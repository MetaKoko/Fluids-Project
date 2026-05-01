#include <omp.h>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <optional>
#include <filesystem>

// =============================================================
// DATA STRUCTURES: MULTI-SWIMMER (NEW CODE)
// =============================================================

struct Sphere_Data {
    // Two coordinate sets
    sf::Vector2<double> Pos_actual;   // coupled universe: full hydro + drawing
    sf::Vector2<double> Pos_iso;      // isolated universe: per-swimmer hydro

    // Forces and velocities
    sf::Vector2<double> F_internal;   // wave forcing (computed from Pos_iso)
    sf::Vector2<double> V_full;       // velocity in coupled universe
    sf::Vector2<double> V_iso;        // velocity in isolated universe

    sf::Color Colour;
    double Rad;
    sf::Vector2<double> F_spring_actual;
    sf::Vector2<double> F_spring_iso;
};

struct Swimmer {
    std::size_t start;     // index of first bead in Spheres_List
    std::size_t count;     // number of beads

    double A;              // amplitude
    double k;              // wavenumber
    double omega;          // angular frequency
    double base_y;         // centerline y
    double phase;          // wave phase
    double start_x;        // starting x position

    // Diagnostics
    double COM_actual_prev_x = 0.0;
    double COM_iso_prev_x = 0.0;
    double accum_time = 0.0;
};


// =============================================================
// GLOBALS: MULTI-SWIMMER
// =============================================================

std::vector<Sphere_Data> Spheres_List;
std::vector<Swimmer> Swimmers;

double Visc = 1e-3;
double Time_Step = 3e-6;

double Domain_Width = 1e-2;
double Domain_Height = 1e-2;
constexpr float VIS_SCALE = 15e5f;

double Global_Spacing = 0.0;
double k_spring_global = 1e-2;
double k_wave_global = 1e-2;

int step_counter = 0;
bool Hydro_On = true;


// =============================================================
// UTILITY HELPERS: MULTI-SWIMMER
// =============================================================

std::size_t swimmerOf(std::size_t bead_index)
{
    for (std::size_t s = 0; s < Swimmers.size(); ++s) {
        const auto& sw = Swimmers[s];
        if (bead_index >= sw.start && bead_index < sw.start + sw.count)
            return s;
    }
    return (std::size_t)-1;
}

sf::Vector2<double> Compute_COM_actual(std::size_t s_index)
{
    const auto& sw = Swimmers[s_index];
    sf::Vector2<double> com{ 0.0,0.0 };

    for (std::size_t i = 0; i < sw.count; ++i)
        com += Spheres_List[sw.start + i].Pos_actual;

    com.x /= sw.count;
    com.y /= sw.count;
    return com;
}

sf::Vector2<double> Compute_COM_iso(std::size_t s_index)
{
    const auto& sw = Swimmers[s_index];
    sf::Vector2<double> com{ 0.0,0.0 };

    for (std::size_t i = 0; i < sw.count; ++i)
        com += Spheres_List[sw.start + i].Pos_iso;

    com.x /= sw.count;
    com.y /= sw.count;
    return com;
}

// =============================================================
// ARC-LENGTH / NEWTON INVERSION (shared)
// =============================================================

double arc_length_integrand(double x, double A, double k)
{
    double dy_dx = A * k * std::cos(k * x);
    return std::sqrt(1.0 + dy_dx * dy_dx);
}

double arc_length_between(double x0, double x1, double A, double k)
{
    int M = 2000;
    double dx = (x1 - x0) / M;
    double s = 0.0;
    double x = x0;
    double f_prev = arc_length_integrand(x, A, k);

    for (int i = 1; i <= M; ++i) {
        x = x0 + i * dx;
        double f_curr = arc_length_integrand(x, A, k);
        s += 0.5 * (f_prev + f_curr) * dx;
        f_prev = f_curr;
    }
    return s;
}

double s_of_x(double x0, double x, double A, double k)
{
    return arc_length_between(x0, x, A, k);
}

double x_of_s(double x0, double s_target, double A, double k)
{
    double x = x0 + s_target; // initial guess: straight chain
    for (int it = 0; it < 8; ++it) {
        double s_val = s_of_x(x0, x, A, k);
        double f = s_val - s_target;
        double fp = arc_length_integrand(x, A, k);
        x -= f / fp;
    }
    return x;
}


// =============================================================
// STOKESLET (shared)
// =============================================================

sf::Vector2<double> Stokeslet(const sf::Vector2<double>& d,
    const sf::Vector2<double>& F,
    double mu)
{
    double x = d.x, y = d.y;
    double r2 = x * x + y * y;
    double r = std::sqrt(r2);

    if (r < 1e-14)
        return { 0.0,0.0 };

    double pi = 3.141592653589793;
    double C = 1.0 / (8.0 * pi * mu);

    double dot = x * F.x + y * F.y;

    return {
        C * (F.x / r + dot * x / (r2 * r)),
        C * (F.y / r + dot * y / (r2 * r))
    };
}

// =============================================================
// STRUCTURAL SPRINGS (MULTI-SWIMMER)
// =============================================================

void Apply_Springs()
{
    for (auto& b : Spheres_List) {
        b.F_spring_actual = { 0.0, 0.0 };
        b.F_spring_iso = { 0.0, 0.0 };
    }

    for (const auto& sw : Swimmers) {
        for (std::size_t i = 0; i + 1 < sw.count; ++i) {

            std::size_t a = sw.start + i;
            std::size_t b = sw.start + i + 1;

            // Actual universe springs
            {
                sf::Vector2<double> pA = Spheres_List[a].Pos_actual;
                sf::Vector2<double> pB = Spheres_List[b].Pos_actual;

                sf::Vector2<double> d = pB - pA;
                double r = std::sqrt(d.x * d.x + d.y * d.y);
                if (r > 1e-14) {
                    double stretch = r - Global_Spacing;
                    double fmag = k_spring_global * stretch;
                    sf::Vector2<double> F = { fmag * d.x / r, fmag * d.y / r };
                    Spheres_List[a].F_spring_actual += F;
                    Spheres_List[b].F_spring_actual -= F;
                }
            }

            // Isolated universe springs
            {
                sf::Vector2<double> pA = Spheres_List[a].Pos_iso;
                sf::Vector2<double> pB = Spheres_List[b].Pos_iso;

                sf::Vector2<double> d = pB - pA;
                double r = std::sqrt(d.x * d.x + d.y * d.y);
                if (r > 1e-14) {
                    double stretch = r - Global_Spacing;
                    double fmag = k_spring_global * stretch;
                    sf::Vector2<double> F = { fmag * d.x / r, fmag * d.y / r };
                    Spheres_List[a].F_spring_iso += F;
                    Spheres_List[b].F_spring_iso -= F;
                }
            }
        }
    }
}


// =============================================================
// WAVE FORCING (MULTI-SWIMMER, from Pos_iso)
// =============================================================

void Apply_Wave_Forcing()
{
    // Clear internal forcing
    for (auto& b : Spheres_List)
        b.F_internal = { 0.0, 0.0 };

    // For each swimmer, compute forcing using its isolated configuration
    for (std::size_t s = 0; s < Swimmers.size(); ++s) {
        auto& sw = Swimmers[s];

        for (std::size_t i = 0; i < sw.count; ++i) {
            std::size_t idx = sw.start + i;
            auto& bead = Spheres_List[idx];

            // Use isolated x-position to define the target waveform
            double x_iso = bead.Pos_iso.x;
            double y_tar = sw.base_y + sw.A * std::sin(sw.k * (x_iso - sw.start_x) + sw.phase);


            double dy = y_tar - bead.Pos_iso.y;

            // This F_internal is then used in BOTH universes
            bead.F_internal.y += k_wave_global * dy;
        }
    }
}



// =============================================================
// SWIMMER INITIALIZATION (MULTI-SWIMMER)
// =============================================================

void InitializeSwimmer_Newton(
    std::size_t N,
    double A,
    double lambda,
    double base_x,
    double base_y,
    double freq,
    double bead_rad)
{
    double pi = 3.141592653589793;
    double k = 2.0 * pi / lambda;
    double omega = 2.0 * pi * freq;

    std::size_t start = Spheres_List.size();

    double x0 = base_x;

    // Total arclength of one wavelength
    double rest_length = lambda / (N - 1);

    Global_Spacing = rest_length; // shared for all swimmers (same geometry)

    for (std::size_t i = 0; i < N; ++i)
    {
        double s_i = i * rest_length;
        double x_i = x_of_s(x0, s_i, A, k);
        double y_i = base_y + A * std::sin(k * (x_i - x0));

        Spheres_List.push_back({
            {x_i, y_i},     // Pos_actual
            {x_i, y_i},     // Pos_iso
            {0.0, 0.0},     // F_internal
            {0.0, 0.0},     // V_full
            {0.0, 0.0},     // V_iso
            sf::Color::Blue,
            bead_rad,
            {0.0, 0.0},     // F_spring_actual
            {0.0, 0.0}      // F_spring_iso
            });
    }

    Swimmers.push_back({
        start,
        N,
        A,
        k,
        omega,
        base_y,
        0.0,   // phase
        0.0,   // COM_actual_prev_x
        0.0,   // COM_iso_prev_x
        0.0    // accum_time
        });
}

// =============================================================
// ADVANCE WAVE PHASES (MULTI-SWIMMER)
// =============================================================
void Advance_Wave_Phases()
{
    for (auto& sw : Swimmers) {
        sw.phase -= sw.omega * Time_Step;
    }
}

// =============================================================
// COMPUTE HYDRODYNAMIC VELOCITIES (MULTI-SWIMMER)
// =============================================================
void Compute_Velocities()
{
    double pi = 3.141592653589793;

    // Reset velocities
    for (auto& b : Spheres_List) {
        b.V_full = { 0.0, 0.0 };
        b.V_iso = { 0.0, 0.0 };
    }

    // Self-mobility in both universes
    for (auto& b : Spheres_List) {
        double zeta = 6.0 * pi * Visc * b.Rad;

        // Coupled universe: internal + actual springs
        b.V_full += (b.F_internal + b.F_spring_actual) / zeta;

        // Isolated universe: internal + isolated springs
        b.V_iso += (b.F_internal + b.F_spring_iso) / zeta;
    }

    std::size_t N = Spheres_List.size();

    // Hydrodynamic interactions
#pragma omp parallel for
    for (int i = 0; i < (int)N; ++i) {

        auto& bi = Spheres_List[i];
        std::size_t si = swimmerOf(i);

        // FULL HYDRODYNAMICS (coupled universe)
        for (std::size_t j = 0; j < N; ++j) {
            if (j == (std::size_t)i) continue;

            const auto& bj = Spheres_List[j];
            sf::Vector2<double> dA = bi.Pos_actual - bj.Pos_actual;

            sf::Vector2<double> vA = Stokeslet(dA, bj.F_internal, Visc);
            bi.V_full += vA;
        }

        // ISOLATED HYDRODYNAMICS (same swimmer only)
        for (std::size_t j = Swimmers[si].start;
            j < Swimmers[si].start + Swimmers[si].count; ++j)
        {
            if (j == (std::size_t)i) continue;

            const auto& bj = Spheres_List[j];
            sf::Vector2<double> dI = bi.Pos_iso - bj.Pos_iso;

            sf::Vector2<double> Fsrc_iso = bj.F_internal + bj.F_spring_iso;  // <-- NEW
            sf::Vector2<double> vI = Stokeslet(dI, Fsrc_iso, Visc);          // <-- use total
            bi.V_iso += vI;
        }

    }
}

// =============================================================
// ONE PHYSICS STEP (MULTI-SWIMMER)
// =============================================================
void Do_Physics_Step()
{
    double pi = 3.141592653589793;

    // 1) Springs
    Apply_Springs();

    // 2) Wave forcing (from isolated universe)
    Apply_Wave_Forcing();

    // 3) Hydrodynamics
    if (Hydro_On) {
        Compute_Velocities();
    }
    else {
        for (auto& b : Spheres_List) {
            double zeta = 6.0 * pi * Visc * b.Rad;
            sf::Vector2<double> selfV = { (b.F_internal.x + b.F_spring_actual.x) / zeta,
                                          (b.F_internal.y + b.F_spring_actual.y) / zeta };
            b.V_full = selfV;
            b.V_iso = selfV;
        }
    }

    // 4) Integrate both coordinate sets
    for (auto& b : Spheres_List) {
        b.Pos_actual += b.V_full * Time_Step;
        b.Pos_iso += b.V_iso * Time_Step;
    }

    // 5) Diagnostics: speeds per swimmer
    for (std::size_t s = 0; s < Swimmers.size(); ++s) {

        auto& sw = Swimmers[s];

        sf::Vector2<double> COM_actual = Compute_COM_actual(s);
        sf::Vector2<double> COM_iso = Compute_COM_iso(s);

        if (sw.accum_time == 0.0) {
            sw.COM_actual_prev_x = COM_actual.x;
            sw.COM_iso_prev_x = COM_iso.x;
        }

        sw.accum_time += Time_Step;

        double T = 2.0 * pi / sw.omega;

        if (sw.accum_time >= T) {

            double dx_actual = COM_actual.x - sw.COM_actual_prev_x;
            double dx_iso = COM_iso.x - sw.COM_iso_prev_x;

            double avg_speed_actual = dx_actual / sw.accum_time;
            double avg_speed_iso = dx_iso / sw.accum_time;

            std::cout << "[Multi] Swimmer " << s
                << " coupled_speed = " << avg_speed_actual
                << " , iso_speed = " << avg_speed_iso << "\n";

            sw.COM_actual_prev_x = COM_actual.x;
            sw.COM_iso_prev_x = COM_iso.x;
            sw.accum_time = 0.0;
        }
    }

    step_counter++;
}

// =============================================================
// WRAPPING FOR DISPLAY (MULTI-SWIMMER)
// =============================================================
sf::Vector2<double> Wrap_For_Display(sf::Vector2<double> p) {
    p.x = fmod(p.x, Domain_Width);
    if (p.x < 0) p.x += Domain_Width;

    p.y = fmod(p.y, Domain_Height);
    if (p.y < 0) p.y += Domain_Height;

    return p;
}

// =============================================================
// DRAWING: BEADS (Pos_actual only)
// =============================================================
void Draw_Sphere(std::vector<sf::Vertex>& lines, std::size_t index) {
    const Sphere_Data& sphere = Spheres_List[index];
    sf::Vector2<double> center = Wrap_For_Display(sphere.Pos_actual);

    double Rad_px = sphere.Rad * VIS_SCALE;
    int Segments = 40;
    double pi = 3.141592653589793;

    float cx = static_cast<float>(center.x * VIS_SCALE);
    float cy = static_cast<float>(center.y * VIS_SCALE);

    for (int i = 0; i < Segments; i++) {
        double a1 = 2.0 * pi * i / Segments;
        double a2 = 2.0 * pi * (i + 1) / Segments;

        sf::Vector2f p1(cx + static_cast<float>(Rad_px * std::cos(a1)),
            cy + static_cast<float>(Rad_px * std::sin(a1)));

        sf::Vector2f p2(cx + static_cast<float>(Rad_px * std::cos(a2)),
            cy + static_cast<float>(Rad_px * std::sin(a2)));

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }
}

// =============================================================
// DRAWING: SPRING CONNECTIONS (Pos_actual only)
// =============================================================
void Draw_Connections(std::vector<sf::Vertex>& lines) {
    for (const auto& sw : Swimmers) {
        std::size_t start = sw.start;
        std::size_t end = start + sw.count;

        for (std::size_t i = start; i + 1 < end; ++i) {
            sf::Vector2<double> pA = Wrap_For_Display(Spheres_List[i].Pos_actual);
            sf::Vector2<double> pB = Wrap_For_Display(Spheres_List[i + 1].Pos_actual);

            sf::Vector2f A_px(static_cast<float>(pA.x * VIS_SCALE),
                static_cast<float>(pA.y * VIS_SCALE));

            sf::Vector2f B_px(static_cast<float>(pB.x * VIS_SCALE),
                static_cast<float>(pB.y * VIS_SCALE));

            lines.push_back(sf::Vertex(A_px, sf::Color::Red));
            lines.push_back(sf::Vertex(B_px, sf::Color::Red));
        }
    }
}

// =============================================================
// DRAWING: WAVEFORM GUIDES (visual, lab-frame using Pos_actual.x)
// =============================================================
void Draw_WaveformGuides(std::vector<sf::Vertex>& lines, double bead_rad)
{
    if (Spheres_List.empty() || Swimmers.empty()) return;

    double offset = 1.0 * bead_rad;

    for (std::size_t s = 0; s < Swimmers.size(); ++s) {
        const auto& sw = Swimmers[s];

        std::size_t start = sw.start;
        std::size_t end = start + sw.count;

        for (std::size_t i = start; i + 1 < end; ++i) {

            double xA = Spheres_List[i].Pos_actual.x;
            double xB = Spheres_List[i + 1].Pos_actual.x;

            double yA = sw.base_y + sw.A * std::sin(sw.k * xA + sw.phase);
            double yB = sw.base_y + sw.A * std::sin(sw.k * xB + sw.phase);

            sf::Vector2<double> uA = Wrap_For_Display({ xA, yA + offset });
            sf::Vector2<double> uB = Wrap_For_Display({ xB, yB + offset });

            sf::Vector2<double> lA = Wrap_For_Display({ xA, yA - offset });
            sf::Vector2<double> lB = Wrap_For_Display({ xB, yB - offset });

            lines.push_back(sf::Vertex(
                { float(uA.x * VIS_SCALE), float(uA.y * VIS_SCALE) },
                sf::Color::Black));
            lines.push_back(sf::Vertex(
                { float(uB.x * VIS_SCALE), float(uB.y * VIS_SCALE) },
                sf::Color::Black));

            lines.push_back(sf::Vertex(
                { float(lA.x * VIS_SCALE), float(lA.y * VIS_SCALE) },
                sf::Color::Black));
            lines.push_back(sf::Vertex(
                { float(lB.x * VIS_SCALE), float(lB.y * VIS_SCALE) },
                sf::Color::Black));
        }
    }
}


// =============================================================
// DRAW EVERYTHING (MULTI-SWIMMER)
// =============================================================
void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();

    if (!Spheres_List.empty())
        Draw_WaveformGuides(lines, Spheres_List[0].Rad);

    Draw_Connections(lines);

    for (std::size_t i = 0; i < Spheres_List.size(); ++i)
        Draw_Sphere(lines, i);
}

// =============================================================
// SAVE FRAME (for video)
// =============================================================
void SaveFrame(sf::RenderWindow& window, int frameNumber)
{
    std::filesystem::create_directories("frames");

    sf::Texture texture;
    texture.resize(window.getSize());
    texture.update(window);

    sf::Image screenshot = texture.copyToImage();

    std::string filename = "frames/frame_" + std::to_string(frameNumber) + ".png";

    if (!screenshot.saveToFile(filename))
        std::cout << "Failed to save: " << filename << "\n";
    else
        std::cout << "Saved: " << filename << "\n";
}


// =============================================================
// SOLO SIMULATION (OLD CODE) EMBEDDED
// =============================================================

struct SoloSphere {
    sf::Vector2<double> Pos_Unwrapped;
    sf::Vector2<double> F_Vector;
    sf::Vector2<double> V;
    double Rad;
};

std::vector<SoloSphere> SoloSpheres;
std::vector<sf::Vector2<double>> Solo_Target_Pos;

double Solo_Global_Spacing = 0.0;
double Solo_base_y_global = 0.0;
double Solo_A_global = 0.0;
double Solo_k_global = 0.0;
double Solo_omega_global = 0.0;
double Solo_k_wave_global = 0.0;
double Solo_phase_global = 0.0;

sf::Vector2<double> Solo_COM_prev = { 0.0, 0.0 };
sf::Vector2<double> Solo_COM_current = { 0.0, 0.0 };

void Solo_Add_Sphere(const sf::Vector2<double>& Pos,
    const sf::Vector2<double>& F_Vector,
    double Rad)
{
    SoloSpheres.push_back({ Pos, F_Vector, sf::Vector2<double>(0.0, 0.0), Rad });
}

sf::Vector2<double> Solo_Compute_COM()
{
    sf::Vector2<double> com = { 0.0, 0.0 };
    for (const auto& s : SoloSpheres)
        com += s.Pos_Unwrapped;

    com.x /= SoloSpheres.size();
    com.y /= SoloSpheres.size();
    return com;
}

void Solo_Apply_Structural_Springs(double k_spring, double rest_length)
{
    for (size_t i = 0; i + 1 < SoloSpheres.size(); i++) {

        sf::Vector2<double> pA = SoloSpheres[i].Pos_Unwrapped;
        sf::Vector2<double> pB = SoloSpheres[i + 1].Pos_Unwrapped;

        sf::Vector2<double> d = pB - pA;
        double r2 = d.x * d.x + d.y * d.y;
        double r = std::sqrt(r2);

        double stretch = r - rest_length;
        double fmag = k_spring * stretch;

        sf::Vector2<double> F = { fmag * d.x / r, fmag * d.y / r };

        SoloSpheres[i].F_Vector += F;
        SoloSpheres[i + 1].F_Vector -= F;
    }
}

void Solo_Calc_Sphere_Velocity(size_t Sphere_Index, sf::Vector2<double>& V_Vector)
{
    SoloSphere target = SoloSpheres[Sphere_Index];
    V_Vector = { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double self_mob = 1.0 / (6.0 * pi * Visc * target.Rad);

    V_Vector += target.F_Vector * self_mob;

    for (size_t i = 0; i < SoloSpheres.size(); i++) {
        if (i == Sphere_Index) continue;

        SoloSphere source = SoloSpheres[i];
        sf::Vector2<double> Dist = target.Pos_Unwrapped - source.Pos_Unwrapped;
        V_Vector += Stokeslet(Dist, source.F_Vector, Visc);
    }
}

void Solo_Update_Position(double k_spring)
{
    double pi = 3.141592653589793;

    for (auto& s : SoloSpheres)
        s.F_Vector = { 0.0, 0.0 };

    double rest = Solo_Global_Spacing;
    Solo_Apply_Structural_Springs(k_spring, rest);

    for (size_t i = 0; i < SoloSpheres.size(); i++)
    {
        double dy = Solo_Target_Pos[i].y - SoloSpheres[i].Pos_Unwrapped.y;
        SoloSpheres[i].F_Vector.y += (Solo_k_wave_global * dy);
    }

    // Hydrodynamics ON (as in old code)
#pragma omp parallel for
    for (int i = 0; i < (int)SoloSpheres.size(); i++)
        Solo_Calc_Sphere_Velocity(i, SoloSpheres[i].V);

    for (auto& s : SoloSpheres)
        s.Pos_Unwrapped += s.V * Time_Step;
}

void Solo_InitializeChain(size_t N,
    double A,
    double lambda,
    double L_arc,
    double base_x,
    double base_y,
    double freq,
    double bead_rad)
{
    double pi = 3.141592653589793;
    double k = 2.0 * pi / lambda;

    Solo_A_global = A;
    Solo_k_global = k;
    Solo_base_y_global = base_y;
    Solo_omega_global = 2.0 * pi * freq;
    Solo_k_wave_global = 1e-2;
    Solo_phase_global = 0.0;

    SoloSpheres.clear();
    Solo_Target_Pos.assign(N, sf::Vector2<double>(0.0, 0.0));

    double x0 = base_x;
    double rest_length = L_arc / (N - 1);
    Solo_Global_Spacing = rest_length;

    for (size_t i = 0; i < N; ++i)
    {
        double s_i = (double)i * rest_length;
        double x_i = x_of_s(x0, s_i, A, k);
        double y_i = base_y + A * std::sin(k * (x_i - x0) + Solo_phase_global);

        Solo_Add_Sphere({ x_i, y_i }, { 0.0, 0.0 }, bead_rad);

        Solo_Target_Pos[i].x = x_i;
        Solo_Target_Pos[i].y = y_i;
    }

    Solo_COM_prev = Solo_Compute_COM();
    Solo_COM_current = Solo_COM_prev;
}

void Solo_Step(double k_spring)
{
    double pi = 3.141592653589793;

    double old_phase = Solo_phase_global;
    (void)old_phase;
    Solo_phase_global -= Solo_omega_global * Time_Step;

    for (size_t i = 0; i < SoloSpheres.size(); i++) {
        Solo_Target_Pos[i].x = SoloSpheres[i].Pos_Unwrapped.x;
        Solo_Target_Pos[i].y =
            Solo_base_y_global +
            Solo_A_global * std::sin(Solo_k_global * (Solo_Target_Pos[i].x) + Solo_phase_global);
    }

    Solo_Update_Position(k_spring);

    Solo_COM_prev = Solo_COM_current;
    Solo_COM_current = Solo_Compute_COM();
}

// =============================================================
// MAIN
// =============================================================
int main() {
    sf::VideoMode mode = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(mode, "Multi-swimmer + Solo reference");
    window.setFramerateLimit(60);

    bool dragging = false;
    sf::Vector2i lastMouse;
    int frame = 0;
    int saveEvery = 20;

    double bead_rad = 7.5e-6;
    double lambda = 5e-4;
    double freq = 1.92;

    k_spring_global = 1e-2;
    k_wave_global = 1e-2;

    // ===================== DEFINE MULTI-SWIMMERS =====================
    struct SwimmerParams {
        std::size_t N;
        double A;
        double base_x;
        double base_y;
    };

    std::vector<SwimmerParams> swimmers_params = {
        {25, 2.0e-5, 0.515 * Domain_Width, 0.507 * Domain_Height},
        {25, 2.0e-5, 0.50 * Domain_Width, 0.493 * Domain_Height}
    };

    for (const auto& sp : swimmers_params) {
        InitializeSwimmer_Newton(
            sp.N,
            sp.A,
            lambda,
            sp.base_x,
            sp.base_y,
            freq,
            bead_rad
        );
    }

    // ===================== SOLO SWIMMER (OLD CODE) =====================
    // Same geometry as swimmer 0, but single chain
    std::size_t N_solo = 25;
    double A_solo = 2.0e-5;
    double L_arc_solo = 5e-4;
    double base_x_solo = 0.50 * Domain_Width;
    double base_y_solo = 0.50 * Domain_Height;
    double k_spring_solo = 1e-2;

    Solo_InitializeChain(
        N_solo,
        A_solo,
        lambda,
        L_arc_solo,
        base_x_solo,
        base_y_solo,
        freq,
        bead_rad
    );

    // Camera: center on first multi-swimmer
    sf::Vector2<double> COM0 = Compute_COM_actual(0);
    sf::Vector2f COM0_px(static_cast<float>(COM0.x * VIS_SCALE),
        static_cast<float>(COM0.y * VIS_SCALE));

    sf::Vector2u winSize = window.getSize();

    sf::FloatRect viewRect(
        sf::Vector2f(
            COM0_px.x - 0.5f * static_cast<float>(winSize.x),
            COM0_px.y - 0.5f * static_cast<float>(winSize.y)
        ),
        sf::Vector2f(static_cast<float>(winSize.x),
            static_cast<float>(winSize.y))
    );

    sf::View camera(viewRect);
    window.setView(camera);

    std::vector<sf::Vertex> allLines;

    double pi = 3.141592653589793;
    double solo_accum_time = 0.0;
    double solo_COM_start_x = Solo_COM_current.x;

    // =============================================================
    // MAIN LOOP
    // =============================================================
    while (window.isOpen()) {

        while (const std::optional<sf::Event> eventOpt = window.pollEvent()) {
            const sf::Event& event = *eventOpt;

            if (event.is<sf::Event::Closed>()) {
                window.close();
                continue;
            }

            if (const auto* mb = event.getIf<sf::Event::MouseButtonPressed>()) {
                if (mb->button == sf::Mouse::Button::Left) {
                    dragging = true;
                    lastMouse = sf::Mouse::getPosition(window);
                }
            }

            if (const auto* mb = event.getIf<sf::Event::MouseButtonReleased>()) {
                if (mb->button == sf::Mouse::Button::Left) {
                    dragging = false;
                }
            }

            if (const auto* mm = event.getIf<sf::Event::MouseMoved>()) {
                if (dragging) {
                    sf::Vector2i current = sf::Mouse::getPosition(window);
                    sf::Vector2i delta = current - lastMouse;
                    camera.move(sf::Vector2f(-(float)delta.x, -(float)delta.y));
                    lastMouse = current;
                }
            }

            if (const auto* mw = event.getIf<sf::Event::MouseWheelScrolled>()) {
                float factor = (mw->delta > 0.f) ? 0.9f : 1.1f;
                camera.zoom(factor);
            }

            if (const auto* key = event.getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::S) {
                    SaveFrame(window, frame);
                }
            }
        }

        int PhysicsStepsPerFrame = 100;
        for (int s = 0; s < PhysicsStepsPerFrame; ++s) {

            // Step SOLO simulation (old code)
            Solo_Step(k_spring_solo);

            // Measure SOLO speed over one period
            solo_accum_time += Time_Step;
            double T_solo = 2.0 * pi / Solo_omega_global;
            if (solo_accum_time >= T_solo) {
                double solo_COM_end_x = Solo_COM_current.x;
                double solo_avg_speed = (solo_COM_end_x - solo_COM_start_x) / solo_accum_time;
                std::cout << "[Solo] avg_speed = " << solo_avg_speed << "\n";
                solo_COM_start_x = solo_COM_end_x;
                solo_accum_time = 0.0;
            }

            // Step MULTI-SWIMMER simulation (new code)
            Advance_Wave_Phases();
            Do_Physics_Step();
        }

        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.setView(camera);
        if (!allLines.empty())
            window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);

        if (frame % saveEvery == 0) {
            SaveFrame(window, frame / saveEvery);
        }

        frame++;
        window.display();
    }

    return 0;
}
