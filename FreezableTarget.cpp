#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

// =============================================================
// Data structures
// =============================================================
struct Sphere_Data {
    sf::Vector2<double> Pos_Unwrapped;   // PATCH: unwrapped physical position
    sf::Vector2<double> F_Vector;        // force (N)
    sf::Color Colour;
    sf::Vector2<double> V;               // velocity (m/s)
    double Rad;                          // radius (meters)
};

std::vector<Sphere_Data> Spheres_List;
std::vector<sf::Vector2<double>> Base_Pos;
double Global_Spacing = 0.0;
std::vector<double> Rest_Lengths;
std::vector<sf::Vector2<double>> Curv_rest;
std::vector<sf::Vector2<double>> Target_Pos;   // current target for each bead
std::vector<bool> Target_Reached;              // whether bead has reached/passed target
double Wave_Phase = 0.0;                       // global wave phase





// =============================================================
// Physical + numerical parameters
// =============================================================
double Visc = 1e-3;
double Time_Step = 1e-4;

double Domain_Width = 2e-3;
double Domain_Height = 2e-3;

constexpr float VIS_SCALE = 2e7f;

sf::Vector2<double> Dir;


// =============================================================
// Helpers
// =============================================================
void Add_Sphere(const sf::Vector2<double>& Pos,
    const sf::Vector2<double>& F_Vector,
    double Rad = 10.0,
    sf::Color Colour = sf::Color::Blue)
{
    Spheres_List.push_back({
        Pos,            // PATCH: store unwrapped position
        F_Vector,
        Colour,
        sf::Vector2<double>(0.0, 0.0),
        Rad
        });
}

sf::Vector2<double> Minimum_Image(sf::Vector2<double> d) {
    if (d.x > 0.5 * Domain_Width)  d.x -= Domain_Width;
    if (d.x < -0.5 * Domain_Width) d.x += Domain_Width;
    if (d.y > 0.5 * Domain_Height) d.y -= Domain_Height;
    if (d.y < -0.5 * Domain_Height) d.y += Domain_Height;
    return d;
}

sf::Vector2<double> Stokeslet(sf::Vector2<double> Dist,
    sf::Vector2<double> F_Vector)
{
    double r2 = Dist.x * Dist.x + Dist.y * Dist.y;
    double r = std::sqrt(r2);
    if (r < 1e-14) return { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double Constant = 1.0 / (8.0 * pi * Visc);
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;

    double r3 = r2 * r;

    return {
        Constant * (F_Vector.x / r + Dot_Prod * Dist.x / r3),
        Constant * (F_Vector.y / r + Dot_Prod * Dist.y / r3)
    };
}

sf::Vector2<double> faxen_Correction(sf::Vector2<double> Dist,
    sf::Vector2<double> F_Vector,
    double a2)
{
    double r2 = Dist.x * Dist.x + Dist.y * Dist.y;
    double r = std::sqrt(r2);
    if (r < 1e-14) return { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = a2 / (48.0 * pi * Visc);

    double r3 = r2 * r;
    double r5 = r3 * r2;

    return {
        Constant * (F_Vector.x / r3 - 3.0 * Dot_Prod * Dist.x / r5),
        Constant * (F_Vector.y / r3 - 3.0 * Dot_Prod * Dist.y / r5)
    };
}

sf::Vector2<double> faxen_Correction_Of_faxen(sf::Vector2<double> Dist,
    sf::Vector2<double> F_Vector,
    double a2_source,
    double a2_target)
{
    double r2 = Dist.x * Dist.x + Dist.y * Dist.y;
    double r = std::sqrt(r2);
    if (r < 1e-14) return { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = (a2_target * a2_source) / (288.0 * pi * Visc);

    double r5 = r2 * r2 * r;
    double r7 = r5 * r2;
    double r9 = r7 * r2;

    return {
        Constant * (9.0 * F_Vector.x / r5 - 45.0 * Dot_Prod * Dist.x / r7 + 105.0 * Dot_Prod * Dist.x / r9),
        Constant * (9.0 * F_Vector.y / r5 - 45.0 * Dot_Prod * Dist.y / r7 + 105.0 * Dot_Prod * Dist.y / r9)
    };
}

void Calc_Sphere_Velocity(size_t Sphere_Index, sf::Vector2<double>& V_Vector) {
    Sphere_Data target = Spheres_List[Sphere_Index];
    V_Vector = { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double self_mob = 1.0 / (6.0 * pi * Visc * target.Rad);

    V_Vector += target.F_Vector * self_mob;

    for (size_t i = 0; i < Spheres_List.size(); i++) {
        if (i == Sphere_Index) continue;

        Sphere_Data source = Spheres_List[i];

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {

                sf::Vector2<double> Image_Pos = source.Pos_Unwrapped; // PATCH
                Image_Pos.x += dx * Domain_Width;
                Image_Pos.y += dy * Domain_Height;

                sf::Vector2<double> Dist =
                    Minimum_Image(target.Pos_Unwrapped - Image_Pos); // PATCH

                V_Vector += Stokeslet(Dist, source.F_Vector);
                V_Vector += faxen_Correction(Dist, source.F_Vector, source.Rad * source.Rad);
                V_Vector += faxen_Correction(Dist, source.F_Vector, target.Rad * target.Rad);
                V_Vector += faxen_Correction_Of_faxen(Dist, source.F_Vector,
                    source.Rad * source.Rad,
                    target.Rad * target.Rad);
            }
        }
    }
}

void Apply_Structural_Springs(double k_struct, double spacing) {
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        sf::Vector2<double> dx =
            Spheres_List[i + 1].Pos_Unwrapped - Spheres_List[i].Pos_Unwrapped; // PATCH

        double L = std::sqrt(dx.x * dx.x + dx.y * dx.y);
        if (L < 1e-14) continue;

        double L0 = Rest_Lengths[i];

        sf::Vector2<double> F = k_struct * (L - L0) * (dx / L);

        Spheres_List[i].F_Vector += F;
        Spheres_List[i + 1].F_Vector -= F;
    }
}

void Apply_Bending_Curvature(double k_bend,
    const std::vector<sf::Vector2<double>>& curv_rest)
{
    size_t N = Spheres_List.size();
    if (curv_rest.size() < N) return;

    for (size_t i = 1; i + 1 < N; i++) {

        sf::Vector2<double> x0 = Spheres_List[i - 1].Pos_Unwrapped;
        sf::Vector2<double> x1 = Spheres_List[i].Pos_Unwrapped;
        sf::Vector2<double> x2 = Spheres_List[i + 1].Pos_Unwrapped;

        // Discrete curvature vector: second difference
        sf::Vector2<double> curv = x0 - 2.0 * x1 + x2;

        sf::Vector2<double> diff = curv - curv_rest[i];

        // Quadratic energy: 0.5 * k_bend * |diff|^2
        // Forces are linear in diff, no singularities:
        sf::Vector2<double> F0 = -k_bend * diff;
        sf::Vector2<double> F1 = 2.0 * k_bend * diff;
        sf::Vector2<double> F2 = -k_bend * diff;

        Spheres_List[i - 1].F_Vector += F0;
        Spheres_List[i].F_Vector += F1;
        Spheres_List[i + 1].F_Vector += F2;
    }
}



sf::Vector2<double> Compute_COM() {
    sf::Vector2<double> com(0.0, 0.0);
    for (const auto& s : Spheres_List)
        com += s.Pos_Unwrapped;
    return com / (double)Spheres_List.size();
}

sf::Vector2<double> Compute_Net_Force() {
    sf::Vector2<double> net(0.0, 0.0);
    for (const auto& s : Spheres_List)
        net += s.F_Vector;
    return net;
}


// =============================================================
// Time stepping + averaged speed
// =============================================================
void Update_Position(double k_struct, double k_aut)
{
    static sf::Vector2<double> Prev_COM = Compute_COM();
    double tol = 0.1 * Global_Spacing;
    static int counter = 0;

    static double accum_disp = 0.0;
    static double accum_time = 0.0;
    static double last_COM_x = 0.0;
    static double last_COM_y = 0.0;
    static bool first = true;

    static sf::Vector2<double> Dir_fixed = Dir;

    for (auto& s : Spheres_List)
        s.F_Vector = { 0.0, 0.0 };

    Apply_Structural_Springs(k_struct, Global_Spacing);
    // --- Wave spring forces (global freeze logic) ---
    for (size_t i = 0; i < Spheres_List.size(); i++) {

        sf::Vector2<double> dx = Target_Pos[i] - Spheres_List[i].Pos_Unwrapped;
        double dist = std::sqrt(dx.x * dx.x + dx.y * dx.y);

        // Only apply force if bead has NOT reached its target
        if (dist > tol) {
            Spheres_List[i].F_Vector += k_aut * dx;   // k_aut is k_wave
        }
    }




    sf::Vector2<double> Current_COM = Compute_COM();

    if (first) {
        last_COM_x = Current_COM.x;
        last_COM_y = Current_COM.y;
        first = false;
    }

    sf::Vector2<double> COM_Vel = (Current_COM - Prev_COM) / Time_Step;

    double Speed_inst = COM_Vel.x * Dir_fixed.x + COM_Vel.y * Dir_fixed.y;

    sf::Vector2<double> dCOM(
        Current_COM.x - last_COM_x,
        Current_COM.y - last_COM_y
    );

    double d_forward = dCOM.x * Dir_fixed.x + dCOM.y * Dir_fixed.y;

    accum_disp += d_forward;
    accum_time += Time_Step;

    last_COM_x = Current_COM.x;
    last_COM_y = Current_COM.y;

    Prev_COM = Current_COM;

    double pi = 3.141592653589793;
    double omega = 2.0 * pi * 25.0;
    double T = 2.0 * pi / omega;

    if (accum_time >= T) {
        double avg_speed = accum_disp / accum_time;

        std::cout << std::scientific << std::setprecision(12)
            << "Averaged speed = " << avg_speed << "\n";

        accum_disp = 0.0;
        accum_time = 0.0;
    }

    counter++;
    if (counter % 100 == 0) {
        sf::Vector2<double> netF = Compute_Net_Force();
        std::cout << std::scientific << std::setprecision(12)
            << "NetF = (" << netF.x << ", " << netF.y << ")"
            << "   Inst speed = " << Speed_inst << "\n";
    }
    // === Structural spring stretch diagnostic ===
    if (counter % 100 == 0) {
        double max_rel_error = 0.0;

        for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {
            sf::Vector2<double> dx =
                Spheres_List[i + 1].Pos_Unwrapped - Spheres_List[i].Pos_Unwrapped;

            double L = std::sqrt(dx.x * dx.x + dx.y * dx.y);
            double L0 = Rest_Lengths[i];

            double rel_err = std::abs(L - L0) / L0;
            if (rel_err > max_rel_error) max_rel_error = rel_err;
        }

        std::cout << "Max relative stretch = " << max_rel_error << "\n";
    }


    // === Semi‑implicit Euler for stiff springs ===
    for (size_t i = 0; i < Spheres_List.size(); i++) {

        double pi = 3.141592653589793;
        double zeta = 6.0 * pi * Visc * Spheres_List[i].Rad;

        sf::Vector2<double> Vh = Spheres_List[i].F_Vector / zeta;
        double denom = 1.0 + (k_struct / zeta) * Time_Step;

        Spheres_List[i].V = Vh / denom;

        // *** THIS WAS MISSING ***
        Spheres_List[i].Pos_Unwrapped += Spheres_List[i].V * Time_Step;
    }


}


// =============================================================
// Visualisation helpers
// =============================================================
sf::Vector2<double> Wrap_For_Display(sf::Vector2<double> p) {
    p.x = fmod(p.x, Domain_Width);
    if (p.x < 0) p.x += Domain_Width;

    p.y = fmod(p.y, Domain_Height);
    if (p.y < 0) p.y += Domain_Height;

    return p;
}

void Draw_Sphere(std::vector<sf::Vertex>& lines, size_t index) {
    const Sphere_Data& sphere = Spheres_List[index];
    sf::Vector2<double> center = Wrap_For_Display(sphere.Pos_Unwrapped);

    double Rad_px = 2.0;
    int Segments = 40;
    double pi = 3.141592653589793;

    float cx = (float)(center.x * VIS_SCALE);
    float cy = (float)(center.y * VIS_SCALE);

    for (int i = 0; i < Segments; i++) {
        double a1 = 2.0 * pi * i / Segments;
        double a2 = 2.0 * pi * (i + 1) / Segments;

        sf::Vector2f p1(
            cx + (float)(Rad_px * std::cos(a1)),
            cy + (float)(Rad_px * std::sin(a1))
        );

        sf::Vector2f p2(
            cx + (float)(Rad_px * std::cos(a2)),
            cy + (float)(Rad_px * std::sin(a2))
        );

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }
}

void Draw_Connections(std::vector<sf::Vertex>& lines) {
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {
        sf::Vector2<double> pA = Wrap_For_Display(Spheres_List[i].Pos_Unwrapped);
        sf::Vector2<double> pB = Wrap_For_Display(Spheres_List[i + 1].Pos_Unwrapped);

        sf::Vector2f A_px((float)(pA.x * VIS_SCALE),
            (float)(pA.y * VIS_SCALE));

        sf::Vector2f B_px((float)(pB.x * VIS_SCALE),
            (float)(pB.y * VIS_SCALE));

        lines.push_back(sf::Vertex(A_px, sf::Color::Red));
        lines.push_back(sf::Vertex(B_px, sf::Color::Red));
    }
}

void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();
    Draw_Connections(lines);
    for (size_t i = 0; i < Spheres_List.size(); i++)
        Draw_Sphere(lines, i);
}


// =============================================================
// main
// =============================================================
int main() {
    sf::VideoMode mode = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(mode, "Travelling Wave Swimmer (Global-Freeze Targets)");
    window.setFramerateLimit(60);

    bool dragging = false;
    sf::Vector2i lastMouse;

    size_t N = 30;
    double radius = 0.3e-6;
    double length = 50e-6 - 2 * radius;
    double spacing = length / (N - 1);
    Global_Spacing = spacing;

    // Structural stiffness + wave spring stiffness
    double k_str = 1;
    double k_wave = 1;   // use this instead of k_bend inside Update_Position

    double base_x = 0.5 * Domain_Width;
    double base_y = 0.5 * Domain_Height;

    double pi = 3.141592653589793;
    double Angle = pi / 4.0;

    // Initial swimming direction and amplitude direction
    Dir = sf::Vector2<double>(std::cos(Angle), std::sin(Angle));
    sf::Vector2<double> Normal(-Dir.y, Dir.x);   // amplitude direction

    // Wave parameters
    double A = 3e-6;        // amplitude (meters)
    double lambda = 30e-6;       // wavelength
    double k = 2.0 * pi / lambda;
    double f = 25.0;
    double omega = 2.0 * pi * f;

    Base_Pos.resize(N);
    std::vector<double> S_param(N);

    // --- ARC-LENGTH INITIALISATION ---
    double ds = spacing * 0.05;
    double s = 0.0;

    for (size_t i = 0; i < N; i++) {
        S_param[i] = s;

        double wave = A * std::sin(k * s);
        Base_Pos[i] = {
            base_x + Dir.x * s + Normal.x * wave,
            base_y + Dir.y * s + Normal.y * wave
        };

        Add_Sphere(Base_Pos[i], { 0.0, 0.0 }, radius, sf::Color::Blue);

        double accumulated = 0.0;
        while (accumulated < spacing) {
            double slope = A * k * std::cos(k * s);
            double ds_arc = std::sqrt(1.0 + slope * slope) * ds;
            accumulated += ds_arc;
            s += ds;
        }
    }

    Rest_Lengths.resize(N - 1);
    for (size_t i = 0; i + 1 < N; ++i) {
        sf::Vector2<double> dx = Base_Pos[i + 1] - Base_Pos[i];
        Rest_Lengths[i] = std::sqrt(dx.x * dx.x + dx.y * dx.y);
    }

    // --- TARGET POSITIONS FOR GLOBAL-FREEZE WAVE ---
    Target_Pos.assign(N, sf::Vector2<double>(0.0, 0.0));
    for (size_t i = 0; i < N; ++i)
        Target_Pos[i] = Base_Pos[i];

    double Wave_Phase = 0.0;                 // global phase of the wave
    double tol = 0.1 * Global_Spacing; // distance tolerance for "reached"

    sf::Vector2<double> COM0 = Compute_COM();
    sf::Vector2f COM0_px((float)(COM0.x * VIS_SCALE),
        (float)(COM0.y * VIS_SCALE));

    sf::Vector2u winSize = window.getSize();

    sf::FloatRect viewRect(
        sf::Vector2f(
            COM0_px.x - 0.5f * (float)winSize.x,
            COM0_px.y - 0.5f * (float)winSize.y
        ),
        sf::Vector2f(
            (float)winSize.x,
            (float)winSize.y
        )
    );

    sf::View camera(viewRect);
    window.setView(camera);

    static double t_wave = 0.0;
    std::vector<sf::Vertex> allLines;

    while (window.isOpen()) {

        // ------------------ EVENTS ------------------
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
        }

        // ------------------ UPDATE GLOBAL WAVE TARGETS (GLOBAL FREEZE) ------------------
        t_wave += Time_Step;

        // Propose next targets based on next phase
        double next_phase = Wave_Phase - omega * Time_Step;
        std::vector<sf::Vector2<double>> Next_Target(N);

        for (size_t i = 0; i < N; i++) {
            double s_i = S_param[i];
            double wave = A * std::sin(k * s_i + next_phase);

            // Amplitude direction is fixed Normal; propagation axis is free to evolve physically
// Compute local tangent from current geometry
            sf::Vector2<double> t;

            if (i == 0) {
                t = Spheres_List[1].Pos_Unwrapped - Spheres_List[0].Pos_Unwrapped;
            }
            else if (i == N - 1) {
                t = Spheres_List[N - 1].Pos_Unwrapped - Spheres_List[N - 2].Pos_Unwrapped;
            }
            else {
                t = Spheres_List[i + 1].Pos_Unwrapped - Spheres_List[i - 1].Pos_Unwrapped;
            }

            double Lt = std::sqrt(t.x * t.x + t.y * t.y);
            if (Lt < 1e-12) Lt = 1e-12;
            t /= Lt;

            // Local normal (perpendicular to tangent)
            sf::Vector2<double> n(-t.y, t.x);

            // Target position using *current* normal
            Next_Target[i] = Spheres_List[i].Pos_Unwrapped + n * wave;

        }

        // Check if all beads are within tol of their proposed next targets
        bool all_reached = true;
        for (size_t i = 0; i < N; i++) {
            sf::Vector2<double> dx = Next_Target[i] - Spheres_List[i].Pos_Unwrapped;
            double dist = std::sqrt(dx.x * dx.x + dx.y * dx.y);
            if (dist > tol) {
                all_reached = false;
                break;
            }
        }

        // If all beads reached → advance wave and commit new targets
        if (all_reached) {
            Wave_Phase = next_phase;
            Target_Pos = Next_Target;
        }
        // else: global freeze → keep Target_Pos as is

        // ------------------ PHYSICS STEP ------------------
        // Inside Update_Position you should:
        //  - zero F_Vector
        //  - Apply_Structural_Springs(k_str, Global_Spacing);
        //  - for each i: apply wave spring only if bead not within tol of Target_Pos[i]:
        //        dx = Target_Pos[i] - Pos_Unwrapped;
        //        if (|dx| > tol) F += k_wave * dx;
        //  - then do your overdamped/semi-implicit position update.
        Update_Position(k_str, k_wave);

        // ------------------ DRAW ------------------
        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.setView(camera);
        if (!allLines.empty())
            window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}
