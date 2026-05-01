#include <omp.h>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <optional>


struct Sphere_Data {
    sf::Vector2<double> Pos_Unwrapped;
    sf::Vector2<double> F_Vector;
    sf::Color Colour;
    sf::Vector2<double> V;
    double Rad;
};

std::vector<Sphere_Data> Spheres_List;


double Visc = 1e-3;
double Time_Step = 3e-6;
double Domain_Width = 1e-2;
double Domain_Height = 1e-2;
constexpr float VIS_SCALE = 15e5f;


std::vector<sf::Vector2<double>> Target_Pos;
double Global_Spacing = 0.0;
double base_y_global = 0.0;

sf::Vector2<double> COM_prev = { 0.0, 0.0 };
sf::Vector2<double> COM_current = { 0.0, 0.0 };
double TranslationSpeed = 0.0;

double A_global = 0.0;
double k_global = 0.0;
double omega_global = 0.0;
double k_wave_global = 0.0;

int step_counter = 0;
double phase_global = 0.0;
int PhysicsStepsPerFrame = 100; 
bool Hydro_On = true;
bool Use_ProjectedX_Mode = false;
bool Diagnostics_On = false;


void Add_Sphere(const sf::Vector2<double>& Pos,
    const sf::Vector2<double>& F_Vector,
    double Rad,
    sf::Color Colour = sf::Color::Blue)
{
    Spheres_List.push_back({ Pos, F_Vector, Colour, sf::Vector2<double>(0.0, 0.0), Rad });
}

//Ignore this, this was something I was experimenting with.
/*sf::Vector2<double> Regularized_Stokeslet(sf::Vector2<double> Dist,
    sf::Vector2<double> F_Vector,
    double epsilon)
{
    double x = Dist.x;
    double y = Dist.y;

    double r2 = x * x + y * y;
    double eps2 = epsilon * epsilon;
    double re2 = r2 + eps2;
    double re = std::sqrt(re2);

    if (re < 1e-14) return { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double Constant = 1.0 / (8.0 * pi * Visc);

    double Dot = x * F_Vector.x + y * F_Vector.y;
    double re3 = re2 * re;

    double iso = (r2 + 2.0 * eps2) / re3;

    return {
        Constant * (iso * F_Vector.x + Dot * x / re3),
        Constant * (iso * F_Vector.y + Dot * y / re3)
    };
}*/

sf::Vector2<double> Stokeslet(const sf::Vector2<double>& Dist,
    const sf::Vector2<double>& F,
    double mu)
{
    double x = Dist.x;
    double y = Dist.y;

    double r2 = x * x + y * y;
    double r = std::sqrt(r2);

    double pi = 3.141592653589793;
    double C = 1.0 / (8.0 * pi * mu);

    double dot = x * F.x + y * F.y;

    return {
        C * (F.x / r + dot * x / (r2 * r)),
        C * (F.y / r + dot * y / (r2 * r))
    };
}


void Calc_Sphere_Velocity(size_t Sphere_Index, sf::Vector2<double>& V_Vector)
{
    Sphere_Data target = Spheres_List[Sphere_Index];
    V_Vector = { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double self_mob = 1.0 / (6.0 * pi * Visc * target.Rad);

    V_Vector += target.F_Vector * self_mob;

    for (size_t i = 0; i < Spheres_List.size(); i++) {
        if (i == Sphere_Index) continue;

        Sphere_Data source = Spheres_List[i];
        sf::Vector2<double> Dist = target.Pos_Unwrapped - source.Pos_Unwrapped;
        double epsilon = source.Rad;
        V_Vector += Stokeslet(Dist, source.F_Vector, Visc);

    }
}

void Apply_Structural_Springs(double k_spring, double rest_length)
{
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        sf::Vector2<double> pA = Spheres_List[i].Pos_Unwrapped;
        sf::Vector2<double> pB = Spheres_List[i + 1].Pos_Unwrapped;

        sf::Vector2<double> d = pB - pA;
        double r2 = d.x * d.x + d.y * d.y;
        double r = std::sqrt(r2);

        double stretch = r - rest_length;
        double fmag = k_spring * stretch;

        sf::Vector2<double> F = { fmag * d.x / r, fmag * d.y / r };

        Spheres_List[i].F_Vector += F;
        Spheres_List[i + 1].F_Vector -= F;
    }
}

sf::Vector2<double> Compute_COM()
{
    sf::Vector2<double> com = { 0.0, 0.0 };
    for (const auto& s : Spheres_List)
        com += s.Pos_Unwrapped;

    com.x /= Spheres_List.size();
    com.y /= Spheres_List.size();
    return com;
}

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
    double x = x0 + s_target;
    for (int it = 0; it < 8; ++it) {
        double s_val = s_of_x(x0, x, A, k);
        double f = s_val - s_target;
        double fp = arc_length_integrand(x, A, k);
        x -= f / fp;
    }
    return x;
}

// =============================================================
// Diagnostics (this function was for debugging only)
// =============================================================
void Diagnostics(const std::vector<double>& x_coord, size_t N)
{
    if (!Diagnostics_On) return;

    double pi = 3.141592653589793;

    // =============================================================
   // Net Force
   // =============================================================
    static int netF_counter = 0;
    netF_counter++;

    sf::Vector2<double> NetF = { 0.0, 0.0 };
    for (const auto& s : Spheres_List)
        NetF += s.F_Vector;

    if (netF_counter >= 5000)
    {
        std::cout << "[Diag] Net Force = ("
            << NetF.x << ", " << NetF.y << ")\n";

        netF_counter = 0;
    }

    // =============================================================
    // Force magnitudes + ratio
    // =============================================================
    static int force_ratio_counter = 0;
    force_ratio_counter++;

    if (force_ratio_counter >= 5000)
    {
        double sumF = 0.0;
        double maxF = 0.0;

        for (const auto& s : Spheres_List) {
            double mag = std::sqrt(s.F_Vector.x * s.F_Vector.x +
                s.F_Vector.y * s.F_Vector.y);
            sumF += mag;
            if (mag > maxF) maxF = mag;
        }

        double netMag = std::sqrt(NetF.x * NetF.x + NetF.y * NetF.y);
        double ratio_sum = (sumF > 0.0) ? netMag / sumF : 0.0;
        double ratio_max = (maxF > 0.0) ? netMag / maxF : 0.0;

        std::cout << "[Diag] |NetF| = " << netMag
            << ", sum |F_i| = " << sumF
            << ", max |F_i| = " << maxF << "\n"
            << "       ratio_sum = " << ratio_sum
            << ", ratio_max = " << ratio_max << "\n";

        force_ratio_counter = 0;
    }

    // =============================================================
    // Target tracking diagnostic
    // =============================================================
    static int track_counter = 0;
    track_counter++;

    if (track_counter >= 5000)
    {
        double tol = 0.001 * A_global;
        int count_good = 0;

        std::cout << "[Diag] Tracking errors: ";

        for (size_t i = 0; i < Spheres_List.size(); i++)
        {
            double dy = Target_Pos[i].y - Spheres_List[i].Pos_Unwrapped.y;

            bool ok = (std::abs(dy) < tol);
            if (ok) count_good++;

            std::cout << (ok ? "." : "x");
        }

        std::cout << "  (" << count_good << "/" << Spheres_List.size()
            << " beads within tolerance)\n";

        track_counter = 0;
    }



    // =============================================================
    // Amplitude
    // =============================================================
    static double y_min = 1e9;
    static double y_max = -1e9;

    size_t mid = N / 2;
    double y_mid = Spheres_List[mid].Pos_Unwrapped.y;

    if (y_mid < y_min) y_min = y_mid;
    if (y_mid > y_max) y_max = y_mid;

    double T_period = 2.0 * pi / omega_global;
    static double accum_amp = 0.0;
    accum_amp += Time_Step;

    if (accum_amp >= T_period)
    {
        double amplitude_measured = 0.5 * (y_max - y_min);
        std::cout << "[Diag] Measured amplitude = "
            << amplitude_measured
            << " (target A = " << A_global << ")\n";

        y_min = 1e9;
        y_max = -1e9;
        accum_amp = 0.0;
    }

    // =============================================================
    // Frequency
    // =============================================================
    static bool first = true;
    static double prev_y = 0.0;
    static double last_crossing_time = 0.0;

    double y_now = Spheres_List[mid].Pos_Unwrapped.y;

    if (!first)
    {
        if (prev_y < base_y_global && y_now >= base_y_global)
        {
            double t = step_counter * Time_Step;

            if (last_crossing_time > 0.0)
            {
                double period = t - last_crossing_time;
                double freq_measured = 1.0 / period;

                std::cout << "[Diag] Measured frequency = "
                    << freq_measured
                    << " Hz (target = " << omega_global / (2 * pi) << ")\n";
            }

            last_crossing_time = t;
        }
    }
    else first = false;

    prev_y = y_now;

    // =============================================================
    // Arclength
    // =============================================================
    double L_now = 0.0;
    for (size_t i = 0; i + 1 < Spheres_List.size(); ++i)
    {
        sf::Vector2<double> pA = Spheres_List[i].Pos_Unwrapped;
        sf::Vector2<double> pB = Spheres_List[i + 1].Pos_Unwrapped;

        double dx = pB.x - pA.x;
        double dy = pB.y - pA.y;
        L_now += std::sqrt(dx * dx + dy * dy);
    }

    static double L0 = -1.0;
    if (L0 < 0.0)
    {
        L0 = L_now;
        std::cout << "[Diag] Initial arclength L0 = " << L0 << "\n";
    }
    else if (step_counter % 5000 == 0)
    {
        double rel = (L_now - L0) / L0;
        std::cout << "[Diag] Arclength now = " << L_now
            << " (relative change = " << rel << ")\n";
    }

    // =============================================================
    // Average bead spacing
    // =============================================================
    {
        double sum = 0.0;
        double dmin = 1e9;
        double dmax = -1e9;

        for (size_t i = 0; i + 1 < Spheres_List.size(); ++i)
        {
            sf::Vector2<double> pA = Spheres_List[i].Pos_Unwrapped;
            sf::Vector2<double> pB = Spheres_List[i + 1].Pos_Unwrapped;

            double dx = pB.x - pA.x;
            double dy = pB.y - pA.y;
            double d = std::sqrt(dx * dx + dy * dy);

            sum += d;
            if (d < dmin) dmin = d;
            if (d > dmax) dmax = d;
        }

        double avg = sum / (Spheres_List.size() - 1);
        double rel = (avg - Global_Spacing) / Global_Spacing;

        if (step_counter % 5000 == 0)
        {
            std::cout << "[Diag] Avg spacing = " << avg
                << " (relative = " << rel << ")\n"
                << "       min = " << dmin
                << ", max = " << dmax << "\n";
        }
    }

    // =============================================================
    // Simple per-bead force array
    // =============================================================
    static int force_array_counter = 0;
    force_array_counter++;

    if (force_array_counter >= 5000)
    {
        std::cout << "[Diag] Force array: [";

        for (size_t i = 0; i < Spheres_List.size(); i++)
        {
            const auto& s = Spheres_List[i];
            std::cout << "(" << s.F_Vector.y << ")";
            if (i + 1 < Spheres_List.size()) std::cout << ",";
        }

        std::cout << "]\n";

        force_array_counter = 0;
    }

}

void Update_Position(double k_spring)
{
    double pi = 3.141592653589793;

    for (auto& s : Spheres_List)
        s.F_Vector = { 0.0, 0.0 };

    double rest = Global_Spacing;
    Apply_Structural_Springs(k_spring, rest);

    for (size_t i = 0; i < Spheres_List.size(); i++)
    {
        double dy = Target_Pos[i].y - Spheres_List[i].Pos_Unwrapped.y;
        Spheres_List[i].F_Vector.y += (k_wave_global * dy);
    }


    if (!Hydro_On) //This is an option to disable hydrodynamic interactions, was used to make sure the springs worked
    {
        for (auto& s : Spheres_List)
        {
            double zeta = 6.0 * pi * Visc * s.Rad;
            s.V.x = 0.0;
            s.V.y = s.F_Vector.y / zeta;
        }
    }
    else
    {
        #pragma omp parallel for
        for (int i = 0; i < (int)Spheres_List.size(); i++)
            Calc_Sphere_Velocity(i, Spheres_List[i].V);
    }

    for (auto& s : Spheres_List)
        s.Pos_Unwrapped += s.V * Time_Step;

    step_counter++;
}

void InitializeChain(size_t N,
    double A,
    double lambda,
    double Lx,
    double L_arc,
    double base_x,
    double base_y,
    double freq,
    double bead_rad,
    std::vector<double>& x_coord_out)
{
    double pi = 3.141592653589793;
    double k = 2.0 * pi / lambda;

    A_global = A;
    k_global = k;
    base_y_global = base_y;

    omega_global = 2.0 * pi * freq;

    //This is c_F
    k_wave_global = 1e-2;

    Spheres_List.clear();
    Target_Pos.assign(N, sf::Vector2<double>(0.0, 0.0));
    x_coord_out.assign(N, 0.0);

    phase_global = 0.0;

    double x0 = base_x;

    double rest_length = 0.0;

                                                              
    if (Use_ProjectedX_Mode)                                  //This is an option to instead of inputting length of the swimmer
    {                                                         //you can input the x-coordinate length that you want your swimmmer
        double x1 = x0 + Lx;                                  //to be when already in a sine wave, just ignore this I don't use it
        double L_total = arc_length_between(x0, x1, A, k);
        rest_length = L_total / (N - 1);
    }
    else
    {
        rest_length = L_arc / (N - 1);
    }

    Global_Spacing = rest_length;

    for (size_t i = 0; i < N; ++i)
    {
        double s_i = (double)i * rest_length;
        double x_i = x_of_s(x0, s_i, A, k);
        double y_i = base_y + A * std::sin(k * (x_i - x0) + phase_global);

        x_coord_out[i] = x_i;

        Add_Sphere({ x_i, y_i }, { 0.0, 0.0 }, bead_rad, sf::Color::Blue);

        Target_Pos[i].x = x_i;
        Target_Pos[i].y = y_i;
    }
}

//This is leftover from when I tried to implement periodic boundaries, you can ignore it
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

    double Rad_px = sphere.Rad * VIS_SCALE;
    int Segments = 40;
    double pi = 3.141592653589793;

    float cx = (float)(center.x * VIS_SCALE);
    float cy = (float)(center.y * VIS_SCALE);

    for (int i = 0; i < Segments; i++) {
        double a1 = 2.0 * pi * i / Segments;
        double a2 = 2.0 * pi * (i + 1) / Segments;

        sf::Vector2f p1(cx + (float)(Rad_px * std::cos(a1)),
            cy + (float)(Rad_px * std::sin(a1)));

        sf::Vector2f p2(cx + (float)(Rad_px * std::cos(a2)),
            cy + (float)(Rad_px * std::sin(a2)));

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
//This is some guidelines for the waveform that we want the swimmer to follow
void Draw_WaveformGuides(std::vector<sf::Vertex>& lines, double bead_rad) {
    if (Target_Pos.size() < 2) return;

    double offset = 1* bead_rad;

    for (size_t i = 0; i + 1 < Target_Pos.size(); ++i) {

        // Upper line
        sf::Vector2<double> uA = Wrap_For_Display({ Target_Pos[i].x,     Target_Pos[i].y + offset });
        sf::Vector2<double> uB = Wrap_For_Display({ Target_Pos[i + 1].x,   Target_Pos[i + 1].y + offset });

        // Lower line
        sf::Vector2<double> lA = Wrap_For_Display({ Target_Pos[i].x,     Target_Pos[i].y - offset });
        sf::Vector2<double> lB = Wrap_For_Display({ Target_Pos[i + 1].x,   Target_Pos[i + 1].y - offset });

        sf::Vector2f uA_px((float)(uA.x * VIS_SCALE), (float)(uA.y * VIS_SCALE));
        sf::Vector2f uB_px((float)(uB.x * VIS_SCALE), (float)(uB.y * VIS_SCALE));

        sf::Vector2f lA_px((float)(lA.x * VIS_SCALE), (float)(lA.y * VIS_SCALE));
        sf::Vector2f lB_px((float)(lB.x * VIS_SCALE), (float)(lB.y * VIS_SCALE));

        lines.push_back(sf::Vertex(uA_px, sf::Color::Black));
        lines.push_back(sf::Vertex(uB_px, sf::Color::Black));

        lines.push_back(sf::Vertex(lA_px, sf::Color::Black));
        lines.push_back(sf::Vertex(lB_px, sf::Color::Black));
    }
}




void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();

    Draw_WaveformGuides(lines,  Spheres_List[0].Rad);
    Draw_Connections(lines);

    for (size_t i = 0; i < Spheres_List.size(); i++)
        Draw_Sphere(lines, i);
}

void SaveFrame(sf::RenderWindow& window, int frameNumber)
{
    std::filesystem::create_directories("frames");

    sf::Texture texture;

    texture.update(window);

    sf::Image screenshot = texture.copyToImage();

    std::string filename = "frames/frame_" + std::to_string(frameNumber) + ".png";

    if (!screenshot.saveToFile(filename))
        std::cout << "Failed to save: " << filename << "\n";
    else
        std::cout << "Saved: " << filename << "\n";
}


int main() {
    sf::VideoMode mode = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(mode, "First microswimmer simulation");
    window.setFramerateLimit(60);

    bool dragging = false;
    sf::Vector2i lastMouse;
    double pi = 3.141592653589793;
    int frame = 0;
    int saveEvery = 20; 

    //Here are most of the inputs
    // ============================================================
    size_t N = 25;

    double A = 2.0e-5;
    double lambda = 5e-4; 

    double bead_rad = 7.5e-6;
    double Lx = 5e-4;
    double L_arc = 5e-4;

    double freq = 1.92;

    double k_spring = 1e-2;

    double base_x = 0.5 * Domain_Width;
    double base_y = 0.5 * Domain_Height;
    // ============================================================


    std::vector<double> x_coord(N);
    InitializeChain(N, A, lambda, Lx, L_arc,
        base_x, base_y,
        freq, bead_rad,
        x_coord);

    //All of these parts are for the camera and mouse controls for when the simulation is running
    double Lx_effective = x_coord.back() - x_coord.front();
    sf::Vector2<double> COM0 = { 0.5 * (x_coord.front() + x_coord.back()), base_y };
    sf::Vector2f COM0_px((float)(COM0.x * VIS_SCALE),
        (float)(COM0.y * VIS_SCALE));

    sf::Vector2u winSize = window.getSize();

    sf::FloatRect viewRect(
        sf::Vector2f(
            COM0_px.x - 0.5f * (float)winSize.x,
            COM0_px.y - 0.5f * (float)winSize.y
        ),
        sf::Vector2f((float)winSize.x, (float)winSize.y)
    );

    sf::View camera(viewRect);
    window.setView(camera);

    std::vector<sf::Vertex> allLines;

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
                    sf::Texture texture;
                    texture.resize(window.getSize());
                    texture.update(window); 


                    sf::Image screenshot = texture.copyToImage();

                    std::string filename = "swimmer_frame_" + std::to_string(step_counter) + ".png";
                    if (screenshot.saveToFile(filename)) {
                        std::cout << "Saved frame to " << filename << "\n";
                    }
                    else {
                        std::cout << "Failed to save screenshot\n";
                    }
                }
            }

        }

        //This part speeds up the simulation a little bit
        for (int s = 0; s < PhysicsStepsPerFrame; s++) {

            double old_phase = phase_global;
            phase_global -= omega_global * Time_Step;
            (void)old_phase;

            double COMx = COM_current.x;

            for (size_t i = 0; i < N; i++) {

                Target_Pos[i].x = Spheres_List[i].Pos_Unwrapped.x;

                Target_Pos[i].y =
                    base_y_global +
                    A_global * std::sin(k_global * (Target_Pos[i].x) + phase_global);
            }


            Update_Position(k_spring);
            Diagnostics(x_coord, N);

            COM_prev = COM_current;
            COM_current = Compute_COM();
            sf::Vector2<double> dCOM = COM_current - COM_prev;
            TranslationSpeed = dCOM.x / Time_Step;

            // Here we get our swimming speeds
            static double accum_time_speed = 0.0;
            static double COM_start_x = COM_current.x;
            accum_time_speed += Time_Step;
            double T_period_speed = 2.0 * pi / omega_global;

            if (accum_time_speed >= T_period_speed)
            {
                double COM_end_x = COM_current.x;
                double avg_speed = (COM_end_x - COM_start_x) / accum_time_speed;

                std::cout << "Average speed over one period: " << avg_speed << "\n";

                COM_start_x = COM_end_x;
                accum_time_speed = 0.0;
            }
        }
        // ========================================================

        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.setView(camera);
        if (!allLines.empty())
            window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        if (frame % saveEvery == 0)
        {
            //SaveFrame(window, frame / saveEvery);
        }
        frame++;
        window.display();
    }

    return 0;
}