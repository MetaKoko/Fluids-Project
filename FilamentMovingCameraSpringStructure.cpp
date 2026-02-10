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
    sf::Vector2<double> Pos_Vector;   // physical position (meters)
    sf::Vector2<double> F_Vector;     // force (N)
    sf::Color Colour;
    sf::Vector2<double> V;            // velocity (m/s)
    double Rad;                       // radius (meters)
};

std::vector<Sphere_Data> Spheres_List;
std::vector<sf::Vector2<double>> Base_Pos;
std::vector<sf::Vector2<double>> Rest_Pos;
double Global_Spacing = 0.0;


// =============================================================
// Physical + numerical parameters
// =============================================================
double Visc = 1e-3;              // physicalWW   viscosity (water-like)
double Time_Step = 1e-10;

// Periodic domain size in *meters*
double Domain_Width = 2e-3;  
double Domain_Height = 2e-3;   

// Rendering scale: meters → pixels
constexpr float VIS_SCALE = 2e7f;

// Direction of swimming (unit vector)
sf::Vector2<double> Dir;

// =============================================================
// Helpers
// =============================================================
void Add_Sphere(const sf::Vector2<double>& Pos_Vector,
    const sf::Vector2<double>& F_Vector,
    double Rad = 10.0,
    sf::Color Colour = sf::Color::Blue)
{
    Spheres_List.push_back({
        Pos_Vector,
        F_Vector,
        Colour,
        sf::Vector2<double>(0.0, 0.0),
        Rad
        });
}

sf::Vector2<double> Minimum_Image(sf::Vector2<double> d) {
    if (d.x > 0.5 * Domain_Width)  d.x -= Domain_Width;
    if (d.x < -0.5 * Domain_Width)  d.x += Domain_Width;
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

                sf::Vector2<double> Image_Pos = source.Pos_Vector;
                Image_Pos.x += dx * Domain_Width;
                Image_Pos.y += dy * Domain_Height;

                sf::Vector2<double> Dist = target.Pos_Vector - Image_Pos;

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

sf::Vector2<double> Relative_Spring(
    const sf::Vector2<double>& A, const sf::Vector2<double>& B,
    const sf::Vector2<double>& RestA, const sf::Vector2<double>& RestB,
    double k_spring)
{
    sf::Vector2<double> dx = Minimum_Image(B - A);
    sf::Vector2<double> dR = Minimum_Image(RestB - RestA);
    return k_spring * (dx - dR);
}

void Apply_Structural_Springs(double k_struct, double spacing) {
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        sf::Vector2<double> dx = Minimum_Image(
            Spheres_List[i + 1].Pos_Vector - Spheres_List[i].Pos_Vector
        );

        double L = std::sqrt(dx.x * dx.x + dx.y * dx.y);
        if (L < 1e-14) continue;

        double L0 = spacing;  // natural spacing

        sf::Vector2<double> F = k_struct * (L - L0) * (dx / L);

        Spheres_List[i].F_Vector += F;
        Spheres_List[i + 1].F_Vector -= F;
    }
}

void Apply_Actuation_Springs(double k_wave) {
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        sf::Vector2<double> dx = Minimum_Image(
            Spheres_List[i + 1].Pos_Vector - Spheres_List[i].Pos_Vector
        );

        sf::Vector2<double> dR = Minimum_Image(
            Rest_Pos[i + 1] - Rest_Pos[i]
        );

        sf::Vector2<double> F = k_wave * (dx - dR);

        Spheres_List[i].F_Vector += F;
        Spheres_List[i + 1].F_Vector -= F;
    }
}



sf::Vector2<double> Compute_COM() {
    sf::Vector2<double> com(0.0, 0.0);
    for (const auto& s : Spheres_List)
        com += s.Pos_Vector;
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
    static int counter = 0;

    // Averaged speed tracking
    static double accum_disp = 0.0;
    static double accum_time = 0.0;
    static double last_COM_x = 0.0;
    static double last_COM_y = 0.0;
    static bool first = true;

    static sf::Vector2<double> Dir_fixed = Dir;

    // Clear forces first
    for (auto& s : Spheres_List)
        s.F_Vector = { 0.0, 0.0 };

    // Strong spacing enforcement
    Apply_Structural_Springs(k_struct, Global_Spacing);

    // Weak travelling-wave enforcement
    Apply_Actuation_Springs(k_aut);


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

    for (size_t i = 0; i < Spheres_List.size(); i++)
        Calc_Sphere_Velocity(i, Spheres_List[i].V);

    for (auto& s : Spheres_List) {
        s.Pos_Vector += s.V * Time_Step;

        // periodic wrapping in *physical* domain
        if (s.Pos_Vector.x < 0)              s.Pos_Vector.x += Domain_Width;
        else if (s.Pos_Vector.x >= Domain_Width)  s.Pos_Vector.x -= Domain_Width;

        if (s.Pos_Vector.y < 0)              s.Pos_Vector.y += Domain_Height;
        else if (s.Pos_Vector.y >= Domain_Height) s.Pos_Vector.y -= Domain_Height;
    }
}

// =============================================================
// Visualisation helpers
// =============================================================
void Draw_Sphere(std::vector<sf::Vertex>& lines, size_t index) {
    const Sphere_Data& sphere = Spheres_List[index];
    sf::Vector2<double> center = sphere.Pos_Vector;

    double Rad_px = 2.0; // visible pixel radius
    int Segments = 40;
    double pi = 3.141592653589793;

    // convert physical → pixel
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
        sf::Vector2<double> pA = Spheres_List[i].Pos_Vector;
        sf::Vector2<double> pB = Spheres_List[i + 1].Pos_Vector;

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
    Draw_Connections(lines);   // <--- add this first
    for (size_t i = 0; i < Spheres_List.size(); i++)
        Draw_Sphere(lines, i);
}


// =============================================================
// main
// =============================================================
int main() {
    sf::VideoMode mode = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(mode, "Travelling Wave Swimmer (SFML 3, Fixed Scaling)");
    window.setFramerateLimit(60);

    bool dragging = false;
    sf::Vector2i lastMouse;

    // =========================================================
    // Swimmer setup
    // =========================================================
    size_t N = 30;
    double radius = 0.3e-6;
    double length = 50e-6 - 2*radius;
    double spacing = length / (N - 1);
    Global_Spacing = spacing;
    double k_str = 5e-13;
    double k_act = 2e-15;

    double base_x = 0.5 * Domain_Width;
    double base_y = 0.5 * Domain_Height;

    double pi = 3.141592653589793;
    double Angle = pi / 4.0;

    Dir = sf::Vector2<double>(std::cos(Angle), std::sin(Angle));
    sf::Vector2<double> Normal(-Dir.y, Dir.x);

    double A = 3e-6;
    double lambda = 30e-6; // 30 µm 
    double k = 2.0 * pi / lambda; // ≈ 2.09e5 1/m 
    double f = 25.0; // 25 Hz 
    double omega = 2.0 * pi * f; // ≈ 157 rad/s

    Base_Pos.resize(N);
    Rest_Pos.resize(N);

    // Build wavy curve FIRST
    for (size_t i = 0; i < N; i++) {
        double s = i * spacing;
        double wave = A * std::sin(k * s);

        Base_Pos[i] = {
            base_x + Dir.x * s + Normal.x * wave,
            base_y + Dir.y * s + Normal.y * wave
        };

        Rest_Pos[i] = Base_Pos[i];
        Add_Sphere(Base_Pos[i], { 0.0, 0.0 }, radius, sf::Color::Blue);
    }

    // CAMERA SETUP
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

    // =========================================================
    // Main loop
    // =========================================================
    sf::Clock clock;
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

        // ------------------ UPDATE REST SHAPE ------------------
        double t = clock.getElapsedTime().asSeconds();

        for (size_t i = 0; i < N; i++) {
            double s = i * spacing;

            double wave0 = A * std::sin(k * s);              // initial wave
            double waveT = A * std::sin(k * s - omega * t);  // travelling wave

            double delta = waveT - wave0;                    // how much to move

            Rest_Pos[i] = {
                Base_Pos[i].x + Normal.x * delta,
                Base_Pos[i].y + Normal.y * delta
            };
        }


        // ------------------ PHYSICS STEP ------------------
        Update_Position(k_str, k_act);

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
