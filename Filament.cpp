#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

// =============================================================
// Data structures
// =============================================================
struct Sphere_Data {
    sf::Vector2<double> Pos_Vector;
    sf::Vector2<double> F_Vector;
    sf::Color Colour;
    sf::Vector2<double> V;
    double Rad;
};

std::vector<Sphere_Data> Spheres_List;
std::vector<sf::Vector2<double>> Base_Pos;
std::vector<sf::Vector2<double>> Rest_Pos;

double Visc = 1e4;
double Time_Step = 5e-8;
double Screen_Width = 1340.0;
double Screen_Height = 700.0;

sf::Vector2<double> Dir;
double Zoom = 1e7;

// =============================================================
// Helpers
// =============================================================
void Add_Sphere(const sf::Vector2<double>& Pos_Vector,
    const sf::Vector2<double>& F_Vector,
    double Rad = 10.0,
    sf::Color Colour = sf::Color::Blue)
{
    Spheres_List.push_back({ Pos_Vector, F_Vector, Colour,
                             sf::Vector2<double>(0.0, 0.0), Rad });
}

sf::Vector2<double> Minimum_Image(sf::Vector2<double> d) {
    if (d.x > 0.5 * Screen_Width)  d.x -= Screen_Width;
    if (d.x < -0.5 * Screen_Width) d.x += Screen_Width;
    if (d.y > 0.5 * Screen_Height) d.y -= Screen_Height;
    if (d.y < -0.5 * Screen_Height) d.y += Screen_Height;
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
                Image_Pos.x += dx * Screen_Width;
                Image_Pos.y += dy * Screen_Height;

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

void Apply_Spring_Forces(double k_spring) {
    for (auto& s : Spheres_List)
        s.F_Vector = { 0.0, 0.0 };

    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        sf::Vector2<double> RestA = Rest_Pos[i];
        sf::Vector2<double> RestB = Rest_Pos[i + 1];

        sf::Vector2<double> F = Relative_Spring(
            Spheres_List[i].Pos_Vector,
            Spheres_List[i + 1].Pos_Vector,
            RestA, RestB,
            k_spring
        );

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
void Update_Position(double k_spring)
{
    static sf::Vector2<double> Prev_COM = Compute_COM();
    static int counter = 0;

    // --- Averaged speed tracking ---
    static double accum_disp = 0.0;
    static double accum_time = 0.0;
    static double last_COM_x = 0.0;
    static double last_COM_y = 0.0;
    static bool first = true;

    // Fixed forward direction (initial body axis)
    static sf::Vector2<double> Dir_fixed = Dir;

    Apply_Spring_Forces(k_spring);

    sf::Vector2<double> Current_COM = Compute_COM();

    // Initialize COM history
    if (first) {
        last_COM_x = Current_COM.x;
        last_COM_y = Current_COM.y;
        first = false;
    }

    // Instantaneous COM velocity
    sf::Vector2<double> COM_Vel = (Current_COM - Prev_COM) / Time_Step;

    // Instantaneous forward speed (may oscillate)
    double Speed_inst = COM_Vel.x * Dir_fixed.x + COM_Vel.y * Dir_fixed.y;

    // Accumulate displacement
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

    // Beat period
    double pi = 3.141592653589793;
    double omega = 2.0 * pi * 25.0;
    double T = 2.0 * pi / omega;

    // Compute averaged speed once per beat
    if (accum_time >= T) {
        double avg_speed = accum_disp / accum_time;

        std::cout << std::scientific << std::setprecision(12)
            << "Averaged speed = " << avg_speed << "\n";

        accum_disp = 0.0;
        accum_time = 0.0;
    }

    // Print instantaneous diagnostics
    counter++;
    if (counter % 100 == 0) {
        sf::Vector2<double> netF = Compute_Net_Force();
        std::cout << std::scientific << std::setprecision(12)
            << "NetF = (" << netF.x << ", " << netF.y << ")"
            << "   Inst speed = " << Speed_inst << "\n";
    }

    // Hydrodynamic velocities
    for (size_t i = 0; i < Spheres_List.size(); i++)
        Calc_Sphere_Velocity(i, Spheres_List[i].V);

    // Update positions
    for (auto& s : Spheres_List) {
        s.Pos_Vector += s.V * Time_Step;

        if (s.Pos_Vector.x < 0) s.Pos_Vector.x += Screen_Width;
        else if (s.Pos_Vector.x >= Screen_Width) s.Pos_Vector.x -= Screen_Width;

        if (s.Pos_Vector.y < 0) s.Pos_Vector.y += Screen_Height;
        else if (s.Pos_Vector.y >= Screen_Height) s.Pos_Vector.y -= Screen_Height;
    }
}

// =============================================================
// Visualisation
// =============================================================
sf::Vector2f ZoomTransform(const sf::Vector2<double>& p)
{
    static sf::Vector2<double> VisualCenter(Screen_Width / 2.0,
        Screen_Height / 2.0);

    sf::Vector2<double> d = p - Compute_COM();
    d *= Zoom;

    return sf::Vector2f(
        (float)(VisualCenter.x + d.x),
        (float)(VisualCenter.y + d.y)
    );
}

void Draw_Sphere(std::vector<sf::Vertex>& lines, size_t index) {
    const Sphere_Data& sphere = Spheres_List[index];
    sf::Vector2<double> center = sphere.Pos_Vector;

    double Rad = sphere.Rad;
    int Segments = 40;
    double pi = 3.141592653589793;

    for (int i = 0; i < Segments; i++) {
        double a1 = 2.0 * pi * i / Segments;
        double a2 = 2.0 * pi * (i + 1) / Segments;

        sf::Vector2<double> p1_phys(
            center.x + Rad * std::cos(a1),
            center.y + Rad * std::sin(a1)
        );

        sf::Vector2<double> p2_phys(
            center.x + Rad * std::cos(a2),
            center.y + Rad * std::sin(a2)
        );

        sf::Vector2f p1 = ZoomTransform(p1_phys);
        sf::Vector2f p2 = ZoomTransform(p2_phys);

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }
}

void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();
    for (size_t i = 0; i < Spheres_List.size(); i++)
        Draw_Sphere(lines, i);
}

// =============================================================
// main
// =============================================================
int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(),
        "Travelling Wave Swimmer (Stokes, Zoomed)");

    size_t N = 150;
    double length = 60e-6;
    double spacing = length / (N - 1);
    double radius = 1.25e-7;
    double spring_k = 1e-6;

    double base_x = Screen_Width / 2.0;
    double base_y = Screen_Height / 2.0;

    double Angle = 3.141592653589793 / 4.0;

    Dir = sf::Vector2<double>(std::cos(Angle), std::sin(Angle));
    sf::Vector2<double> Normal(-Dir.y, Dir.x);

    double pi = 3.141592653589793;
    double A = 4e-6;
    double lambda = 35e-6;
    double k = 2.0 * pi / lambda;
    double f = 25.0;
    double omega = 2.0 * pi * f;

    Base_Pos.resize(N);
    Rest_Pos.resize(N);

    for (size_t i = 0; i < N; i++) {
        sf::Vector2<double> base(
            base_x + Dir.x * (double)i * spacing,
            base_y + Dir.y * (double)i * spacing
        );

        Base_Pos[i] = base;

        double wave = A * std::sin(k * (double)i);
        Rest_Pos[i] = base + Normal * wave;

        Add_Sphere(Rest_Pos[i], { 0.0, 0.0 }, radius, sf::Color::Blue);
    }

    sf::Clock clock;
    std::vector<sf::Vertex> allLines;

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
        }

        double t = clock.getElapsedTime().asSeconds();

        for (size_t i = 0; i < N; i++) {
            double wave = A * std::sin(k * (double)i - omega * t);
            Rest_Pos[i] = Base_Pos[i] + Normal * wave;
        }

        Update_Position(spring_k);

        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}
