#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

// =============================================================
// Data structures
struct Sphere_Data {
    sf::Vector2<double> Pos_Unwrapped;
    sf::Vector2<double> F_Vector;
    sf::Color Colour;
    sf::Vector2<double> V;
    double Rad;
};

std::vector<Sphere_Data> Spheres_List;
// =============================================================

// =============================================================
//Numerical parameters and viscosity

double Visc = 1e-3;
double Time_Step = 1e-6;
double Domain_Width = 2e-1;
double Domain_Height = 2e-1;
constexpr float VIS_SCALE = 2e7f;
constexpr double DRAW_SCALE = 2e7;

// =============================================================

// =============================================================
// Global variables
std::vector<sf::Vector2<double>> Target_Pos;
double base_y_global = 0.0;
double A_global = 0.0;
double k_global = 0.0;
double omega_global = 0.0;
double k_wave_global = 0.0;
int step_counter = 0;
double phase_global = 0.0;
std::vector<double> k_structural;
std::vector<double> rest_structural;
std::vector<double> k_bend_structural;
bool Hydro_On = true;
bool Start_With_Sine = true;
bool Use_Tapered_Stiffness = false;
bool Use_Tapered_Bending = false;
// =============================================================

// =============================================================
// Functions (the physics and etc)
void Add_Sphere(const sf::Vector2<double>& Pos,
    const sf::Vector2<double>& F_Vector,
    double Rad = 10.0,
    sf::Color Colour = sf::Color::Blue) {
    Spheres_List.push_back({ Pos,F_Vector,Colour,sf::Vector2<double>(0.0, 0.0),Rad });
}

sf::Vector2<double> Minimum_Image(sf::Vector2<double> d) {
    if (d.x > 0.5 * Domain_Width)  d.x -= Domain_Width;
    if (d.x < -0.5 * Domain_Width)  d.x += Domain_Width;
    if (d.y > 0.5 * Domain_Height) d.y -= Domain_Height;
    if (d.y < -0.5 * Domain_Height) d.y += Domain_Height;
    return d;
}


sf::Vector2<double> Regularized_Stokeslet(sf::Vector2<double> Dist,
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
}


void Calc_Sphere_Velocity(size_t Sphere_Index, sf::Vector2<double>& V_Vector)
{
    Sphere_Data target = Spheres_List[Sphere_Index];
    V_Vector = { 0.0, 0.0 };

    double pi = 3.141592653589793;
    double self_mob = 1.0 / (6.0 * pi * Visc * target.Rad);

    // Self mobility
    V_Vector += target.F_Vector * self_mob;

    // Direct interactions only — NO periodic images
    for (size_t i = 0; i < Spheres_List.size(); i++) {
        if (i == Sphere_Index) continue;

        Sphere_Data source = Spheres_List[i];

        sf::Vector2<double> Dist = target.Pos_Unwrapped - source.Pos_Unwrapped;

        double epsilon = source.Rad;
        V_Vector += Regularized_Stokeslet(Dist, source.F_Vector, epsilon);
    }
}


void Apply_Structural_Springs()
{
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {
        double k_spring = k_structural[i];
        double rest_length = rest_structural[i];
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

void Apply_Bending_Springs()
{
    for (size_t i = 0; i + 2 < Spheres_List.size(); i++) {
        auto& s1 = Spheres_List[i];
        auto& s3 = Spheres_List[i + 2];
        double rest2 = rest_structural[i] + rest_structural[i + 1];
        double k_bend = k_bend_structural[i];
        double dx = s3.Pos_Unwrapped.x - s1.Pos_Unwrapped.x;
        double dy = s3.Pos_Unwrapped.y - s1.Pos_Unwrapped.y;

        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist == 0.0)
            continue;

        double diff = dist - rest2;
        if (diff > 0.0)
            continue;

        double fx = k_bend * diff * (dx / dist);
        double fy = k_bend * diff * (dy / dist);
        s1.F_Vector.x += fx;
        s1.F_Vector.y += fy;
        s3.F_Vector.x -= fx;
        s3.F_Vector.y -= fy;
    }
}
// =============================================================

// =============================================================
// Time stepping
void Update_Position()
{
    for (auto& s : Spheres_List)
        s.F_Vector = { 0.0, 0.0 };
    //Apply_Structural_Springs();
    //Apply_Bending_Springs();

    for (size_t i = 0; i < Spheres_List.size(); i++) {
        if (i < 2) { 
            Spheres_List[i].F_Vector.y = 0.0;
            continue; 
        }
        double y_cur = Spheres_List[i].Pos_Unwrapped.y;
        double y_tar = Target_Pos[i].y;
        double dy = y_tar - y_cur;
        double Fy = k_wave_global * dy;
        Spheres_List[i].F_Vector.y = Fy;
    }

    if (step_counter == 0) {
        std::cout << "Step 0 forces: ";
        for (size_t i = 0; i < Spheres_List.size(); i++)
            std::cout << Spheres_List[i].F_Vector.y << " ";
        std::cout << "\n";
    }

    step_counter++;
    if (step_counter % 200 == 0) {
        std::cout << "Step " << step_counter << " forces: ";
        for (size_t i = 0; i < Spheres_List.size(); i++) {
            std::cout << Spheres_List[i].F_Vector.y << " ";
        }
        std::cout << "\n";
    }

    sf::Vector2<double> NetF = { 0.0, 0.0 };
    if (step_counter % 200 == 0) {
        for (const auto& s : Spheres_List)
            NetF += s.F_Vector;

        std::cout << "Net force: (" << NetF.x << ", " << NetF.y << ")\n";
    }

    double pi = 3.141592653589793;

    if (!Hydro_On) {
        for (auto& s : Spheres_List) {
            double zeta = 6.0 * pi * Visc * s.Rad;
            s.V.x = 1e-8;
            s.V.y = s.F_Vector.y / zeta;
        }
    }
    else {
        for (size_t i = 0; i < Spheres_List.size(); i++) {
            Calc_Sphere_Velocity(i, Spheres_List[i].V);
        }
    }
    for (auto& s : Spheres_List)
        s.Pos_Unwrapped += s.V * Time_Step;
}
// =============================================================

// =============================================================
// Functions for drawing
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

    double Rad_px = sphere.Rad * DRAW_SCALE;
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

void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();
    Draw_Connections(lines);
    for (size_t i = 0; i < Spheres_List.size(); i++)
        Draw_Sphere(lines, i);
}
// =============================================================

int main() {
    sf::VideoMode mode = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(mode, "Regulised stokeslet swimmer");
    window.setFramerateLimit(60);

    bool dragging = false;
    sf::Vector2i lastMouse;

    // Swimmer geometry
    size_t N = 80;
    double head_radius = 6e-6;
    double torso_radius = 3e-6;
    double tail_radius = 0.3e-6;

    double length = 50e-6 - 2 * tail_radius;

	// Rest lengths
    double rest_head_torso = head_radius + torso_radius;
    double rest_torso_tail = torso_radius + tail_radius; 
    int N_tail = N - 2;
    double rest_tail_tail = 2 * tail_radius;

    // Structural springs
    double k_head_torso = 1e-7;
    double k_torso_tail = 1e-7;

    k_structural.resize(N - 1);
    rest_structural.resize(N - 1);
    k_bend_structural.resize(N - 2);

    // Head–torso
    k_structural[0] = k_head_torso;
    rest_structural[0] = rest_head_torso;

    // Torso–first tail bead
    k_structural[1] = k_torso_tail;
    rest_structural[1] = rest_torso_tail;

    double k_tail_const = 1e-7;
    double k_tail_base = 1e-7;
    double k_tail_tip = 1e-7; 

    for (size_t i = 2; i < N - 1; i++) {

        if (!Use_Tapered_Stiffness) {         
            k_structural[i] = k_tail_const;
        }
        else {
            double s = double(i - 2) / double((N - 1) - 2);
            k_structural[i] = k_tail_base * std::pow(k_tail_tip / k_tail_base, s);
        }
        rest_structural[i] = rest_tail_tail;
    }


    double k_bend_const = 1.5e-7;
    double k_bend_base = 1e-8;
    double k_bend_tip = 2e-9;

    for (size_t i = 0; i + 2 < N; i++) {
        if (!Use_Tapered_Bending) {
            k_bend_structural[i] = k_bend_const;
        }
        else {
            double s = double(i) / double(N - 3);
            k_bend_structural[i] =
                k_bend_base * std::pow(k_bend_tip / k_bend_base, s);
        }
    }



    double base_x = 0.5 * Domain_Width;
    double base_y = 0.5 * Domain_Height;
    base_y_global = base_y;
    
    // Wave parameters
    double pi = 3.141592653589793;
    double A = 3e-6;
    double lambda = 30e-6;
    double k = 2.0 * pi / lambda;
    double f = 25;
    double omega = 2 * pi * f;

    A_global = A;
    k_global = k;
    omega_global = omega;
    k_wave_global = 2.5e-3; 

    // Build swimmer
    Spheres_List.clear();
    Target_Pos.assign(N, sf::Vector2<double>(0.0, 0.0));

    phase_global = 0.0;

    // Head
    double x0 = base_x;
    double y0 = base_y;
    Add_Sphere({ x0, y0 }, { 0.0, 0.0 }, head_radius, sf::Color::Red);
    Target_Pos[0] = { x0, y0 };

    // Torso bead
    double x1 = x0 + rest_head_torso;
    double y1 = base_y;

    Add_Sphere({ x1, y1 }, { 0.0, 0.0 }, torso_radius, sf::Color::Magenta);
    Target_Pos[1] = { x1, y1 };

    // Tail beads
    double x_prev = x1 + rest_torso_tail;

    for (size_t i = 2; i < N; i++) {
        double x = x_prev;
        double y;
        if (Start_With_Sine) {
            double s = x - x1;
            y = y1 + A * std::sin(k * s + phase_global);
        }
        else {
            y = y1;
        }
        Add_Sphere({ x, y }, { 0.0, 0.0 }, tail_radius, sf::Color::Blue);
        Target_Pos[i] = { x, y };
        x_prev += rest_tail_tail;
    }


    // Camera setup
    sf::Vector2<double> COM0 = { base_x + 0.5 * length, base_y };
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

        // Mouse controls
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

        int physics_steps_per_frame = 5;

        for (int step = 0; step < physics_steps_per_frame; step++) {
            Update_Position();
        }


        // COM velocity
        static double last_COMx = 0.0;
        static bool first = true;

        double COMx = 0.0;
        for (const auto& s : Spheres_List)
            COMx += s.Pos_Unwrapped.x;
        COMx /= Spheres_List.size();

        double Vx = 0.0;
        if (!first) {
            Vx = (COMx - last_COMx) / Time_Step;
            if (step_counter % 100 == 0)
                std::cout << "Average x-velocity: " << Vx << "\n";
        }

        last_COMx = COMx;
        first = false;


        double x_torso = Spheres_List[1].Pos_Unwrapped.x;

        for (size_t i = 0; i < N; i++) {
            double x_i = Spheres_List[i].Pos_Unwrapped.x;
            double s = x_i - x_torso; 
            Target_Pos[i].y =
                base_y_global + A_global * std::sin(k_global * s + phase_global);
        }

        phase_global -= omega_global * Time_Step;

        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.setView(camera);
        if (!allLines.empty())
            window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}
