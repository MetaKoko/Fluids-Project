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
    sf::Vector2<double> Pos_Unwrapped;
    sf::Vector2<double> F_Vector;
    sf::Color Colour;
    sf::Vector2<double> V;
    double Rad;
};

std::vector<Sphere_Data> Spheres_List;

// =============================================================
// Physical + numerical parameters
// =============================================================
double Visc = 1e-3;
double Time_Step = 10;

double Domain_Width = 2e-3;
double Domain_Height = 2e-3;

constexpr float VIS_SCALE = 2e7f;

// =============================================================
// Wave / target globals
// =============================================================
std::vector<sf::Vector2<double>> Target_Pos;  // only y is meaningful
double Global_Spacing = 0.0;
double base_y_global = 0.0;
double A_global = 0.0;
double k_global = 0.0;
double omega_global = 0.0;
double tol_global = 0.0;
double k_wave_global = 0.0;
int step_counter = 0;
std::vector<double> Prev_Force_Y;
std::vector<double> Prev_dy;
std::vector<bool> Bead_Done;
double phase_global = 0.0;
bool Hydro_On = false;








// =============================================================
// Helpers
// =============================================================
void Add_Sphere(const sf::Vector2<double>& Pos,
    const sf::Vector2<double>& F_Vector,
    double Rad = 10.0,
    sf::Color Colour = sf::Color::Blue)
{
    Spheres_List.push_back({
        Pos,
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
    double Dot = Dist.x * F_Vector.x + Dist.y * F_Vector.y;

    double r3 = r2 * r;

    return {
        Constant * (F_Vector.x / r + Dot * Dist.x / r3),
        Constant * (F_Vector.y / r + Dot * Dist.y / r3)
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
    double Dot = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = a2 / (48.0 * pi * Visc);

    double r3 = r2 * r;
    double r5 = r3 * r2;

    return {
        Constant * (F_Vector.x / r3 - 3.0 * Dot * Dist.x / r5),
        Constant * (F_Vector.y / r3 - 3.0 * Dot * Dist.y / r5)
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
    double Dot = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = (a2_target * a2_source) / (288.0 * pi * Visc);

    double r5 = r2 * r2 * r;
    double r7 = r5 * r2;
    double r9 = r7 * r2;

    return {
        Constant * (9.0 * F_Vector.x / r5 - 45.0 * Dot * Dist.x / r7 + 105.0 * Dot * Dist.x / r9),
        Constant * (9.0 * F_Vector.y / r5 - 45.0 * Dot * Dist.y / r7 + 105.0 * Dot * Dist.y / r9)
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

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {

                sf::Vector2<double> Image_Pos = source.Pos_Unwrapped;
                Image_Pos.x += dx * Domain_Width;
                Image_Pos.y += dy * Domain_Height;

                sf::Vector2<double> Dist =
                    Minimum_Image(target.Pos_Unwrapped - Image_Pos);

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

// =============================================================
// Time stepping (hydrodynamics + y‑only target forces)
// =============================================================
void Update_Position()
{
    // Reset forces
    for (auto& s : Spheres_List)
        s.F_Vector = { 0.0, 0.0 };

    for (size_t i = 0; i < Spheres_List.size(); i++) {

        double y_cur = Spheres_List[i].Pos_Unwrapped.y;
        double y_tar = Target_Pos[i].y;
        double dy = y_tar - y_cur;

        // Detect overshoot
        bool overshot = (Prev_dy[i] != 0.0) && ((dy > 0.0) != (Prev_dy[i] > 0.0));

        if (overshot) {
            Spheres_List[i].F_Vector.y = 0.0;
            Prev_Force_Y[i] = 0.0;
            Bead_Done[i] = true;     // <-- NEW
            Prev_dy[i] = dy;
            continue;
        }

        // Reached target
        if (std::abs(dy) <= tol_global) {
            Spheres_List[i].F_Vector.y = 0.0;
            Prev_Force_Y[i] = 0.0;
            Bead_Done[i] = true;     // <-- NEW
            Prev_dy[i] = dy;
            continue;
        }

        // Still behind target → apply force
        double alpha = 1;

        if (Prev_Force_Y[i] == 0.0) {
            Prev_Force_Y[i] = k_wave_global * dy;
        }
        else {
            Prev_Force_Y[i] *= alpha;
        }

        // Wave-spring force threshold (ignore structural forces)
        double wave_force = k_wave_global * dy;

        constexpr double WAVE_FORCE_EPS = 1e-25;
        if (std::abs(wave_force) < WAVE_FORCE_EPS) {
            Bead_Done[i] = true;
            Prev_Force_Y[i] = 0.0;
        }


        Spheres_List[i].F_Vector.y = Prev_Force_Y[i];
        Prev_dy[i] = dy;
    }


    if (step_counter == 0) {
        std::cout << "Step 0 forces: ";
        for (size_t i = 0; i < Spheres_List.size(); i++)
            std::cout << Spheres_List[i].F_Vector.y << " ";
        std::cout << "\n";
    }


	step_counter++;
    // --- DEBUG: PRINT FORCES EVERY 200 STEPS ---
    if (step_counter % 200 == 0) {
        std::cout << "Step " << step_counter << " forces: ";
        for (size_t i = 0; i < Spheres_List.size(); i++) {
            std::cout << Spheres_List[i].F_Vector.y << " ";
        }
        std::cout << "\n";
    }



    double pi = 3.141592653589793;

    if (!Hydro_On) {
        // Local drag model (no hydrodynamics)
        for (auto& s : Spheres_List) {
            double zeta = 6.0 * pi * Visc * s.Rad;
            s.V.x = 1e-8;
            s.V.y = s.F_Vector.y / zeta;
        }
    }
    else {
        // Full hydrodynamics: Stokeslets + Faxén only
        for (size_t i = 0; i < Spheres_List.size(); i++) {
            Calc_Sphere_Velocity(i, Spheres_List[i].V);
        }
    }


    // Integrate
    for (auto& s : Spheres_List)
        s.Pos_Unwrapped += s.V * Time_Step;
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
// main
// =============================================================
int main() {
    sf::VideoMode mode = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(mode, "Y‑target wave — hydrodynamics + global freeze");
    window.setFramerateLimit(60);

    bool dragging = false;
    sf::Vector2i lastMouse;

    size_t N = 30;
    double radius = 0.3e-6;

    double length = 50e-6 - 2 * radius;
    double spacing = length / (N - 1);
    Global_Spacing = spacing;

    double base_x = 0.5 * Domain_Width;
    double base_y = 0.5 * Domain_Height;
    base_y_global = base_y;

    double pi = 3.141592653589793;
    double A = 3e-6;
    double lambda = 30e-6;
    double k = 2.0 * pi / lambda;
    double f = 25;
    double omega = 2*pi*f;

    A_global = A;
    k_global = k;
    omega_global = omega;
    tol_global = 1e-30 * spacing;
    k_wave_global = 1e-11;

    // --- INITIALISE BEADS USING ACTUAL X POSITION ---
    Spheres_List.clear();
    for (size_t i = 0; i < N; i++) {

        double x = base_x + i * spacing;
        double x_i = x;

        double y = base_y + A * std::sin(k * x_i);

        Add_Sphere({ x, y }, { 0.0, 0.0 }, radius, sf::Color::Blue);
    }

    Prev_Force_Y.assign(N, 0.0);
    Prev_dy.assign(N, 0.0);
    Bead_Done.assign(N, false);

    // --- INITIAL TARGETS USING ACTUAL X POSITION ---
    Target_Pos.assign(N, sf::Vector2<double>(0.0, 0.0));

    double phase0 = -omega_global * Time_Step + 0.5;

    for (size_t i = 0; i < N; i++) {
        double x_i = Spheres_List[i].Pos_Unwrapped.x;
        double target_y = base_y + A * std::sin(k * x_i + phase0);

        Target_Pos[i].x = x_i;
        Target_Pos[i].y = target_y;
    }

    double phase_global = phase0;

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

        // ------------------ PHYSICS STEP ------------------
        Update_Position();

        // ------------------ UPDATE GLOBAL WAVE TARGETS ------------------
        double phase = phase_global;

        std::vector<sf::Vector2<double>> Next_Target(N);

        for (size_t i = 0; i < N; i++) {
            double x_i = Spheres_List[i].Pos_Unwrapped.x;

            double target_y =
                base_y_global + A_global * std::sin(k_global * x_i + phase);

            Next_Target[i].x = x_i;
            Next_Target[i].y = target_y;
        }

        // Check if all beads are done
        bool all_reached = true;
        for (size_t i = 0; i < N; i++) {
            if (!Bead_Done[i]) {
                all_reached = false;
                break;
            }
        }

        if (all_reached) {
            phase_global -= omega_global * Time_Step;

            Target_Pos = Next_Target;

            Bead_Done.assign(N, false);
            Prev_Force_Y.assign(N, 0.0);
            Prev_dy.assign(N, 0.0);
        }

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
