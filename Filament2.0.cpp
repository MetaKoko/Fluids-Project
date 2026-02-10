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
std::vector<sf::Vector2<double>> Rest_Offset;

double Visc = 1.0;
double Time_Step = 1e-6;
double Screen_Width = 1340.0;
double Screen_Height = 700.0;

sf::Vector2<double> Dir;

// Your old zoom factor — now used by the SFML camera
double Zoom = 1e6;


// =============================================================
// Helpers
// =============================================================

sf::FloatRect ComputeBoundingBox()
{
    double minX = 1e300, maxX = -1e300;
    double minY = 1e300, maxY = -1e300;

    for (const auto& s : Spheres_List) {
        minX = std::min(minX, s.Pos_Vector.x);
        maxX = std::max(maxX, s.Pos_Vector.x);
        minY = std::min(minY, s.Pos_Vector.y);
        maxY = std::max(maxY, s.Pos_Vector.y);
    }

    sf::Vector2f pos((float)minX, (float)minY);
    sf::Vector2f size((float)(maxX - minX), (float)(maxY - minY));

    return sf::FloatRect(pos, size);
}



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

        sf::Vector2<double> RestA = Base_Pos[i] + Rest_Offset[i];
        sf::Vector2<double> RestB = Base_Pos[i + 1] + Rest_Offset[i + 1];

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
// Time stepping
// =============================================================
void Update_Position(double k_spring)
{
    static sf::Vector2<double> Prev_COM = Compute_COM();
    static int counter = 0;

    Apply_Spring_Forces(k_spring);

    sf::Vector2<double> Current_COM = Compute_COM();
    sf::Vector2<double> COM_Vel = (Current_COM - Prev_COM) / Time_Step;

    sf::Vector2<double> body = Spheres_List.back().Pos_Vector - Spheres_List.front().Pos_Vector;
    double blen = std::sqrt(body.x * body.x + body.y * body.y);

    sf::Vector2<double> Dir_inst = (blen > 1e-12)
        ? sf::Vector2<double>(body.x / blen, body.y / blen)
        : Dir;

    double Speed = COM_Vel.x * Dir_inst.x + COM_Vel.y * Dir_inst.y;

    Prev_COM = Current_COM;

    counter++;
    if (counter % 100 == 0) {
        sf::Vector2<double> netF = Compute_Net_Force();
        std::cout << std::scientific << std::setprecision(12)
            << "NetF = (" << netF.x << ", " << netF.y << ")"
            << "   Forward speed = " << Speed << "\n";
    }

    for (size_t i = 0; i < Spheres_List.size(); i++)
        Calc_Sphere_Velocity(i, Spheres_List[i].V);

    for (auto& s : Spheres_List) {
        s.Pos_Vector += s.V * Time_Step;

        if (s.Pos_Vector.x < 0) s.Pos_Vector.x += Screen_Width;
        else if (s.Pos_Vector.x >= Screen_Width) s.Pos_Vector.x -= Screen_Width;

        if (s.Pos_Vector.y < 0) s.Pos_Vector.y += Screen_Height;
        else if (s.Pos_Vector.y >= Screen_Height) s.Pos_Vector.y -= Screen_Height;
    }
}

// =============================================================
// Drawing using SFML View zoom
// =============================================================
void Draw_Everything(sf::RenderWindow& window,
    double visualRadiusMultiplier)
{
    for (const auto& s : Spheres_List) {

        double drawRadius = s.Rad * visualRadiusMultiplier;

        sf::CircleShape bead;
        bead.setRadius((float)drawRadius);
        bead.setOrigin(sf::Vector2f((float)drawRadius, (float)drawRadius));
        bead.setPosition(sf::Vector2f((float)s.Pos_Vector.x,
            (float)s.Pos_Vector.y));
        bead.setFillColor(s.Colour);

        window.draw(bead);
    }
}

// =============================================================
// main
// =============================================================
int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(),
        "Travelling Wave Swimmer (Stokes, Zoomed)");
	sf::View view;

    size_t N = 30;
    double spacing = 5e-6;
    double radius = 2.5e-7;
    double spring_k = 5e-9;

    double base_x = Screen_Width / 2.0;
    double base_y = Screen_Height / 2.0;

    double Angle = 3.141592653589793 / 4.0;

    Dir = sf::Vector2<double>(std::cos(Angle), std::sin(Angle));
    sf::Vector2<double> Normal(-Dir.y, Dir.x);

    double A = 5e-6;
    double omega = 1.0;
    double k = 1.0;

    Base_Pos.resize(N);
    Rest_Offset.resize(N);

    for (size_t i = 0; i < N; i++) {
        sf::Vector2<double> base(
            base_x + Dir.x * (double)i * spacing,
            base_y + Dir.y * (double)i * spacing
        );

        double wave = A * std::sin(k * (double)i);
        sf::Vector2<double> offset = Normal * wave;

        Base_Pos[i] = base;
        Rest_Offset[i] = offset;

        sf::Vector2<double> Pos = base + offset;

        Add_Sphere(Pos, { 0.0, 0.0 }, radius, sf::Color::Blue);
    }

    // =========================================================
    // Auto-sized SFML View (cleanest version)
    // =========================================================
    double visualRadiusMultiplier = 5.0;

    sf::Clock clock;

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
        }

        double t = clock.getElapsedTime().asSeconds();

        for (size_t i = 0; i < N; i++) {
            double wave = A * std::sin(k * (double)i - omega * t);
            Rest_Offset[i] = Normal * wave;
        }

        Update_Position(spring_k);

        // --- Dynamic view sizing (SFML 3.0.2 compatible) ---
        sf::Vector2<double> com = Compute_COM();
        sf::FloatRect box = ComputeBoundingBox();

        // physical size of swimmer
        double Lx = box.size.x;
        double Ly = box.size.y;

        // world size implied by zoom
        double zoomWorldWidth = Screen_Width / Zoom;
        double zoomWorldHeight = Screen_Height / Zoom;

        // final view size = max(zoom window, swimmer size)
        double viewWidth = std::max(zoomWorldWidth, Lx * 1.2);
        double viewHeight = std::max(zoomWorldHeight, Ly * 1.2);

        // update view
        view.setSize(sf::Vector2f((float)viewWidth, (float)viewHeight));
        view.setCenter(sf::Vector2f((float)com.x, (float)com.y));
        window.setView(view);


        window.clear(sf::Color::White);
        Draw_Everything(window, visualRadiusMultiplier);
        window.display();
    }

    return 0;
}
