#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>

struct Sphere_Data {
    sf::Vector2f Pos_Vector;
    sf::Vector2f F_Vector;
    sf::Color Colour;
    sf::Vector2f V;
    float Rad;
};

std::vector<Sphere_Data> Spheres_List;
std::vector<sf::Vector2f> Base_Pos;
std::vector<sf::Vector2f> Rest_Offset;

float Visc = 70.0f;
float Time_Step = 1.0f;
float Screen_Width = 1340.0f;
float Screen_Height = 700.0f;

void Add_Sphere(const sf::Vector2f& Pos_Vector, const sf::Vector2f& F_Vector,
    float Rad = 10.0f, sf::Color Colour = sf::Color::Blue) {
    Spheres_List.push_back({ Pos_Vector, F_Vector, Colour,
                             sf::Vector2f(0.0f, 0.0f), Rad });
}

sf::Vector2f Minimum_Image(sf::Vector2f d) {
    if (d.x > 0.5f * Screen_Width)  d.x -= Screen_Width;
    if (d.x < -0.5f * Screen_Width) d.x += Screen_Width;
    if (d.y > 0.5f * Screen_Height) d.y -= Screen_Height;
    if (d.y < -0.5f * Screen_Height) d.y += Screen_Height;
    return d;
}

sf::Vector2f Stokeslet(sf::Vector2f Dist, sf::Vector2f F_Vector) {
    float r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);
    if (r < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Constant = 1.0f / (8.0f * pi * Visc);
    float Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;

    float r3 = r * r * r;

    sf::Vector2f V;
    V.x = Constant * (F_Vector.x / r + Dot_Prod * Dist.x / r3);
    V.y = Constant * (F_Vector.y / r + Dot_Prod * Dist.y / r3);
    return V;
}

sf::Vector2f faxen_Correction(sf::Vector2f Dist, sf::Vector2f F_Vector, float a2) {
    float r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);
    if (r < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    float Constant = a2 / (48.0f * pi * Visc);

    float r3 = r * r * r;
    float r5 = r3 * r * r;

    sf::Vector2f V;
    V.x = Constant * (F_Vector.x / r3 - 3.0f * Dot_Prod * Dist.x / r5);
    V.y = Constant * (F_Vector.y / r3 - 3.0f * Dot_Prod * Dist.y / r5);
    return V;
}

sf::Vector2f faxen_Correction_Of_faxen(sf::Vector2f Dist, sf::Vector2f F_Vector,
    float a2_source, float a2_target) {
    float r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);
    if (r < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    float Constant = (a2_target * a2_source) / (288.0f * pi * Visc);

    float r5 = r * r * r * r * r;
    float r7 = r5 * r * r;
    float r9 = r7 * r * r;

    sf::Vector2f V;
    V.x = Constant * (9.0f * F_Vector.x / r5 - 45.0f * Dot_Prod * Dist.x / r7 + 105.0f * Dot_Prod * Dist.x / r9);
    V.y = Constant * (9.0f * F_Vector.y / r5 - 45.0f * Dot_Prod * Dist.y / r7 + 105.0f * Dot_Prod * Dist.y / r9);
    return V;
}

void Calc_Sphere_Velocity(size_t Sphere_Index, sf::Vector2f& V_Vector) {
    Sphere_Data target = Spheres_List[Sphere_Index];
    V_Vector = sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float self_mob = 1.0f / (6.0f * pi * Visc * target.Rad);

    V_Vector += target.F_Vector * self_mob;

    for (size_t i = 0; i < Spheres_List.size(); i++) {
        if (i == Sphere_Index) continue;

        Sphere_Data source = Spheres_List[i];

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {

                sf::Vector2f Image_Pos = source.Pos_Vector;
                Image_Pos.x += dx * Screen_Width;
                Image_Pos.y += dy * Screen_Height;

                sf::Vector2f Dist = target.Pos_Vector - Image_Pos;

                sf::Vector2f S = Stokeslet(Dist, source.F_Vector);
                sf::Vector2f C1 = faxen_Correction(Dist, source.F_Vector, source.Rad * source.Rad);
                sf::Vector2f C2 = faxen_Correction(Dist, source.F_Vector, target.Rad * target.Rad);
                sf::Vector2f C3 = faxen_Correction_Of_faxen(Dist, source.F_Vector,
                    source.Rad * source.Rad,
                    target.Rad * target.Rad);

                V_Vector += (S + C1 + C2 + C3);
            }
        }
    }
}

//
// ⭐ NEW INTERNAL SHAPE‑RESTORING SPRING FORCE
//
sf::Vector2f Shape_Spring(
    const sf::Vector2f& A, const sf::Vector2f& B,
    const sf::Vector2f& RestA, const sf::Vector2f& RestB,
    float k_spring)
{
    sf::Vector2f dx = Minimum_Image(B - A);
    sf::Vector2f dR = RestB - RestA;

    return k_spring * (dx - dR);
}

void Apply_Spring_Forces(float k_spring) {
    for (auto& s : Spheres_List)
        s.F_Vector = sf::Vector2f(0, 0);

    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        sf::Vector2f RestA = Base_Pos[i] + Rest_Offset[i];
        sf::Vector2f RestB = Base_Pos[i + 1] + Rest_Offset[i + 1];

        sf::Vector2f F = Shape_Spring(
            Spheres_List[i].Pos_Vector,
            Spheres_List[i + 1].Pos_Vector,
            RestA, RestB,
            k_spring
        );

        Spheres_List[i].F_Vector += F;
        Spheres_List[i + 1].F_Vector -= F;
    }
}

sf::Vector2f Compute_Net_Force() {
    sf::Vector2f net(0, 0);
    for (const auto& s : Spheres_List)
        net += s.F_Vector;
    return net;
}

void Update_Position(float k_spring) {
    Apply_Spring_Forces(k_spring);

    sf::Vector2f netF = Compute_Net_Force();
    static int counter = 0;
    counter++;

    if (counter % 40 == 0) {   // print every 20 frames
        std::cout << "Net force: " << netF.x << ", " << netF.y << "\n";
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

void Draw_Sphere(std::vector<sf::Vertex>& lines, const Sphere_Data& sphere) {
    float Rad = sphere.Rad;
    int Segments = 40;
    float pi = 3.14159f;

    for (int i = 0; i < Segments; i++) {
        float a1 = 2 * pi * i / Segments;
        float a2 = 2 * pi * (i + 1) / Segments;

        sf::Vector2f p1 = sphere.Pos_Vector + sf::Vector2f(Rad * std::cos(a1), Rad * std::sin(a1));
        sf::Vector2f p2 = sphere.Pos_Vector + sf::Vector2f(Rad * std::cos(a2), Rad * std::sin(a2));

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }
}

void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();
    for (size_t i = 0; i < Spheres_List.size(); i++)
        Draw_Sphere(lines, Spheres_List[i]);
}

sf::Vector2f Compute_COM() {
    sf::Vector2f com(0, 0);
    for (const auto& s : Spheres_List)
        com += s.Pos_Vector;
    return com / (float)Spheres_List.size();
}

int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(),
        "Travelling Wave Swimmer — Internal Spring Model");

    size_t N = 10;
    float spacing = 15.0f;
    float radius = 1.0f;
    float spring_k = 0.02f;

    float base_x = 600.0f;
    float base_y = 350.0f;

    float Angle = 3.14159f / 4.0f;

    sf::Vector2f Dir(std::cos(Angle), std::sin(Angle));
    sf::Vector2f Normal(-Dir.y, Dir.x);

    Base_Pos.resize(N);
    Rest_Offset.resize(N);

    for (size_t i = 0; i < N; i++) {
        sf::Vector2f Pos = sf::Vector2f(
            base_x + Dir.x * (float)i * spacing,
            base_y + Dir.y * (float)i * spacing
        );

        Base_Pos[i] = Pos;
        Rest_Offset[i] = sf::Vector2f(0, 0);

        Add_Sphere(Pos, sf::Vector2f(0, 0), radius, sf::Color::Blue);
    }

    sf::Clock clock;
    std::vector<sf::Vertex> allLines;

    sf::Vector2f Prev_COM = Compute_COM();

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
        }

        float t = clock.getElapsedTime().asSeconds();

        float A = 100.0f;
        float omega = 2.0f;
        float k = 1.0f;

        for (size_t i = 0; i < N; i++) {
            float wave = A * std::sin(k * (float)i - omega * t);
            Rest_Offset[i] = Normal * wave;
        }

        Update_Position(spring_k);

        sf::Vector2f Current_COM = Compute_COM();
        Prev_COM = Current_COM;

        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}

