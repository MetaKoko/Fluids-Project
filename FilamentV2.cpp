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
float Time_Step = 15.0f;
float Screen_Width = 1340.0f;
float Screen_Height = 700.0f;

// For adding spheres
void Add_Sphere(const sf::Vector2f& Pos_Vector, const sf::Vector2f& F_Vector,
    float Rad = 10.0f, sf::Color Colour = sf::Color::Blue) {
    Spheres_List.push_back({ Pos_Vector, F_Vector, Colour,
                             sf::Vector2f(0.0f, 0.0f), Rad });
}

// Makes a bit bigger domain of filament calculations
sf::Vector2f Minimum_Image(sf::Vector2f d) {
    if (d.x > 0.5f * Screen_Width)  d.x = d.x - Screen_Width;
    if (d.x < -0.5f * Screen_Width)  d.x = d.x + Screen_Width;

    if (d.y > 0.5f * Screen_Height) d.y = d.y - Screen_Height;
    if (d.y < -0.5f * Screen_Height) d.y = d.y + Screen_Height;

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
    V_Vector.x = V_Vector.x + target.F_Vector.x * self_mob;
    V_Vector.y = V_Vector.y + target.F_Vector.y * self_mob;

    for (size_t i = 0; i < Spheres_List.size(); i = i + 1) {
        if (i == Sphere_Index) continue;

        Sphere_Data source = Spheres_List[i];

        for (int dx = -1; dx <= 1; dx = dx + 1) {
            for (int dy = -1; dy <= 1; dy = dy + 1) {

                sf::Vector2f Image_Pos = source.Pos_Vector;
                Image_Pos.x = Image_Pos.x + dx * Screen_Width;
                Image_Pos.y = Image_Pos.y + dy * Screen_Height;

                sf::Vector2f Dist;
                Dist.x = target.Pos_Vector.x - Image_Pos.x;
                Dist.y = target.Pos_Vector.y - Image_Pos.y;

                sf::Vector2f S = Stokeslet(Dist, source.F_Vector);
                sf::Vector2f C1 = faxen_Correction(Dist, source.F_Vector, source.Rad * source.Rad);
                sf::Vector2f C2 = faxen_Correction(Dist, source.F_Vector, target.Rad * target.Rad);
                sf::Vector2f C3 = faxen_Correction_Of_faxen(Dist, source.F_Vector,
                    source.Rad * source.Rad,
                    target.Rad * target.Rad);

                V_Vector.x = V_Vector.x + S.x + C1.x + C2.x + C3.x;
                V_Vector.y = V_Vector.y + S.y + C1.y + C2.y + C3.y;
            }
        }
    }
}


sf::Vector2f Spring_Force(const sf::Vector2f& A, const sf::Vector2f& B,
    float rest_length, float k_spring,
    const sf::Vector2f& RestA,
    const sf::Vector2f& RestB) {
    sf::Vector2f diff;
    diff.x = B.x - A.x;
    diff.y = B.y - A.y;

    diff = Minimum_Image(diff);

    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    if (dist < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float stretch = dist - rest_length;

    sf::Vector2f dir;
    dir.x = diff.x / dist;
    dir.y = diff.y / dist;

    sf::Vector2f F;
    F.x = k_spring * stretch * dir.x;
    F.y = k_spring * stretch * dir.y;

    sf::Vector2f FA;
    FA.x = k_spring * (RestA.x - A.x);
    FA.y = k_spring * (RestA.y - A.y);

    sf::Vector2f FB;
    FB.x = k_spring * (RestB.x - B.x);
    FB.y = k_spring * (RestB.y - B.y);

    F.x = F.x + FA.x - FB.x;
    F.y = F.y + FA.y - FB.y;

    return F;
}

void Apply_Spring_Forces(float rest_length, float k_spring) {
    for (size_t i = 0; i < Spheres_List.size(); i = i + 1) {
        Spheres_List[i].F_Vector = sf::Vector2f(0.0f, 0.0f);
    }

    for (size_t i = 0; i + 1 < Spheres_List.size(); i = i + 1) {
        sf::Vector2f RestA;
        RestA.x = Base_Pos[i].x + Rest_Offset[i].x;
        RestA.y = Base_Pos[i].y + Rest_Offset[i].y;

        sf::Vector2f RestB;
        RestB.x = Base_Pos[i + 1].x + Rest_Offset[i + 1].x;
        RestB.y = Base_Pos[i + 1].y + Rest_Offset[i + 1].y;

        sf::Vector2f F = Spring_Force(Spheres_List[i].Pos_Vector,
            Spheres_List[i + 1].Pos_Vector,
            rest_length, k_spring,
            RestA, RestB);

        Spheres_List[i].F_Vector.x = Spheres_List[i].F_Vector.x + F.x;
        Spheres_List[i].F_Vector.y = Spheres_List[i].F_Vector.y + F.y;

        Spheres_List[i + 1].F_Vector.x = Spheres_List[i + 1].F_Vector.x - F.x;
        Spheres_List[i + 1].F_Vector.y = Spheres_List[i + 1].F_Vector.y - F.y;
    }
}

// ------------------------------------------------------------
// UPDATE POSITIONS
// ------------------------------------------------------------
void Update_Position(float rest_length, float k_spring) {
    Apply_Spring_Forces(rest_length, k_spring);

    for (size_t i = 0; i < Spheres_List.size(); i = i + 1) {
        Calc_Sphere_Velocity(i, Spheres_List[i].V);
    }
    // For periodic boundaries
    for (size_t i = 0; i < Spheres_List.size(); i = i + 1) {
        Spheres_List[i].Pos_Vector.x = Spheres_List[i].Pos_Vector.x + Spheres_List[i].V.x * Time_Step;
        Spheres_List[i].Pos_Vector.y = Spheres_List[i].Pos_Vector.y + Spheres_List[i].V.y * Time_Step;

        if (Spheres_List[i].Pos_Vector.x < 0.0f) Spheres_List[i].Pos_Vector.x = Spheres_List[i].Pos_Vector.x + Screen_Width;
        else if (Spheres_List[i].Pos_Vector.x >= Screen_Width) Spheres_List[i].Pos_Vector.x = Spheres_List[i].Pos_Vector.x - Screen_Width;

        if (Spheres_List[i].Pos_Vector.y < 0.0f) Spheres_List[i].Pos_Vector.y = Spheres_List[i].Pos_Vector.y + Screen_Height;
        else if (Spheres_List[i].Pos_Vector.y >= Screen_Height) Spheres_List[i].Pos_Vector.y = Spheres_List[i].Pos_Vector.y - Screen_Height;
    }
}


void Draw_Sphere(std::vector<sf::Vertex>& lines, const Sphere_Data& sphere) {
    float Rad = sphere.Rad;
    int Segments = 40;
    float pi = 3.14159f;

    for (int i = 0; i < Segments; i = i + 1) {
        float a1 = 2.0f * pi * i / Segments;
        float a2 = 2.0f * pi * (i + 1) / Segments;

        sf::Vector2f p1 = sphere.Pos_Vector + sf::Vector2f(Rad * std::cos(a1), Rad * std::sin(a1));
        sf::Vector2f p2 = sphere.Pos_Vector + sf::Vector2f(Rad * std::cos(a2), Rad * std::sin(a2));

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }
}

void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();
    for (size_t i = 0; i < Spheres_List.size(); i = i + 1) {
        Draw_Sphere(lines, Spheres_List[i]);
    }
}


sf::Vector2f Compute_COM() {
    sf::Vector2f com(0.0f, 0.0f);
    for (size_t i = 0; i < Spheres_List.size(); i = i + 1) {
        com.x = com.x + Spheres_List[i].Pos_Vector.x;
        com.y = com.y + Spheres_List[i].Pos_Vector.y;
    }
    com.x = com.x / (float)Spheres_List.size();
    com.y = com.y / (float)Spheres_List.size();
    return com;
}

int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(),
        "Orientation-Aware Travelling Wave Swimmer");

    size_t N = 10;
    float spacing = 15.0f;
    float radius = 1.0f;

    float base_x = 600.0f;
    float base_y = 350.0f;

    float spring_k = 0.02f;

    float Angle = 3.14159f / 4.0f;

    sf::Vector2f Dir;
    Dir.x = std::cos(Angle);
    Dir.y = std::sin(Angle);

    sf::Vector2f Normal;
    Normal.x = -Dir.y;
    Normal.y = Dir.x;

    Base_Pos.resize(N);
    Rest_Offset.resize(N);

    for (size_t i = 0; i < N; i = i + 1) {
        sf::Vector2f Pos;
        Pos.x = base_x + Dir.x * (float)i * spacing;
        Pos.y = base_y + Dir.y * (float)i * spacing;

        Base_Pos[i] = Pos;
        Rest_Offset[i] = sf::Vector2f(0.0f, 0.0f);

        Add_Sphere(Pos, sf::Vector2f(0.0f, 0.0f), radius, sf::Color::Blue);
    }

    sf::Clock clock;
    std::vector<sf::Vertex> allLines;

    sf::Vector2f Prev_COM = Compute_COM();

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
        }

        float t = clock.getElapsedTime().asSeconds();

        float A = 50.0f;
        float omega = 2.0f;
        float k = 1.0f;

        for (size_t i = 0; i < N; i = i + 1) {
            float wave = A * std::sin(k * (float)i - omega * t);

            Rest_Offset[i].x = Normal.x * wave;
            Rest_Offset[i].y = Normal.y * wave;
        }

        Update_Position(spacing, spring_k);

        sf::Vector2f Current_COM = Compute_COM();

        sf::Vector2f COM_Vel;
        COM_Vel.x = (Current_COM.x - Prev_COM.x) / Time_Step;
        COM_Vel.y = (Current_COM.y - Prev_COM.y) / Time_Step;

        Prev_COM = Current_COM;

        float Speed = std::sqrt(COM_Vel.x * COM_Vel.x + COM_Vel.y * COM_Vel.y);
        std::cout << "Swimming speed: " << Speed << "\n";


        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}
