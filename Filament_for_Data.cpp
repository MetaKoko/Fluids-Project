#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>

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

double Visc = 70.0;
double Time_Step = 1.0;
double Screen_Width = 1340.0;
double Screen_Height = 700.0;

// ⭐ Make Dir global so Update_Position can use it
sf::Vector2<double> Dir;

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
    double r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);

    double pi = 3.141592653589793;
    double Constant = 1.0 / (8.0 * pi * Visc);
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;

    double r3 = r * r * r;

    sf::Vector2<double> V;
    V.x = Constant * (F_Vector.x / r + Dot_Prod * Dist.x / r3);
    V.y = Constant * (F_Vector.y / r + Dot_Prod * Dist.y / r3);
    return V;
}

sf::Vector2<double> faxen_Correction(sf::Vector2<double> Dist,
    sf::Vector2<double> F_Vector,
    double a2)
{
    double r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);

    double pi = 3.141592653589793;
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = a2 / (48.0 * pi * Visc);

    double r3 = r * r * r;
    double r5 = r3 * r * r;

    sf::Vector2<double> V;
    V.x = Constant * (F_Vector.x / r3 - 3.0 * Dot_Prod * Dist.x / r5);
    V.y = Constant * (F_Vector.y / r3 - 3.0 * Dot_Prod * Dist.y / r5);
    return V;
}

sf::Vector2<double> faxen_Correction_Of_faxen(sf::Vector2<double> Dist,
    sf::Vector2<double> F_Vector,
    double a2_source,
    double a2_target)
{
    double r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);

    double pi = 3.141592653589793;
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = (a2_target * a2_source) / (288.0 * pi * Visc);

    double r5 = r * r * r * r * r;
    double r7 = r5 * r * r;
    double r9 = r7 * r * r;

    sf::Vector2<double> V;
    V.x = Constant * (9.0 * F_Vector.x / r5 - 45.0 * Dot_Prod * Dist.x / r7 + 105.0 * Dot_Prod * Dist.x / r9);
    V.y = Constant * (9.0 * F_Vector.y / r5 - 45.0 * Dot_Prod * Dist.y / r7 + 105.0 * Dot_Prod * Dist.y / r9);
    return V;
}

void Calc_Sphere_Velocity(size_t Sphere_Index, sf::Vector2<double>& V_Vector) {
    Sphere_Data target = Spheres_List[Sphere_Index];
    V_Vector = sf::Vector2<double>(0.0, 0.0);

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

                sf::Vector2<double> S = Stokeslet(Dist, source.F_Vector);
                sf::Vector2<double> C1 = faxen_Correction(Dist, source.F_Vector, source.Rad * source.Rad);
                sf::Vector2<double> C2 = faxen_Correction(Dist, source.F_Vector, target.Rad * target.Rad);
                sf::Vector2<double> C3 = faxen_Correction_Of_faxen(Dist, source.F_Vector,
                    source.Rad * source.Rad,
                    target.Rad * target.Rad);

                V_Vector += (S + C1 + C2 + C3);
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
        s.F_Vector = sf::Vector2<double>(0.0, 0.0);

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

void Update_Position(double k_spring,
    double& Speed,
    sf::Vector2<double>& Prev_COM,
    double dt)
{
    Apply_Spring_Forces(k_spring);

    sf::Vector2<double> Current_COM = Compute_COM();
    sf::Vector2<double> COM_Vel = (Current_COM - Prev_COM) / dt;

    sf::Vector2<double> body = Spheres_List.back().Pos_Vector - Spheres_List.front().Pos_Vector;
    double blen = std::sqrt(body.x * body.x + body.y * body.y);

    sf::Vector2<double> Dir_inst = (blen > 1e-12)
        ? sf::Vector2<double>(body.x / blen, body.y / blen)
        : Dir;

    // Project COM velocity onto instantaneous body axis
    double forward_speed = COM_Vel.x * Dir_inst.x + COM_Vel.y * Dir_inst.y;
    Speed = forward_speed;

    Speed = forward_speed;

    Prev_COM = Current_COM;

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

int main() {

    std::ofstream outfile("swimmer_data.csv");
    outfile << "A,A2,avg_speed\n";

    std::vector<double> amplitude_list;
    int num_points = 20;
    double A_min = 0.0001;
    double A_max = 0.0002;

    for (int i = 0; i < num_points; i++) {
        double A = A_min + (A_max - A_min) * (double(i) / (num_points - 1));
        amplitude_list.push_back(A);
    }

    for (double A : amplitude_list) {

        Spheres_List.clear();
        Base_Pos.clear();
        Rest_Offset.clear();

        size_t N = 30;
        double spacing = 15.0;
        double radius = 1.0;
        double spring_k = 5.0;

        double base_x = 600.0;
        double base_y = 350.0;

        double Angle = 3.141592653589793 / 4.0;

        Dir = sf::Vector2<double>(std::cos(Angle), std::sin(Angle));
        sf::Vector2<double> Normal(-Dir.y, Dir.x);

        Base_Pos.resize(N);
        Rest_Offset.resize(N);

        for (size_t i = 0; i < N; i++) {
            sf::Vector2<double> Pos(
                base_x + Dir.x * (double)i * spacing,
                base_y + Dir.y * (double)i * spacing
            );

            Base_Pos[i] = Pos;
            Rest_Offset[i] = sf::Vector2<double>(0.0, 0.0);

            Add_Sphere(Pos, sf::Vector2<double>(0.0, 0.0), radius, sf::Color::Blue);
        }

        sf::Clock clock;

        sf::Vector2<double> Prev_COM = Compute_COM();
        double Prev_t = 0.0;
        double Speed = 0.0;

        double speed_accumulator = 0.0;
        int speed_count = 0;

        double omega = 1.0;
        double k = 1.0;
        double period = 2.0 * 3.141592653589793 / omega;

        int periods_completed = 0;
        double last_period_time = 0.0;

        while (true) {

            double t = clock.getElapsedTime().asSeconds();
            double dt = t - Prev_t;
            Prev_t = t;
            if (dt <= 0.0)
                dt = 1e-12;

            for (size_t i = 0; i < N; i++) {
                double wave = A * std::sin(k * (double)i - omega * t);
                Rest_Offset[i] = Normal * wave;
            }

            Update_Position(spring_k, Speed, Prev_COM, dt);

            speed_accumulator += Speed;
            speed_count++;

            if (t - last_period_time >= period) {
                last_period_time = t;
                periods_completed++;

                if (periods_completed >= 10) {
                    double avg_speed = speed_accumulator / speed_count;
                    outfile << A << "," << (A * A) << "," << avg_speed << "\n";
                    break;
                }
            }
        }
    }

    outfile.close();
    return 0;
}