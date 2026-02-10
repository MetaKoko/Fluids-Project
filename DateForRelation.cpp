#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

// ===============================
// Simple 2D vector
// ===============================
struct Vec2 {
    double x, y;

    Vec2(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(double a) const { return Vec2(a * x, a * y); }
    Vec2 operator/(double a) const { return Vec2(x / a, y / a); }

    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
};

inline Vec2 operator*(double a, const Vec2& v) { return Vec2(a * v.x, a * v.y); }

// ===============================
// Data structures
// ===============================
struct Sphere_Data {
    Vec2 Pos_Unwrapped;   // physical position (unwrapped)
    Vec2 F_Vector;        // force
    Vec2 V;               // velocity
    double Rad;           // radius
};

std::vector<Sphere_Data> Spheres_List;
std::vector<Vec2> Rest_Pos;      // time-dependent rest positions (actuation)
std::vector<double> Rest_Lengths;
std::vector<double> S_param;     // arc-length parameter along body

double Global_Spacing = 0.0;

// ===============================
// Physical + numerical parameters
// ===============================
double Visc = 1e-3;
double Time_Step = 1e-4;   // you can reduce this if needed

double Domain_Width = 2e-3;
double Domain_Height = 2e-3;

Vec2 Dir;      // swimming direction
Vec2 Normal;   // normal to Dir

// travelling wave parameters (dimensionless here)
double k_wave = 1.0;   // k
double omega = 1.0;   // omega

// spring stiffness
double k_struct = 5e-13;
double k_act = 2e-15;

// ===============================
// Helpers
// ===============================
void Add_Sphere(const Vec2& Pos, const Vec2& F_Vector, double Rad = 1e-6)
{
    Spheres_List.push_back({ Pos, F_Vector, Vec2(0.0, 0.0), Rad });
}

Vec2 Minimum_Image(Vec2 d) {
    if (d.x > 0.5 * Domain_Width)  d.x -= Domain_Width;
    if (d.x < -0.5 * Domain_Width)  d.x += Domain_Width;
    if (d.y > 0.5 * Domain_Height) d.y -= Domain_Height;
    if (d.y < -0.5 * Domain_Height) d.y += Domain_Height;
    return d;
}

Vec2 Stokeslet(Vec2 Dist, Vec2 F_Vector)
{
    double r2 = Dist.x * Dist.x + Dist.y * Dist.y;
    double r = std::sqrt(r2);
    if (r < 1e-14) return Vec2(0.0, 0.0);

    double pi = 3.141592653589793;
    double Constant = 1.0 / (8.0 * pi * Visc);
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double r3 = r2 * r;

    return Vec2(
        Constant * (F_Vector.x / r + Dot_Prod * Dist.x / r3),
        Constant * (F_Vector.y / r + Dot_Prod * Dist.y / r3)
    );
}

Vec2 faxen_Correction(Vec2 Dist, Vec2 F_Vector, double a2)
{
    double r2 = Dist.x * Dist.x + Dist.y * Dist.y;
    double r = std::sqrt(r2);
    if (r < 1e-14) return Vec2(0.0, 0.0);

    double pi = 3.141592653589793;
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = a2 / (48.0 * pi * Visc);

    double r3 = r2 * r;
    double r5 = r3 * r2;

    return Vec2(
        Constant * (F_Vector.x / r3 - 3.0 * Dot_Prod * Dist.x / r5),
        Constant * (F_Vector.y / r3 - 3.0 * Dot_Prod * Dist.y / r5)
    );
}

Vec2 faxen_Correction_Of_faxen(Vec2 Dist, Vec2 F_Vector,
    double a2_source, double a2_target)
{
    double r2 = Dist.x * Dist.x + Dist.y * Dist.y;
    double r = std::sqrt(r2);
    if (r < 1e-14) return Vec2(0.0, 0.0);

    double pi = 3.141592653589793;
    double Dot_Prod = Dist.x * F_Vector.x + Dist.y * F_Vector.y;
    double Constant = (a2_target * a2_source) / (288.0 * pi * Visc);

    double r5 = r2 * r2 * r;
    double r7 = r5 * r2;
    double r9 = r7 * r2;

    return Vec2(
        Constant * (9.0 * F_Vector.x / r5 - 45.0 * Dot_Prod * Dist.x / r7 + 105.0 * Dot_Prod * Dist.x / r9),
        Constant * (9.0 * F_Vector.y / r5 - 45.0 * Dot_Prod * Dist.y / r7 + 105.0 * Dot_Prod * Dist.y / r9)
    );
}

void Calc_Sphere_Velocity(size_t Sphere_Index, Vec2& V_Vector) {
    Sphere_Data target = Spheres_List[Sphere_Index];
    V_Vector = Vec2(0.0, 0.0);

    double pi = 3.141592653589793;
    double self_mob = 1.0 / (6.0 * pi * Visc * target.Rad);

    V_Vector += self_mob * target.F_Vector;

    for (size_t i = 0; i < Spheres_List.size(); i++) {
        if (i == Sphere_Index) continue;

        Sphere_Data source = Spheres_List[i];

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {

                Vec2 Image_Pos = source.Pos_Unwrapped;
                Image_Pos.x += dx * Domain_Width;
                Image_Pos.y += dy * Domain_Height;

                Vec2 Dist = Minimum_Image(target.Pos_Unwrapped - Image_Pos);

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

void Apply_Structural_Springs(double k_struct)
{
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        Vec2 dx = Spheres_List[i + 1].Pos_Unwrapped - Spheres_List[i].Pos_Unwrapped;
        double L = std::sqrt(dx.x * dx.x + dx.y * dx.y);
        if (L < 1e-14) continue;

        double L0 = Rest_Lengths[i];

        Vec2 F = k_struct * (L - L0) * (dx / L);

        Spheres_List[i].F_Vector += F;
        Spheres_List[i + 1].F_Vector -= F;
    }
}

void Apply_Actuation_Springs(double k_wave)
{
    for (size_t i = 0; i + 1 < Spheres_List.size(); i++) {

        Vec2 dx = Spheres_List[i + 1].Pos_Unwrapped - Spheres_List[i].Pos_Unwrapped;
        Vec2 dR = Rest_Pos[i + 1] - Rest_Pos[i];

        Vec2 F = k_wave * (dx - dR);

        Spheres_List[i].F_Vector += F;
        Spheres_List[i + 1].F_Vector -= F;
    }
}

Vec2 Compute_COM() {
    Vec2 com(0.0, 0.0);
    for (const auto& s : Spheres_List)
        com += s.Pos_Unwrapped;
    return com / (double)Spheres_List.size();
}

// ===============================
// One full simulation for a given A
// ===============================
double Run_Simulation_For_Amplitude(double A)
{
    const size_t N = 30;
    Spheres_List.clear();
    Rest_Pos.clear();
    Rest_Lengths.clear();
    S_param.clear();

    // geometry
    double length = 50e-6;
    double spacing = length / (N - 1);
    Global_Spacing = spacing;

    // base position
    double base_x = 0.5 * Domain_Width;
    double base_y = 0.5 * Domain_Height;

    // direction and normal
    double pi = 3.141592653589793;
    double Angle = pi / 4.0;
    Dir = Vec2(std::cos(Angle), std::sin(Angle));
    Normal = Vec2(-Dir.y, Dir.x);

    // allocate
    Rest_Pos.resize(N);
    S_param.resize(N);

    // arc-length-ish initialisation along Dir with small wave at t=0
    double ds = spacing * 0.05;
    double s = 0.0;

    for (size_t i = 0; i < N; i++) {
        S_param[i] = s;

        double wave = A * std::sin(k_wave * s);  // phase at t=0
        Vec2 pos(
            base_x + Dir.x * s + Normal.x * wave,
            base_y + Dir.y * s + Normal.y * wave
        );

        Rest_Pos[i] = pos;
        Add_Sphere(pos, Vec2(0.0, 0.0), 0.3e-6);

        double accumulated = 0.0;
        while (accumulated < spacing) {
            double slope = A * k_wave * std::cos(k_wave * s);
            double ds_arc = std::sqrt(1.0 + slope * slope) * ds;
            accumulated += ds_arc;
            s += ds;
        }
    }

    // structural rest lengths
    Rest_Lengths.resize(N - 1);
    for (size_t i = 0; i + 1 < N; ++i) {
        Vec2 dx = Rest_Pos[i + 1] - Rest_Pos[i];
        Rest_Lengths[i] = std::sqrt(dx.x * dx.x + dx.y * dx.y);
    }

    // time integration
    double period = 2.0 * pi / omega;
    int    periods_total = 40;
    int    periods_discard = 20;

    int steps_per_period = (int)std::round(period / Time_Step);
    int total_steps = periods_total * steps_per_period;

    Vec2 Prev_COM = Compute_COM();

    double accum_disp = 0.0;
    double accum_time = 0.0;

    for (int step = 0; step < total_steps; ++step) {
        double t = step * Time_Step;

        // update rest shape (actuation)
        for (size_t i = 0; i < N; i++) {
            double s_i = S_param[i];
            double waveT = A * std::sin(k_wave * s_i - omega * t);

            Rest_Pos[i] = Vec2(
                base_x + Dir.x * s_i + Normal.x * waveT,
                base_y + Dir.y * s_i + Normal.y * waveT
            );
        }

        // reset forces
        for (auto& s : Spheres_List)
            s.F_Vector = Vec2(0.0, 0.0);

        // structural + actuation springs
        Apply_Structural_Springs(k_struct);
        Apply_Actuation_Springs(k_act);

        // hydrodynamic velocities
        for (size_t i = 0; i < N; i++)
            Calc_Sphere_Velocity(i, Spheres_List[i].V);

        // update positions
        for (auto& s : Spheres_List) {
            s.Pos_Unwrapped += s.V * Time_Step;
        }

        // measure COM displacement after transient
        Vec2 Current_COM = Compute_COM();
        Vec2 dCOM = Current_COM - Prev_COM;

        if (step >= periods_discard * steps_per_period) {
            double d_forward = dCOM.x * Dir.x + dCOM.y * Dir.y;
            accum_disp += d_forward;
            accum_time += Time_Step;
        }

        Prev_COM = Current_COM;
    }

    double avg_speed = accum_disp / accum_time;
    return avg_speed;
}

// ===============================
// main: sweep amplitudes, write CSV
// ===============================
int main()
{
    std::ofstream outfile("swimmer_data.csv");
    outfile << "A,A2,avg_speed\n";

    int num_points = 10;
    double A_min = 1e-7;
    double A_max = 5e-7;

    for (int i = 0; i < num_points; ++i) {
        double A = A_min + (A_max - A_min) * (double(i) / (num_points - 1));

        double avg_speed = Run_Simulation_For_Amplitude(A);

        std::cout << "A = " << A
            << "  A^2 = " << (A * A)
            << "  avg_speed = " << std::scientific << avg_speed << "\n";

        outfile << std::setprecision(16)
            << A << "," << (A * A) << "," << avg_speed << "\n";
    }

    outfile.close();
    return 0;
}
