#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

// =============================================================
// Data structures
// =============================================================
struct Sphere_Data {
    double x, y;      // position
    double fx, fy;    // force
    double vx, vy;    // velocity
    double Rad;
};

std::vector<Sphere_Data> Spheres_List;

// body-frame coordinates of beads
std::vector<double> Body_X;
std::vector<double> Body_Y;   // time-dependent wave shape

// parameters
double Visc = 1.0;
double Time_Step = 0.001;

// body pose (fixed orientation)
double Body_COM_x = 0.0;
double Body_COM_y = 0.0;

// =============================================================
// Helpers
// =============================================================
void Add_Sphere(double x, double y, double Rad) {
    Sphere_Data s;
    s.x = x;
    s.y = y;
    s.fx = 0.0;
    s.fy = 0.0;
    s.vx = 0.0;
    s.vy = 0.0;
    s.Rad = Rad;
    Spheres_List.push_back(s);
}

inline void Compute_COM(double& cx, double& cy) {
    cx = 0.0;
    cy = 0.0;
    for (const auto& s : Spheres_List) {
        cx += s.x;
        cy += s.y;
    }
    double invN = 1.0 / static_cast<double>(Spheres_List.size());
    cx *= invN;
    cy *= invN;
}

inline void Compute_Net_Force(double& fx, double& fy) {
    fx = 0.0;
    fy = 0.0;
    for (const auto& s : Spheres_List) {
        fx += s.fx;
        fy += s.fy;
    }
}

// body-frame rest positions → lab-frame (NO rotation)
inline void BodyToLab(double bx, double by, double& lx, double& ly) {
    lx = Body_COM_x + bx;
    ly = Body_COM_y + by;
}

// Stokeslet (3D Oseen tensor projected in 2D)
inline void Stokeslet(double dx, double dy,
    double Fx, double Fy,
    double& vx, double& vy)
{
    double r2 = dx * dx + dy * dy;
    double r = std::sqrt(r2);
    if (r < 1e-12) return;

    double pi = 3.141592653589793;
    double Constant = 1.0 / (8.0 * pi * Visc);
    double Dot_Prod = dx * Fx + dy * Fy;
    double r3 = r2 * r;

    vx += Constant * (Fx / r + Dot_Prod * dx / r3);
    vy += Constant * (Fy / r + Dot_Prod * dy / r3);
}

void Calc_Sphere_Velocity(std::size_t i, double& vx, double& vy) {
    Sphere_Data target = Spheres_List[i];
    vx = 0.0;
    vy = 0.0;

    double pi = 3.141592653589793;
    double self_mob = 1.0 / (6.0 * pi * Visc * target.Rad);

    // self mobility
    vx += target.fx * self_mob;
    vy += target.fy * self_mob;

    // pairwise Stokeslets
    for (std::size_t j = 0; j < Spheres_List.size(); ++j) {
        if (j == i) continue;
        const Sphere_Data& src = Spheres_List[j];
        double dx = target.x - src.x;
        double dy = target.y - src.y;
        Stokeslet(dx, dy, src.fx, src.fy, vx, vy);
    }
}

// springs enforce body-frame shape only (internal forces)
void Apply_Spring_Forces(double k_spring) {
    for (auto& s : Spheres_List) {
        s.fx = 0.0;
        s.fy = 0.0;
    }

    std::size_t N = Spheres_List.size();
    for (std::size_t i = 0; i + 1 < N; ++i) {
        double RestAx, RestAy, RestBx, RestBy;
        BodyToLab(Body_X[i], Body_Y[i], RestAx, RestAy);
        BodyToLab(Body_X[i + 1], Body_Y[i + 1], RestBx, RestBy);

        double dx = Spheres_List[i + 1].x - Spheres_List[i].x;
        double dy = Spheres_List[i + 1].y - Spheres_List[i].y;

        double dRx = RestBx - RestAx;
        double dRy = RestBy - RestAy;

        double Fx = k_spring * (dx - dRx);
        double Fy = k_spring * (dy - dRy);

        Spheres_List[i].fx += Fx;
        Spheres_List[i].fy += Fy;
        Spheres_List[i + 1].fx -= Fx;
        Spheres_List[i + 1].fy -= Fy;
    }
}

// =============================================================
// Global accumulators for averaged COM speed
// =============================================================
double g_speed_accum = 0.0;
int    g_speed_count = 0;

void Reset_Accumulators() {
    g_speed_accum = 0.0;
    g_speed_count = 0;
}

double Get_Averaged_COM_Speed() {
    if (g_speed_count == 0) return 0.0;
    return g_speed_accum / (double)g_speed_count;
}

// =============================================================
// Time stepping
// =============================================================
void Update_Position(double k_spring, double t) {
    static bool first_call = true;
    static double Prev_COM_x = 0.0;
    static double Prev_COM_y = 0.0;
    static int counter = 0;

    // --- Compute COM ---
    Compute_COM(Body_COM_x, Body_COM_y);

    if (first_call) {
        Prev_COM_x = Body_COM_x;
        Prev_COM_y = Body_COM_y;
        first_call = false;
    }

    // --- Apply internal forces ---
    Apply_Spring_Forces(k_spring);

    // --- Compute COM velocity ---
    double Current_COM_x = Body_COM_x;
    double Current_COM_y = Body_COM_y;

    double COM_Vel_x = (Current_COM_x - Prev_COM_x) / Time_Step;

    Prev_COM_x = Current_COM_x;
    Prev_COM_y = Current_COM_y;

    double Speed_COM = COM_Vel_x;

    // --- Compute bead-average x velocity ---
    double bead_avg_vx = 0.0;
    for (auto& s : Spheres_List) bead_avg_vx += s.vx;
    bead_avg_vx /= (double)Spheres_List.size();

    // --- Accumulate after transient ---
    if (t > 50.0) {
        g_speed_accum += Speed_COM;
        g_speed_count++;
    }

    counter++;  // always increment

    if (t > 50.0 && counter % 5000 == 0) {
        double netFx, netFy;
        Compute_Net_Force(netFx, netFy);

        std::cout << std::scientific << std::setprecision(12)
            << "t = " << t
            << "   NetF = (" << netFx << ", " << netFy << ")"
            << "   COM speed = " << Speed_COM
            << "   bead-avg vx = " << bead_avg_vx;

        if (g_speed_count > 0) {
            double U_avg_COM = g_speed_accum / g_speed_count;
            std::cout << "   <U_COM> = " << U_avg_COM;
        }

        std::cout << "\n";
    }

    // --- Compute velocities for next step ---
    for (std::size_t i = 0; i < Spheres_List.size(); ++i) {
        double vx, vy;
        Calc_Sphere_Velocity(i, vx, vy);
        Spheres_List[i].vx = vx;
        Spheres_List[i].vy = vy;
    }

    // --- Update positions ---
    for (auto& s : Spheres_List) {
        s.x += s.vx * Time_Step;
        s.y += s.vy * Time_Step;
    }
}



int main() {
    std::size_t N = 400;
    double pi = 3.141592653589793;
    double L = 2.0 * pi;
    double spacing = L / (N - 1);
    double radius = 1e-4;
    double spring_k = 10.0;

    Body_X.resize(N);
    Body_Y.resize(N);

    Body_COM_x = 0.0;
    Body_COM_y = 0.0;

    Spheres_List.clear();
    for (std::size_t i = 0; i < N; ++i) {
        Body_X[i] = i * spacing;
        Body_Y[i] = 0.0;

        double lx, ly;
        BodyToLab(Body_X[i], Body_Y[i], lx, ly);
        Add_Sphere(lx, ly, radius);
    }

    double A = 0.01;
    double omega = 1.0;
    double k = 10.0;

    double T_final = 200.0;
    int steps = static_cast<int>(T_final / Time_Step);

    Reset_Accumulators();

    std::cout << "Running with N = " << N
        << ", spacing = " << spacing
        << ", A = " << A
        << ", dt = " << Time_Step
        << ", steps = " << steps << "\n";

    double t = 0.0;
    for (int n = 0; n < steps; ++n) {
        t = n * Time_Step;

        for (std::size_t i = 0; i < N; ++i) {
            Body_Y[i] = A * std::sin(k * Body_X[i] - omega * t);
        }

        Update_Position(spring_k, t);
    }

    double U_avg = Get_Averaged_COM_Speed();   // your measured <U_COM>
    double S_prefactor = U_avg / (omega * k * A * A);

    std::cout << std::scientific << std::setprecision(12);
    std::cout << "Average COM speed <U> = " << U_avg << "\n";
    std::cout << "Taylor prefactor S = <U> / (omega * k * A^2) = "
        << S_prefactor << "\n";
    std::cout << "Taylor prediction: S = -0.5\n";


    std::cout << "Simulation finished.\n";
    return 0;
}
