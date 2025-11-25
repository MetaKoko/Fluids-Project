#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

struct Sphere_Data {
    sf::Vector2f Pos_Vector;
    sf::Vector2f F_Vector;
    float Torque;
    sf::Color Colour;
    sf::Vector2f V;
    float Angular_V;
    float Radius;
    float Angle;
    float F_Magnitude;
    sf::Vector2f F_Direction;
    sf::Vector2f OGF_Direction;
};

std::vector<Sphere_Data> Spheres_List;
float Visc = 70.0f;
float Time_Step = 25.0f;
float Screen_Width = 1340.0f;
float Screen_Height = 700.0f;

void Add_Sphere(const sf::Vector2f& Pos_Vector, float F_Magnitude, const sf::Vector2f& OGF_Direction_V,
    float torque = 0.0f, float Radius = 20.0f, sf::Color Colour = sf::Color::Red){

    //All of this deals with the visualization of the original force direction and new force direction
    sf::Vector2f NOGF_Direction = OGF_Direction_V;
    float Mag = std::sqrt(NOGF_Direction.x*NOGF_Direction.x+NOGF_Direction.y*NOGF_Direction.y);
    if (Mag > 0.0001f){
        NOGF_Direction.x /= Mag;
        NOGF_Direction.y /= Mag;
    }

    float OG_Angle = std::atan2(NOGF_Direction.y, NOGF_Direction.x);
    sf::Vector2f OG_F = NOGF_Direction*F_Magnitude;
    sf::Vector2f New_F_dir(1.0f, 0.0f);

    Spheres_List.push_back({
        Pos_Vector,
        OG_F,
        torque,
        Colour,
        sf::Vector2f(0.0f, 0.0f),
        0.0f,
        Radius,
        OG_Angle,              
        F_Magnitude,
        New_F_dir,
        NOGF_Direction
        });
}

void New_Force_Directions() {
    for (auto& sphere : Spheres_List) {
        float cos_angle = std::cos(sphere.Angle);
        float sin_angle = std::sin(sphere.Angle);

        sf::Vector2f world_F_dir;
        world_F_dir.x = sphere.F_Direction.x*cos_angle-sphere.F_Direction.y*sin_angle;
        world_F_dir.y = sphere.F_Direction.x*sin_angle+sphere.F_Direction.y*cos_angle;

        sphere.F_Vector = world_F_dir*sphere.F_Magnitude;
    }
}

// This is for calculating stokeslet part of the velocity
sf::Vector2f Stokeslet(sf::Vector2f Dist, sf::Vector2f F_Vector) {
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Constant = 1.0f/(8.0f*pi*Visc);
    float Dot_Prod = Dist.x*F_Vector.x+Dist.y*F_Vector.y;

    return sf::Vector2f(
        Constant*(F_Vector.x/r+Dot_Prod*Dist.x*1.0f/(r*r*r)),
        Constant*(F_Vector.y/r+Dot_Prod*Dist.y*1.0f/(r*r*r))
    );
}

// This is for calculating rotlet used for rotation
sf::Vector2f Rotlet(sf::Vector2f Dist, float torque) {
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Constant = torque/(8.0f*pi*Visc);

    return sf::Vector2f(
        -Constant*Dist.y*1.0f/(r*r*r),
        Constant*Dist.x*1.0f/(r*r*r)
    );
}

// This is for calculating the faxien correction part of the velocity
sf::Vector2f Faxien_Correction(sf::Vector2f Dist, sf::Vector2f F_Vector, float a_squared) {
    float r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);
    if (r < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Dot_Prod = Dist.x*F_Vector.x+Dist.y*F_Vector.y;
    float Constant = a_squared/(24.0f*pi*Visc);

    return sf::Vector2f(
        Constant*(F_Vector.x*1.0f/(r*r*r)-3.0f*Dot_Prod*Dist.x*1.0f/(r*r*r*r*r)),
        Constant*(F_Vector.y*1.0f/(r*r*r)-3.0f*Dot_Prod*Dist.y*1.0f/(r*r*r*r*r))
    );
}

// This is to calculate vorticity for torque coupling
float Coupling_Vorticity(sf::Vector2f Dist, const Sphere_Data& source_sphere) {
    float r = std::sqrt(Dist.x * Dist.x + Dist.y * Dist.y);
    if (r < 0.0001f) return 0.0f;

    float pi = 3.14159265358979323846f;
    float Constant = 1.0f/(8.0f*pi*Visc);

    float Stokeslet_Vorticity = (Dist.x*source_sphere.F_Vector.y-Dist.y*source_sphere.F_Vector.x)*1.0f/(r*r*r);
    Stokeslet_Vorticity = Stokeslet_Vorticity*Constant;

    float Rotlet_Vorticity = source_sphere.Torque/(4.0f*pi*Visc*r*r*r);

    return Stokeslet_Vorticity+Rotlet_Vorticity;
}

// Velocity calculation for sphere motion
void Calc_Sphere_Velocity_And_Vorticity(size_t Sphere, sf::Vector2f& V_Vector, float& Vorticity) {
    const Sphere_Data& target_sphere = Spheres_List[Sphere];
    V_Vector = sf::Vector2f(0.0f, 0.0f);
    Vorticity = 0.0f;

    float pi = 3.14159265358979323846f;

    // This is to calculate the stokes drag due to its own force
    float self_mob = 1.0f/(6.0f*pi*Visc*target_sphere.Radius);
    V_Vector = V_Vector+target_sphere.F_Vector*self_mob;

    // Self rotation term
    float rotational_mob = 1.0f/(8.0f*pi*Visc*std::pow(target_sphere.Radius, 3.0f));
    Vorticity = Vorticity+target_sphere.Torque*rotational_mob;

    // Rotational contribution from other spheres
    float Coupling_Vorticity_Total = 0.0f;

    for (size_t i = 0; i<Spheres_List.size(); i = i+1){
        if (i == Sphere) continue;

        const Sphere_Data& source_sphere = Spheres_List[i];

        // This is again for periodic boundary conditions
        for (int dx = -1; dx<=1; dx = dx+1){
            for (int dy = -1; dy<=1; dy = dy+1){
                sf::Vector2f Image_Pos = source_sphere.Pos_Vector;
                Image_Pos.x += dx*Screen_Width;
                Image_Pos.y += dy*Screen_Height;

                sf::Vector2f Dist = target_sphere.Pos_Vector-Image_Pos;

                // Velocity contributions from other spheres
                sf::Vector2f Stokes_V = Stokeslet(Dist, source_sphere.F_Vector);
                sf::Vector2f Correction_V = Faxien_Correction(Dist, source_sphere.F_Vector, target_sphere.Radius*target_sphere.Radius);
                V_Vector = V_Vector+Stokes_V+Correction_V;

                // Vorticity from torque coupling
                Coupling_Vorticity_Total = Coupling_Vorticity_Total+Coupling_Vorticity(Dist, source_sphere);
            }
        }
    }

    // Torque coupling term
    Vorticity = Vorticity+0.5f*Coupling_Vorticity_Total;
}

// This is our iterate function to update sphere positions
void Update_Position(){
    New_Force_Directions();

    for (size_t i = 0; i<Spheres_List.size(); i = i+1) {
        sf::Vector2f V_Vector;
        float Angular_V;
        Calc_Sphere_Velocity_And_Vorticity(i, V_Vector, Angular_V);

        Spheres_List[i].V = V_Vector;
        Spheres_List[i].Angular_V = Angular_V;
    }

    for (auto& sphere : Spheres_List) {
        sphere.Pos_Vector = sphere.Pos_Vector+sphere.V*Time_Step;
        sphere.Angle = sphere.Angle+sphere.Angular_V*Time_Step;

        if (sphere.Angle > 2.0f * 3.14159f) sphere.Angle -= 2.0f * 3.14159f;
        if (sphere.Angle < 0.0f) sphere.Angle += 2.0f * 3.14159f;

        if (sphere.Pos_Vector.x<0.0f) sphere.Pos_Vector.x = sphere.Pos_Vector.x+Screen_Width;
        else if (sphere.Pos_Vector.x>=Screen_Width) sphere.Pos_Vector.x = sphere.Pos_Vector.x-Screen_Width;
        if (sphere.Pos_Vector.y<0.0f) sphere.Pos_Vector.y = sphere.Pos_Vector.y+Screen_Height;
        else if (sphere.Pos_Vector.y>=Screen_Height) sphere.Pos_Vector.y = sphere.Pos_Vector.y-Screen_Height;
    }
}

//Velocity and Vorticity calculation from all spheres
void Velocity_From_Sphere(const sf::Vector2f& Point, sf::Vector2f& V_Vector, float& Vort){
    V_Vector = sf::Vector2f(0.0f, 0.0f);
    Vort = 0.0f;

    // To implement periodic boundary conditions, we consider a 3x3 grid, were we have 8 images around our main box
    for (const auto& sphere : Spheres_List){
        for (int dx = -1; dx<=1; dx = dx+1){
            for (int dy = -1; dy<=1; dy = dy+1){
                sf::Vector2f Image_Pos = sphere.Pos_Vector;
                Image_Pos.x = Image_Pos.x+dx*Screen_Width;
                Image_Pos.y = Image_Pos.y+dy*Screen_Height;

                sf::Vector2f Dist = Point-Image_Pos;
                sf::Vector2f stokes_flow = Stokeslet(Dist, sphere.F_Vector);
                V_Vector = V_Vector+stokes_flow;
                Vort = Vort+Coupling_Vorticity(Dist, sphere);
            }
        }
    }
}

void Arrow_Head(std::vector<sf::Vertex>& lines, sf::Vector2f tip, sf::Vector2f Dire, sf::Color colour) {
    float headSize = 12.0f;
    float headAngle = 0.7f;
    float angle = std::atan2(Dire.y, Dire.x);

    // This is for drawing our arrowheads
    sf::Vector2f head1 = tip + sf::Vector2f(-headSize * std::cos(angle - headAngle), -headSize * std::sin(angle - headAngle));
    sf::Vector2f head2 = tip + sf::Vector2f(-headSize * std::cos(angle + headAngle), -headSize * std::sin(angle + headAngle));

    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head1, colour));
    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head2, colour));
}

void Draw_Sphere(std::vector<sf::Vertex>& lines, const Sphere_Data& sphere) {
    // We draw circles as our spheres for visualization
    float Radius = sphere.Radius;
    int Segments = 32;
    float pi = 3.14159f;

    // Draw sphere body (rotates with orientation)
    for (int i = 0; i < Segments; i = i+1) {
        float angle1 = 2.0f*pi*i/Segments+sphere.Angle;
        float angle2 = 2.0f*pi*(i+1)/Segments+sphere.Angle;
        sf::Vector2f p1 = sphere.Pos_Vector+sf::Vector2f(Radius*std::cos(angle1), Radius*std::sin(angle1));
        sf::Vector2f p2 = sphere.Pos_Vector+sf::Vector2f(Radius*std::cos(angle2), Radius*std::sin(angle2));

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }

    // BLACK LINE: Shows the Original force direction
    sf::Vector2f initial_F_end = sphere.Pos_Vector + sphere.OGF_Direction * Radius * 1.5f;
    lines.push_back(sf::Vertex(sphere.Pos_Vector, sf::Color::Black));
    lines.push_back(sf::Vertex(initial_F_end, sf::Color::Black));

    // RED LINE: Shows current force direction
    float Force_Scale = 0.03f;
    sf::Vector2f current_F_end = sphere.Pos_Vector + sphere.F_Vector * Force_Scale;
    lines.push_back(sf::Vertex(sphere.Pos_Vector, sf::Color::Red));
    lines.push_back(sf::Vertex(current_F_end, sf::Color::Red));
}

// Create all the arrows and circles for visualization
void Draw_Everything(std::vector<sf::Vertex>& lines){
    lines.clear();

    // This is to create our grid
    for (int x = 0; x<Screen_Width; x = x+20){
        for (int y = 0; y<Screen_Height; y = y+20){
            sf::Vector2f point((float)x, (float)y);
            sf::Vector2f V_Vector;
            float Vort;
            Velocity_From_Sphere(point, V_Vector, Vort);

            float speed = std::sqrt(V_Vector.x*V_Vector.x+V_Vector.y*V_Vector.y);

            // Makes the visualization nicer by removing very small arrows
            if (speed < 0.0001f) continue;

            // This is for drawing our arrows length based on speed
            float scale = 5000.0f;
            sf::Vector2f Dire = V_Vector/speed;
            float Arrow_Length = speed*scale;
            if (Arrow_Length < 8.0f) Arrow_Length = 8.0f;
            if (Arrow_Length > 30.0f) Arrow_Length = 30.0f;

            sf::Vector2f Arrow_Tip = point+Dire*Arrow_Length;
            lines.push_back(sf::Vertex(point, sf::Color::Green));
            lines.push_back(sf::Vertex(Arrow_Tip, sf::Color::Green));

            Arrow_Head(lines, Arrow_Tip, Dire, sf::Color::Green);
        }
    }

    // Drawing our spheres
    for (const auto& sphere : Spheres_List){
        Draw_Sphere(lines, sphere);
    }
}

int main() {
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(desktop, "Original Force Direction Reference");
    window.setFramerateLimit(60);

    // Here is where we add our spheres
    Add_Sphere(sf::Vector2f(200.0f, 200.0f), 1000.0f, sf::Vector2f(1.0f, 0.0f), 0.0f, 20.0f, sf::Color::Blue);      // Black line: right
    Add_Sphere(sf::Vector2f(600.0f, 400.0f), 800.0f, sf::Vector2f(0.707f, 0.707f), 0.0f, 20.0f, sf::Color::Red);    // Black line: up-right 45°
    Add_Sphere(sf::Vector2f(1000.0f, 300.0f), 1200.0f, sf::Vector2f(0.0f, 1.0f), 0.0f, 20.0f, sf::Color::Green);    // Black line: up

    std::vector<sf::Vertex> allLines;

    while (window.isOpen()) {
        if (auto event = window.pollEvent()){
            if (event->is<sf::Event::Closed>()){
                window.close();
            }
        }

        Update_Position();
        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        if (!allLines.empty()) {
            window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        }
        window.display();
    }

    return 0;
}