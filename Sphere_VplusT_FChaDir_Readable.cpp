/*#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

struct Sphere_Data{
    sf::Vector2f Pos_Vector;
    sf::Vector2f F_Vector;
    float Torque;
    sf::Color Colour;
    sf::Vector2f V_Vector;
    float Angular_V;
    float Rad;
    float Angle;
    float F_Mag;
    sf::Vector2f F_Dire;
    sf::Vector2f OGF_Dire;
};

std::vector<Sphere_Data> Spheres_List;
float Visc = 70.0f;
float Time_Step = 25.0f;
float Screen_Width = 1340.0f;
float Screen_Height = 700.0f;

void Add_Sphere(const sf::Vector2f& Pos_Vector, float F_Mag, const sf::Vector2f& OGF_Dire_V,
    float Torque = 0.0f, float Rad = 20.0f, sf::Color Colour = sf::Color::Red){

    // All of this deals with the visualization of the original force direction and new force direction
    sf::Vector2f NOGF_Dire = OGF_Dire_V;
    float Mag = std::sqrt(NOGF_Dire.x*NOGF_Dire.x+NOGF_Dire.y*NOGF_Dire.y);
    if (Mag>0.0001f){
        NOGF_Dire.x = NOGF_Dire.x/Mag;
        NOGF_Dire.y = NOGF_Dire.y/Mag;
    }

    sf::Vector2f OG_F = NOGF_Dire*F_Mag;

    Spheres_List.push_back({
        Pos_Vector,
        OG_F,
        Torque,
        Colour,
        sf::Vector2f(0.0f, 0.0f),
        0.0f,
        Rad,
        0.0f,
        F_Mag,
        NOGF_Dire,
        NOGF_Dire
        });
}

// This is for calculating stokeslet part of the velocity
sf::Vector2f Stokeslet(sf::Vector2f Dist, sf::Vector2f F_Vector){
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r<0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Constant = 1.0f/(8.0f*pi*Visc);
    float Dot_Prod = Dist.x*F_Vector.x+Dist.y*F_Vector.y;

    float r3 = r*r*r;

    return sf::Vector2f(
        Constant*(F_Vector.x/r+Dot_Prod*Dist.x*1.0f/r3),
        Constant*(F_Vector.y/r+Dot_Prod*Dist.y*1.0f/r3)
    );
}

// This is for calculating the faxen correction part of the velocity
sf::Vector2f Faxen_Correction(sf::Vector2f Dist, sf::Vector2f F_Vector, float a_squared){
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r<0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Dot_Prod = Dist.x*F_Vector.x+Dist.y*F_Vector.y;
    float Constant = a_squared/(48.0f*pi*Visc);

    float r3 = r*r*r;
    float r5 = r*r*r*r*r;

    return sf::Vector2f(
        Constant*(F_Vector.x*1.0f/r3-3.0f*Dot_Prod*Dist.x*1.0f/r5),
        Constant*(F_Vector.y*1.0f/r3-3.0f*Dot_Prod*Dist.y*1.0f/r5)
    );
}

// This is for the terms where we have to apply a faxen correction of a faxen correction
sf::Vector2f Faxen_Correction_Of_Faxen(sf::Vector2f Dist, sf::Vector2f F_Vector, float a_squared_source, float a_squared_target){
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r<0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Dot_Prod = Dist.x*F_Vector.x+Dist.y*F_Vector.y;
    float Constant = (a_squared_target*a_squared_source)/(288.0f*pi*Visc);

    float r5 = r*r*r*r*r;
    float r7 = r*r*r*r*r*r*r;
    float r9 = r*r*r*r*r*r*r*r*r;

    return sf::Vector2f(
        Constant*(9.0f*F_Vector.x/r5-45.0f*Dot_Prod*Dist.x/r7+105.0f*Dot_Prod*Dist.x/r9),
        Constant*(9.0f*F_Vector.y/r5-45.0f*Dot_Prod*Dist.y/r7+105.0f*Dot_Prod*Dist.y/r9)
    );
}

// This is to calculate the vorticity contributions from other spheres
float Complete_Disturbance_Vorticity(sf::Vector2f Dist, const Sphere_Data& source_sphere, float target_Rad){
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r<0.0001f) return 0.0f;

    float pi = 3.14159f;

    float Constant = 1.0f/(8.0f*pi*Visc);
    float Cross_Prod = (Dist.x*source_sphere.F_Vector.y-Dist.y*source_sphere.F_Vector.x);
    float Stokeslet_Vorticity = Constant*Cross_Prod/(r*r*r);
    float Rotlet_Vorticity = source_sphere.Torque/(8.0f*pi*Visc*r*r*r);
    float Dot_Prod = Dist.x*source_sphere.F_Vector.x+Dist.y*source_sphere.F_Vector.y;
    float Source_Faxen_Vorticity_Constant = source_sphere.Rad*source_sphere.Rad/(48.0f*pi*Visc);

    float r5 = r*r*r*r*r;
    float r7 = r*r*r*r*r*r*r;
    float Source_Faxen_Vorticity = Source_Faxen_Vorticity_Constant*(-3.0f*Cross_Prod/r5+15.0f*Dot_Prod*Cross_Prod/r7);

    float Target_Faxen_Vorticity_Constant = target_Rad*target_Rad/(48.0f*pi*Visc);
    float Target_Faxen_Vorticity_Stokes = Target_Faxen_Vorticity_Constant*(-3.0f*Cross_Prod/r5+15.0f*Dot_Prod*Cross_Prod/r7);

    return Stokeslet_Vorticity+Rotlet_Vorticity+Source_Faxen_Vorticity+Target_Faxen_Vorticity_Stokes;
}

// Velocity calculation for sphere motion
void Calc_Sphere_Velocity_And_Vorticity(size_t Sphere, sf::Vector2f& V_Vector, float& Angular_V){
    const Sphere_Data& target_sphere = Spheres_List[Sphere];
    V_Vector = sf::Vector2f(0.0f, 0.0f);
    Angular_V = 0.0f;

    float pi = 3.14159265358979323846f;

    // This is to calculate the stokes drag due to its own force
    float self_mob = 1.0f/(6.0f*pi*Visc*target_sphere.Rad);
    V_Vector = V_Vector+target_sphere.F_Vector*self_mob;

    // Self rotation term
    float rotational_mob = 1.0f/(8.0f*pi*Visc*std::pow(target_sphere.Rad, 3.0f));
    Angular_V = Angular_V+target_sphere.Torque*rotational_mob;

    for (size_t i = 0; i<Spheres_List.size(); i = i+1){
        if (i == Sphere) continue;

        const Sphere_Data& source_sphere = Spheres_List[i];

        // To implement periodic boundary conditions, we consider a 3x3 grid, were we have 8 images around our main box
        for (int dx = -1; dx<=1; dx = dx+1){
            for (int dy = -1; dy<=1; dy = dy+1){
                sf::Vector2f Image_Pos = source_sphere.Pos_Vector;
                Image_Pos.x = Image_Pos.x+dx*Screen_Width;
                Image_Pos.y = Image_Pos.y+dy*Screen_Height;

                sf::Vector2f Dist = target_sphere.Pos_Vector-Image_Pos;

                sf::Vector2f Stokes_V = Stokeslet(Dist, source_sphere.F_Vector);
                sf::Vector2f S_Correction = Faxen_Correction(Dist, source_sphere.F_Vector, source_sphere.Rad*source_sphere.Rad);

                sf::Vector2f T_Correction_Stokes = Faxen_Correction(Dist, source_sphere.F_Vector, target_sphere.Rad*target_sphere.Rad);
                sf::Vector2f T_Correction_SourceFaxen = Faxen_Correction_Of_Faxen(Dist, source_sphere.F_Vector,
                    source_sphere.Rad*source_sphere.Rad, target_sphere.Rad*target_sphere.Rad);
                
                V_Vector.x = V_Vector.x+Stokes_V.x+S_Correction.x+T_Correction_Stokes.x+T_Correction_SourceFaxen.x;
                V_Vector.y = V_Vector.y+Stokes_V.y+S_Correction.y+T_Correction_Stokes.y+T_Correction_SourceFaxen.y;

                float complete_vorticity = Complete_Disturbance_Vorticity(Dist, source_sphere, target_sphere.Rad);
                Angular_V = Angular_V+0.5f*complete_vorticity;
            }
        }
    }
}

void Update_Force_Directions(){
    for (auto& sphere : Spheres_List){
        float cos_angle = std::cos(sphere.Angle);
        float sin_angle = std::sin(sphere.Angle);

        sf::Vector2f New_F_dir;
        New_F_dir.x = sphere.OGF_Dire.x*cos_angle-sphere.OGF_Dire.y*sin_angle;
        New_F_dir.y = sphere.OGF_Dire.x*sin_angle+sphere.OGF_Dire.y*cos_angle;

        sphere.F_Vector = New_F_dir*sphere.F_Mag;
        sphere.F_Dire = New_F_dir;
    }
}

// This is our iterate function to update sphere positions
void Update_Position(){
    Update_Force_Directions();

    for (size_t i = 0; i<Spheres_List.size(); i = i+1){
        sf::Vector2f V_Vector;
        float Angular_V;
        Calc_Sphere_Velocity_And_Vorticity(i, V_Vector, Angular_V);

        Spheres_List[i].V_Vector = V_Vector;
        Spheres_List[i].Angular_V = Angular_V;
    }

    for (auto& sphere : Spheres_List){
        sphere.Pos_Vector = sphere.Pos_Vector+sphere.V_Vector*Time_Step;
        sphere.Angle = sphere.Angle+sphere.Angular_V*Time_Step;
        float pi = 3.14159f;
        if (sphere.Angle>2.0f*pi) sphere.Angle = sphere.Angle-2.0f*pi;
        if (sphere.Angle<0.0f) sphere.Angle = sphere.Angle+2.0f*pi;

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

    // This is again for periodic boundary conditions
    for (const auto& sphere : Spheres_List){
        for (int dx = -1; dx<=1; dx = dx+1){
            for (int dy = -1; dy<=1; dy = dy+1){
                sf::Vector2f Image_Pos = sphere.Pos_Vector;
                Image_Pos.x = Image_Pos.x+dx*Screen_Width;
                Image_Pos.y = Image_Pos.y+dy*Screen_Height;

                sf::Vector2f Dist = Point-Image_Pos;
                float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
                //if (r<sphere.Rad*1.1f) continue;
                sf::Vector2f stokes_flow = Stokeslet(Dist, sphere.F_Vector);
                sf::Vector2f faxen_correction = Faxen_Correction(Dist, sphere.F_Vector, sphere.Rad*sphere.Rad);
                V_Vector = V_Vector+stokes_flow+faxen_correction;

                float pi = 3.14159f;
                float Constant = 1.0f/(8.0f*pi*Visc);
                float Stokeslet_Vorticity = (Dist.x*sphere.F_Vector.y-Dist.y*sphere.F_Vector.x)*1.0f/(r*r*r);
                Stokeslet_Vorticity = Stokeslet_Vorticity*Constant;
                float Rotlet_Vorticity = sphere.Torque/(8.0f*pi*Visc*r*r*r);
                Vort = Vort+Stokeslet_Vorticity+Rotlet_Vorticity;
            }
        }
    }
}

void Arrow_Head(std::vector<sf::Vertex>& lines, sf::Vector2f tip, sf::Vector2f Dire, sf::Color colour){
    float headSize = 12.0f;
    float headAngle = 0.7f;
    float angle = std::atan2(Dire.y, Dire.x);

    // This is for drawing our arrowheads
    sf::Vector2f head1 = tip+sf::Vector2f(-headSize*std::cos(angle-headAngle), -headSize*std::sin(angle-headAngle));
    sf::Vector2f head2 = tip+sf::Vector2f(-headSize*std::cos(angle+headAngle), -headSize*std::sin(angle+headAngle));

    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head1, colour));
    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head2, colour));
}

void Draw_Sphere(std::vector<sf::Vertex>& lines, const Sphere_Data& sphere){
    // We draw circles as our spheres for visualization
    float Rad = sphere.Rad;
    int Segments = 32;
    float pi = 3.14159f;

    for (int i = 0; i<Segments; i = i+1){
        float angle1 = 2.0f*pi*i/Segments+sphere.Angle;
        float angle2 = 2.0f*pi*(i+1)/Segments+sphere.Angle;
        sf::Vector2f p1 = sphere.Pos_Vector+sf::Vector2f(Rad*std::cos(angle1), Rad*std::sin(angle1));
        sf::Vector2f p2 = sphere.Pos_Vector+sf::Vector2f(Rad*std::cos(angle2), Rad*std::sin(angle2));

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }

    // BLACK LINE: Shows original force direction
    float Force_Scale = 0.03f;
    sf::Vector2f original_F_end = sphere.Pos_Vector+sphere.OGF_Dire*sphere.F_Mag*Force_Scale;
    lines.push_back(sf::Vertex(sphere.Pos_Vector, sf::Color::Black));
    lines.push_back(sf::Vertex(original_F_end, sf::Color::Black));

    // RED LINE: Shows current force direction
    sf::Vector2f current_F_end = sphere.Pos_Vector+sphere.F_Vector*Force_Scale;
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

            // This is for drawing our arrows length based on speed
            float scale = 500.0f;
            sf::Vector2f Dire = V_Vector/speed;
            float Arrow_Length = speed*scale;
            if (Arrow_Length<1.0f) Arrow_Length = 1.0f;
            if (Arrow_Length>50.0f) Arrow_Length = 50.0f;

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

int main(){
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(desktop, "Spheres with changing force direction");
    window.setFramerateLimit(60);

    // Here is where we add our spheres
    Add_Sphere(sf::Vector2f(200.0f, 200.0f), 1000.0f, sf::Vector2f(1.0f, 0.0f), 0.0f, 10.0f, sf::Color::Blue);
    Add_Sphere(sf::Vector2f(600.0f, 400.0f), 800.0f, sf::Vector2f(0.707f, 0.707f), 0.0f, 10.0f, sf::Color::Red);
    Add_Sphere(sf::Vector2f(1000.0f, 300.0f), 1200.0f, sf::Vector2f(0.0f, 1.0f), 0.0f, 10.0f, sf::Color::Green);

    std::vector<sf::Vertex> allLines;

    while (window.isOpen()){
        if (auto event = window.pollEvent()){
            if (event->is<sf::Event::Closed>()){
                window.close();
            }
        }

        Update_Position();
        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        if (!allLines.empty()){
            window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        }
        window.display();
    }

    return 0;
}*/