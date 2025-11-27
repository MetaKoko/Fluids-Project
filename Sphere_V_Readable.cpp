#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>

struct Sphere_Data{
    sf::Vector2f Pos_Vector;
    sf::Vector2f F_Vector;
    sf::Color Colour;
    sf::Vector2f V;
    float Rad;
};

std::vector<Sphere_Data> Spheres_List;
float Visc = 70.0f;
float Time_Step = 20.0f;
float Screen_Width = 1340.0f;
float Screen_Height = 700.0f;

void Add_Sphere(const sf::Vector2f& Pos_Vector, const sf::Vector2f& F_Vector, float Rad = 20.0f, sf::Color Colour = sf::Color::Red){
    Spheres_List.push_back({ Pos_Vector, F_Vector, Colour, sf::Vector2f(0.0f, 0.0f), Rad });
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
        Constant*(F_Vector.x*1.0f/r+Dot_Prod*Dist.x*1.0f/r3),
        Constant*(F_Vector.y*1.0f/r+Dot_Prod*Dist.y*1.0f/r3)
    );
}

// This is for calculating the faxen correction
sf::Vector2f faxen_Correction(sf::Vector2f Dist, sf::Vector2f F_Vector, float a_squared){
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r<0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Dot_Prod = Dist.x*F_Vector.x+Dist.y*F_Vector.y;
    float Constant = a_squared/(48.0f*pi*Visc);
    
    float r3 = r*r*r;
    float r5 = r*r*r*r*r;

    return sf::Vector2f(
        Constant*((F_Vector.x*1.0f/r3)-3.0f*Dot_Prod*Dist.x*1.0f/r5),
        Constant*((F_Vector.y*1.0f/r3)-3.0f*Dot_Prod*Dist.y*1.0f/r5)
    );
}

// This is for the terms where we have to apply a fexien correction a faxen correction
sf::Vector2f faxen_Correction_Of_faxen(sf::Vector2f Dist, sf::Vector2f F_Vector, float a_squared_source, float a_squared_target){
    float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
    if (r<0.0001f) return sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;
    float Dot_Prod = Dist.x*F_Vector.x+Dist.y*F_Vector.y;
    float Constant = (a_squared_target*a_squared_source)/(288.0f*pi*Visc);

    float r5 = r*r*r*r*r;
    float r7 = r*r*r*r*r*r*r;
    float r9 = r*r*r*r*r*r*r*r*r;

    return sf::Vector2f(
        Constant*((9.0f*F_Vector.x/r5)-45.0f*Dot_Prod*Dist.x/r7+105.0f*Dot_Prod*Dist.x/r9),
        Constant*((9.0f*F_Vector.y/r5)-45.0f*Dot_Prod*Dist.y/r7+105.0f*Dot_Prod*Dist.y/r9)
    );
}

//Quiver plot velocity calculation from all spheres
void Velocity_From_Sphere(const sf::Vector2f& Point, sf::Vector2f& V_Vector){
    V_Vector = sf::Vector2f(0.0f, 0.0f);

    // To implement periodic boundary conditions, we consider a 3x3 grid, were we have 8 images around our main box
    for (const auto& Sphere : Spheres_List){
        for (int dx = -1; dx <= 1; dx = dx+1){
            for (int dy = -1; dy <= 1; dy = dy+1){
                sf::Vector2f Image_Pos = Sphere.Pos_Vector;
                Image_Pos.x = Image_Pos.x+dx*Screen_Width;
                Image_Pos.y = Image_Pos.y+dy*Screen_Height;

                sf::Vector2f Dist = Point-Image_Pos;
                sf::Vector2f Stokes = Stokeslet(Dist, Sphere.F_Vector);
                sf::Vector2f Correction = faxen_Correction(Dist, Sphere.F_Vector, Sphere.Rad*Sphere.Rad);

                V_Vector.x = V_Vector.x+Stokes.x+Correction.x;
                V_Vector.y = V_Vector.y+Stokes.y+Correction.y;
            }
        }
    }
}

//Velocity calculation for sphere motion
void Calc_Sphere_Velocity(size_t Sphere_Index, sf::Vector2f& V_Vector){
    Sphere_Data target_sphere = Spheres_List[Sphere_Index];
    V_Vector = sf::Vector2f(0.0f, 0.0f);

    float pi = 3.14159f;

    // This is to calculate the stokes drag due to its own force
    float self_mob = 1.0f/(6.0f*pi*Visc*target_sphere.Rad);
    V_Vector.x = V_Vector.x+target_sphere.F_Vector.x*self_mob;
    V_Vector.y = V_Vector.y+target_sphere.F_Vector.y*self_mob;

    // Contributions from other spheres
    for (size_t i = 0; i<Spheres_List.size(); i = i+1){
        if (i == Sphere_Index) continue;
        Sphere_Data source_sphere = Spheres_List[i];

        // This is again for periodic boundary conditions
        for (int dx = -1; dx <= 1; dx = dx+1){
            for (int dy = -1; dy <= 1; dy = dy+1){
                sf::Vector2f Image_pos = source_sphere.Pos_Vector;
                Image_pos.x = Image_pos.x+dx*Screen_Width;
                Image_pos.y = Image_pos.y+dy*Screen_Height;

                sf::Vector2f Dist = target_sphere.Pos_Vector-Image_pos;

                // This puts all of the terms together
                sf::Vector2f Stokes = Stokeslet(Dist, source_sphere.F_Vector);
                sf::Vector2f S_Correction = faxen_Correction(Dist, source_sphere.F_Vector, source_sphere.Rad*source_sphere.Rad);

                sf::Vector2f T_Correction_Stokes = faxen_Correction(Dist, source_sphere.F_Vector, target_sphere.Rad*target_sphere.Rad);
                sf::Vector2f T_Correction_of_S_Correction = faxen_Correction_Of_faxen(Dist, source_sphere.F_Vector,
                    source_sphere.Rad*source_sphere.Rad, target_sphere.Rad*target_sphere.Rad);

                V_Vector.x = V_Vector.x+Stokes.x+S_Correction.x+T_Correction_Stokes.x+T_Correction_of_S_Correction.x;
                V_Vector.y = V_Vector.y+Stokes.y+S_Correction.y+T_Correction_Stokes.y+T_Correction_of_S_Correction.y;
            }
        }
    }
}

//This is our itterate function to update sphere positions
void Update_Position(){
    for (size_t i = 0; i<Spheres_List.size(); i = i+1){
        Calc_Sphere_Velocity(i, Spheres_List[i].V);
    }
    for (size_t i = 0; i<Spheres_List.size(); i = i+1){
        auto& sphere = Spheres_List[i];
        sphere.Pos_Vector.x = sphere.Pos_Vector.x+sphere.V.x*Time_Step;
        sphere.Pos_Vector.y = sphere.Pos_Vector.y+sphere.V.y*Time_Step;

        if (sphere.Pos_Vector.x<0.0f) sphere.Pos_Vector.x = sphere.Pos_Vector.x+Screen_Width;
        else if (sphere.Pos_Vector.x >= Screen_Width) sphere.Pos_Vector.x = sphere.Pos_Vector.x-Screen_Width;

        if (sphere.Pos_Vector.y<0.0f) sphere.Pos_Vector.y = sphere.Pos_Vector.y+Screen_Height;
        else if (sphere.Pos_Vector.y >= Screen_Height) sphere.Pos_Vector.y = sphere.Pos_Vector.y-Screen_Height;
    }
}

void Arrow_Head(std::vector<sf::Vertex>& lines, sf::Vector2f tip, sf::Vector2f Dire, sf::Color colour){
    float headSize = 10.0f;
    float headAngle = 0.6f;
    float angle = std::atan2(Dire.y, Dire.x);

    // This is for drawing our arrrowheads
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
    int Segments = 40;
    float pi = 3.14159f;

    for (int i = 0; i<Segments; i = i+1){
        float angle1 = 2.0f*pi*i/Segments;
        float angle2 = 2.0f*pi*(i+1)/Segments;

        sf::Vector2f p1 = sphere.Pos_Vector+sf::Vector2f(Rad*std::cos(angle1), Rad*std::sin(angle1));
        sf::Vector2f p2 = sphere.Pos_Vector+sf::Vector2f(Rad*std::cos(angle2), Rad*std::sin(angle2));

        lines.push_back(sf::Vertex(p1, sphere.Colour));
        lines.push_back(sf::Vertex(p2, sphere.Colour));
    }

    // This is to represent the force direction
    float Force_Scale = 0.03f;
    sf::Vector2f force_end = sphere.Pos_Vector+sphere.F_Vector*Force_Scale;
    lines.push_back(sf::Vertex(sphere.Pos_Vector, sf::Color::Black));
    lines.push_back(sf::Vertex(force_end, sf::Color::Black));

    // Draw arrowhead for force direction
    sf::Vector2f force_dir = force_end-sphere.Pos_Vector;
    float force_length = std::sqrt(force_dir.x*force_dir.x+force_dir.y*force_dir.y);
    if (force_length>0.1f){
        sf::Vector2f force_dir_normalized = force_dir/force_length;
        Arrow_Head(lines, force_end, force_dir_normalized, sf::Color::Black);
    }
}

// Create all the arrows and circles for visualization
void Draw_Everything(std::vector<sf::Vertex>& lines){
    lines.clear();

    //This is to create our grid
    for (int x = 0; x<Screen_Width; x = x+20){
        for (int y = 0; y<Screen_Height; y = y+20){
            sf::Vector2f point((float)x, (float)y);
            sf::Vector2f V;
            Velocity_From_Sphere(point, V);

            float speed = std::sqrt(V.x*V.x+V.y*V.y);

            //This is for drawing our arrows length based on speed
            float scale = 5000.0f;
            sf::Vector2f Dire = V/speed;
            float Arrow_Length = speed*scale;
            if (Arrow_Length<8.0f) Arrow_Length = 8.0f;
            if (Arrow_Length>30.0f) Arrow_Length = 30.0f;

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
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Spheres moving in viscous fluid");

    // Here is where we add our spheres
    Add_Sphere(sf::Vector2f(200.0f, 200.0f), sf::Vector2f(1000.0f, 0.0f), 10.0f, sf::Color::Blue);
    Add_Sphere(sf::Vector2f(600.0f, 400.0f), sf::Vector2f(-500.0f, -500.0f), 10.0f, sf::Color::Red);
    Add_Sphere(sf::Vector2f(1000.0f, 300.0f), sf::Vector2f(-400.0f, 500.0f), 10.0f, sf::Color::Green);

    std::vector<sf::Vertex> allLines;

    while (window.isOpen()){
        while (auto event = window.pollEvent()){
            if (event->is<sf::Event::Closed>()) window.close();
        }

        Update_Position();
        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}