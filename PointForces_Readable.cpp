/*#include<vector>
#include<cmath>
#include<SFML/Graphics.hpp>

// Point force data
struct Point_Force_Data {
    sf::Vector2f Pos_Vector;
    sf::Vector2f F_Vector;
    sf::Color Colour;
    sf::Vector2f V;
};

std::vector<Point_Force_Data> Point_Force_List;
float Visc = 1.0f;
float Time_Step = 1.0f;
float Screen_Width = 1340.0f;
float Screen_Height = 700.0f;

void Point_Force(const sf::Vector2f& Pos_Vector, const sf::Vector2f& F_Vector, sf::Color Colour = sf::Color::Red) {
    Point_Force_List.push_back({Pos_Vector, F_Vector, Colour, sf::Vector2f(0.0f, 0.0f)});
}
// This is for calculating velocity at each point induced by a point force for quiver plot
void Velocity_From_Point_Force(const sf::Vector2f& Point, sf::Vector2f& V_Vector){
    V_Vector = sf::Vector2f(0.0f, 0.0f);

	// To implement periodic boundary conditions, we consider a 3x3 grid, were we have 8 images around our main box
    for (const auto& Point_Force : Point_Force_List){
        for (int dx = -1; dx<=1; dx = dx+1) {
            for (int dy = -1; dy<=1; dy = dy+1) {
                sf::Vector2f Image_Pos = Point_Force.Pos_Vector;
                Image_Pos.x = Image_Pos.x+dx*Screen_Width;
                Image_Pos.y = Image_Pos.y+dy*Screen_Height;
                sf::Vector2f Dist = Point-Image_Pos;
                float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);

                if (r < 0.0001f) continue;

                float pi = 3.14159f;
                float Constant = 1.0f/(8.0f*pi*Visc);
                float Dot_Prod = Dist.x*Point_Force.F_Vector.x+Dist.y*Point_Force.F_Vector.y;

                V_Vector.x = V_Vector.x+Constant*(Point_Force.F_Vector.x*1.0f/r+Dot_Prod*Dist.x*1.0f/(r*r*r));
                V_Vector.y = V_Vector.y+Constant*(Point_Force.F_Vector.y*1.0f/r+Dot_Prod*Dist.y*1.0f/(r*r*r));
            }
        }
    }
}
//This is so that we don't have the non image point force affecting its own velocity for point force motion
void Velocity_From_Point_Force_Excluding_Self(const sf::Vector2f& Point, size_t Ignore, sf::Vector2f& V_Vector) {
    V_Vector = sf::Vector2f(0.0f, 0.0f);

    for (size_t i = 0; i<Point_Force_List.size(); i = i+1){
        const auto& Point_Force = Point_Force_List[i];

        for (int dx = -1; dx<=1; dx = dx+1) {
            for (int dy = -1; dy<=1; dy = dy+1) {
                if (i == Ignore && dx == 0 && dy == 0) {
                    continue;
                }

                sf::Vector2f Image_Pos = Point_Force.Pos_Vector;
                Image_Pos.x = Image_Pos.x+dx*Screen_Width;
                Image_Pos.y = Image_Pos.y+dy*Screen_Height;

                sf::Vector2f Dist = Point-Image_Pos;
                float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);

                if (r < 0.0001f) continue;

                float pi = 3.14159f;
                float Constant = 1.0f/(8.0f*pi*Visc);
                float Dot_Prod = Dist.x* Point_Force.F_Vector.x+Dist.y* Point_Force.F_Vector.y;

                V_Vector.x = V_Vector.x+Constant*(Point_Force.F_Vector.x*1.0f/r+Dot_Prod*Dist.x*1.0f/(r*r*r));
                V_Vector.y = V_Vector.y+Constant*(Point_Force.F_Vector.y*1.0f/r+Dot_Prod*Dist.y*1.0f/(r*r*r));
            }
        }
    }
}

//This is our itterate function to update point force positions
void Update_Position(){
    for (size_t i = 0; i<Point_Force_List.size(); i = i+1){
        auto& Point_Force = Point_Force_List[i];

        sf::Vector2f velocity;
        Velocity_From_Point_Force_Excluding_Self(Point_Force.Pos_Vector, i, velocity);

        Point_Force.V = velocity;
    }

    for (size_t i = 0; i<Point_Force_List.size(); i = i+1){
        auto& Point_Force = Point_Force_List[i];
        Point_Force.Pos_Vector = Point_Force.Pos_Vector+Point_Force.V*Time_Step;

        if (Point_Force.Pos_Vector.x<0.0f){
            Point_Force.Pos_Vector.x = Point_Force.Pos_Vector.x+Screen_Width;
        }
        else if (Point_Force.Pos_Vector.x>=Screen_Width) {
            Point_Force.Pos_Vector.x = Point_Force.Pos_Vector.x-Screen_Width;
        }

        if (Point_Force.Pos_Vector.y<0.0f){
            Point_Force.Pos_Vector.y = Point_Force.Pos_Vector.y+Screen_Height;
        }
        else if (Point_Force.Pos_Vector.y >= Screen_Height){
            Point_Force.Pos_Vector.y = Point_Force.Pos_Vector.y-Screen_Height;
        }
    }
}

void Arrow_Head(std::vector<sf::Vertex>& lines, sf::Vector2f tip, sf::Vector2f Dire, sf::Color colour) {
    float headSize = 10.0f;
    float headAngle = 0.6f;
    float angle = std::atan2(Dire.y, Dire.x);

    // This is for drawing our arrrowheads
    sf::Vector2f head1 = tip+sf::Vector2f(
        -headSize*std::cos(angle-headAngle), 
        -headSize*std::sin(angle-headAngle));
    sf::Vector2f head2 = tip+sf::Vector2f(
        -headSize*std::cos(angle+headAngle), 
        -headSize*std::sin(angle+headAngle));

    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head1, colour));
    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head2, colour));
}

void Draw_Point_Force(std::vector<sf::Vertex>& lines, const Point_Force_Data& PF){
    // We draw circles to as out point forces for visualization
    float Radius = 10.0f;
    int Circle_Segments = 40;
	float pi = 3.14159f;
    
    for (int i = 0; i<Circle_Segments; i = i+1) {
        float angle1 = 2.0f*pi*i/Circle_Segments;
        float angle2 = 2.0f*pi*(i+1)/Circle_Segments;
        sf::Vector2f Line_Start = PF.Pos_Vector+sf::Vector2f(Radius*std::cos(angle1), Radius*std::sin(angle1));
        sf::Vector2f Line_End = PF.Pos_Vector+sf::Vector2f(Radius*std::cos(angle2), Radius*std::sin(angle2));
        lines.push_back(sf::Vertex(Line_Start, PF.Colour));
        lines.push_back(sf::Vertex(Line_End, PF.Colour));
    }

    // This is to represent the force direction
    float Force_Scale = 0.03f;
    sf::Vector2f Force_End = PF.Pos_Vector+PF.F_Vector*Force_Scale;
    lines.push_back(sf::Vertex(PF.Pos_Vector, sf::Color::Black));
    lines.push_back(sf::Vertex(Force_End, sf::Color::Black));

    // This is to represent the velocity direction
    float speed = std::sqrt(PF.V.x*PF.V.x+PF.V.y*PF.V.y);
    if (speed>0.1f) {
        sf::Vector2f velocity_end = PF.Pos_Vector+PF.V*20.0f;
        lines.push_back(sf::Vertex(PF.Pos_Vector, sf::Color::Magenta));
        lines.push_back(sf::Vertex(velocity_end, sf::Color::Magenta));
    }
}
// Create all the arrows and circles for visualization
void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();

    //This is to create our grid
    for (int x = 0; x<Screen_Width; x = x+20) {
        for (int y = 0; y < Screen_Height; y = y+20) {
            float fx = static_cast<float>(x);
            float fy = static_cast<float>(y);
            sf::Vector2f point(fx, fy);
            sf::Vector2f V;
            Velocity_From_Point_Force(point, V);

            float speed = std::sqrt(V.x* V.x+ V.y* V.y);
            
            // Makes the visualization nicer by removing very small arrows
            if (speed < 1e-10f) continue;

            //This is for drawing our arrows length based on speed
            float scale = 5000.0f;
            sf::Vector2f Dire = V/speed;
            float Arrow_Length = speed*scale;
            if (Arrow_Length <8.0f) Arrow_Length = 8.0f;
            if (Arrow_Length >30.0f) Arrow_Length = 30.0f;
            sf::Vector2f Arrow_Tip = point+Dire*Arrow_Length;

            lines.push_back(sf::Vertex(point, sf::Color::Green));
            lines.push_back(sf::Vertex(Arrow_Tip, sf::Color::Green));

            Arrow_Head(lines, Arrow_Tip, Dire, sf::Color::Green);
        }
    }

    // Drawing our point forces
    for (const auto& Point_Force : Point_Force_List) {
        Draw_Point_Force(lines, Point_Force);
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Stokes Flow");

    // Here is where we can add our point forces, first vector is the postion and the second is the force of it
    Point_Force(sf::Vector2f(50.0f, 200.0f), sf::Vector2f(200.0f, 0.0f), sf::Color::Blue);
    Point_Force(sf::Vector2f(50.0f, 375.0f), sf::Vector2f(200.0f, 0.0f), sf::Color::Red);
    Point_Force(sf::Vector2f(50.0f, 500.0f), sf::Vector2f(200.0f, 0.0f), sf::Color::Green);

    std::vector<sf::Vertex> allLines;

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
        }

        Update_Position();
        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}*/