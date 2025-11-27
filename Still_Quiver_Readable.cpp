#include<vector>
#include<cmath>
#include<SFML/Graphics.hpp>

// Point force data
struct PF_Data{
    sf::Vector2f Pos_Vector;
    sf::Vector2f F_Vector;
    sf::Color Colour;
};
std::vector<PF_Data> PF_List;
float Visc = 70.0f;

void Point_Force(sf::Vector2f Pos_Vector, sf::Vector2f F_Vector, sf::Color Colour){
    PF_List.push_back({Pos_Vector, F_Vector, Colour});
}
// This is for calculating velocity at each point in our fluid
sf::Vector2f Fluid_Velocity(sf::Vector2f Point){
    sf::Vector2f V_atPoint(0, 0);
    // This is the stokes flow formula
    for(const auto& PF:PF_List){
        float pi = 3.14159f;
        sf::Vector2f Dist = Point-PF.Pos_Vector;
        float r = std::sqrt(Dist.x*Dist.x+Dist.y*Dist.y);
        float Constant = 1.0f/(8*pi*Visc);
        float Dot_Prod = Dist.x*PF.F_Vector.x+Dist.y*PF.F_Vector.y;
        V_atPoint.x = V_atPoint.x+Constant*(PF.F_Vector.x*1.0f/r+Dot_Prod*Dist.x*1.0f/(r*r*r));
        V_atPoint.y = V_atPoint.y+Constant*(PF.F_Vector.y*1.0f/r+Dot_Prod*Dist.y*1.0f/(r*r*r));
    }
    return V_atPoint;
}

void Arrow_Head(std::vector<sf::Vertex>& lines, sf::Vector2f tip,
    sf::Vector2f Dire, sf::Color colour){
    float headSize = 10.0f;
    float headAngle = 0.6f;

    float angle = std::atan2(Dire.y, Dire.x);

    // This is for drawing our arrrowheads
    sf::Vector2f head1 = tip+sf::Vector2f(
        -headSize * std::cos(angle-headAngle),
        -headSize * std::sin(angle-headAngle));

    sf::Vector2f head2 = tip+sf::Vector2f(
        -headSize*std::cos(angle+headAngle),
        -headSize*std::sin(angle+headAngle));

    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head1, colour));
    lines.push_back(sf::Vertex(tip, colour));
    lines.push_back(sf::Vertex(head2, colour));
}

void Draw_Point_Force(std::vector<sf::Vertex>& lines, PF_Data PF) {
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
    sf::Vector2f Line_End = PF.Pos_Vector+ PF.F_Vector*0.05f;
    lines.push_back(sf::Vertex(PF.Pos_Vector, PF.Colour));
    lines.push_back(sf::Vertex(Line_End, PF.Colour));
}

// Create all the arrows and circles for visualization
void Draw_Everything(std::vector<sf::Vertex>& lines) {
    lines.clear();

    //This is to create our grid
    for (int x = 0; x<1400; x = x+20) {
        for (int y = 0; y<700; y = y+20) {
            sf::Vector2f point((float)x, (float)y);
            sf::Vector2f V = Fluid_Velocity(point);

            float speed = std::sqrt(V.x*V.x+V.y*V.y);

            // Makes the visualization nicer by removing very small arrows
            if (speed<1e-10f) continue;

            //This is for drawing our arrows length based on speed
            float scale = 5000.0f;
            sf::Vector2f Dire = V/speed;
            float Arrow_Length = speed*scale;
            if (Arrow_Length<8) Arrow_Length = 8;
            if (Arrow_Length>30) Arrow_Length = 30;
            sf::Vector2f Arrow_Tip = point+Dire*Arrow_Length;

            lines.push_back(sf::Vertex(point, sf::Color::Green));
            lines.push_back(sf::Vertex(Arrow_Tip, sf::Color::Green));

            Arrow_Head(lines, Arrow_Tip, Dire, sf::Color::Green);
        }
    }

    // Drawing our point forces
    for (const auto& PF:PF_List) {
        Draw_Point_Force(lines, PF);
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Fluid Flow Visualization");

    // Here is where we can add our point forces, first vector is the postion and the second is the force of it
    Point_Force(sf::Vector2f(300, 200), sf::Vector2f(700, 0), sf::Color::Blue);
    Point_Force(sf::Vector2f(600, 500), sf::Vector2f(-200, -500), sf::Color::Red);
    Point_Force(sf::Vector2f(1000, 400), sf::Vector2f(-300, 800), sf::Color::Magenta);

    std::vector<sf::Vertex> allLines;
    Draw_Everything(allLines);

    while (window.isOpen()) {
        if (auto event = window.waitEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();

            // Press any key to redraw (useful if we change things)
            if (event->is<sf::Event::KeyPressed>()) {
                Draw_Everything(allLines);
            }
        }

        window.clear(sf::Color::White);
        window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}