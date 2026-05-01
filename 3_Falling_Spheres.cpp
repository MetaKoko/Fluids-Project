/*
#include <vector>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <iostream>
#include <SFML/Config.hpp>
#include <filesystem>
#include <chrono>

//This physical and screen coordinate setup up is just so that the visualization looks more like the paper figure.
// ------------------------------------------------------------
// Physical coordinates
// ------------------------------------------------------------
float Y_max = 0.0f;
float Y_min = -800.0f;
float X_min = -15.0f;
float X_max = 15.0f;

// ------------------------------------------------------------
// Screen coordinates
// ------------------------------------------------------------
float Screen_Width = 1340.0f;
float Screen_Height = 760.0f;

// ------------------------------------------------------------
// Mapping: Physical → Screen
// ------------------------------------------------------------
sf::Vector2f PhysToScreen(const sf::Vector2f& P)
{
    float X = P.x;
    float Y = P.y;

    float sx = (((X - X_min) / (X_max - X_min)) * Screen_Width);
    float sy = ((1.0f - ((Y - Y_min) / (Y_max - Y_min))) * Screen_Height)*1.5;

    return sf::Vector2f(sx, sy);
}

// ------------------------------------------------------------
// Point force data
// ------------------------------------------------------------
struct Point_Force_Data {
    sf::Vector2f Pos_Vector;     
    sf::Vector2f F_Vector;       
    sf::Color    Colour;
    sf::Vector2f V;
    std::vector<sf::Vector2f> Path;
};

std::vector<Point_Force_Data> Point_Force_List;

float Visc = 1.0f;
float Time_Step = 10.0f;

void Point_Force(const sf::Vector2f& Pos_Vector,
    const sf::Vector2f& F_Vector,
    sf::Color Colour)
{
    Point_Force_Data PF;
    PF.Pos_Vector = Pos_Vector;
    PF.F_Vector = F_Vector;
    PF.Colour = Colour;
    PF.V = sf::Vector2f(0.0f, 0.0f);
    PF.Path.push_back(Pos_Vector);
    Point_Force_List.push_back(PF);
}

void Velocity_From_Point_Force(const sf::Vector2f& Point, sf::Vector2f& V_Vector)
{
    V_Vector = sf::Vector2f(0.0f, 0.0f);

    for (const auto& PF : Point_Force_List)
    {
        sf::Vector2f Dist = Point - PF.Pos_Vector;
        float r2 = Dist.x * Dist.x + Dist.y * Dist.y;
        float r = std::sqrt(r2);
        if (r < 1e-6f) continue;

        float pi = 3.14159f;
        float C = 1.0f / (8.0f * pi * Visc);

        float dot = Dist.x * PF.F_Vector.x + Dist.y * PF.F_Vector.y;

        float inv_r = 1.0f / r;
        float inv_r3 = 1.0f / (r2 * r);
        float r4 = r2 * r2;
        float inv_r5 = 1.0f / (r4 * r);
        float r6 = r4 * r2;
        float inv_r7 = 1.0f / (r6 * r);

        float a2 = 1.0f;
        float a4 = a2 * a2;

        float S1 = inv_r;
        float S2 = inv_r3; 

        float F1a = (a2 / 3.0f) * (-2.0f * inv_r3); 
        float F2a = (a2 / 3.0f) * (6.0f * inv_r5);   

        float F1b = 0.0f;
        float F2b = (a4 / 36.0f) * (-60.0f * inv_r7);

        float coeff_F = S1 + F1a + F1b;
        float coeff_Fr = S2 + F2a + F2b;

        V_Vector.x += C * (coeff_F * PF.F_Vector.x + coeff_Fr * dot * Dist.x);
        V_Vector.y += C * (coeff_F * PF.F_Vector.y + coeff_Fr * dot * Dist.y);
    }
}

void Velocity_From_Point_Force_Excluding_Self(const sf::Vector2f& Point,
    size_t Ignore,
    sf::Vector2f& V_Vector)
{
    V_Vector = sf::Vector2f(0.0f, 0.0f);

    for (size_t i = 0; i < Point_Force_List.size(); i++)
    {
        if (i == Ignore) continue;

        const auto& PF = Point_Force_List[i];
        sf::Vector2f Dist = Point - PF.Pos_Vector;
        float r2 = Dist.x * Dist.x + Dist.y * Dist.y;
        float r = std::sqrt(r2);
        if (r < 1e-6f) continue;

        float pi = 3.14159f;
        float C = 1.0f / (8.0f * pi * Visc);

        float dot = Dist.x * PF.F_Vector.x + Dist.y * PF.F_Vector.y;

        float inv_r = 1.0f / r;
        float inv_r3 = 1.0f / (r2 * r);
        float r4 = r2 * r2;
        float inv_r5 = 1.0f / (r4 * r);
        float r6 = r4 * r2;
        float inv_r7 = 1.0f / (r6 * r);

        float a2 = 1.0f;
        float a4 = a2 * a2;

        float S1 = inv_r;
        float S2 = inv_r3;

        float F1a = (a2 / 3.0f) * (-2.0f * inv_r3);
        float F2a = (a2 / 3.0f) * (6.0f * inv_r5);

        float F1b = 0.0f;
        float F2b = (a4 / 36.0f) * (-60.0f * inv_r7);

        float coeff_F = S1 + F1a + F1b;
        float coeff_Fr = S2 + F2a + F2b;

        V_Vector.x += C * (coeff_F * PF.F_Vector.x + coeff_Fr * dot * Dist.x);
        V_Vector.y += C * (coeff_F * PF.F_Vector.y + coeff_Fr * dot * Dist.y);
    }
}

void Draw_Vector_Field(std::vector<sf::Vertex>& lines)
{
    float dx = 1.0f; 
    float dy = 10.0f; 

    for (float X = X_min; X <= X_max; X += dx)
    {
        for (float Y = Y_max; Y >= Y_min; Y -= dy)
        {
            sf::Vector2f P_phys(X, Y);
            sf::Vector2f V_phys;

            Velocity_From_Point_Force(P_phys, V_phys);

            float speed = std::sqrt(V_phys.x * V_phys.x + V_phys.y * V_phys.y);
            if (speed < 1e-10f) continue; //just in case

            sf::Vector2f dir = V_phys / speed;

            float L = std::clamp(speed * 50.0f, 0.2f, 3.0f);

            sf::Vector2f tip_phys = P_phys + dir * L;

            // convert to screen
            sf::Vector2f P_scr = PhysToScreen(P_phys);
            sf::Vector2f tip_scr = PhysToScreen(tip_phys);

            // draw main arrow line
            lines.push_back(sf::Vertex(P_scr, sf::Color::Green));
            lines.push_back(sf::Vertex(tip_scr, sf::Color::Green));

            // arrowhead
            sf::Vector2f shaft = tip_scr - P_scr;
            float shaft_len = std::sqrt(shaft.x * shaft.x + shaft.y * shaft.y);
            if (shaft_len < 1e-6f) continue; //just in case

            sf::Vector2f shaft_dir = shaft / shaft_len;

            float angle = std::atan2(shaft_dir.y, shaft_dir.x);
            float headSize = shaft_len * 0.5f;
            float headAngle = 0.6f;

            sf::Vector2f h1(
                tip_scr.x - headSize * std::cos(angle - headAngle),
                tip_scr.y - headSize * std::sin(angle - headAngle)
            );
            sf::Vector2f h2(
                tip_scr.x - headSize * std::cos(angle + headAngle),
                tip_scr.y - headSize * std::sin(angle + headAngle)
            );

            lines.push_back(sf::Vertex(tip_scr, sf::Color::Green));
            lines.push_back(sf::Vertex(h1, sf::Color::Green));
            lines.push_back(sf::Vertex(tip_scr, sf::Color::Green));
            lines.push_back(sf::Vertex(h2, sf::Color::Green));
        }
    }
}


void Update_Position()
{
    // compute velocities
    for (size_t i = 0; i < Point_Force_List.size(); i++)
    {
        sf::Vector2f vel;
        Velocity_From_Point_Force_Excluding_Self(Point_Force_List[i].Pos_Vector, i, vel);

         float pi = 3.14159f;
         float a  = 1.0f; //This is the radius
         sf::Vector2f self_drag =
             Point_Force_List[i].F_Vector * (1.0f / (6.0f * pi * Visc * a));
         Point_Force_List[i].V = vel + self_drag;

    }

    for (auto& PF : Point_Force_List)
    {
        PF.Pos_Vector += PF.V * Time_Step;
        PF.Path.push_back(PF.Pos_Vector);
    }
}

// ------------------------------------------------------------
// Draw a point force (convert to screen coordinates)
// ------------------------------------------------------------
void Draw_Point_Force(std::vector<sf::Vertex>& lines, const Point_Force_Data& PF)
{
    for (size_t i = 1; i < PF.Path.size(); i++)
    {
        sf::Vector2f p1 = PhysToScreen(PF.Path[i - 1]);
        sf::Vector2f p2 = PhysToScreen(PF.Path[i]);
        lines.push_back(sf::Vertex(p1, PF.Colour));
        lines.push_back(sf::Vertex(p2, PF.Colour));
    }

    float R = 5.0f;
    int   seg = 40;
    float pi = 3.14159f;

    for (int i = 0; i < seg; i++)
    {
        float a1 = 2.0f * pi * i / seg;
        float a2 = 2.0f * pi * (i + 1) / seg;

        sf::Vector2f c1 = PhysToScreen(PF.Pos_Vector) +
            sf::Vector2f(R * std::cos(a1), R * std::sin(a1));
        sf::Vector2f c2 = PhysToScreen(PF.Pos_Vector) +
            sf::Vector2f(R * std::cos(a2), R * std::sin(a2));

        lines.push_back(sf::Vertex(c1, PF.Colour));
        lines.push_back(sf::Vertex(c2, PF.Colour));
    }
}


void Draw_Everything(std::vector<sf::Vertex>& lines)
{
    lines.clear();

    Draw_Vector_Field(lines);

    for (const auto& PF : Point_Force_List)
        Draw_Point_Force(lines, PF);
}

void SaveFrame(sf::RenderWindow& window, int frameNumber) //This was for making the videos
{
    std::filesystem::create_directories("frames");

    sf::Texture texture;
    bool ok = texture.resize(window.getSize());
    if (!ok)
    {
        std::cout << "Failed to resize texture\n";
        return;
    }

    texture.update(window);
    sf::Image screenshot = texture.copyToImage();

    std::string filename = "frames/frame_" + std::to_string(frameNumber) + ".png";

    std::cout << "Saving to absolute path: "
        << std::filesystem::absolute(filename) << "\n";

    if (!screenshot.saveToFile(filename))
        std::cout << "Failed to save: " << filename << "\n";
    else
        std::cout << "Saved: " << filename << "\n";
}


int main()
{
    std::cout << std::filesystem::current_path().string() << "\n";

    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Stokes Flow with Faxen (Sphere–Sphere)");

    float y0 = 0.0f;

    //Adding spheres here
    Point_Force(sf::Vector2f(-5.0f, y0), sf::Vector2f(0.0f, -1.0f), sf::Color::Blue);
    Point_Force(sf::Vector2f(0.0f, y0), sf::Vector2f(0.0f, -1.0f), sf::Color::Red);
    Point_Force(sf::Vector2f(7.0f, y0), sf::Vector2f(0.0f, -1.0f), sf::Color::Black);

    std::vector<sf::Vertex> allLines;

    // Real-time timer for video length
    auto  start = std::chrono::high_resolution_clock::now();
    float maxSeconds = 30.0f;
    int frame = 0;
    int saveEvery = 10;

    while (window.isOpen())
    {
        while (auto event = window.pollEvent())
            if (event->is<sf::Event::Closed>())
                window.close();

        auto  now = std::chrono::high_resolution_clock::now();
        float elapsed = std::chrono::duration<float>(now - start).count();

        if (elapsed >= maxSeconds)
            window.close();

        Update_Position();
        Draw_Everything(allLines);

        window.clear(sf::Color::White);
        window.draw(allLines.data(), allLines.size(), sf::PrimitiveType::Lines);


        if (frame % saveEvery == 0)
            //SaveFrame(window, frame / saveEvery);
        frame++;

        window.display();
    }


    return 0;
}
*/