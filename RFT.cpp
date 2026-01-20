#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>

constexpr double PI_VALUE = 3.14159265358979323846;
constexpr float POINTS_PER_UNIT = 0.1f;
constexpr float SWIMMER_HEAD_RADIUS = 8.0f;
constexpr float FLUID_VISCOSITY = 0.1f;

//Swimmer data
struct SwimmerData {
    std::vector<sf::Vector2f> filament;
    std::vector<sf::Vector2f> restPositions;
    float waveNumber = 0.0f;
    float amplitude = 0.0f;
    float frequency = 0.0f;
    float length = 0.0f;
    size_t numPoints = 0;
    float xi_perp = 0.0f, xi_parallel = 0.0f, xi0 = 0.0f;
    float time = 0.0f;
    float swimming_speed = 0.0f;
    sf::Color color;
};

std::vector<SwimmerData> swimmers;
sf::RenderWindow window;

// Calculates number of points based on length
size_t calculateNumPoints(float length) {
    return static_cast<size_t>(length * POINTS_PER_UNIT);
}

// Calculates drag coefficients
void calculateDragCoefficients(SwimmerData& swimmer) {
    constexpr float filament_radius = 1.0f;
    const float wavelength = 2.0f * static_cast<float>(PI_VALUE) / swimmer.waveNumber;

    float log_term = std::log(2.0f * wavelength / filament_radius) - 0.5f;
    swimmer.xi_parallel = 2.0f * static_cast<float>(PI_VALUE) * FLUID_VISCOSITY / log_term;
    swimmer.xi_perp = 2.0f * swimmer.xi_parallel;
    swimmer.xi0 = 6.0f * static_cast<float>(PI_VALUE) * FLUID_VISCOSITY;
}

// This is to place the filament
void PlaceFilament(SwimmerData& swimmer, sf::Vector2f startPos) {
    swimmer.filament.clear();
    swimmer.restPositions.clear();

    swimmer.numPoints = calculateNumPoints(swimmer.length);
    float spacing = swimmer.length / static_cast<float>(swimmer.numPoints - 1);

    for (size_t i = 0; i < swimmer.numPoints; i++) {
        float x = startPos.x + static_cast<float>(i) * spacing;
        float y = startPos.y;
        swimmer.filament.push_back(sf::Vector2f(x, y));
        swimmer.restPositions.push_back(sf::Vector2f(x, y));
    }
}

// Calculate the averaged swimming speed
void calculateSwimmingSpeed(SwimmerData& swimmer) {
    const float numerator = (swimmer.xi_perp - swimmer.xi_parallel) / (2.0f * swimmer.xi_parallel) *
        swimmer.frequency * swimmer.waveNumber * swimmer.amplitude * swimmer.amplitude;
    const float denominator = 1.0f + (swimmer.xi0 * SWIMMER_HEAD_RADIUS) / (swimmer.xi_parallel * swimmer.length);
    swimmer.swimming_speed = -numerator / denominator;
}

// For account a new swimmer
void addSwimmer(sf::Vector2f position, sf::Color color, float k, float b, float omega, float length) {
    SwimmerData newSwimmer;
    newSwimmer.waveNumber = k;
    newSwimmer.amplitude = b;
    newSwimmer.frequency = omega;
    newSwimmer.length = length;
    newSwimmer.time = 0.0f;
    newSwimmer.color = color;

    calculateDragCoefficients(newSwimmer);
    PlaceFilament(newSwimmer, position);
    calculateSwimmingSpeed(newSwimmer);

    swimmers.push_back(newSwimmer);
}

// This places the swimmer back if it leaves the screen (not seen this happen yet)
void resetSwimmerPosition(SwimmerData& swimmer) {
    float currentX = swimmer.filament[0].x;
    float offset = 800.0f;
    for (size_t i = 0; i < swimmer.numPoints; i++) {
        swimmer.filament[i].x -= currentX - offset;
        swimmer.restPositions[i].x -= currentX - offset;
    }
}

// This is for inducing the wave onto the filament
void applyTravelingWave(SwimmerData& swimmer) {
    const float spacing = swimmer.length / static_cast<float>(swimmer.numPoints - 1);
    for (size_t i = 0; i < swimmer.numPoints; i++) {
        const float x_local = static_cast<float>(i) * spacing;
        const float wave = swimmer.amplitude * std::sin(swimmer.waveNumber * x_local - swimmer.frequency * swimmer.time);
        swimmer.filament[i].y = swimmer.restPositions[i].y + wave;
    }
}

// This moves the swimmer in the x direction based on its speed
void moveSwimmer(SwimmerData& swimmer, float dt) {
    const float dx = swimmer.swimming_speed * dt;
    for (size_t i = 0; i < swimmer.numPoints; i++) {
        swimmer.filament[i].x += dx;
        swimmer.restPositions[i].x += dx;
    }

    if (swimmer.filament[0].x > 1000.0f) {
        resetSwimmerPosition(swimmer);
    }
}

// This itterates the swimmer
void updateSwimmer(SwimmerData& swimmer, float dt) {
    swimmer.time += dt;
    applyTravelingWave(swimmer);
    moveSwimmer(swimmer, dt);
}

// For Drawing
void drawSwimmer(const SwimmerData& swimmer) {
    // Draw filament
    for (size_t i = 0; i < swimmer.filament.size() - 1; i++) {
        std::array<sf::Vertex, 2> line = {
            sf::Vertex(swimmer.filament[i], swimmer.color),
            sf::Vertex(swimmer.filament[i + 1], swimmer.color)
        };
        window.draw(line.data(), line.size(), sf::PrimitiveType::LineStrip);
    }

    // Draw points
    sf::Color pointColor = swimmer.color;
    pointColor.r = static_cast<uint8_t>(std::min(255, static_cast<int>(swimmer.color.r) + 50));
    pointColor.g = static_cast<uint8_t>(std::min(255, static_cast<int>(swimmer.color.g) + 50));
    pointColor.b = static_cast<uint8_t>(std::min(255, static_cast<int>(swimmer.color.b) + 50));

    for (size_t i = 0; i < swimmer.filament.size(); i++) {
        sf::CircleShape point(2.0f);
        point.setPosition(sf::Vector2f(swimmer.filament[i].x - 2.0f, swimmer.filament[i].y - 2.0f));
        point.setFillColor(pointColor);
        window.draw(point);
    }

    // Draw head
    sf::Color headColor = swimmer.color;
    headColor.r = static_cast<uint8_t>(std::max(0, static_cast<int>(swimmer.color.r) - 50));
    headColor.g = static_cast<uint8_t>(std::max(0, static_cast<int>(swimmer.color.g) - 50));
    headColor.b = static_cast<uint8_t>(std::max(0, static_cast<int>(swimmer.color.b) - 50));

    sf::CircleShape head(SWIMMER_HEAD_RADIUS);
    head.setPosition(sf::Vector2f(swimmer.filament[0].x - SWIMMER_HEAD_RADIUS,
        swimmer.filament[0].y - SWIMMER_HEAD_RADIUS));
    head.setFillColor(headColor);
    window.draw(head);
}

// Update physics for all swimmers
void updatePhysics(float dt) {
    for (auto& swimmer : swimmers) {
        updateSwimmer(swimmer, dt);
    }
}

// Render all swimmers
void render() {
    window.clear(sf::Color::Black);

    for (const auto& swimmer : swimmers) {
        drawSwimmer(swimmer);
    }

    window.display();
}

// To show use swimmer info when simulating
void printInitialInfo() {
    std::cout << "\n=== Swimmer Information ===" << std::endl;
    for (size_t i = 0; i < swimmers.size(); i++) {
        const auto& swimmer = swimmers[i];
        std::cout << "Swimmer " << i + 1 << ":" << std::endl;
        std::cout << "  Speed: " << swimmer.swimming_speed << std::endl;
        std::cout << std::endl;
    }
    std::cout << "==========================" << std::endl;
}


int main() {
    window.create(sf::VideoMode({ 1200u, 800u }), "Taylor Swimming Simulation - Variable Length & Points");

    // Add swimmers here
    addSwimmer(sf::Vector2f(100.0f, 200.0f), sf::Color::Green, 0.05f, 8.0f, 1.0f, 200.0f);
    addSwimmer(sf::Vector2f(100.0f, 400.0f), sf::Color::Blue, 0.05f, 8.0f, 1.0f, 400.0f);
    addSwimmer(sf::Vector2f(100.0f, 600.0f), sf::Color::Magenta, 0.05f, 8.0f, 1.0f, 800.0f);

    printInitialInfo();

    sf::Clock clock;

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (const auto* closeEvent = event->getIf<sf::Event::Closed>()) {
                window.close();
            }
        }

        float dt = clock.restart().asSeconds();
        updatePhysics(dt);
        render();
    }

    return 0;
}