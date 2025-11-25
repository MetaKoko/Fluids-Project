/*#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>

constexpr double PI_VALUE = 3.14159265358979323846;

class Swimmer {
public:
    std::vector<sf::Vector2f> filament;
    std::vector<sf::Vector2f> restPositions;

    // Parameters
    static constexpr size_t N = 50;
    static constexpr float L = 400.0f;
    static constexpr float Rb = 8.0f;
    static constexpr float eta = 0.1f;

    // Wave parameters
    float k;
    float b;
    float omega;

    // Drag coefficients
    float xi_perp, xi_parallel, xi0;

    float time = 0.0f;
    float swimming_speed = 0.0f;
    sf::Color swimmer_color;

    // Net force (as number, not vector)
    float net_force;

    // Periodic boundary parameters
    static constexpr float BOX_WIDTH = 1200.0f;
    static constexpr float BOX_HEIGHT = 800.0f;

public:
    Swimmer(sf::Vector2f startPos, sf::Color color,
        float waveNumber = 0.05f, float amplitude = 8.0f, float frequency = 1.0f)
        : swimmer_color(color), k(waveNumber), b(amplitude), omega(frequency) {

        calculateDragCoefficients();
        initializeFilament(startPos);
        calculateSwimmingSpeed();
        calculateNetForce();
    }

    void calculateDragCoefficients() {
        constexpr float a = 1.0f;
        const float wavelength = 2.0f * static_cast<float>(PI_VALUE) / k;

        // Check for valid log argument to avoid domain errors
        float log_arg = 2.0f * wavelength / a;
        if (log_arg <= 0.0f) {
            log_arg = 1.0f; // Default safe value
        }

        float log_term = std::log(log_arg) - 0.5f;
        xi_parallel = 2.0f * static_cast<float>(PI_VALUE) * eta / log_term;
        xi_perp = 2.0f * xi_parallel;
        xi0 = 6.0f * static_cast<float>(PI_VALUE) * eta * Rb;
    }

    void initializeFilament(sf::Vector2f startPos) {
        filament.clear();
        restPositions.clear();

        float spacing = L / static_cast<float>(N - 1);
        for (size_t i = 0; i < N; i++) {
            float x = startPos.x + static_cast<float>(i) * spacing;
            float y = startPos.y;
            filament.push_back(sf::Vector2f(x, y));
            restPositions.push_back(sf::Vector2f(x, y));
        }
    }

    void calculateNetForce() {
        float thrust = (xi_perp - xi_parallel) / (2.0f * xi_parallel) * omega * k * b * b * xi_parallel;
        float drag = xi0 * swimming_speed;
        net_force = thrust - drag;
    }

    void update(float dt) {
        time += dt;
        applyTravelingWave();
        moveSwimmer(dt);
        applyPeriodicBoundary();
    }

    void applyTravelingWave() {
        const float spacing = L / static_cast<float>(N - 1);
        for (size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            const float wave = b * std::sin(k * x_local - omega * time);
            filament[i].y = restPositions[i].y + wave;
        }
    }

    void calculateSwimmingSpeed() {
        const float numerator = (xi_perp - xi_parallel) / (2.0f * xi_parallel) * omega * k * b * b;
        const float denominator = 1.0f + (xi0 * Rb) / (xi_parallel * L);
        swimming_speed = -numerator / denominator * 0.1f;
    }

    void moveSwimmer(float dt) {
        const float dx = swimming_speed * dt;
        for (size_t i = 0; i < N; i++) {
            filament[i].x += dx;
            restPositions[i].x += dx;
        }
    }

    void applyPeriodicBoundary() {
        // Check if head has crossed the right boundary
        if (filament[0].x > BOX_WIDTH) {
            // Removed unused 'offset' variable
            for (size_t i = 0; i < N; i++) {
                filament[i].x -= BOX_WIDTH;
                restPositions[i].x -= BOX_WIDTH;
            }
        }
        // Check if head has crossed the left boundary
        else if (filament[0].x < 0.0f) {
            // Removed unused 'offset' variable
            for (size_t i = 0; i < N; i++) {
                filament[i].x += BOX_WIDTH;
                restPositions[i].x += BOX_WIDTH;
            }
        }

        // Apply periodic boundary to each individual point as well
        for (size_t i = 0; i < N; i++) {
            if (filament[i].x > BOX_WIDTH) {
                filament[i].x -= BOX_WIDTH;
            }
            else if (filament[i].x < 0.0f) {
                filament[i].x += BOX_WIDTH;
            }
        }
    }

    void draw(sf::RenderWindow& window) {
        // Draw filament
        for (size_t i = 0; i < filament.size() - 1; i++) {
            std::array<sf::Vertex, 2> line = {
                sf::Vertex(filament[i], swimmer_color),
                sf::Vertex(filament[i + 1], swimmer_color)
            };
            window.draw(line.data(), line.size(), sf::PrimitiveType::LineStrip);
        }

        // Draw points
        sf::Color pointColor = swimmer_color;
        pointColor.r = static_cast<uint8_t>(std::min(255, static_cast<int>(swimmer_color.r) + 50));
        pointColor.g = static_cast<uint8_t>(std::min(255, static_cast<int>(swimmer_color.g) + 50));
        pointColor.b = static_cast<uint8_t>(std::min(255, static_cast<int>(swimmer_color.b) + 50));

        for (size_t i = 0; i < filament.size(); i++) {
            sf::CircleShape point(2.0f);
            point.setPosition(sf::Vector2f(filament[i].x - 2.0f, filament[i].y - 2.0f));
            point.setFillColor(pointColor);
            window.draw(point);
        }

        // Draw head
        sf::Color headColor = swimmer_color;
        headColor.r = static_cast<uint8_t>(std::max(0, static_cast<int>(swimmer_color.r) - 50));
        headColor.g = static_cast<uint8_t>(std::max(0, static_cast<int>(swimmer_color.g) - 50));
        headColor.b = static_cast<uint8_t>(std::max(0, static_cast<int>(swimmer_color.b) - 50));

        sf::CircleShape head(Rb);
        head.setPosition(sf::Vector2f(filament[0].x - Rb, filament[0].y - Rb));
        head.setFillColor(headColor);
        window.draw(head);
    }

    // Getters
    float getSpeed() const { return swimming_speed; }
    float getNetForce() const { return net_force; }
    sf::Vector2f getHeadPosition() const { return filament[0]; }
};

class TaylorSwimmingSimulation {
private:
    sf::RenderWindow window;
    std::vector<Swimmer> swimmers;

public:
    TaylorSwimmingSimulation() : window(sf::VideoMode({ 1200u, 800u }), "Taylor Swimming Simulation") {
        // Add swimmers with different parameters
        addSwimmer(sf::Vector2f(50.0f, 80.0f), sf::Color::Red, 0.2f, 2.0f, 4.0f);
        addSwimmer(sf::Vector2f(50.0f, 380.0f), sf::Color::Yellow, 0.005f, 30.0f, 0.1f);
        addSwimmer(sf::Vector2f(50.0f, 680.0f), sf::Color(255, 165, 0), 0.07f, 20.0f, 0.4f);

        std::cout << "Taylor Swimming Simulation Started" << std::endl;
        std::cout << "Number of swimmers: " << swimmers.size() << std::endl;
        std::cout << "Periodic Boundary: " << Swimmer::BOX_WIDTH << " x " << Swimmer::BOX_HEIGHT << std::endl;

        // Print initial values once
        printInitialInfo();
    }

    void addSwimmer(sf::Vector2f position, sf::Color color, float k, float b, float omega) {
        swimmers.emplace_back(position, color, k, b, omega);
    }

    void run() {
        sf::Clock clock;
        size_t frame = 0;

        while (window.isOpen()) {
            // SFML 3.0.2 event handling
            while (auto event = window.pollEvent()) {
                if (const auto* closeEvent = event->getIf<sf::Event::Closed>()) {
                    window.close();
                }
            }

            float dt = clock.restart().asSeconds();
            updatePhysics(dt);
            render();

            // Print periodic updates
            if (frame++ % 2000 == 0) {
                printSwimmerInfo();
            }
        }
    }

private:
    void updatePhysics(float dt) {
        for (auto& swimmer : swimmers) {
            swimmer.update(dt);
        }
    }

    void render() {
        window.clear(sf::Color::Black);

        for (auto& swimmer : swimmers) {
            swimmer.draw(window);
        }

        window.display();
    }

    void printInitialInfo() {
        std::cout << "\n=== Initial Swimmer Parameters ===" << std::endl;
        for (size_t i = 0; i < swimmers.size(); i++) {
            const auto& swimmer = swimmers[i];
            std::cout << "Swimmer " << i + 1 << ":" << std::endl;
            std::cout << "  Speed: " << swimmer.getSpeed() << std::endl;
            std::cout << "  Net Force: " << swimmer.getNetForce() << std::endl;
            std::cout << "  Head Position: (" << swimmer.getHeadPosition().x << ", " << swimmer.getHeadPosition().y << ")" << std::endl;
        }
        std::cout << "=================================" << std::endl;
    }

    void printSwimmerInfo() {
        std::cout << "\n=== Current Swimmer Status ===" << std::endl;
        for (size_t i = 0; i < swimmers.size(); i++) {
            const auto& swimmer = swimmers[i];
            std::cout << "Swimmer " << i + 1 << ": ";
            std::cout << "Position: " << swimmer.getHeadPosition().x << " | ";
            std::cout << "Speed: " << swimmer.getSpeed() << " | ";
            std::cout << "Net Force: " << swimmer.getNetForce() << std::endl;
        }
    }
};

int main() {
    TaylorSwimmingSimulation simulation;
    simulation.run();
    return 0;
}*/