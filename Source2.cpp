#include <SFML/Graphics.hpp>
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

public:
    Swimmer(sf::Vector2f startPos, sf::Color color,
        float waveNumber = 0.05f, float amplitude = 8.0f, float frequency = 1.0f)
        : swimmer_color(color), k(waveNumber), b(amplitude), omega(frequency) {

        calculateDragCoefficients();
        initializeFilament(startPos);
        calculateSwimmingSpeed();
    }

    void calculateDragCoefficients() {
        constexpr float a = 1.0f;
        const float wavelength = 2.0f * static_cast<float>(PI_VALUE) / k;

        float log_term = std::log(2.0f * wavelength / a) - 0.5f;
        xi_parallel = 2.0f * static_cast<float>(PI_VALUE) * eta / log_term;
        xi_perp = 2.0f * xi_parallel;
        xi0 = 6.0f * static_cast<float>(PI_VALUE) * eta;
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

    void calculateSwimmingSpeed() {
        const float numerator = (xi_perp - xi_parallel) / (2.0f * xi_parallel) * omega * k * b * b;
        const float denominator = 1.0f + (xi0 * Rb) / (xi_parallel * L);
        swimming_speed = -numerator / denominator;
    }

    void update(float dt) {
        time += dt;
        applyTravelingWave();
        moveSwimmer(dt);
    }

    void applyTravelingWave() {
        const float spacing = L / static_cast<float>(N - 1);
        for (size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            const float wave = b * std::sin(k * x_local - omega * time);
            filament[i].y = restPositions[i].y + wave;
        }
    }

    void moveSwimmer(float dt) {
        const float dx = swimming_speed * dt;
        for (size_t i = 0; i < N; i++) {
            filament[i].x += dx;
            restPositions[i].x += dx;
        }

        if (filament[0].x > 1000.0f) {
            resetSwimmerPosition();
        }
    }

    void resetSwimmerPosition() {
        float currentX = filament[0].x;
        float offset = 800.0f;
        for (size_t i = 0; i < N; i++) {
            filament[i].x -= currentX - offset;
            restPositions[i].x -= currentX - offset;
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
};

class TaylorSwimmingSimulation {
private:
    sf::RenderWindow window;
    std::vector<Swimmer> swimmers;

public:
    TaylorSwimmingSimulation() : window(sf::VideoMode({ 1200u, 800u }), "") {
        // Add swimmers with different parameters
        addSwimmer(sf::Vector2f(100.0f, 200.0f), sf::Color::Green, 0.05f, 8.0f, 1.0f);
        addSwimmer(sf::Vector2f(100.0f, 400.0f), sf::Color::Blue, 0.03f, 12.0f, 0.8f);
        addSwimmer(sf::Vector2f(100.0f, 600.0f), sf::Color::Magenta, 0.08f, 6.0f, 1.2f);

        printInitialInfo();
    }

    void addSwimmer(sf::Vector2f position, sf::Color color, float k, float b, float omega) {
        swimmers.emplace_back(position, color, k, b, omega);
    }

    void run() {
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
        std::cout << "\n=== Swimmer Speeds ===" << std::endl;
        for (size_t i = 0; i < swimmers.size(); i++) {
            const auto& swimmer = swimmers[i];
            std::cout << "Swimmer " << i + 1 << ": " << swimmer.getSpeed() << std::endl;
        }
        std::cout << "=====================" << std::endl;
    }
};

int main() {
    TaylorSwimmingSimulation simulation;
    simulation.run();
    return 0;
}