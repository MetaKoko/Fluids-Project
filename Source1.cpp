/*#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>

constexpr double PI_VALUE = 3.14159265358979323846;

class TaylorSwimmingSimulation {
private:
    sf::RenderWindow window;
    std::vector<sf::Vector2f> filament;
    std::vector<sf::Vector2f> restPositions;

    // Parameters
    static constexpr size_t N = 50;
    static constexpr float L = 400.0f;
    static constexpr float Rb = 8.0f;
    static constexpr float eta = 0.1f;

    // Wave parameters
    static constexpr float k = 0.05f;
    static constexpr float b = 8.0f;
    static constexpr float omega = 1.0f;

    // Drag coefficients
    float xi_perp, xi_parallel, xi0;

    float time = 0.0f;
    float swimming_speed = 0.0f;
    float efficiency = 0.0f;

public:
    TaylorSwimmingSimulation() : window(sf::VideoMode({ 800u, 600u }), "Taylor Swimming Theory") {
        calculateDragCoefficients();
        initializeFilament();
        calculateSwimmingSpeed();
    }

    void calculateDragCoefficients() {
        // Using Gray and Hancock theory
        // xi_parallel = 2*pi*eta / [ln(2*wavelength/a) - 1/2], xi_perp = 2*xi_parallel

        constexpr float a = 1.0f;  // Segment radius
        const float wavelength = 2.0f * static_cast<float>(PI_VALUE) / k;

        // Gray-Hancock coefficients
        float log_term = std::log(2.0f * wavelength / a) - 0.5f;
        xi_parallel = 2.0f * static_cast<float>(PI_VALUE) * eta / log_term;
        xi_perp = 2.0f * xi_parallel;

        // Sphere drag: xi0 = 6*pi*eta*Rb
        xi0 = 6.0f * static_cast<float>(PI_VALUE) * eta * Rb;

        std::cout << "Drag coefficients - xi_perp: " << xi_perp
            << ", xi_parallel: " << xi_parallel
            << ", xi0: " << xi0 << std::endl;
        std::cout << "Ratio xi_perp/xi_parallel = " << xi_perp / xi_parallel << " (should be 2.0)" << std::endl;
    }

    void initializeFilament() {
        float spacing = L / static_cast<float>(N - 1);
        for (size_t i = 0; i < N; i++) {
            float x = 200.0f + static_cast<float>(i) * spacing;
            filament.push_back(sf::Vector2f(x, 300.0f));
            restPositions.push_back(sf::Vector2f(x, 300.0f));
        }
    }

    void run() {
        sf::Clock clock;
        while (window.isOpen()) {
            for (auto event = window.pollEvent(); event.has_value(); event = window.pollEvent()) {
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
        time += dt;
        applyTravelingWave();
        moveSwimmer(dt);
        calculateEfficiency();
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
        // Corrected equation from theory
        const float numerator = (xi_perp - xi_parallel) / (2.0f * xi_parallel) * omega * k * b * b;
        const float denominator = 1.0f + (xi0 * Rb) / (xi_parallel * L);

        swimming_speed = -numerator / denominator;

        // Scale for visualization
        swimming_speed *= 0.1f;

        std::cout << "Theoretical swimming speed U = " << swimming_speed << " pixels/sec" << std::endl;
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

    void calculateEfficiency() {
        const float spacing = L / static_cast<float>(N - 1);
        float integral_h_dot_squared = 0.0f;

        for (size_t i = 0; i < N - 1; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            const float h_dot = -omega * b * std::cos(k * x_local - omega * time);
            integral_h_dot_squared += h_dot * h_dot * spacing;
        }

        const float numerator = (xi_parallel * L + xi0 * Rb) * swimming_speed * swimming_speed;
        const float denominator = xi_perp * integral_h_dot_squared;

        if (denominator > 0.0f) {
            efficiency = numerator / denominator;

            static int frameCount = 0;
            if (frameCount++ % 300 == 0) {
               // std::cout << "Current efficiency e = " << efficiency << std::endl;
            }
        }
    }

    void render() {
        window.clear(sf::Color::Black);

        // Draw filament
        for (size_t i = 0; i < filament.size() - 1; i++) {
            sf::Vertex line[] = {
                sf::Vertex(filament[i], sf::Color::Green),
                sf::Vertex(filament[i + 1], sf::Color::Green)
            };
            window.draw(line, 2, sf::PrimitiveType::Lines);
        }

        // Draw points
        for (size_t i = 0; i < filament.size(); i++) {
            sf::CircleShape point(2.0f);
            point.setPosition(sf::Vector2f(filament[i].x - 2.0f, filament[i].y - 2.0f));
            point.setFillColor(sf::Color::Yellow);
            window.draw(point);
        }

        // Draw head
        sf::CircleShape head(Rb);
        head.setPosition(sf::Vector2f(filament[0].x - Rb, filament[0].y - Rb));
        head.setFillColor(sf::Color::Red);
        window.draw(head);

        window.display();
    }
};

int main() {
    TaylorSwimmingSimulation simulation;
    simulation.run();
    return 0;
}*/