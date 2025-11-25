/*#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>

struct PointForce {
    sf::Vector2f position;
    sf::Vector2f force;
    sf::Color color;
};

class StokesFlow {
private:
    std::vector<PointForce> forces_info;
    float viscosity_;

public:
    StokesFlow(float viscosity = 1.0f) : viscosity_(viscosity) {}

    void addForce(const sf::Vector2f& position, const sf::Vector2f& force, sf::Color color = sf::Color::Red) {
        forces_info.push_back({ position, force, color });
    }

    // Calculate velocity at point x due to ALL point forces
    sf::Vector2f calculateVelocity(const sf::Vector2f& x) {
        sf::Vector2f velocity(0.0f, 0.0f);

        for (const auto& force : forces_info) {
            sf::Vector2f r = x - force.position;
            float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);

            // Avoid division by zero at force locations
            if (r_magnitude < 0.00000001f) {
                continue;
            }

            // Stokeslet formula: u = G(r) · F
            const float pi = 3.14159265358979323846f;
            float prefactor = 1.0f / (8.0f * pi * viscosity_);
            float r_inv = 1.0f / r_magnitude;
            float r3_inv = 1.0f / (r_magnitude * r_magnitude * r_magnitude);

            float r_dot_f = r.x * force.force.x + r.y * force.force.y;

            velocity.x += prefactor * (force.force.x * r_inv + r_dot_f * r.x * r3_inv);
            velocity.y += prefactor * (force.force.y * r_inv + r_dot_f * r.y * r3_inv);
        }

        return velocity;
    }

    void generateFlowField(std::vector<sf::Vertex>& arrows) {
        arrows.clear();

        // Create grid of observation points
        for (int x = 0; x < 1400; x += 20) {
            for (int y = 0; y < 700; y += 20) {
                float fx = static_cast<float>(x);
                float fy = static_cast<float>(y);
                sf::Vector2f point(fx, fy);
                sf::Vector2f velocity = calculateVelocity(point);

                float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

                // Skip very small velocities
                if (speed < 0.0000000001f) continue;

                // Arrow length proportional to speed
                float scale = 5000.0f;
                sf::Vector2f direction = velocity / speed;

                // Arrow length based on speed with limits
                float arrow_length = speed * scale;
                if (arrow_length < 8.0f) arrow_length = 8.0f;
                if (arrow_length > 30.0f) arrow_length = 30.0f;

                sf::Vector2f arrow_end = point + direction * arrow_length;

                sf::Color color = sf::Color::Green;

                // Draw arrow line
                arrows.push_back(sf::Vertex(point, color));
                arrows.push_back(sf::Vertex(arrow_end, color));

                // Draw arrow head
                drawArrowHead(arrows, arrow_end, direction, color);
            }
        }

        // Draw force points
        for (const auto& force : forces_info) {
            drawForcePoint(arrows, force);
        }
    }

private:

    void drawArrowHead(std::vector<sf::Vertex>& arrows, const sf::Vector2f& tip,
        const sf::Vector2f& direction, sf::Color color) {
        const float head_length = 12.0f;
        const float head_angle = 0.7f;

        float angle = std::atan2(direction.y, direction.x);

        sf::Vector2f head1 = tip + sf::Vector2f(
            -head_length * std::cos(angle - head_angle),
            -head_length * std::sin(angle - head_angle));

        sf::Vector2f head2 = tip + sf::Vector2f(
            -head_length * std::cos(angle + head_angle),
            -head_length * std::sin(angle + head_angle));

        arrows.push_back(sf::Vertex(tip, color));
        arrows.push_back(sf::Vertex(head1, color));
        arrows.push_back(sf::Vertex(tip, color));
        arrows.push_back(sf::Vertex(head2, color));
    }

    void drawForcePoint(std::vector<sf::Vertex>& arrows, const PointForce& force) {
        // Draw a circle around the force point
        const float radius = 10.0f;
        const int segments = 32;
        const float pi = 3.14159265358979323846f;

        for (int i = 0; i < segments; i++) {
            float angle1 = 2.0f * pi * static_cast<float>(i) / static_cast<float>(segments);
            float angle2 = 2.0f * pi * static_cast<float>(i + 1) / static_cast<float>(segments);

            sf::Vector2f point1 = force.position + sf::Vector2f(
                radius * std::cos(angle1),
                radius * std::sin(angle1)
            );

            sf::Vector2f point2 = force.position + sf::Vector2f(
                radius * std::cos(angle2),
                radius * std::sin(angle2)
            );

            arrows.push_back(sf::Vertex(point1, force.color));
            arrows.push_back(sf::Vertex(point2, force.color));
        }

        // Draw force direction indicator
        sf::Vector2f force_end = force.position + force.force * 0.05f;
        arrows.push_back(sf::Vertex(force.position, force.color));
        arrows.push_back(sf::Vertex(force_end, force.color));
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Stokes Flow - Length Indicates Speed");

    StokesFlow flow(70.0f);

    // Add point forces
    flow.addForce(sf::Vector2f(200.0f, 200.0f), sf::Vector2f(800.0f, 0.0f), sf::Color::Blue);
    flow.addForce(sf::Vector2f(600.0f, 400.0f), sf::Vector2f(-500.0f, -500.0f), sf::Color::Red);
    flow.addForce(sf::Vector2f(1000.0f, 300.0f), sf::Vector2f(-400.0f, 500.0f), sf::Color::Magenta);

    std::vector<sf::Vertex> arrows;
    flow.generateFlowField(arrows);

    while (window.isOpen()) {
        if (auto event = window.waitEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            if (event->is<sf::Event::KeyPressed>()) {
                flow.generateFlowField(arrows);
            }
        }

        window.clear(sf::Color::White);
        window.draw(arrows.data(), arrows.size(), sf::PrimitiveType::Lines);
        window.display();
    }

    return 0;
}*/