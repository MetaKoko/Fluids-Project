/*#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

struct PointForce {
    sf::Vector2f position;
    sf::Vector2f force;
    sf::Color color;
    sf::Vector2f velocity;
};

class StokesFlow {
private:
    std::vector<PointForce> forces_info;
    float viscosity_;
    float time_step_;
    float screen_width_;
    float screen_height_;

public:
    StokesFlow(float viscosity = 1.0f, float time_step = 0.1f, float screen_width = 1400.0f, float screen_height = 700.0f)
        : viscosity_(viscosity), time_step_(time_step), screen_width_(screen_width), screen_height_(screen_height) {
    }

    void addForce(const sf::Vector2f& position, const sf::Vector2f& force, sf::Color color = sf::Color::Red) {
        forces_info.push_back({ position, force, color, sf::Vector2f(0.0f, 0.0f) });
    }

    void calculateVelocity(const sf::Vector2f& x, sf::Vector2f& velocity, float& vorticity) const {
        velocity = sf::Vector2f(0.0f, 0.0f);
        vorticity = 0.0f;

        // Consider 3x3 grid of periodic images (center box + 8 surrounding boxes)
        for (const auto& force : forces_info) {
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    // Create image position in surrounding box
                    sf::Vector2f image_position = force.position;
                    image_position.x += dx * screen_width_;
                    image_position.y += dy * screen_height_;

                    sf::Vector2f r = x - image_position;
                    float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);

                    if (r_magnitude < 0.0001f) continue;

                    const float pi = 3.14159265358979323846f;
                    float r_inv = 1.0f / r_magnitude;
                    float r2 = r_magnitude * r_magnitude;
                    float r3_inv = 1.0f / (r_magnitude * r2);

                    float prefactor = 1.0f / (8.0f * pi * viscosity_);
                    float r_dot_f = r.x * force.force.x + r.y * force.force.y;

                    velocity.x += prefactor * (force.force.x * r_inv + r_dot_f * r.x * r3_inv);
                    velocity.y += prefactor * (force.force.y * r_inv + r_dot_f * r.y * r3_inv);

                    float stokeslet_vorticity = (r.x * force.force.y - r.y * force.force.x) / (8.0f * pi * viscosity_) * r3_inv;
                    vorticity += stokeslet_vorticity;
                }
            }
        }
    }

    void calculateVelocityExcludingSelf(const sf::Vector2f& x, size_t exclude_index, sf::Vector2f& velocity, float& vorticity) const {
        velocity = sf::Vector2f(0.0f, 0.0f);
        vorticity = 0.0f;

        for (size_t i = 0; i < forces_info.size(); ++i) {
            const auto& force = forces_info[i];

            // Consider 3x3 grid of periodic images for each force
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    // For self-interaction (i == exclude_index), exclude only the exact same position
                    // but include all other periodic images
                    if (i == exclude_index && dx == 0 && dy == 0) {
                        continue; // Skip the exact self-position
                    }

                    sf::Vector2f image_position = force.position;
                    image_position.x += dx * screen_width_;
                    image_position.y += dy * screen_height_;

                    sf::Vector2f r = x - image_position;
                    float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);

                    if (r_magnitude < 0.0001f) continue;

                    const float pi = 3.14159265358979323846f;
                    float r_inv = 1.0f / r_magnitude;
                    float r2 = r_magnitude * r_magnitude;
                    float r3_inv = 1.0f / (r_magnitude * r2);

                    float prefactor = 1.0f / (8.0f * pi * viscosity_);
                    float r_dot_f = r.x * force.force.x + r.y * force.force.y;

                    velocity.x += prefactor * (force.force.x * r_inv + r_dot_f * r.x * r3_inv);
                    velocity.y += prefactor * (force.force.y * r_inv + r_dot_f * r.y * r3_inv);

                    float stokeslet_vorticity = (r.x * force.force.y - r.y * force.force.x) / (8.0f * pi * viscosity_) * r3_inv;
                    vorticity += stokeslet_vorticity;
                }
            }
        }
    }

    void updateForcePositions() {
        for (size_t i = 0; i < forces_info.size(); ++i) {
            auto& force_i = forces_info[i];

            sf::Vector2f velocity;
            float vorticity;
            calculateVelocityExcludingSelf(force_i.position, i, velocity, vorticity);

            force_i.velocity = velocity;
        }

        for (size_t i = 0; i < forces_info.size(); ++i) {
            auto& force = forces_info[i];
            force.position += force.velocity * time_step_;

            // Apply periodic boundary conditions - wrap around to opposite side
            if (force.position.x < 0.0f) {
                force.position.x += screen_width_;
            }
            else if (force.position.x >= screen_width_) {
                force.position.x -= screen_width_;
            }

            if (force.position.y < 0.0f) {
                force.position.y += screen_height_;
            }
            else if (force.position.y >= screen_height_) {
                force.position.y -= screen_height_;
            }
        }
    }

    void generateFlowField(std::vector<sf::Vertex>& arrows) const {
        arrows.clear();

        // Visualize flow field in the central box only
        for (int x = 0; x < screen_width_; x += 20) {
            for (int y = 0; y < screen_height_; y += 20) {
                float fx = static_cast<float>(x);
                float fy = static_cast<float>(y);
                sf::Vector2f point(fx, fy);
                sf::Vector2f velocity;
                float vorticity;
                calculateVelocity(point, velocity, vorticity);

                float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
                if (speed < 0.0001f) continue;

                float scale = 5000.0f;
                sf::Vector2f direction = velocity / speed;
                float arrow_length = std::clamp(speed * scale, 8.0f, 30.0f);
                sf::Vector2f arrow_end = point + direction * arrow_length;

                sf::Color color;
                if (vorticity > 0.1f) color = sf::Color::Red;
                else if (vorticity < -0.1f) color = sf::Color::Blue;
                else color = sf::Color::Green;

                arrows.push_back(sf::Vertex(point, color));
                arrows.push_back(sf::Vertex(arrow_end, color));
                drawArrowHead(arrows, arrow_end, direction, color);
            }
        }

        // Draw only the forces in the central box
        for (const auto& force : forces_info) {
            drawForcePoint(arrows, force);
        }
    }

    const std::vector<PointForce>& getForces() const { return forces_info; }
    void setTimeStep(float time_step) { time_step_ = time_step; }

private:
    void drawArrowHead(std::vector<sf::Vertex>& arrows, const sf::Vector2f& tip,
        const sf::Vector2f& direction, sf::Color color) const {
        const float head_length = 12.0f;
        const float head_angle = 0.7f;
        float angle = std::atan2(direction.y, direction.x);

        sf::Vector2f head1 = tip + sf::Vector2f(
            -head_length * std::cos(angle - head_angle), -head_length * std::sin(angle - head_angle));
        sf::Vector2f head2 = tip + sf::Vector2f(
            -head_length * std::cos(angle + head_angle), -head_length * std::sin(angle + head_angle));

        arrows.push_back(sf::Vertex(tip, color));
        arrows.push_back(sf::Vertex(head1, color));
        arrows.push_back(sf::Vertex(tip, color));
        arrows.push_back(sf::Vertex(head2, color));
    }

    void drawForcePoint(std::vector<sf::Vertex>& arrows, const PointForce& force) const {
        const float radius = 12.0f;
        const int segments = 16;

        for (int i = 0; i < segments; i++) {
            float angle1 = 2.0f * 3.14159f * i / segments;
            float angle2 = 2.0f * 3.14159f * (i + 1) / segments;
            sf::Vector2f point1 = force.position + sf::Vector2f(radius * std::cos(angle1), radius * std::sin(angle1));
            sf::Vector2f point2 = force.position + sf::Vector2f(radius * std::cos(angle2), radius * std::sin(angle2));
            arrows.push_back(sf::Vertex(point1, force.color));
            arrows.push_back(sf::Vertex(point2, force.color));
        }

        float force_scale = 0.03f;
        sf::Vector2f force_end = force.position + force.force * force_scale;
        arrows.push_back(sf::Vertex(force.position, sf::Color::Black));
        arrows.push_back(sf::Vertex(force_end, sf::Color::Black));

        float speed = std::sqrt(force.velocity.x * force.velocity.x + force.velocity.y * force.velocity.y);
        if (speed > 0.1f) {
            sf::Vector2f velocity_end = force.position + force.velocity * 20.0f;
            arrows.push_back(sf::Vertex(force.position, sf::Color::Magenta));
            arrows.push_back(sf::Vertex(velocity_end, sf::Color::Magenta));
        }
    }
};

int main() {
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(desktop, "Stokes Flow - 3x3 Periodic Images");
    window.setFramerateLimit(60);

    StokesFlow flow(70.0f, 1000.0f, 1340.0f, 700.0f);

    // Test with a single point force - it should now move!
    flow.addForce(sf::Vector2f(200.0f, 200.0f), sf::Vector2f(1000.0f, 0.0f), sf::Color::Blue);

    // You can also test with multiple forces:
    flow.addForce(sf::Vector2f(600.0f, 400.0f), sf::Vector2f(-500.0f, -500.0f), sf::Color::Red);
    flow.addForce(sf::Vector2f(1000.0f, 300.0f), sf::Vector2f(-400.0f, 500.0f), sf::Color::Green);

    std::vector<sf::Vertex> arrows;
    flow.generateFlowField(arrows);

    while (window.isOpen()) {
        if (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        flow.updateForcePositions();
        flow.generateFlowField(arrows);

        window.clear(sf::Color::White);
        if (!arrows.empty()) {
            window.draw(arrows.data(), arrows.size(), sf::PrimitiveType::Lines);
        }
        window.display();
    }

    return 0;
}*/