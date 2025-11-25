/*#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

struct Sphere {
    sf::Vector2f position;
    sf::Vector2f force;
    sf::Color color;
    sf::Vector2f velocity;
    float radius;
};

class StokesFlow {
private:
    std::vector<Sphere> spheres;
    float viscosity_;
    float time_step_;
    float screen_width_;
    float screen_height_;

public:
    StokesFlow(float viscosity = 1.0f, float time_step = 0.1f, float screen_width = 1400.0f, float screen_height = 700.0f)
        : viscosity_(viscosity), time_step_(time_step), screen_width_(screen_width), screen_height_(screen_height) {
    }

    void addSphere(const sf::Vector2f& position, const sf::Vector2f& force, float radius = 20.0f, sf::Color color = sf::Color::Red) {
        spheres.push_back({ position, force, color, sf::Vector2f(0.0f, 0.0f), radius });
    }

    // Compute J_ij(r) * F_j for 3D Stokeslet (projected to 2D)
    sf::Vector2f applyStokeslet(const sf::Vector2f& r, const sf::Vector2f& F) const {
        float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);

        if (r_magnitude < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

        float r_inv = 1.0f / r_magnitude;
        float r3_inv = 1.0f / (r_magnitude * r_magnitude * r_magnitude);
        float r_dot_f = r.x * F.x + r.y * F.y;

        const float pi = 3.14159265358979323846f;
        float prefactor = 1.0f / (8.0f * pi * viscosity_); // 8π for 3D Stokeslet

        return sf::Vector2f(
            prefactor * (F.x * r_inv + r_dot_f * r.x * r3_inv),
            prefactor * (F.y * r_inv + r_dot_f * r.y * r3_inv)
        );
    }

    // Compute the Laplacian correction term: (a^2/3) * (I/r^3 - 3*r*r/r^5) * F
    sf::Vector2f applyLaplacianCorrection(const sf::Vector2f& r, const sf::Vector2f& F, float a_squared) const {
        float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);

        if (r_magnitude < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

        float r2 = r_magnitude * r_magnitude;
        float r3_inv = 1.0f / (r_magnitude * r2);
        float r5_inv = 1.0f / (r2 * r2 * r_magnitude);

        float r_dot_f = r.x * F.x + r.y * F.y;

        const float pi = 3.14159265358979323846f;
        float prefactor = a_squared / (24.0f * pi * viscosity_); // a^2/(3 * 8πμ)

        return sf::Vector2f(
            prefactor * (F.x * r3_inv - 3.0f * r_dot_f * r.x * r5_inv),
            prefactor * (F.y * r3_inv - 3.0f * r_dot_f * r.y * r5_inv)
        );
    }

    void calculateSphereVelocity(size_t sphere_index, sf::Vector2f& velocity) const {
        const Sphere& target_sphere = spheres[sphere_index];
        velocity = sf::Vector2f(0.0f, 0.0f);

        const float pi = 3.14159265358979323846f;

        // SELF-MOBILITY: Sphere's response to its own force (Stokes drag)
        float self_mobility = 1.0f / (6.0f * pi * viscosity_ * target_sphere.radius);
        velocity += target_sphere.force * self_mobility;

        // INTERACTIONS: Contributions from other spheres
        for (size_t i = 0; i < spheres.size(); ++i) {
            const Sphere& source_sphere = spheres[i];

            // Skip self-interaction (already handled by self-mobility above)
            if (i == sphere_index) continue;

            // Consider 3x3 grid of periodic images
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    sf::Vector2f image_position = source_sphere.position;
                    image_position.x += dx * screen_width_;
                    image_position.y += dy * screen_height_;

                    sf::Vector2f r = target_sphere.position - image_position;

                    // Basic Stokeslet contribution
                    sf::Vector2f stokeslet_flow = applyStokeslet(r, source_sphere.force);

                    // Faxén correction for finite size
                    sf::Vector2f correction_flow = applyLaplacianCorrection(r, source_sphere.force,
                        target_sphere.radius * target_sphere.radius);

                    velocity += stokeslet_flow + correction_flow;
                }
            }
        }
    }

    void updateSpherePositions() {
        // First calculate all velocities
        for (size_t i = 0; i < spheres.size(); ++i) {
            calculateSphereVelocity(i, spheres[i].velocity);
        }

        // Then update all positions
        for (auto& sphere : spheres) {
            sphere.position += sphere.velocity * time_step_;

            // Apply periodic boundary conditions
            if (sphere.position.x < 0.0f) {
                sphere.position.x += screen_width_;
            }
            else if (sphere.position.x >= screen_width_) {
                sphere.position.x -= screen_width_;
            }

            if (sphere.position.y < 0.0f) {
                sphere.position.y += screen_height_;
            }
            else if (sphere.position.y >= screen_height_) {
                sphere.position.y -= screen_height_;
            }
        }
    }

    void calculateFlowField(const sf::Vector2f& x, sf::Vector2f& velocity) const {
        velocity = sf::Vector2f(0.0f, 0.0f);

        for (const auto& sphere : spheres) {
            // Consider 3x3 grid of periodic images
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    sf::Vector2f image_position = sphere.position;
                    image_position.x += dx * screen_width_;
                    image_position.y += dy * screen_height_;

                    sf::Vector2f r = x - image_position;
                    float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);

                    if (r_magnitude < 0.0001f) continue;

                    // Basic Stokeslet flow
                    sf::Vector2f stokeslet_flow = applyStokeslet(r, sphere.force);
                    velocity += stokeslet_flow;
                }
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
                calculateFlowField(point, velocity);

                float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
                if (speed < 0.0001f) continue;

                float scale = 5000.0f;
                sf::Vector2f direction = velocity / speed;
                float arrow_length = std::clamp(speed * scale, 8.0f, 30.0f);
                sf::Vector2f arrow_end = point + direction * arrow_length;

                // Fixed color for all flow field arrows (removed vorticity-based color changing)
                sf::Color color = sf::Color::Green;

                arrows.push_back(sf::Vertex(point, color));
                arrows.push_back(sf::Vertex(arrow_end, color));
                drawArrowHead(arrows, arrow_end, direction, color);
            }
        }

        // Draw spheres in the central box
        for (const auto& sphere : spheres) {
            drawSphere(arrows, sphere);
        }
    }

    const std::vector<Sphere>& getSpheres() const { return spheres; }
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

    void drawSphere(std::vector<sf::Vertex>& arrows, const Sphere& sphere) const {
        const int segments = 32;

        // Draw sphere circle
        for (int i = 0; i < segments; i++) {
            float angle1 = 2.0f * 3.14159f * i / segments;
            float angle2 = 2.0f * 3.14159f * (i + 1) / segments;
            sf::Vector2f point1 = sphere.position + sf::Vector2f(sphere.radius * std::cos(angle1),
                sphere.radius * std::sin(angle1));
            sf::Vector2f point2 = sphere.position + sf::Vector2f(sphere.radius * std::cos(angle2),
                sphere.radius * std::sin(angle2));
            arrows.push_back(sf::Vertex(point1, sphere.color));
            arrows.push_back(sf::Vertex(point2, sphere.color));
        }

        // Draw force vector
        float force_scale = 0.03f;
        sf::Vector2f force_end = sphere.position + sphere.force * force_scale;
        arrows.push_back(sf::Vertex(sphere.position, sf::Color::Black));
        arrows.push_back(sf::Vertex(force_end, sf::Color::Black));

        // Draw velocity vector
        float speed = std::sqrt(sphere.velocity.x * sphere.velocity.x + sphere.velocity.y * sphere.velocity.y);
        if (speed > 0.1f) {
            sf::Vector2f velocity_end = sphere.position + sphere.velocity * 20.0f;
            arrows.push_back(sf::Vertex(sphere.position, sf::Color::Magenta));
            arrows.push_back(sf::Vertex(velocity_end, sf::Color::Magenta));
        }

        // Draw radius indicator
        arrows.push_back(sf::Vertex(sphere.position, sf::Color::Black));
        arrows.push_back(sf::Vertex(sphere.position + sf::Vector2f(sphere.radius, 0), sf::Color::Black));
    }
};

int main() {
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(desktop, "Stokes Flow - Finite Size Spheres with Self-Mobility Correction");
    window.setFramerateLimit(60);

    StokesFlow flow(70.0f, 20.0f, 1340.0f, 700.0f);

    // Test spheres with different radii to see finite-size effects
    flow.addSphere(sf::Vector2f(200.0f, 200.0f), sf::Vector2f(1000.0f, 0.0f), 10.0f, sf::Color::Blue);
    flow.addSphere(sf::Vector2f(600.0f, 400.0f), sf::Vector2f(-500.0f, -500.0f), 10.0f, sf::Color::Red);
    flow.addSphere(sf::Vector2f(1000.0f, 300.0f), sf::Vector2f(-400.0f, 500.0f), 10.0f, sf::Color::Green);

    std::vector<sf::Vertex> arrows;

    while (window.isOpen()) {
        if (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        flow.updateSpherePositions();
        flow.generateFlowField(arrows);

        window.clear(sf::Color::White);
        if (!arrows.empty()) {
            window.draw(arrows.data(), arrows.size(), sf::PrimitiveType::Lines);
        }
        window.display();
    }

    return 0;
}*/