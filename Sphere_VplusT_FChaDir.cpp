/*#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

struct Sphere {
    sf::Vector2f position;
    sf::Vector2f force;
    float torque;
    sf::Color color;
    sf::Vector2f velocity;
    float angular_velocity;
    float radius;
    float angle;
    float force_magnitude;
    sf::Vector2f force_direction;
    sf::Vector2f initial_world_force_dir;  // Store the initial world-space force direction (normalized)
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

    void addSphere(const sf::Vector2f& position, float force_magnitude, const sf::Vector2f& world_force_direction,
        float torque = 0.0f, float radius = 20.0f, sf::Color color = sf::Color::Red) {

        // Use the world-space force direction directly (no rotation applied initially)
        sf::Vector2f normalized_world_dir = world_force_direction;
        float dir_length = std::sqrt(normalized_world_dir.x * normalized_world_dir.x + normalized_world_dir.y * normalized_world_dir.y);
        if (dir_length > 0.0001f) {
            normalized_world_dir.x /= dir_length;
            normalized_world_dir.y /= dir_length;
        }

        // Calculate initial angle from the world force direction
        float initial_angle = std::atan2(normalized_world_dir.y, normalized_world_dir.x);

        // Initial force is exactly the world direction * magnitude
        sf::Vector2f initial_force = normalized_world_dir * force_magnitude;

        // For the relative force direction, use (1,0) since we want force to be "forward" relative to orientation
        sf::Vector2f relative_force_dir(1.0f, 0.0f);

        spheres.push_back({
            position,
            initial_force,                    // Current force
            torque,
            color,
            sf::Vector2f(0.0f, 0.0f),
            0.0f,
            radius,
            initial_angle,                    // Initial orientation matches world force direction
            force_magnitude,
            relative_force_dir,               // Force is always "forward" relative to orientation
            normalized_world_dir              // Store the exact initial world direction for black line
            });

        std::cout << "Added sphere at (" << position.x << ", " << position.y << ")" << std::endl;
        std::cout << "  Initial world force direction: (" << normalized_world_dir.x << ", " << normalized_world_dir.y << ")" << std::endl;
        std::cout << "  Initial angle: " << initial_angle * 180.0f / 3.14159f << " degrees" << std::endl;
    }

    void updateForceDirections() {
        for (auto& sphere : spheres) {
            float cos_angle = std::cos(sphere.angle);
            float sin_angle = std::sin(sphere.angle);

            // Rotate the relative force direction (1,0) by the current orientation
            sf::Vector2f world_force_dir;
            world_force_dir.x = sphere.force_direction.x * cos_angle - sphere.force_direction.y * sin_angle;
            world_force_dir.y = sphere.force_direction.x * sin_angle + sphere.force_direction.y * cos_angle;

            sphere.force = world_force_dir * sphere.force_magnitude;
        }
    }

    // [ALL THE PHYSICS METHODS - same as before]
    sf::Vector2f applyStokeslet(const sf::Vector2f& r, const sf::Vector2f& F) const {
        float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);
        if (r_magnitude < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

        float r_inv = 1.0f / r_magnitude;
        float r3_inv = 1.0f / (r_magnitude * r_magnitude * r_magnitude);
        float r_dot_f = r.x * F.x + r.y * F.y;

        const float pi = 3.14159265358979323846f;
        float prefactor = 1.0f / (8.0f * pi * viscosity_);

        return sf::Vector2f(
            prefactor * (F.x * r_inv + r_dot_f * r.x * r3_inv),
            prefactor * (F.y * r_inv + r_dot_f * r.y * r3_inv)
        );
    }

    sf::Vector2f applyRotlet(const sf::Vector2f& r, float torque) const {
        float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);
        if (r_magnitude < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

        float r3_inv = 1.0f / (r_magnitude * r_magnitude * r_magnitude);
        const float pi = 3.14159265358979323846f;
        float prefactor = torque / (8.0f * pi * viscosity_);

        return sf::Vector2f(
            -prefactor * r.y * r3_inv,
            prefactor * r.x * r3_inv
        );
    }

    sf::Vector2f applyLaplacianCorrection(const sf::Vector2f& r, const sf::Vector2f& F, float a_squared) const {
        float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);
        if (r_magnitude < 0.0001f) return sf::Vector2f(0.0f, 0.0f);

        float r2 = r_magnitude * r_magnitude;
        float r3_inv = 1.0f / (r_magnitude * r2);
        float r5_inv = 1.0f / (r2 * r2 * r_magnitude);
        float r_dot_f = r.x * F.x + r.y * F.y;

        const float pi = 3.14159265358979323846f;
        float prefactor = a_squared / (24.0f * pi * viscosity_);

        return sf::Vector2f(
            prefactor * (F.x * r3_inv - 3.0f * r_dot_f * r.x * r5_inv),
            prefactor * (F.y * r3_inv - 3.0f * r_dot_f * r.y * r5_inv)
        );
    }

    float calculateVorticity(const sf::Vector2f& r, const Sphere& source_sphere) const {
        float r_magnitude = std::sqrt(r.x * r.x + r.y * r.y);
        if (r_magnitude < 0.0001f) return 0.0f;

        float r3_inv = 1.0f / (r_magnitude * r_magnitude * r_magnitude);
        const float pi = 3.14159265358979323846f;

        float stokeslet_vorticity = (r.x * source_sphere.force.y - r.y * source_sphere.force.x) * r3_inv;
        stokeslet_vorticity /= (8.0f * pi * viscosity_);

        float rotlet_vorticity = source_sphere.torque / (4.0f * pi * viscosity_ * r_magnitude * r_magnitude * r_magnitude);

        return stokeslet_vorticity + rotlet_vorticity;
    }

    void calculateSphereVelocity(size_t sphere_index, sf::Vector2f& velocity, float& angular_velocity) const {
        const Sphere& target_sphere = spheres[sphere_index];
        velocity = sf::Vector2f(0.0f, 0.0f);
        angular_velocity = 0.0f;

        const float pi = 3.14159265358979323846f;

        // Self-mobility
        float self_mobility = 1.0f / (6.0f * pi * viscosity_ * target_sphere.radius);
        velocity += target_sphere.force * self_mobility;

        // Self-rotation
        float rotational_mobility = 1.0f / (8.0f * pi * viscosity_ * std::pow(target_sphere.radius, 3.0f));
        angular_velocity += target_sphere.torque * rotational_mobility;

        // Interactions with other spheres
        float background_vorticity = 0.0f;

        for (size_t i = 0; i < spheres.size(); ++i) {
            if (i == sphere_index) continue;

            const Sphere& source_sphere = spheres[i];

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    sf::Vector2f image_position = source_sphere.position;
                    image_position.x += dx * screen_width_;
                    image_position.y += dy * screen_height_;

                    sf::Vector2f r = target_sphere.position - image_position;

                    // Force contributions
                    sf::Vector2f stokeslet_flow = applyStokeslet(r, source_sphere.force);
                    sf::Vector2f correction_flow = applyLaplacianCorrection(r, source_sphere.force,
                        target_sphere.radius * target_sphere.radius);
                    velocity += stokeslet_flow + correction_flow;

                    // Torque contributions
                    sf::Vector2f rotlet_flow = applyRotlet(r, source_sphere.torque);
                    velocity += rotlet_flow;

                    // Vorticity for torque coupling
                    background_vorticity += calculateVorticity(r, source_sphere);
                }
            }
        }

        // Torque coupling
        angular_velocity += 0.5f * background_vorticity;
    }

    void updateSpherePositions() {
        updateForceDirections();

        for (size_t i = 0; i < spheres.size(); ++i) {
            sf::Vector2f velocity;
            float angular_velocity;
            calculateSphereVelocity(i, velocity, angular_velocity);

            spheres[i].velocity = velocity;
            spheres[i].angular_velocity = angular_velocity;
        }

        for (auto& sphere : spheres) {
            sphere.position += sphere.velocity * time_step_;
            sphere.angle += sphere.angular_velocity * time_step_;

            if (sphere.angle > 2.0f * 3.14159f) sphere.angle -= 2.0f * 3.14159f;
            if (sphere.angle < 0.0f) sphere.angle += 2.0f * 3.14159f;

            if (sphere.position.x < 0.0f) sphere.position.x += screen_width_;
            else if (sphere.position.x >= screen_width_) sphere.position.x -= screen_width_;
            if (sphere.position.y < 0.0f) sphere.position.y += screen_height_;
            else if (sphere.position.y >= screen_height_) sphere.position.y -= screen_height_;
        }
    }

    void calculateFlowField(const sf::Vector2f& x, sf::Vector2f& velocity, float& vorticity) const {
        velocity = sf::Vector2f(0.0f, 0.0f);
        vorticity = 0.0f;

        for (const auto& sphere : spheres) {
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    sf::Vector2f image_position = sphere.position;
                    image_position.x += dx * screen_width_;
                    image_position.y += dy * screen_height_;

                    sf::Vector2f r = x - image_position;

                    sf::Vector2f stokeslet_flow = applyStokeslet(r, sphere.force);
                    sf::Vector2f rotlet_flow = applyRotlet(r, sphere.torque);
                    velocity += stokeslet_flow + rotlet_flow;
                    vorticity += calculateVorticity(r, sphere);
                }
            }
        }
    }

    void generateFlowField(std::vector<sf::Vertex>& arrows) const {
        arrows.clear();

        // Flow field visualization
        for (int x = 0; x < screen_width_; x += 20) {
            for (int y = 0; y < screen_height_; y += 20) {
                float fx = static_cast<float>(x);
                float fy = static_cast<float>(y);
                sf::Vector2f point(fx, fy);
                sf::Vector2f velocity;
                float vorticity;
                calculateFlowField(point, velocity, vorticity);

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

        // Draw spheres
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

        // Draw sphere body (rotates with orientation)
        for (int i = 0; i < segments; i++) {
            float angle1 = 2.0f * 3.14159f * i / segments + sphere.angle;
            float angle2 = 2.0f * 3.14159f * (i + 1) / segments + sphere.angle;
            sf::Vector2f point1 = sphere.position + sf::Vector2f(sphere.radius * std::cos(angle1),
                sphere.radius * std::sin(angle1));
            sf::Vector2f point2 = sphere.position + sf::Vector2f(sphere.radius * std::cos(angle2),
                sphere.radius * std::sin(angle2));
            arrows.push_back(sf::Vertex(point1, sphere.color));
            arrows.push_back(sf::Vertex(point2, sphere.color));
        }

        // BLACK LINE: Shows the EXACT INITIAL world force direction
        sf::Vector2f initial_force_end = sphere.position + sphere.initial_world_force_dir * sphere.radius * 1.5f;
        arrows.push_back(sf::Vertex(sphere.position, sf::Color::Black));
        arrows.push_back(sf::Vertex(initial_force_end, sf::Color::Black));

        // RED LINE: Shows CURRENT force direction
        float force_scale = 0.03f;
        sf::Vector2f current_force_end = sphere.position + sphere.force * force_scale;
        arrows.push_back(sf::Vertex(sphere.position, sf::Color::Red));
        arrows.push_back(sf::Vertex(current_force_end, sf::Color::Red));

        // Magenta velocity vector
        float speed = std::sqrt(sphere.velocity.x * sphere.velocity.x + sphere.velocity.y * sphere.velocity.y);
        if (speed > 0.1f) {
            sf::Vector2f velocity_end = sphere.position + sphere.velocity * 20.0f;
            arrows.push_back(sf::Vertex(sphere.position, sf::Color::Magenta));
            arrows.push_back(sf::Vertex(velocity_end, sf::Color::Magenta));
        }

        // Rotation indicator
        if (std::abs(sphere.angular_velocity) > 0.01f) {
            float rotation_indicator_length = sphere.radius * 0.8f;
            sf::Vector2f rotation_indicator = sphere.position + sf::Vector2f(
                rotation_indicator_length * std::cos(sphere.angle + 3.14159f * 0.5f),
                rotation_indicator_length * std::sin(sphere.angle + 3.14159f * 0.5f)
            );
            sf::Color rotation_color = (sphere.angular_velocity > 0) ? sf::Color::Cyan : sf::Color::Yellow;
            arrows.push_back(sf::Vertex(sphere.position, rotation_color));
            arrows.push_back(sf::Vertex(rotation_indicator, rotation_color));
        }
    }
};

int main() {
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(desktop, "Stokes Flow - Force Direction Reference");
    window.setFramerateLimit(60);

    StokesFlow flow(70.0f, 25.0f, 1340.0f, 700.0f);

    // Now we pass the EXACT world-space directions we want for the black lines:
    flow.addSphere(sf::Vector2f(200.0f, 200.0f), 1000.0f, sf::Vector2f(1.0f, 0.0f), 0.0f, 20.0f, sf::Color::Blue);      // Black line: right
    flow.addSphere(sf::Vector2f(600.0f, 400.0f), 800.0f, sf::Vector2f(0.707f, 0.707f), 0.0f, 20.0f, sf::Color::Red);    // Black line: up-right 45°
    flow.addSphere(sf::Vector2f(1000.0f, 300.0f), 1200.0f, sf::Vector2f(0.0f, 1.0f), 0.0f, 20.0f, sf::Color::Green);    // Black line: up

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