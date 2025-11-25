/*#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>

// Use Eigen for linear algebra
#include <Eigen/Dense>

constexpr double PI_VALUE = 3.14159265358979323846;

class SlenderBodySwimmer {
private:
    std::vector<sf::Vector2f> filament;
    std::vector<sf::Vector2f> restPositions;
    std::vector<sf::Vector2f> forces;
    std::vector<sf::Vector2f> velocities;
    std::vector<sf::Vector2f> segment_velocities;
    std::vector<sf::Vector2f> tangents;

    // Parameters
    static constexpr std::size_t N = 20;
    static const float L;
    static const float Rb;
    static const float eta;
    static const float a;  // filament radius

    // Wave parameters
    float k, b, omega;
    float time = 0.0f;
    sf::Color color;

    // Velocity
    sf::Vector2f swimming_velocity;

    // Slender body parameters from the paper
    static const float delta;  // δ = a√ε/2, using ε=2
    static constexpr float q = 4.0f;  // Cutoff parameter from paper

public:
    SlenderBodySwimmer(sf::Vector2f startPos, sf::Color swimmerColor,
        float waveNumber = 0.05f, float amplitude = 8.0f, float frequency = 1.0f)
        : color(swimmerColor), k(waveNumber), b(amplitude), omega(frequency) {

        initializeFilament(startPos);
        forces.resize(N, sf::Vector2f(0, 0));
        velocities.resize(N, sf::Vector2f(0, 0));
        segment_velocities.resize(N, sf::Vector2f(0, 0));
        tangents.resize(N, sf::Vector2f(0, 0));
        calculateTangents();
    }

    void initializeFilament(sf::Vector2f startPos) {
        filament.clear();
        restPositions.clear();

        float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            float x = startPos.x + static_cast<float>(i) * spacing;
            float y = startPos.y;
            filament.push_back(sf::Vector2f(x, y));
            restPositions.push_back(sf::Vector2f(x, y));
        }
    }

    void calculateTangents() {
        for (std::size_t i = 0; i < N; i++) {
            if (i == 0) {
                tangents[i] = filament[1] - filament[0];
            }
            else if (i == N - 1) {
                tangents[i] = filament[N - 1] - filament[N - 2];
            }
            else {
                tangents[i] = (filament[i + 1] - filament[i - 1]) / 2.0f;
            }

            // Normalize tangent
            float length = std::sqrt(tangents[i].x * tangents[i].x + tangents[i].y * tangents[i].y);
            if (length > 1e-10f) {
                tangents[i] /= length;
            }
        }
    }

    void update(float dt, const std::vector<SlenderBodySwimmer*>& otherSwimmers = {}) {
        time += dt;
        applyTravelingWave();
        calculateTangents();  // Recalculate tangents after deformation
        calculateDesiredVelocities();
        solveSlenderBodyEquationWithInteractions(otherSwimmers);
        calculateSegmentVelocities(otherSwimmers);
        moveSwimmer(dt);
    }

    void applyTravelingWave() {
        const float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            const float wave = b * std::sin(k * x_local - omega * time);
            filament[i].y = restPositions[i].y + wave;
        }
    }

    void calculateDesiredVelocities() {
        const float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            float wave_vel_y = -omega * b * std::cos(k * x_local - omega * time);
            velocities[i] = sf::Vector2f(0.0f, wave_vel_y);
        }
    }

    // Calculate perpendicular component: f⟂ = (I - tt)·f
    sf::Vector2f calculatePerpendicularComponent(sf::Vector2f f, sf::Vector2f t) const {
        float tx = t.x, ty = t.y;
        float dot_product = tx * f.x + ty * f.y;
        return sf::Vector2f(
            f.x - dot_product * tx,
            f.y - dot_product * ty
        );
    }

    // Oseen tensor (free-space Green's function for Stokes flow)
    sf::Vector2f oseenTensor(sf::Vector2f r, sf::Vector2f f) const {
        float r_sq = r.x * r.x + r.y * r.y;

        if (r_sq < 1e-10f) {
            // Self-interaction handled separately in slender body operator
            return sf::Vector2f(0, 0);
        }

        float r_mag = std::sqrt(r_sq);
        float dot_product = r.x * f.x + r.y * f.y;
        float factor = 1.0f / (8.0f * static_cast<float>(PI_VALUE) * eta * r_mag);

        return sf::Vector2f(
            factor * (f.x + dot_product * r.x / r_sq),
            factor * (f.y + dot_product * r.y / r_sq)
        );
    }

    // Slender body operator as in equation (46)
    sf::Vector2f slenderBodyOperator(std::size_t i, std::size_t j, sf::Vector2f f_j) const {
        if (i == j) {
            // Self-interaction: local term f⟂/(4πη)
            sf::Vector2f f_perp = calculatePerpendicularComponent(f_j, tangents[i]);
            return f_perp / (4.0f * static_cast<float>(PI_VALUE) * eta);
        }
        else {
            // Non-local interaction: G(rᵢ - rⱼ)·fⱼ
            sf::Vector2f r_vec = filament[i] - filament[j];
            float distance_sq = r_vec.x * r_vec.x + r_vec.y * r_vec.y;

            // Apply cutoff as mentioned in paper (|r₀ - r| > q)
            if (distance_sq <= q * q) {
                return sf::Vector2f(0, 0);
            }

            return oseenTensor(r_vec, f_j);
        }
    }

    void solveSlenderBodyEquationWithInteractions(const std::vector<SlenderBodySwimmer*>& otherSwimmers) {
        Eigen::MatrixXf mobility(2 * N, 2 * N);
        Eigen::VectorXf rhs(2 * N);
        Eigen::VectorXf solution(2 * N);

        // Build mobility matrix using slender body operator
        mobility.setZero();
        for (std::size_t i = 0; i < N; i++) {
            for (std::size_t j = 0; j < N; j++) {
                // Test with unit forces in x and y directions
                sf::Vector2f unit_x(1, 0);
                sf::Vector2f unit_y(0, 1);

                sf::Vector2f response_x = slenderBodyOperator(i, j, unit_x);
                sf::Vector2f response_y = slenderBodyOperator(i, j, unit_y);

                Eigen::Index i_idx = static_cast<Eigen::Index>(i);
                Eigen::Index j_idx = static_cast<Eigen::Index>(j);

                mobility(2 * i_idx, 2 * j_idx) = response_x.x;
                mobility(2 * i_idx, 2 * j_idx + 1) = response_y.x;
                mobility(2 * i_idx + 1, 2 * j_idx) = response_x.y;
                mobility(2 * i_idx + 1, 2 * j_idx + 1) = response_y.y;
            }
        }

        // Build right-hand side
        for (std::size_t i = 0; i < N; i++) {
            // Start with desired kinematic velocity
            sf::Vector2f external_flow = velocities[i];

            // Subtract flow from other swimmers (these are known forces from previous step)
            for (const auto& other : otherSwimmers) {
                for (std::size_t j = 0; j < other->N; j++) {
                    sf::Vector2f r_vec = filament[i] - other->filament[j];
                    float distance_sq = r_vec.x * r_vec.x + r_vec.y * r_vec.y;

                    if (distance_sq > q * q) {
                        external_flow -= oseenTensor(r_vec, other->forces[j]);
                    }
                }
            }

            Eigen::Index i_idx = static_cast<Eigen::Index>(i);
            rhs(2 * i_idx) = external_flow.x;
            rhs(2 * i_idx + 1) = external_flow.y;
        }

        // Solve for forces
        solution = mobility.colPivHouseholderQr().solve(rhs);

        for (std::size_t i = 0; i < N; i++) {
            Eigen::Index i_idx = static_cast<Eigen::Index>(i);
            forces[i] = sf::Vector2f(solution(2 * i_idx), solution(2 * i_idx + 1));
        }
    }

    void calculateSegmentVelocities(const std::vector<SlenderBodySwimmer*>& otherSwimmers) {
        for (std::size_t i = 0; i < N; i++) {
            sf::Vector2f total_velocity(0.0f, 0.0f);

            // Self-interactions using slender body operator
            for (std::size_t j = 0; j < N; j++) {
                total_velocity += slenderBodyOperator(i, j, forces[j]);
            }

            // Interactions with other swimmers
            for (const auto& other : otherSwimmers) {
                for (std::size_t j = 0; j < other->N; j++) {
                    sf::Vector2f r_vec = filament[i] - other->filament[j];
                    float distance_sq = r_vec.x * r_vec.x + r_vec.y * r_vec.y;

                    if (distance_sq > q * q) {
                        total_velocity += oseenTensor(r_vec, other->forces[j]);
                    }
                }
            }

            segment_velocities[i] = total_velocity;
        }

        // Calculate overall swimming velocity (average of all segments)
        swimming_velocity = sf::Vector2f(0.0f, 0.0f);
        for (const auto& vel : segment_velocities) {
            swimming_velocity += vel;
        }
        swimming_velocity /= static_cast<float>(N);
    }

    void moveSwimmer(float dt) {
        for (std::size_t i = 0; i < N; i++) {
            filament[i].x += swimming_velocity.x * dt;
            restPositions[i].x += swimming_velocity.x * dt;
        }

        // Boundary conditions - wrap around screen
        if (filament[0].x > 1200.0f) {
            float offset = filament[0].x - 100.0f;
            for (std::size_t i = 0; i < N; i++) {
                filament[i].x -= offset;
                restPositions[i].x -= offset;
            }
        }
        else if (filament[0].x < -200.0f) {
            float offset = 1200.0f - filament[0].x;
            for (std::size_t i = 0; i < N; i++) {
                filament[i].x += offset;
                restPositions[i].x += offset;
            }
        }
    }

    void draw(sf::RenderWindow& window) {
        // Draw filament
        for (std::size_t i = 0; i < filament.size() - 1; i++) {
            std::array<sf::Vertex, 2> line = {
                sf::Vertex(filament[i], color),
                sf::Vertex(filament[i + 1], color)
            };
            window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
        }

        // Draw force vectors (red)
        for (std::size_t i = 0; i < filament.size(); i++) {
            sf::Vector2f force_end = filament[i] + forces[i] * 0.001f;
            std::array<sf::Vertex, 2> force_line = {
                sf::Vertex(filament[i], sf::Color::Red),
                sf::Vertex(force_end, sf::Color::Red)
            };
            window.draw(force_line.data(), force_line.size(), sf::PrimitiveType::Lines);
        }

        // REMOVED: Green tangent vectors

        // Draw points
        sf::Color pointColor = color;
        pointColor.r = static_cast<std::uint8_t>(std::min(255, static_cast<int>(color.r) + 50));
        pointColor.g = static_cast<std::uint8_t>(std::min(255, static_cast<int>(color.g) + 50));
        pointColor.b = static_cast<std::uint8_t>(std::min(255, static_cast<int>(color.b) + 50));

        for (std::size_t i = 0; i < filament.size(); i++) {
            sf::CircleShape point(3.0f);
            point.setPosition(sf::Vector2f(filament[i].x - 3.0f, filament[i].y - 3.0f));
            point.setFillColor(pointColor);
            window.draw(point);
        }

        // Draw head
        sf::Color headColor = color;
        headColor.r = static_cast<std::uint8_t>(std::max(0, static_cast<int>(color.r) - 50));
        headColor.g = static_cast<std::uint8_t>(std::max(0, static_cast<int>(color.g) - 50));
        headColor.b = static_cast<std::uint8_t>(std::max(0, static_cast<int>(color.b) - 50));

        sf::CircleShape head(Rb);
        head.setPosition(sf::Vector2f(filament[0].x - Rb, filament[0].y - Rb));
        head.setFillColor(headColor);
        window.draw(head);
    }

    // Getters
    sf::Vector2f getNetForce() const {
        sf::Vector2f net(0, 0);
        for (const auto& f : forces) net += f;
        return net;
    }

    sf::Vector2f getSwimmingVelocity() const { return swimming_velocity; }
    const std::vector<sf::Vector2f>& getSegmentVelocities() const { return segment_velocities; }
    const std::vector<sf::Vector2f>& getFilament() const { return filament; }
    const std::vector<sf::Vector2f>& getForces() const { return forces; }

    std::size_t getN() const { return N; }
};

// Define static constants outside the class
const float SlenderBodySwimmer::L = 200.0f;
const float SlenderBodySwimmer::Rb = 8.0f;
const float SlenderBodySwimmer::eta = 0.1f;
const float SlenderBodySwimmer::a = 1.0f;
const float SlenderBodySwimmer::delta = SlenderBodySwimmer::a * std::sqrt(2.0f) / 2.0f;

class MultiSwimmerSimulation {
private:
    sf::RenderWindow window;
    std::vector<SlenderBodySwimmer> swimmers;

public:
    MultiSwimmerSimulation() : window(sf::VideoMode(sf::Vector2u(1200, 800)), "Multi-Swimmer Slender Body Theory") {
        // Add multiple swimmers with different parameters and positions
        addSwimmer(sf::Vector2f(100.0f, 200.0f), sf::Color::Green, 0.05f, 8.0f, 1.0f);
        addSwimmer(sf::Vector2f(100.0f, 400.0f), sf::Color::Blue, 0.03f, 12.0f, 0.8f);
        addSwimmer(sf::Vector2f(100.0f, 600.0f), sf::Color::Magenta, 0.08f, 6.0f, 1.2f);

        std::cout << "Multi-Swimmer Slender Body Theory Simulation Started" << std::endl;
        std::cout << "Now implementing Lighthill's slender body theory with perpendicular components" << std::endl;
        std::cout << "Number of swimmers: " << swimmers.size() << std::endl;
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
        // Create list of pointers to all other swimmers for each swimmer
        std::vector<std::vector<SlenderBodySwimmer*>> otherSwimmersLists;

        for (std::size_t i = 0; i < swimmers.size(); i++) {
            std::vector<SlenderBodySwimmer*> others;
            for (std::size_t j = 0; j < swimmers.size(); j++) {
                if (i != j) {
                    others.push_back(&swimmers[j]);
                }
            }
            otherSwimmersLists.push_back(others);
        }

        // Update each swimmer with interactions
        for (std::size_t i = 0; i < swimmers.size(); i++) {
            swimmers[i].update(dt, otherSwimmersLists[i]);
        }

        // Print status occasionally
        static std::size_t frame = 0;
        if (frame++ % 120 == 0) {
            std::cout << "\n=== Frame " << frame << " ===" << std::endl;
            for (std::size_t i = 0; i < swimmers.size(); i++) {
                sf::Vector2f net_force = swimmers[i].getNetForce();
                sf::Vector2f swim_vel = swimmers[i].getSwimmingVelocity();
                sf::Vector2f head_pos = swimmers[i].getFilament()[0];

                std::cout << "Swimmer " << i + 1 << ":" << std::endl;
                std::cout << "  Position: (" << head_pos.x << ", " << head_pos.y << ")" << std::endl;
                std::cout << "  Net force: (" << net_force.x << ", " << net_force.y << ")" << std::endl;
                std::cout << "  Velocity: (" << swim_vel.x << ", " << swim_vel.y << ")" << std::endl;

                // Show perpendicular components
                auto forces = swimmers[i].getForces();
                float total_force_mag = 0;
                for (std::size_t j = 0; j < forces.size(); j++) {
                    total_force_mag += std::sqrt(forces[j].x * forces[j].x + forces[j].y * forces[j].y);
                }
                std::cout << "  Average force magnitude: " << total_force_mag / forces.size() << std::endl;
            }
            std::cout << "=====================" << std::endl;
        }
    }

    void render() {
        window.clear(sf::Color::Black);

        for (auto& swimmer : swimmers) {
            swimmer.draw(window);
        }

        window.display();
    }
};

int main() {
    MultiSwimmerSimulation simulation;
    simulation.run();
    return 0;
}*/