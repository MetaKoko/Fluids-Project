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
    std::vector<sf::Vector2f> forces;
    std::vector<sf::Vector2f> velocities;
    std::vector<sf::Vector2f> segment_velocities;
    std::vector<sf::Vector2f> tangents;

    // Parameters
    static constexpr std::size_t N = 20;
    static const float L;
    static const float Rb;
    static const float eta;
    static const float a;

    // Wave parameters
    float k, b, omega;
    float time = 0.0f;
    sf::Color color;

    // Swimming state - position is the HEAD position
    sf::Vector2f position;
    sf::Vector2f swimming_velocity;

    // Slender body parameters
    static const float delta;
    static constexpr float q = 4.0f;

    // Periodic boundary parameters
    static constexpr float BOX_WIDTH = 1200.0f;
    static constexpr float BOX_HEIGHT = 800.0f;
    static constexpr int IMAGE_RANGE = 1;

public:
    SlenderBodySwimmer(sf::Vector2f headPos, sf::Color swimmerColor,
        float waveNumber = 0.05f, float amplitude = 8.0f, float frequency = 1.0f)
        : position(headPos), color(swimmerColor), k(waveNumber), b(amplitude), omega(frequency) {

        initializeFilament();
        forces.resize(N, sf::Vector2f(0, 0));
        velocities.resize(N, sf::Vector2f(0, 0));
        segment_velocities.resize(N, sf::Vector2f(0, 0));
        tangents.resize(N, sf::Vector2f(0, 0));
    }

    void initializeFilament() {
        filament.clear();
        float spacing = L / static_cast<float>(N - 1);

        // Head at index 0 is the leading point
        // Tail at index N-1 is the trailing point
        for (std::size_t i = 0; i < N; i++) {
            float x = position.x - static_cast<float>(i) * spacing;  // Head at pos.x, tail at pos.x - L
            float y = position.y;
            filament.push_back(sf::Vector2f(x, y));
        }
    }

    void calculateTangents() {
        for (std::size_t i = 0; i < N; i++) {
            if (i == 0) {
                tangents[i] = filament[1] - filament[0];  // Head tangent
            }
            else if (i == N - 1) {
                tangents[i] = filament[N - 1] - filament[N - 2];  // Tail tangent
            }
            else {
                tangents[i] = (filament[i + 1] - filament[i - 1]) / 2.0f;
            }

            float length = std::sqrt(tangents[i].x * tangents[i].x + tangents[i].y * tangents[i].y);
            if (length > 1e-10f) {
                tangents[i] /= length;
            }
        }
    }

    void update(float dt, const std::vector<SlenderBodySwimmer*>& otherSwimmers = {}) {
        time += dt;

        applyTravelingWave();
        calculateTangents();
        calculateDesiredVelocities();

        solveSlenderBodyEquationWithInteractions(otherSwimmers);
        calculateSegmentVelocities(otherSwimmers);

        // Ensure forward motion (positive x velocity)
        //if (swimming_velocity.x < 20.0f) {
        //    swimming_velocity.x = 50.0f;
        //}

        moveSwimmer(dt);
    }

    void applyTravelingWave() {
        const float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            // Wave propagates FROM HEAD TO TAIL (backwards relative to swimming direction)
            const float wave = b * std::sin(k * x_local - omega * time);

            filament[i].x = position.x - static_cast<float>(i) * spacing;
            filament[i].y = position.y + wave;
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

    sf::Vector2f calculatePerpendicularComponent(sf::Vector2f f, sf::Vector2f t) const {
        float tx = t.x, ty = t.y;
        float dot_product = tx * f.x + ty * f.y;
        return sf::Vector2f(
            f.x - dot_product * tx,
            f.y - dot_product * ty
        );
    }

    sf::Vector2f oseenTensor(sf::Vector2f r, sf::Vector2f f) const {
        float r_sq = r.x * r.x + r.y * r.y;

        if (r_sq < 1e-10f) {
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

    sf::Vector2f getImagePosition(sf::Vector2f pos, int dx, int dy) const {
        return sf::Vector2f(
            pos.x + dx * BOX_WIDTH,
            pos.y + dy * BOX_HEIGHT
        );
    }

    sf::Vector2f slenderBodyOperator(std::size_t i, std::size_t j, sf::Vector2f f_j) const {
        if (i == j) {
            sf::Vector2f f_perp = calculatePerpendicularComponent(f_j, tangents[i]);
            return f_perp / (4.0f * static_cast<float>(PI_VALUE) * eta);
        }
        else {
            sf::Vector2f total_response(0, 0);

            for (int dx = -IMAGE_RANGE; dx <= IMAGE_RANGE; dx++) {
                for (int dy = -IMAGE_RANGE; dy <= IMAGE_RANGE; dy++) {
                    sf::Vector2f image_pos_j = getImagePosition(filament[j], dx, dy);
                    sf::Vector2f r_vec = filament[i] - image_pos_j;
                    float distance_sq = r_vec.x * r_vec.x + r_vec.y * r_vec.y;

                    if (distance_sq > q * q) {
                        total_response += oseenTensor(r_vec, f_j);
                    }
                }
            }

            return total_response;
        }
    }

    void solveSlenderBodyEquationWithInteractions(const std::vector<SlenderBodySwimmer*>& otherSwimmers) {
        Eigen::MatrixXf mobility(2 * static_cast<int>(N), 2 * static_cast<int>(N));
        Eigen::VectorXf rhs(2 * static_cast<int>(N));
        Eigen::VectorXf solution(2 * static_cast<int>(N));

        mobility.setZero();
        for (std::size_t i = 0; i < N; i++) {
            for (std::size_t j = 0; j < N; j++) {
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

        for (std::size_t i = 0; i < N; i++) {
            sf::Vector2f external_flow = velocities[i];

            for (const auto& other : otherSwimmers) {
                for (std::size_t j = 0; j < other->N; j++) {
                    for (int dx = -IMAGE_RANGE; dx <= IMAGE_RANGE; dx++) {
                        for (int dy = -IMAGE_RANGE; dy <= IMAGE_RANGE; dy++) {
                            if (dx == 0 && dy == 0 && this == other) continue;

                            sf::Vector2f image_pos_j = other->getImagePosition(other->filament[j], dx, dy);
                            sf::Vector2f r_vec = filament[i] - image_pos_j;
                            float distance_sq = r_vec.x * r_vec.x + r_vec.y * r_vec.y;

                            if (distance_sq > q * q) {
                                external_flow -= oseenTensor(r_vec, other->forces[j]);
                            }
                        }
                    }
                }
            }

            Eigen::Index i_idx = static_cast<Eigen::Index>(i);
            rhs(2 * i_idx) = external_flow.x;
            rhs(2 * i_idx + 1) = external_flow.y;
        }

        solution = mobility.colPivHouseholderQr().solve(rhs);

        for (std::size_t i = 0; i < N; i++) {
            Eigen::Index i_idx = static_cast<Eigen::Index>(i);
            forces[i] = sf::Vector2f(solution(2 * i_idx), solution(2 * i_idx + 1));
        }
    }

    void calculateSegmentVelocities(const std::vector<SlenderBodySwimmer*>& otherSwimmers) {
        for (std::size_t i = 0; i < N; i++) {
            sf::Vector2f total_velocity(0.0f, 0.0f);

            for (std::size_t j = 0; j < N; j++) {
                total_velocity += slenderBodyOperator(i, j, forces[j]);
            }

            for (const auto& other : otherSwimmers) {
                for (std::size_t j = 0; j < other->N; j++) {
                    for (int dx = -IMAGE_RANGE; dx <= IMAGE_RANGE; dx++) {
                        for (int dy = -IMAGE_RANGE; dy <= IMAGE_RANGE; dy++) {
                            if (dx == 0 && dy == 0 && this == other) continue;

                            sf::Vector2f image_pos_j = other->getImagePosition(other->filament[j], dx, dy);
                            sf::Vector2f r_vec = filament[i] - image_pos_j;
                            float distance_sq = r_vec.x * r_vec.x + r_vec.y * r_vec.y;

                            if (distance_sq > q * q) {
                                total_velocity += oseenTensor(r_vec, other->forces[j]);
                            }
                        }
                    }
                }
            }

            segment_velocities[i] = total_velocity;
        }

        swimming_velocity = sf::Vector2f(0.0f, 0.0f);
        for (const auto& vel : segment_velocities) {
            swimming_velocity += vel;
        }
        swimming_velocity /= static_cast<float>(N);
    }

    void moveSwimmer(float dt) {
        // Update the HEAD position
        position += swimming_velocity * dt;

        // Apply periodic boundary to HEAD
        if (position.x > BOX_WIDTH) {
            position.x = 0.0f;
        }
        else if (position.x < 0.0f) {
            position.x = BOX_WIDTH;
        }

        // Update all filament points relative to new head position
        const float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            const float wave = b * std::sin(k * x_local - omega * time);

            filament[i].x = position.x - static_cast<float>(i) * spacing;
            filament[i].y = position.y + wave;

            // Apply periodic boundary to each point
            if (filament[i].x > BOX_WIDTH) {
                filament[i].x -= BOX_WIDTH;
            }
            else if (filament[i].x < 0.0f) {
                filament[i].x += BOX_WIDTH;
            }
        }
    }

    void draw(sf::RenderWindow& window) {
        // Draw filament from head to tail
        for (std::size_t i = 0; i < filament.size() - 1; i++) {
            std::array<sf::Vertex, 2> line = {
                sf::Vertex(filament[i], color),
                sf::Vertex(filament[i + 1], color)
            };
            window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
        }

        // Draw points on tail and head - all the same color as the swimmer
        for (std::size_t i = 0; i < filament.size(); i++) {
            sf::CircleShape point(3.0f);
            point.setPosition(sf::Vector2f(filament[i].x - 3.0f, filament[i].y - 3.0f));

            // All points use the swimmer's color
            point.setFillColor(color);
            window.draw(point);
        }

        // Draw head with the same color as the tail
        sf::CircleShape head(Rb);
        head.setPosition(sf::Vector2f(filament[0].x - Rb, filament[0].y - Rb));
        head.setFillColor(color);
        window.draw(head);

        // Calculate net force
        sf::Vector2f net_force(0.0f, 0.0f);
        for (const auto& force : forces) {
            net_force += force;
        }

        // Draw net force vector (red) from head position
        sf::Vector2f force_end = filament[0] + net_force * 0.0001f;
        std::array<sf::Vertex, 2> force_line = {
            sf::Vertex(filament[0], sf::Color::Red),
            sf::Vertex(force_end, sf::Color::Red)
        };
        window.draw(force_line.data(), force_line.size(), sf::PrimitiveType::Lines);

        // Print position, net force, and velocity information
        float force_magnitude = std::sqrt(net_force.x * net_force.x + net_force.y * net_force.y);
        float velocity_magnitude = std::sqrt(swimming_velocity.x * swimming_velocity.x + swimming_velocity.y * swimming_velocity.y);

        static std::size_t frame = 0;
        if (frame++ % 60 == 0) {
            std::cout << "Position: (" << position.x << ", " << position.y << ") | ";
            std::cout << "Net Force: " << force_magnitude << " | ";
            std::cout << "Velocity: " << velocity_magnitude << std::endl;
        }
    }

    // Getters
    sf::Vector2f getPosition() const { return position; }
    sf::Vector2f getSwimmingVelocity() const { return swimming_velocity; }

    std::size_t getN() const { return N; }
};

// Define static constants
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
    MultiSwimmerSimulation() : window(sf::VideoMode(sf::Vector2u(1200, 800)), "Swimmer Physics Simulation") {
        // Start all swimmers with heads on the LEFT side
        // Optimized parameters for better propulsion and distinct behaviors:

        // Swimmer 1: High frequency, moderate amplitude - fast swimmer
        addSwimmer(sf::Vector2f(100.0f, 200.0f), sf::Color::Green, 0.08f, 12.0f, 2.5f);

        // Swimmer 2: Large amplitude, lower frequency - powerful swimmer  
        addSwimmer(sf::Vector2f(100.0f, 400.0f), sf::Color::Blue, 0.12f, 18.0f, 1.8f);

        // Swimmer 3: Short wavelength, high frequency - agile swimmer
        addSwimmer(sf::Vector2f(100.0f, 600.0f), sf::Color::Magenta, 0.15f, 10.0f, 3.0f);

        std::cout << "=== SWIMMER PHYSICS SIMULATION ===" << std::endl;
        std::cout << "Green: Fast swimmer (high frequency)" << std::endl;
        std::cout << "Blue: Powerful swimmer (large amplitude)" << std::endl;
        std::cout << "Magenta: Agile swimmer (short wavelength)" << std::endl;
        std::cout << "Red vectors: Net Force" << std::endl;
        std::cout << "Colored circles: Head and tail segments" << std::endl;
        std::cout << "================================" << std::endl;
    }

    void addSwimmer(sf::Vector2f headPosition, sf::Color color, float k, float b, float omega) {
        swimmers.emplace_back(headPosition, color, k, b, omega);
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

        for (std::size_t i = 0; i < swimmers.size(); i++) {
            swimmers[i].update(dt, otherSwimmersLists[i]);
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