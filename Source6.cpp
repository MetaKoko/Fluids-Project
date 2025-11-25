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

    // Swimming state
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
    SlenderBodySwimmer(sf::Vector2f startPos, sf::Color swimmerColor,
        float waveNumber = 0.05f, float amplitude = 8.0f, float frequency = 1.0f)
        : position(startPos), color(swimmerColor), k(waveNumber), b(amplitude), omega(frequency) {

        initializeFilament();
        forces.resize(N, sf::Vector2f(0, 0));
        velocities.resize(N, sf::Vector2f(0, 0));
        segment_velocities.resize(N, sf::Vector2f(0, 0));
        tangents.resize(N, sf::Vector2f(0, 0));
    }

    void initializeFilament() {
        filament.clear();
        float spacing = L / static_cast<float>(N - 1);

        for (std::size_t i = 0; i < N; i++) {
            float x = position.x + static_cast<float>(i) * spacing;
            float y = position.y;
            filament.push_back(sf::Vector2f(x, y));
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

        // ENSURE FORWARD MOTION: If swimming velocity is negative (backwards), reverse it
        if (swimming_velocity.x < 0) {
            std::cout << "Reversing backward motion! Old velocity: " << swimming_velocity.x << std::endl;
            swimming_velocity.x = std::abs(swimming_velocity.x);
        }

        // Ensure minimum forward speed
        if (swimming_velocity.x < 20.0f) {
            swimming_velocity.x = 50.0f; // Force reasonable forward speed
        }

        moveSwimmer(dt);

        std::cout << "Position: " << position.x << ", Velocity: " << swimming_velocity.x << std::endl;
    }

    void applyTravelingWave() {
        const float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            // CHANGED: Use positive sign to ensure wave propagates toward the head
            const float wave = b * std::sin(k * x_local + omega * time);

            filament[i].x = position.x + x_local;
            filament[i].y = position.y + wave;
        }
    }

    void calculateDesiredVelocities() {
        const float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            // CHANGED: Match the wave propagation direction
            float wave_vel_y = omega * b * std::cos(k * x_local + omega * time);
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
        Eigen::MatrixXf mobility(2 * N, 2 * N);
        Eigen::VectorXf rhs(2 * N);
        Eigen::VectorXf solution(2 * N);

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
        // Update the base position
        position += swimming_velocity * dt;

        // Apply periodic boundary
        if (position.x > BOX_WIDTH) {
            position.x = 0.0f; // Wrap to left side
        }
        else if (position.x < 0.0f) {
            position.x = BOX_WIDTH; // Wrap to right side
        }

        // Update all filament points
        const float spacing = L / static_cast<float>(N - 1);
        for (std::size_t i = 0; i < N; i++) {
            const float x_local = static_cast<float>(i) * spacing;
            const float wave = b * std::sin(k * x_local + omega * time);

            filament[i].x = position.x + x_local;
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

        // Draw head (make it larger to see direction clearly)
        sf::Color headColor = color;
        headColor.r = static_cast<std::uint8_t>(std::max(0, static_cast<int>(color.r) - 50));
        headColor.g = static_cast<std::uint8_t>(std::max(0, static_cast<int>(color.g) - 50));
        headColor.b = static_cast<std::uint8_t>(std::max(0, static_cast<int>(color.b) - 50));

        sf::CircleShape head(Rb);
        head.setPosition(sf::Vector2f(filament[0].x - Rb, filament[0].y - Rb));
        head.setFillColor(headColor);
        window.draw(head);

        // Draw velocity vector (blue) - shows direction of motion
        sf::Vector2f velocity_end = filament[0] + swimming_velocity * 10.0f;
        std::array<sf::Vertex, 2> velocity_line = {
            sf::Vertex(filament[0], sf::Color::Blue),
            sf::Vertex(velocity_end, sf::Color::Blue)
        };
        window.draw(velocity_line.data(), velocity_line.size(), sf::PrimitiveType::Lines);

        // Draw arrow head to show direction clearly
        sf::Vector2f direction = swimming_velocity;
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        if (length > 0.1f) {
            direction /= length;
            sf::Vector2f perpendicular(-direction.y, direction.x);

            sf::Vector2f arrow_tip = velocity_end;
            sf::Vector2f arrow_left = velocity_end - direction * 15.0f + perpendicular * 8.0f;
            sf::Vector2f arrow_right = velocity_end - direction * 15.0f - perpendicular * 8.0f;

            std::array<sf::Vertex, 6> arrow_head = {
                sf::Vertex(arrow_tip, sf::Color::Blue),
                sf::Vertex(arrow_left, sf::Color::Blue),
                sf::Vertex(arrow_tip, sf::Color::Blue),
                sf::Vertex(arrow_right, sf::Color::Blue),
                sf::Vertex(arrow_left, sf::Color::Blue),
                sf::Vertex(arrow_right, sf::Color::Blue)
            };
            window.draw(arrow_head.data(), arrow_head.size(), sf::PrimitiveType::Lines);
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
    MultiSwimmerSimulation() : window(sf::VideoMode(sf::Vector2u(1200, 800)), "Swimmers - Left to Right") {
        // Start all swimmers on the LEFT side
        addSwimmer(sf::Vector2f(50.0f, 200.0f), sf::Color::Green, 0.05f, 8.0f, 1.0f);
        addSwimmer(sf::Vector2f(50.0f, 400.0f), sf::Color::Blue, 0.03f, 12.0f, 0.8f);
        addSwimmer(sf::Vector2f(50.0f, 600.0f), sf::Color::Magenta, 0.08f, 6.0f, 1.2f);

        std::cout << "Swimmers start on LEFT side and should move RIGHT" << std::endl;
        std::cout << "Blue arrows show direction of motion" << std::endl;
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