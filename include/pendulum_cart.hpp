#ifndef PENDULUM_CART_HPP
#define PENDULUM_CART_HPP

#include <cmath>

template <typename T>
class PendulumOnCart {
public:
    PendulumOnCart(T mass_cart, T mass_pend, T length, T friction_cart, T inertia, T gravity = 9.81)
        : M(mass_cart), m(mass_pend), l(length), b(friction_cart), I(inertia), g(gravity),
          x(0), v(0), theta(M_PI + 0.01), omega(0) {}

    void reset(T x0 = 0, T v0 = 0, T theta0 = M_PI + 0.01, T omega0 = 0) {
        x = x0;
        v = v0;
        theta = theta0;
        omega = omega0;
    }

    void update(T force, T dt) {
        // Compute intermediates for Euler update
        T sin_theta = std::sin(theta);
        T cos_theta = std::cos(theta);
        T denom = (I + m * l * l) * (m + M) - m * m * l * l * cos_theta * cos_theta;

        // Acceleration of cart
        T a = ((-b * v + force) * (I + m * l * l) + m * l * sin_theta * (omega * omega * (I + m * l * l) + g * m * l * cos_theta)) / denom;

        // Angular acceleration of pendulum
        T alpha = (2 * l * m * (g * (M + m) * sin_theta + cos_theta * (-b * v + force + omega * omega * l * m * sin_theta))) /
                  (-2 * I * (m + M) - l * l * m * (m + 2 * M) + l * l * m * m * std::cos(2 * theta));

        // Euler integration
        x     += dt * v;
        v     += dt * a;
        theta += dt * omega;
        omega += dt * alpha;
    }

    // Accessors
    T get_position() const { return x; }
    T get_velocity() const { return v; }
    T get_angle() const { return theta; }
    T get_angular_velocity() const { return omega; }

private:
    T M, m, l, b, I, g;
    T x, v;         // Cart position & velocity
    T theta, omega; // Pendulum angle & angular velocity
};

#endif // PENDULUM_CART_HPP
