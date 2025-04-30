#ifndef STATE_FEEDBACK_HPP
#define STATE_FEEDBACK_HPP

#include <Eigen/Dense>

template <typename T, int N>
class StateFeedback {
public:
    using Vector = Eigen::Matrix<T, N, 1>;

    explicit StateFeedback(const Vector& K)
        : K_(K), reference_(Vector::Zero()) {}

    void set_reference(const Vector& reference) {
        reference_ = reference;
    }

    T compute(const Vector& state) const {
        return -K_.dot(state - reference_);
    }

private:
    Vector K_;
    Vector reference_;
};

#endif // STATE_FEEDBACK_HPP
