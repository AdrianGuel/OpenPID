#ifndef GENERIC_CONTROLLER_HPP
#define GENERIC_CONTROLLER_HPP

#include <Eigen/Dense>
#include <functional>

template <typename T>
class GenericController {
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using ControlFunc = std::function<T(const Vector&, const Vector&)>;

    GenericController(ControlFunc control_func)
        : control_fn(control_func), reference(Vector::Zero(0)) {}

    void set_reference(const Vector& ref) {
        reference = ref;
    }

    T compute(const Vector& state) const {
        return control_fn(state, reference);
    }

private:
    ControlFunc control_fn;
    Vector reference;
};

#endif // GENERIC_CONTROLLER_HPP
