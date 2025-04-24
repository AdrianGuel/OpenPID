# 🚀 Classical 6-DOF Rigid Body Model with Quaternions

## 📌 Overview

The **6 degrees of freedom (6-DOF)** model describes the motion of a rigid body (like a missile or rocket) in 3D space. These degrees include:
- **3 translational**: movement in x, y, and z directions.
- **3 rotational**: orientation changes (roll, pitch, yaw).

This model uses **quaternions** instead of Euler angles to represent orientation, avoiding gimbal lock and improving numerical stability during fast or complex rotations.

---

## 🔧 State Vector

The full state of the missile is stored as a 13-dimensional vector:

```
x = [position (3), velocity (3), quaternion (4), angular velocity (3)]
  = [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
```

| Component      | Description                                |
|----------------|--------------------------------------------|
| `x, y, z`      | Position in inertial/world coordinates      |
| `vx, vy, vz`   | Velocity in inertial/world coordinates      |
| `qw, qx, qy, qz` | Unit quaternion (orientation)             |
| `wx, wy, wz`   | Angular velocity in body-fixed coordinates |

---

## 🔄 Equations of Motion

### 1. **Translational Kinematics**
Updates position based on velocity:
```
dx/dt = v
```

### 2. **Translational Dynamics**
Newton's second law, with gravity applied in inertial Z-axis:
```
dv/dt = (1/m) * R(q) * F_body - g
```
- `R(q)` is the rotation matrix from body frame to world frame.
- `F_body` is force applied in body coordinates.
- `g` is gravitational acceleration (usually `[0, 0, -9.81]`).

### 3. **Rotational Kinematics**
Quaternion derivative:
```
dq/dt = 0.5 * q ⊗ ω
```
Where:
- `⊗` is the quaternion product.
- `ω = [0, wx, wy, wz]` is a pure quaternion of angular velocity.

### 4. **Rotational Dynamics**
Euler’s rotational equation:
```
dω/dt = J⁻¹ * (τ - ω × (J * ω))
```
- `J` is the inertia matrix.
- `τ` is torque applied in body coordinates.

---

## ⚙️ Why Quaternions?

| Advantage             | Benefit                                |
|-----------------------|----------------------------------------|
| No Gimbal Lock        | Stable under all orientations          |
| Efficient to compute  | Especially for high-speed rotation     |
| Compact & normalized  | Only 4 components, one constraint      |

The quaternion must always be **unit length**, which is enforced by normalization.

---

## 🧠 Assumptions

- Rigid body dynamics (no flexing or deformation).
- Forces and torques are provided in the **body frame**.
- Gravity is constant and aligned with inertial Z.
- No atmospheric drag, engine plume effects, or mass loss (unless explicitly added).

---

## 🧪 Used in

- Aerospace simulation software (e.g., NASA, SpaceX, ESA)
- Game engines and robotics
- Drones, spacecraft, and missile simulators

---

## 🧩 Example in Code

```cpp
Eigen::Quaterniond q(x(6), x(7), x(8), x(9));
Eigen::Vector3d omega(x(10), x(11), x(12));

Eigen::Quaterniond omega_q(0, omega.x(), omega.y(), omega.z());
Eigen::Quaterniond q_dot = 0.5 * q * omega_q;
```

This evolves the orientation over time using quaternion multiplication.