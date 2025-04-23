![image](https://github.com/user-attachments/assets/52681e9f-b44d-404b-b3c7-ba64e3289e18)

📄 **`OpenPID_README.md`**
```markdown
# OpenPID

**OpenPID** is a lightweight and modern PID controller library written in C++ with simulation tools for mass-spring-damper systems. It supports both real-time C++ applications and Python visualization via `pybind11`.

---

## 🚀 Features

- Header-only, templated `PID<T>` controller
- Mass-Spring-Damper simulation class for testing
- CMake-based build system
- Python bindings using `pybind11`
- Interactive animations and plots using Plotly
- Poetry-managed Python environment

---

## 🗂️ Project Structure

```
OpenPID/
├── include/           # Header files (pid.hpp, msd_system.hpp)
├── src/               # (Optional) C++ implementation files
├── examples/          # C++ examples
├── simulate/          # Python simulation scripts
├── tests/             # Unit tests
├── build/             # CMake build output (ignored)
├── pybind11/          # pybind11 submodule
├── bindings.cpp       # C++ ↔ Python interface
├── CMakeLists.txt     # CMake configuration
├── README.md          # This file
└── pyproject.toml     # Poetry config
```

---

## ⚙️ Building the C++ Library & Python Bindings

### 🔧 Prerequisites

- CMake ≥ 3.14
- Python ≥ 3.8
- Poetry (`pip install poetry`)
- `pybind11` as a submodule:

```bash
git submodule update --init --recursive
```

### 🛠️ Build the C++ & Python Module

```bash
mkdir -p build && cd build
cmake ..
make
```

This will produce `openpid*.so`, the Python module.

---

## 🧪 Python Simulation & Visualization

### 🔁 Setup Python Environment (using Poetry)

```bash
poetry install --no-root
```

### ▶️ Run the animated simulation

```bash
poetry run python simulate/simulate.py
```

This launches an animated Plotly graph of the PID-controlled mass-spring-damper system.

---

## 🧩 Example: C++ Usage

```cpp
#include "pid.hpp"
PID<float> pid(1.0f, 0.5f, 0.1f);
float control = pid.compute(1.0f, 0.8f, 0.01f);
```

---

## 📚 License

This project is licensed under the MIT License — see `LICENSE` for details.

---

## ✨ Credits

Created by [Adrián Guel](https://github.com/AdrianGuel). Inspired by the need for intuitive control simulation tools in both research and teaching.
```
