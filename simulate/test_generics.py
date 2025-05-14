import sys
import os
import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import imageio
import numpy as np

# Add path to compiled openpid module
build_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'build'))
sys.path.append(build_path)
import openpid

def pendulum_cart_dynamics(x, u):
    # State variables
    pos, vel, theta, omega = x
    F = u[0]

    # Parameters
    M = 0.5     # cart mass
    m = 0.2     # pendulum mass
    l = 0.3     # length to pendulum center of mass
    b = 0.1     # friction coefficient
    I = 0.006   # moment of inertia
    g = 9.81    # gravity

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    denom = (I + m * l**2) * (m + M) - (m**2) * l**2 * cos_theta**2

    # Avoid divide-by-zero (just in case)
    if abs(denom) < 1e-6:
        denom = 1e-6

    a = ((-b * vel + F) * (I + m * l**2) + m * l * sin_theta *
         (omega**2 * (I + m * l**2) + g * m * l * cos_theta)) / denom

    alpha = (2 * l * m * (g * (M + m) * sin_theta +
             cos_theta * (-b * vel + F + omega**2 * l * m * sin_theta))) / \
            (-2 * I * (m + M) - l**2 * m * (m + 2 * M) + l**2 * m**2 * np.cos(2 * theta))

    dx = np.zeros_like(x)
    dx[0] = vel
    dx[1] = a
    dx[2] = omega
    dx[3] = alpha

    return dx.astype(np.float32)

# Define the same full-state feedback control law in Python
def control_fn(x, r):
    return -np.dot(K, x - r)

# Parameters from Processing sketch
g = 9.81
M = 0.5
m = 0.2
b = 0.1
I = 0.006
l = 0.3
dt = 0.05

# Full-state feedback gains
K = np.array([-1.0000, -1.6567, 18.6854, 3.4594], dtype=np.float32)
reference = np.array([0.0, 0.0, math.pi, 0.0], dtype=np.float32)

# pend = openpid.PendulumOnCart(M, m, l, b, I, g)
# pend.reset(x0=1.5, v0=0.0, theta0=math.pi + 0.01, omega0=0.2)
x0 = np.array([1.5, 0.0, math.pi + 0.01, 0.2], dtype=np.float32)
pend = openpid.GenericSystem(pendulum_cart_dynamics, x0)
controller = openpid.GenericController(control_fn)
controller.set_reference(reference)

steps = 160
times, angles, positions, forces = [], [], [], []

for i in range(steps):
    t = i * dt
    state = pend.get_state()
    F = controller.compute(state)
    pend.update(np.array([F], dtype=np.float32), dt)

    times.append(t)
    angles.append(state[2])
    positions.append(state[0])
    forces.append(F)

# Layout: stacked rows
fig = make_subplots(
    rows=3, cols=1,
    row_heights=[0.3, 0.3, 0.4],
    specs=[[{"type": "xy"}],
           [{"type": "xy"}],
           [{"type": "xy"}]],
    subplot_titles=("Pendulum Angle (rad)", "Control Force (N)", "Animation")
)

# Static traces
fig.add_trace(go.Scatter(x=[], y=[], name="Angle", mode="lines"), row=1, col=1)
fig.add_trace(go.Scatter(x=[], y=[], name="Force", mode="lines"), row=2, col=1)
fig.add_trace(go.Scatter(x=[], y=[], mode="lines+markers", line=dict(width=4), marker=dict(size=10)), row=3, col=1)
fig.add_trace(go.Scatter(x=[], y=[], fill='toself', mode='lines', line=dict(color="gray"), fillcolor='lightgray'), row=3, col=1)

# Cart visual constants
cart_width = 0.4
cart_height = 0.2

# Animation frames
frames = []
for i in range(2, steps):
    x = positions[i]
    theta = angles[i]
    px = x + l * math.sin(theta)
    py = -l * math.cos(theta)

    cart_xs = [x - cart_width/2, x + cart_width/2, x + cart_width/2, x - cart_width/2, x - cart_width/2]
    cart_ys = [0, 0, -cart_height, -cart_height, 0]

    angle_y = angles[:i]
    force_y = forces[:i]
    time_x = times[:i]

    angle_min, angle_max = min(angle_y), max(angle_y)
    force_min, force_max = min(force_y), max(force_y)
    x_min, x_max = time_x[0], time_x[-1]

    frames.append(go.Frame(
        data=[
            go.Scatter(x=time_x, y=angle_y),
            go.Scatter(x=time_x, y=force_y),
            go.Scatter(x=[x, px], y=[0, py], mode="lines+markers"),
            go.Scatter(x=cart_xs, y=cart_ys, fill='toself', mode='lines')
        ],
        layout=go.Layout(
            xaxis=dict(range=[x_min, x_max]),
            yaxis=dict(range=[angle_min - 0.1, angle_max + 0.1]),
            xaxis2=dict(range=[x_min, x_max]),
            yaxis2=dict(range=[force_min - 1, force_max + 1])
        ),
        name=str(i)
    ))

fig.frames = frames

fig.update_layout(
    updatemenus=[{
        "type": "buttons",
        "buttons": [{
            "label": "Play",
            "method": "animate",
            "args": [None, {
                "frame": {"duration": 10, "redraw": True},
                "fromcurrent": True,
                "transition": {"duration": 0}
            }]
        }]
    }],
    height=900,
    title="Inverted Pendulum with Full-State Feedback: Plots + Animated Cart",
    showlegend=False,
    xaxis3=dict(range=[-2, 2], anchor="y3", title="Position (m)"),
    yaxis3=dict(range=[-2, 2], anchor="x3", title="Height (m)")
)

# Axes
fig.update_yaxes(title_text="Angle (rad)", row=1, col=1)
fig.update_yaxes(title_text="Force (N)", row=2, col=1)
fig.update_xaxes(title_text="Time (s)", row=2, col=1)
fig.update_xaxes(title_text="Position (m)", row=3, col=1, range=[-2, 2])
fig.update_yaxes(title_text="Height (m)", row=3, col=1, range=[-2, 2])

fig.show()

os.makedirs("gif_frames", exist_ok=True)
gif_path = "pendulum_animation.gif"

for i, frame in enumerate(fig.frames):
    fig.update(data=frame.data, layout=frame.layout)
    fig.write_image(f"gif_frames/frame_{i:04d}.png", width=800, height=900)

with imageio.get_writer(gif_path, mode='I', duration=0.01) as writer:
    for i in range(len(fig.frames)):
        filename = f"gif_frames/frame_{i:04d}.png"
        image = imageio.imread(filename)
        writer.append_data(image)

print(f"GIF saved to: {gif_path}")