import sys
import os
import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Add path to compiled openpid module
build_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'build'))
sys.path.append(build_path)

import openpid

# Parameters from Processing sketch
g = 9.81
M = 0.5
m = 0.2
b = 0.1
I = 0.006
l = 0.3
dt = 0.05

kp = 10
ki = 10
kd = 2

# System + controller
pend = openpid.PendulumOnCart(M, m, l, b, I, g)
pend.reset(x0=0, v0=0.0, theta0=math.pi + 0.01, omega0=0.2)
pid = openpid.PID(kp, ki, kd)
pid.reset()

r = math.pi
steps = 1000
times, angles, positions, forces = [], [], [], []

for i in range(steps):
    t = i * dt
    theta = pend.get_angle()
    F = pid.compute_recursive(r, theta, dt)
    pend.update(F, dt)

    times.append(t)
    angles.append(theta)
    positions.append(pend.get_position())
    forces.append(F)

# Layout with 2 columns: plots left, animation right
fig = make_subplots(
    rows=2, cols=2,
    column_widths=[0.5, 0.5],
    specs=[[{"type": "xy"}, {"type": "xy", "rowspan": 2}],
           [{"type": "xy"}, None]],
    subplot_titles=("Pendulum Angle (rad)", "Animation", "Control Force (N)")
)

# Add traces
fig.add_trace(go.Scatter(x=[], y=[], name="Angle", mode="lines"), row=1, col=1)
fig.add_trace(go.Scatter(x=[], y=[], name="Force", mode="lines"), row=2, col=1)

# Pendulum line
pend_line = go.Scatter(x=[], y=[], mode="lines+markers",
                       line=dict(width=4, color="black"),
                       marker=dict(size=10, color="blue"),
                       name="Pendulum", showlegend=False)

# Cart rectangle as a filled polygon
cart_rect = go.Scatter(x=[], y=[], fill='toself', mode='lines',
                       line=dict(color="gray"),
                       fillcolor='lightgray',
                       name="Cart", showlegend=False)

fig.add_trace(pend_line, row=1, col=2)
fig.add_trace(cart_rect, row=1, col=2)

# Constants for visual
cart_width = 0.4
cart_height = 0.2

# Create animation frames
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
    height=600,
    title="Inverted Pendulum with Recursive PID: Plots + Animation",
    showlegend=False
)

# Axes
fig.update_yaxes(title_text="Angle (rad)", row=1, col=1)
fig.update_yaxes(title_text="Force (N)", row=2, col=1)
fig.update_xaxes(title_text="Time (s)", row=2, col=1)
fig.update_xaxes(title_text="Position (m)", row=1, col=2, range=[-2, 2])
fig.update_yaxes(title_text="Height (m)", row=1, col=2, range=[-1, 0.5])

fig.show()
