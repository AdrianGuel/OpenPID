import sys
import os
import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import imageio
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

# Full-state feedback gains
k1 = -1.0000
k2 = -1.6567
k3 = 18.6854
k4 = 3.4594

# System + controller
pend = openpid.PendulumOnCart(M, m, l, b, I, g)
pend.reset(x0=1.5, v0=0.0, theta0=math.pi + 0.01, omega0=0.2)
sf = openpid.StateFeedback(k1, k2, k3, k4)
sf.set_reference(math.pi)

steps = 160
times, angles, positions, forces = [], [], [], []

for i in range(steps):
    t = i * dt
    state = [pend.get_position(), pend.get_velocity(), pend.get_angle(), pend.get_angular_velocity()]
    F = sf.compute(state)
    pend.update(F, dt)

    times.append(t)
    angles.append(state[2])
    positions.append(state[0])
    forces.append(F)

# Layout: plots (left) + animation (right)
fig = make_subplots(
    rows=2, cols=2,
    column_widths=[0.5, 0.5],
    specs=[[{"type": "xy"}, {"type": "xy", "rowspan": 2}],
           [{"type": "xy"}, None]],
    subplot_titles=("Pendulum Angle (rad)", "Animation", "Control Force (N)")
)

# Static traces
fig.add_trace(go.Scatter(x=[], y=[], name="Angle", mode="lines"), row=1, col=1)
fig.add_trace(go.Scatter(x=[], y=[], name="Force", mode="lines"), row=2, col=1)
fig.add_trace(go.Scatter(x=[], y=[], mode="lines+markers", line=dict(width=4), marker=dict(size=10)), row=1, col=2)
fig.add_trace(go.Scatter(x=[], y=[], fill='toself', mode='lines', line=dict(color="gray"), fillcolor='lightgray'), row=1, col=2)

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
            xaxis=dict(range=[x_min, x_max], anchor="y"),
            yaxis=dict(range=[angle_min - 0.1, angle_max + 0.1], anchor="x"),
            xaxis2=dict(range=[x_min, x_max], anchor="y2"),
            yaxis2=dict(range=[force_min - 1, force_max + 1], anchor="x2"),
            xaxis3=dict(range=[-2, 2], anchor="y3"),
            yaxis3=dict(range=[-2, 2], anchor="x3")
        ),
        name=str(i)
    ))

fig.frames = frames

# Layout
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
    height=700,
    title="Inverted Pendulum with Full-State Feedback: Plots + Animated Cart",
    showlegend=False
)

# Axes
fig.update_yaxes(title_text="Angle (rad)", row=1, col=1)
fig.update_yaxes(title_text="Force (N)", row=2, col=1)
fig.update_xaxes(title_text="Time (s)", row=2, col=1)
fig.update_xaxes(title_text="Position (m)", row=1, col=2, range=[-2, 2])
fig.update_yaxes(title_text="Height (m)", row=1, col=2, range=[-2, 2])

fig.show()

os.makedirs("gif_frames", exist_ok=True)
gif_path = "pendulum_animation.gif"

for i, frame in enumerate(fig.frames):
    fig.update(data=frame.data, layout=frame.layout)
    fig.write_image(f"gif_frames/frame_{i:04d}.png", width=800, height=600)

with imageio.get_writer(gif_path, mode='I', duration=0.01) as writer:
    for i in range(len(fig.frames)):
        filename = f"gif_frames/frame_{i:04d}.png"
        image = imageio.imread(filename)
        writer.append_data(image)

print(f"✅ GIF saved to: {gif_path}")