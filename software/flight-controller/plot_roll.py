import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


csv_path = Path(sys.argv[1])

with csv_path.open(newline="") as file:
    rows = list(csv.DictReader(file))

if not rows:
    raise SystemExit("The CSV contains no samples.")


# ------------------------------------------------------------
# Read simulation data
# ------------------------------------------------------------

time = [float(row["time"]) for row in rows]

time = [float(row["time"]) for row in rows]

true_roll = [float(row["true_roll"]) for row in rows]
estimated_roll = [float(row["estimated_roll"]) for row in rows]
target_roll = [float(row["target_roll"]) for row in rows]
roll_control = [float(row["control"]) for row in rows]

true_pitch = [float(row["true_pitch"]) for row in rows]
estimated_pitch = [float(row["estimated_pitch"]) for row in rows]
target_pitch = [float(row["target_pitch"]) for row in rows]
pitch_control = [float(row["pitch_control"]) for row in rows]

true_yaw = [float(row["true_yaw"]) for row in rows]
true_yaw_rate = [float(row["true_yaw_rate"]) for row in rows]
target_yaw_rate = [float(row["target_yaw_rate"]) for row in rows]
yaw_control = [float(row["yaw_control"]) for row in rows]


# ============================================================
# Roll graph
# ============================================================

fig, (angles, output) = plt.subplots(
    2,
    1,
    sharex=True,
    figsize=(10, 7),
)

angles.plot(
    time,
    true_roll,
    label="True roll",
)

angles.plot(
    time,
    estimated_roll,
    "--",
    label="Estimated roll",
)

angles.plot(
    time,
    target_roll,
    ":",
    color="black",
    label="Target",
)

angles.fill_between(
    time,
    [x - 1 for x in target_roll],
    [x + 1 for x in target_roll],
    color="green",
    alpha=0.12,
    label="Target +/- 1 degree",
)

angles.set_ylabel("Roll (degrees)")
angles.set_title("Simulated roll control")
angles.legend()

output.plot(
    time,
    roll_control,
    color="tab:orange",
    label="Control",
)

output.axhline(
    100,
    color="gray",
    linestyle=":",
)

output.axhline(
    -100,
    color="gray",
    linestyle=":",
)

output.set_ylabel("Control (simulation units)")
output.set_xlabel("Time (seconds)")
output.legend()

for axis in (angles, output):
    axis.grid(True, alpha=0.3)

fig.tight_layout()

roll_image_path = csv_path.with_suffix(".png")

fig.savefig(
    roll_image_path,
    dpi=160,
)

plt.close(fig)


# ============================================================
# Yaw graph
# ============================================================

fig, (yaw_angle_plot, yaw_rate_plot, yaw_output_plot) = plt.subplots(
    3,
    1,
    sharex=True,
    figsize=(10, 9),
)


# ------------------------------------------------------------
# Absolute simulated yaw angle
# ------------------------------------------------------------

yaw_angle_plot.plot(
    time,
    true_yaw,
    label="True yaw",
)

yaw_angle_plot.set_ylabel("Yaw angle (degrees)")
yaw_angle_plot.set_title("Simulated yaw-rate control")
yaw_angle_plot.legend()


# ------------------------------------------------------------
# Yaw-rate tracking
# ------------------------------------------------------------

yaw_rate_plot.plot(
    time,
    true_yaw_rate,
    label="Actual yaw rate",
)

yaw_rate_plot.plot(
    time,
    target_yaw_rate,
    "--",
    label="Target yaw rate",
)

yaw_rate_plot.set_ylabel("Yaw rate (deg/s)")
yaw_rate_plot.legend()


# ------------------------------------------------------------
# Yaw controller output
# ------------------------------------------------------------

yaw_output_plot.plot(
    time,
    yaw_control,
    label="Yaw control",
)

yaw_output_plot.axhline(
    100,
    color="gray",
    linestyle=":",
)

yaw_output_plot.axhline(
    -100,
    color="gray",
    linestyle=":",
)

yaw_output_plot.set_ylabel("Control")
yaw_output_plot.set_xlabel("Time (seconds)")
yaw_output_plot.legend()


for axis in (
        yaw_angle_plot,
        yaw_rate_plot,
        yaw_output_plot,
):
    axis.grid(True, alpha=0.3)


fig.tight_layout()

yaw_image_path = csv_path.with_name(
    f"{csv_path.stem}_yaw.png"
)

fig.savefig(
    yaw_image_path,
    dpi=160,
)

plt.close(fig)

# ============================================================
# Combined 3-axis control graph
# ============================================================

fig, (
    roll_plot,
    pitch_plot,
    yaw_rate_plot,
) = plt.subplots(
    3,
    1,
    sharex=True,
    figsize=(11, 9),
)


# ------------------------------------------------------------
# Roll
# ------------------------------------------------------------

roll_plot.plot(
    time,
    true_roll,
    label="True roll",
)

roll_plot.plot(
    time,
    estimated_roll,
    "--",
    label="Estimated roll",
)

roll_plot.plot(
    time,
    target_roll,
    ":",
    label="Target roll",
)

roll_plot.axhline(
    0.0,
    linewidth=0.8,
)

roll_plot.set_ylabel("Roll (deg)")
roll_plot.set_title("3-axis flight-control response")
roll_plot.legend()
roll_plot.grid(True, alpha=0.3)


# ------------------------------------------------------------
# Pitch
# ------------------------------------------------------------

pitch_plot.plot(
    time,
    true_pitch,
    label="True pitch",
)

pitch_plot.plot(
    time,
    estimated_pitch,
    "--",
    label="Estimated pitch",
)

pitch_plot.plot(
    time,
    target_pitch,
    ":",
    label="Target pitch",
)

pitch_plot.axhline(
    0.0,
    linewidth=0.8,
)

pitch_plot.set_ylabel("Pitch (deg)")
pitch_plot.legend()
pitch_plot.grid(True, alpha=0.3)


# ------------------------------------------------------------
# Yaw rate
# ------------------------------------------------------------

yaw_rate_plot.plot(
    time,
    true_yaw_rate,
    label="Actual yaw rate",
)

yaw_rate_plot.plot(
    time,
    target_yaw_rate,
    "--",
    label="Target yaw rate",
)

yaw_rate_plot.axhline(
    0.0,
    linewidth=0.8,
)

yaw_rate_plot.set_ylabel("Yaw rate (deg/s)")
yaw_rate_plot.set_xlabel("Time (seconds)")
yaw_rate_plot.legend()
yaw_rate_plot.grid(True, alpha=0.3)


fig.tight_layout()

three_axis_image_path = csv_path.with_name(
    f"{csv_path.stem}_3axis.png"
)

fig.savefig(
    three_axis_image_path,
    dpi=160,
)

plt.close(fig)


# ============================================================
# Summary
# ============================================================

print(f"Saved {roll_image_path}")
print(f"Saved {yaw_image_path}")
print(f"Saved {three_axis_image_path}")

print(
    f"Final true roll: "
    f"{true_roll[-1]:.3f} degrees"
)

print(
    f"Final estimate error: "
    f"{estimated_roll[-1] - true_roll[-1]:.3f} degrees"
)

print(
    f"Final yaw angle: "
    f"{true_yaw[-1]:.3f} degrees"
)

print(
    f"Final yaw rate: "
    f"{true_yaw_rate[-1]:.3f} deg/s"
)

print(
    f"Final yaw-rate error: "
    f"{true_yaw_rate[-1] - target_yaw_rate[-1]:.3f} deg/s"
)