import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

csv_path = Path(sys.argv[1])

with csv_path.open(newline="") as file:
    rows = list(csv.DictReader(file))

if not rows:
    raise SystemExit("The CSV contains no samples.")

time = [float(row["time"]) for row in rows]
true_roll = [float(row["true_roll"]) for row in rows]
estimated_roll = [float(row["estimated_roll"]) for row in rows]
target = [float(row["target_roll"]) for row in rows]
control = [float(row["control"]) for row in rows]

fig, (angles, output) = plt.subplots(
    2, 1, sharex=True, figsize=(10, 7)
)

angles.plot(time, true_roll, label="True roll")
angles.plot(time, estimated_roll, "--", label="Estimated roll")
angles.plot(time, target, ":", color="black", label="Target")
angles.fill_between(
    time,
    [x - 1 for x in target],
    [x + 1 for x in target],
    color="green",
    alpha=0.12,
    label="Target +/- 1 degree",
)
angles.set_ylabel("Roll (degrees)")
angles.set_title("Simulated roll control")
angles.legend()

output.plot(time, control, color="tab:orange", label="Control")
output.axhline(100, color="gray", linestyle=":")
output.axhline(-100, color="gray", linestyle=":")
output.set_ylabel("Control (simulation units)")
output.set_xlabel("Time (seconds)")
output.legend()

for axis in (angles, output):
    axis.grid(True, alpha=0.3)

fig.tight_layout()

image_path = csv_path.with_suffix(".png")
fig.savefig(image_path, dpi=160)
plt.close(fig)

print(f"Saved {image_path}")
print(f"Final true roll: {true_roll[-1]:.3f} degrees")
print(
    f"Final estimate error: "
    f"{estimated_roll[-1] - true_roll[-1]:.3f} degrees"
)