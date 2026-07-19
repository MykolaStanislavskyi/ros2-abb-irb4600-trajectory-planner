import csv
import matplotlib.pyplot as plt

LOG_PATH = "/home/user/robotics_ws/trajectory_log.csv"
OUT_DIR = "/home/user/robotics_ws"

samples = []
motions = []

q = {i: [] for i in range(1, 7)}
dq = {i: [] for i in range(1, 7)}
ddq = {i: [] for i in range(1, 7)}

with open(LOG_PATH, "r") as f:
    reader = csv.DictReader(f)

    for idx, row in enumerate(reader):
        samples.append(idx)
        motions.append(row["motion"])

        for i in range(1, 7):
            q[i].append(float(row[f"j{i}"]))
            dq[i].append(float(row[f"dj{i}"]))
            ddq[i].append(float(row[f"ddj{i}"]))


def add_segment_lines(ax):
    """Додає вертикальні лінії на межах motion-сегментів."""
    if not motions:
        return

    last_motion = motions[0]

    for idx, motion in enumerate(motions):
        if motion != last_motion:
            ax.axvline(x=idx, linestyle="--", linewidth=0.8)
            ax.text(
                idx,
                ax.get_ylim()[1],
                motion,
                rotation=90,
                verticalalignment="top",
                fontsize=8,
            )
            last_motion = motion


for joint in range(1, 7):
    fig, ax = plt.subplots(figsize=(14, 7))

    ax.plot(samples, q[joint], label="position [rad]")
    ax.plot(samples, dq[joint], label="velocity [rad/s]")
    ax.plot(samples, ddq[joint], label="acceleration [rad/s²]")

    ax.set_title(f"Full trajectory - Joint {joint}")
    ax.set_xlabel("sample")
    ax.set_ylabel("value")
    ax.grid(True)
    ax.legend()

    add_segment_lines(ax)

    fig.tight_layout()

    output_path = f"{OUT_DIR}/joint{joint}_full.png"
    fig.savefig(output_path)
    plt.close(fig)

    print(f"Saved: {output_path}")