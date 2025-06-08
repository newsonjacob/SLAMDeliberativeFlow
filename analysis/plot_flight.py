import os
import math
import pandas as pd
import matplotlib.pyplot as plt


def main():
    csv_path = os.path.join(os.path.dirname(__file__), '..', 'logs', 'flight_log.csv')
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots()
    ax.plot(df["x"], df["y"], marker="o")
    ax.scatter(df["x"].iloc[0], df["y"].iloc[0], c="green", label="Start")
    ax.scatter(df["x"].iloc[-1], df["y"].iloc[-1], c="red", label="End")

    draw_heading(ax, df)

    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    ax.set_title("Drone Flight Path")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    plt.show()


def draw_heading(ax, df, step: int = 10, length: float = 0.5) -> None:
    """Overlay heading arrows along the path."""
    if "yaw" not in df.columns:
        return

    for i in range(0, len(df), step):
        x = df["x"].iloc[i]
        y = df["y"].iloc[i]
        yaw = df["yaw"].iloc[i]
        dx = length * math.cos(yaw)
        dy = length * math.sin(yaw)
        ax.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1,
                 fc="orange", ec="orange")


if __name__ == "__main__":
    main()
