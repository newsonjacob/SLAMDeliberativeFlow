import os
import pandas as pd
import matplotlib.pyplot as plt


def main():
    csv_path = os.path.join(os.path.dirname(__file__), '..', 'logs', 'flight_log.csv')
    df = pd.read_csv(csv_path)

    plt.plot(df["x"], df["y"], marker="o")
    plt.scatter(df["x"].iloc[0], df["y"].iloc[0], c="green", label="Start")
    plt.scatter(df["x"].iloc[-1], df["y"].iloc[-1], c="red", label="End")
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.title("Drone Flight Path")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
