from matplotlib import pyplot as plt
import csv
import numpy as np


def main():
    """
    Plots Torque, Position, and Velocity against Time
    :return: 3 Plots
    """
    data = np.genfromtxt("test.csv", delimiter=",", names=["time", "pos", "vel", "torque"])
    plt.plot(data['time'], data['torque'])
    plt.legend()
    plt.title("Time - Torque")
    plt.show()
    print("plot1 generated.")

    plt.plot(data['time'], data['pos'])
    plt.legend()
    plt.title("Time - Position")
    plt.show()
    print("plot2 generated.")

    plt.plot(data['time'], data['vel'])
    plt.legend()
    plt.title("Time - Velocity")
    plt.show()
    print("plot3 generated.")


main()
