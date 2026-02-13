#!/usr/bin/env python3

from glob import glob
from sys import argv

from matplotlib import animation
from matplotlib import pyplot as plt


def get_configs(file_path):
    configs = dict()
    with open(file_path, "r") as f:
        header = f.readline()
        header = header.replace("#", "").strip().split(";")[:-1]
        for pair in header:
            key, val = pair.split("=")
            configs[key] = val
    return configs


def plot_trees(file_path):
    tree_x = []
    tree_y = []
    sizes = []
    with open(file_path, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            tmp = line.split(",")
            tree_x.append(-(float(tmp[0]) - 42))
            tree_y.append(float(tmp[1]))
            sizes.append(int(float(tmp[3]) * 10))
    plt.scatter(tree_x, tree_y, c="green", s=sizes, marker="^", zorder=5)


def plot_positions(x, y, bool_array, **kwargs):
    x_tmp = []
    y_tmp = []
    for i in range(len(x)):
        if bool_array[i]:
            x_tmp.append(x[i])
            y_tmp.append(y[i])
    plt.scatter(x_tmp, y_tmp, **kwargs)


def plot_robot_data(file_path):
    x = []
    y = []
    # yaw = []
    collided = []
    out_of_bounds = []
    with open(file_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith("#"):
                continue
            tmp = line.split(",")
            x.append(float(tmp[0]))
            y.append(float(tmp[1]))
            # yaw.append(float(tmp[2]))
            collided.append(bool(int(tmp[3])))
            out_of_bounds.append(bool(int(tmp[4])))
    plot_positions(x, y, collided, c="lightgrey", s=500, marker="o", zorder=0)
    plot_positions(x, y, collided, c="red", s=50, marker="x", zorder=7)
    plot_positions(x, y, out_of_bounds, c="steelblue", s=1, marker=",", zorder=3)
    nothing = [
        True if not collided[i] and not out_of_bounds[i] else False
        for i in range(len(x))
    ]
    plot_positions(x, y, nothing, c="black", s=1, marker=",", zorder=3)


def show_plot(robot_data_path, tree_data_path):
    plot_trees(tree_data_path)
    plot_robot_data(robot_data_path)
    plt.xlim(-25, 25)
    plt.ylim(-25, 25)
    plt.show()


def anim_plot(robot_data_path, tree_data_path):
    def animate(_):
        plot_trees(tree_data_path)
        plot_robot_data(robot_data_path)

    fig, ax = plt.subplots()
    ax.set_xlim([-25, 25])
    ax.set_ylim([-25, 25])
    anim = animation.FuncAnimation(fig, animate, repeat=True, interval=1000)
    configs = get_configs(robot_data_path)
    title = f"SEED={configs['RRFM_SEED']}   "
    title += f"ALWAYS_RANDOM_ROTATION={configs['RRFM_ALWAYS_RANDOM_ROTATION']}   "
    title += f"DURATION={configs['RRFM_TTL']}s   "
    plt.title(title)
    plt.show()
    return anim


def main(args=None):
    """
    command line args:
        animate    ... to update plot live (optional)
        <filepath> ... to plot specific file (optional)
    If no args are provided the last file in data folder
    will be plotted.
    """
    animate = True
    robot_data_path = sorted(glob("data/recored_positions_*_*.csv"))[-1]
    if args and len(args) >= 2:
        args = args[1:]
        for i in range(len(args)):
            if args[i] == "animate":
                animate = True
            else:
                robot_data_path = args[i]
    tree_data_path = "install/ros_gz_lehrforst_sim/share/ros_gz_lehrforst_sim/models/lehrforst_trees/tree_position_filtered.csv"
    if animate:
        anim_plot(robot_data_path, tree_data_path)
    else:
        show_plot(robot_data_path, tree_data_path)


if __name__ == "__main__":
    main(argv)
