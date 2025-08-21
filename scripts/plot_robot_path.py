#!/usr/bin/env python3

from matplotlib import pyplot as plt
from glob import glob
from sys import argv


# TODO: correct mapping from 3D to 2D space (include z position)


def plot_trees(file_path="models/lehrforst_trees/tree_position_filtered.csv"):
    tree_x = []
    tree_y = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines[1:]:
            tmp = line.split(',')
            tree_x.append(float(tmp[0])-42)
            tree_y.append(float(tmp[1]))
    plt.scatter(tree_x, tree_y, c='green', s=50)


def plot_robot(file_path):
    x = []
    y = []
    # yaw = []
    col = []
    oob = []
    with open(file_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            tmp = line.split(',')
            x.append(float(tmp[0]))
            y.append(float(tmp[1]))
            # yaw.append(float(tmp[2]))
            col.append(int(tmp[3]))
            oob.append(int(tmp[4]))
    dot_colors = []
    sizes = []
    for i in range(len(col)):
        if col[i]:
            c = 'red'
            s = 50
        elif oob[i]:
            c = 'blue'
            s = 10
        else:
            c = 'black'
            s = 5
        dot_colors.append(c)
        sizes.append(s)
    plt.scatter(x, y, c=dot_colors, s=sizes)


def main(args=None):
    if args and len(args) >= 2:
        file_path = args[1]
    else:
        file_path = sorted(glob("data/recored_positions_*_*.csv"))[-1]
    plt.title(file_path)
    #plot_trees()
    plot_robot(file_path)
    plt.xlim(-30, 30)
    plt.ylim(-30, 30)
    plt.show()


if __name__ == '__main__':
    main(argv)