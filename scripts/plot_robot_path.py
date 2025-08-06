from matplotlib import pyplot as plt

tree_x = []
tree_y = []

with open("models/lehrforst_trees/tree_position_filtered.csv", 'r') as f:
    lines = f.readlines()
    for line in lines[1:]:
        tmp = line.split(',')
        tree_x.append(float(tmp[0])-42)
        tree_y.append(float(tmp[1]))

x = []
y = []
c = []

with open("data/recored_positions.csv", "r") as f:
    lines = f.readlines()
    for line in lines:
        tmp = line.split(',')
        x.append(float(tmp[0]))
        y.append(float(tmp[1]))
        c.append(int(tmp[2]))

plt.scatter(x, y, c=['red' if i else 'black' for i in c], s=3)
plt.scatter(tree_x, tree_y, c='green', s=50)
plt.xlim(-30, 30)
plt.ylim(-30, 30)
plt.show()
