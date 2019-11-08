import argparse
import matplotlib
# matplotlib.use('Qt4Agg')
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
from sklearn.decomposition import PCA
from mpl_toolkits.mplot3d import Axes3D


arg_parser = argparse.ArgumentParser(description="Plots the trajectory of a ORB-SLAM2 session.",
                                     formatter_class=argparse.RawTextHelpFormatter)
arg_parser.add_argument("input_path", help="Path to KeyFrameTrajectory.txt")
arg_parser.add_argument('-pca', '--pca', action='store_true', help="Creates 2D plot from 3D"
        + "trajectory. WARNING: This only makes sense if the trajectory was actually 2D!")

args       = arg_parser.parse_args()
input_path = args.input_path
use_pca    = args.pca

# check if input is a file
if not os.path.isfile(input_path):
    print("Input needs to be a log file. \"{}\" is not a file.".format(input_path))

# extract position from input file
with open(input_path, 'r') as file:
    content = file.read()

lines     = content.splitlines()
positions = [row[1:4] for row in [line.split() for line in lines]]
x_coords  = [float(pos[0]) for pos in positions]
y_coords  = [float(pos[1]) for pos in positions]
z_coords  = [float(pos[2]) for pos in positions]
time      = range(len(x_coords))

# create the plot, either 2D or 3D based on use_pca
if use_pca:
    A     = np.array((x_coords, y_coords, z_coords))
    pca   = PCA(n_components=2, svd_solver='full')
    A_pca = pca.fit_transform(np.transpose(A))
    A_pca = np.transpose(A_pca)
    fig   = plt.figure()
    ax    = fig.gca()

    scatter = ax.scatter(A_pca[0], A_pca[1], c=time, cmap='jet')
else:
    fig = plt.figure()
    ax  = fig.gca(projection='3d')

    ax._axis3don = False
    scatter = ax.scatter(x_coords, y_coords, z_coords, c=time, cmap='jet')

plt.colorbar(scatter)
plt.show()
plt.savefig("trajectory.pdf")
