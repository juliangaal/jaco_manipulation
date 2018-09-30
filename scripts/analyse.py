import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd


class Point:
    def __init__(self, x, y, z, result):
        self.x = x
        self.y = y
        self.z = z
        self.success = result

    def __str__(self):
        return "({} {} {})".format(self.x, self.y, self.z)


class ResultPlotter:
    def __init__(self, file, labels, delimiter=','):
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.file = self.current_dir + '/' + file
        self.figure_path = self.current_dir + '/fig.png'
        self.labels = labels
        self.delimiter = delimiter
        self.points = []
        self.df = pd.read_csv(file, names=self.labels, sep=self.delimiter)

    def __del__(self):
        print "Generated figure with", len(self.points), "data points saved to:", self.figure_path

    def __extract_point(self, key):
        data = self.df[key]
        results = self.df['Result']

        for d, r in zip(data, results):
            if d == key or r == "Result":
                continue

            point, _ = d.split('/')
            point = point.replace('(', '').replace(')', '')
            x, y, z = point.split(',')
            self.points.append(Point(x, y, z, True if r == 'success' else False))

    def saveResultFrom(self, key):
        self.__extract_point(key)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        X = [float(p.x) for p in self.points if p.success]
        Y = [float(p.y) for p in self.points if p.success]
        Z = [float(p.z) for p in self.points if p.success]

        ax.scatter(X, Y, Z, c='g', marker='o')

        X = [float(p.x) for p in self.points if not p.success]
        Y = [float(p.y) for p in self.points if not p.success]
        Z = [float(p.z) for p in self.points if not p.success]

        ax.scatter(X, Y, Z, c='r', marker='o')

        # in visualization, x and y axis are flipped
        ax.set_xlim3d(0.2, 0.7)
        ax.set_ylim3d(0.0, 0.58)
        ax.set_zlim3d(0.15, 0.3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.savefig('fig.png', dpi=300)