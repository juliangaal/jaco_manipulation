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

class AnchorPoint:
    def __init__(self, x, y, z, result):
        self.x = x
        self.y = y
        self.z = z
        self.result = result

    def __str__(self):
        return "({} {} {} {})".format(self.x, self.y, self.z, self.result)

class Color:
    success = 'limegreen'
    failure = 'r'
    kinda = 'yellow'

class ResultPlotter:
    def __init__(self, file, labels, delimiter=','):
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.file = self.current_dir + '/' + file
        self.figure_path_3d = self.current_dir + '/3d_fig.png'
        self.figure_path_2d = self.current_dir + '/2d_fig.png'
        self.labels = labels
        self.delimiter = delimiter
        self.points = []
        self.df = pd.read_csv(file, names=self.labels, sep=self.delimiter)

    def __del__(self):
        print "Done: generated plots"

    def __extract_point(self, key, result_key):
        data = self.df[key]
        results = self.df[result_key]

        for d, r in zip(data, results):
            if d == key or r == result_key:
                continue

            point, _ = d.split('/')
            point = point.replace('(', '').replace(')', '')
            x, y, z = point.split(',')
            if result_key == 'Result':
                self.points.append(Point(x, y, z, True if r == 'success' else False))
            else:
                self.points.append(AnchorPoint(x,y,z,r))

    def save3DResultFrom(self, key, result_key='Result'):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        self.__extract_point(key, result_key)

        if result_key == 'Result':
            X = [float(p.x) for p in self.points if p.success]
            Y = [float(p.y) for p in self.points if p.success]
            Z = [float(p.z) for p in self.points if p.success]
            ax.scatter(X, Y, Z, c=Color.success, marker='o')

            X = [float(p.x) for p in self.points if not p.success]
            Y = [float(p.y) for p in self.points if not p.success]
            Z = [float(p.z) for p in self.points if not p.success]
            ax.scatter(X, Y, Z, c=Color.failure, marker='o')
        else:
            X = [float(p.x) for p in self.points if p.result == 'success']
            Y = [float(p.y) for p in self.points if p.result == 'success']
            Z = [float(p.z) for p in self.points if p.result == 'success']
            ax.scatter(X, Y, Z, c=Color.success, marker='o')

            X = [float(p.x) for p in self.points if p.result == 'failure']
            Y = [float(p.y) for p in self.points if p.result == 'failure']
            Z = [float(p.z) for p in self.points if p.result == 'failure']
            ax.scatter(X, Y, Z, c=Color.failure, marker='o')

            X = [float(p.x) for p in self.points if p.result == 'kinda']
            Y = [float(p.y) for p in self.points if p.result == 'kinda']
            Z = [float(p.z) for p in self.points if p.result == 'kinda']
            ax.scatter(X, Y, Z, c=Color.kinda, marker='o')


        # in visualization, x and y axis are flipped
        ax.set_xlim3d(0.2, 0.7)
        ax.set_ylim3d(0.0, 0.58)
        ax.set_zlim3d(0.15, 0.3)
        ax.set_xlabel('robotic arm          <- X ->          kinect')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.savefig(self.figure_path_3d, dpi=300)
        print "Generated figure with", len(self.points), "data points saved to:", self.figure_path_3d

    def save2DResultFrom(self, key, result_key='Result'):
        plt.figure()
        plt.ylabel('Y')
        plt.xlabel('robotic arm          <- X ->          kinect')

        if not self.points:
            self.__extract_point(key, result_key)

        if result_key == 'Result':
            X = [float(p.x) for p in self.points if p.success]
            Y = [float(p.y) for p in self.points if p.success]
            plt.scatter(X, Y, marker='o', c=Color.success)

            X = [float(p.x) for p in self.points if not p.success]
            Y = [float(p.y) for p in self.points if not p.success]
            plt.scatter(X, Y, marker='o', c=Color.failure)
        else:
            X = [float(p.x) for p in self.points if p.result == 'success']
            Y = [float(p.y) for p in self.points if p.result == 'success']
            plt.scatter(X, Y, marker='o', c=Color.success)

            X = [float(p.x) for p in self.points if p.result == 'failure']
            Y = [float(p.y) for p in self.points if p.result == 'failure']
            plt.scatter(X, Y, marker='o', c=Color.failure)

            X = [float(p.x) for p in self.points if p.result == 'kinda']
            Y = [float(p.y) for p in self.points if p.result == 'kinda']
            plt.scatter(X, Y, marker='o', c=Color.kinda)


        plt.savefig(self.figure_path_2d, dpi=300)
        print "Generated figure with", len(self.points), "data points saved to:", self.figure_path_2d
