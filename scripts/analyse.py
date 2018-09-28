import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Point:
	def __init__(self, x, y, z, result):
		self.x = x
		self.y = y
		self.z = z
		self.result = result


class ResultPlotter:
	def __init__(self, file, labels, delimiter=','):
		self.file = file
		self.labels = labels
		self.delimiter = delimiter
		self.points = []
		self.df = pd.read_csv(file, names=self.labels, sep=self.delimiter)
 	
	def __extract_point(self, key):
		data = self.df[key]
		results = self.df['Result']

		for d,r in zip(data,results):
			if d == key or r == key:
				continue
			
			point, _ = d.split('/')
			point = point.replace('(','').replace(')','')
			x, y, z = point.split(',')
			self.points.append(Point(x,y,z,r))			

	def saveResultFrom(self, key):
		self.__extract_point(key)
		
		X = [float(p.x) for p in self.points]
		Y = [float(p.y) for p in self.points]
		Z = [float(p.z) for p in self.points]
		Results = [p.result for p in self.points]

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		x = [1,2,3,4,5,6,7,8,9,10]
		y = [5,6,2,3,13,4,1,2,4,8]
		z = [2,3,3,3,5,7,9,11,9,10]

		ax.scatter(X, Y, Z, c='r', marker='o')

		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')

		plt.show()
		plt.savefig('fig.png')

		
plotter = ResultPlotter('test.csv',['Time','Current Pose','Target Pose','Result'],';')
plotter.saveResultFrom('Target Pose');
