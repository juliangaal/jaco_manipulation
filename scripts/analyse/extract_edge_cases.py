import pandas as pd

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def str(self):
        return "{},{},{},0.06,0.06,0.06\n".format(self.x, self.y, self.z)


def extract_points(df, points, result_key = "TargetPose", grip_result_key = "Gripped"):
    data = df[result_key]
    results = df[grip_result_key]

    for d, r in zip(data, results):
        if d == result_key or r == grip_result_key:
            continue

        if r == 'kinda' or r == 'failure':
            point, _ = d.split('/')
            point = point.replace('(', '').replace(')', '')
            x, y, z = point.split(',')
            points.append(Point(x,y,z))

if __name__ == "__main__":
    points = []
    df = pd.read_csv("../results/anchoring/combined/combined_anchoring_test_recording.csv", sep=";")
    extract_points(df, points)

    with open("../edge_case_poses.csv", "w") as f:
        f.write("x,y,z,dim_x,dim_y,dim_z\n")
        for point in points:
            f.write(point.str())
