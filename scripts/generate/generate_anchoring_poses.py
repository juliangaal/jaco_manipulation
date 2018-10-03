from pose_generator import PoseGenerator

# generated 50 drop poses for anchoring test
if __name__ == "__main__":
    g = PoseGenerator('anchoring_poses.csv', total_poses=100)
    g.generate()