from pose_generator import PoseGenerator

# generates 100 poses: 50 gripping, 50 dropping poses
if __name__ == "__main__":
    g = PoseGenerator('baseline_poses.csv', total_poses=100)
    g.generate()