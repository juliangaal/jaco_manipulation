from analyse import ResultPlotter
import os

if __name__ == "__main__":
    file = '../baseline_test_recording.csv'
    print '\nAnalysing', os.path.basename(file)
    plotter = ResultPlotter(file, ['Time', 'Type', 'CurrentPose', 'TargetPose', 'Result'], delimiter=';', template=True)
    plotter.save3DResultFrom('TargetPose')