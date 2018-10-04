from analyse import ResultPlotter
from analyse import FileReader
import os

if __name__ == "__main__":
    reader = FileReader('anchoring_recordings.txt')
    for file in reader.files:
        print '\nAnalysing', os.path.basename(file)
        plotter = ResultPlotter(file, ['Time', 'Type', 'CurrentPose', 'TargetPose', 'Result', 'Gripped'], delimiter=';')
        plotter.save3DResultFrom('TargetPose','Gripped')
        plotter.save2DResultFrom('TargetPose','Gripped')