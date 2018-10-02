from analyse import ResultPlotter

plotter = ResultPlotter('results/anchoring/01/01_anchoring_test_recording.csv', ['Time', 'Type', 'CurrentPose', 'TargetPose', 'Result', 'Gripped'], ';')
plotter.save3DResultFrom('TargetPose','Gripped')
plotter.save2DResultFrom('TargetPose','Gripped')