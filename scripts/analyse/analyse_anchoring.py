from analyse import ResultPlotter

plotter = ResultPlotter('results/anchoring/combined/anchoring_test_combined_recording.csv', ['Time', 'Type', 'CurrentPose', 'TargetPose', 'Result', 'Gripped'], ';')
plotter.save3DResultFrom('TargetPose','Gripped')
plotter.save2DResultFrom('TargetPose','Gripped')