from analyse import ResultPlotter

plotter = ResultPlotter('anchoring_test_recording.csv', ['Time', 'Command', 'Type', 'CurrentPose', 'TargetPose', 'Result'], ';')
plotter.saveResultFrom('TargetPose')