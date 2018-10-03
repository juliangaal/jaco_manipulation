from analyse import ResultPlotter

plotter = ResultPlotter('results/baseline/combined/baseline_combined_recording.csv', ['Time', 'CurrentPose', 'TargetPose', 'Result'], ';')
plotter.saveResultFrom('TargetPose')