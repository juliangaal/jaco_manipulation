import pandas as pd
from collections import defaultdict
import os

file_descriptions = {'../results/baseline/01/01_baseline_test_recording.csv': 'Result',
                    '../results/baseline/02/02_baseline_test_recording.csv': 'Result',
                    '../results/baseline/combined/combined_baseline_test_recording.csv': 'Result',
                    '../results/anchoring/01/01_anchoring_test_recording.csv': 'Gripped',
                    '../results/anchoring/02/02_anchoring_test_recording.csv': 'Gripped',
                    '../results/anchoring/combined/combined_anchoring_test_recording.csv': 'Gripped'}


def create_summary(output_file, name, result, trials):
    space = '  '
    with open(output_file, "a") as f:
        f.write("---\n" + name + "\n")
        for k, v in result.items():
            f.write(space + str(k) + ": " + str((float(v)/trials)*100.0) + "%\n")

        f.write(space + "Total grasp trials: " + str(trials) + "\n")


if __name__ == "__main__":
    outputfile = "stats.txt"
    try:
        os.remove(outputfile)
    except OSError:
        pass

    for filename, target_label in file_descriptions.items():
        total_trials = 0

        df = pd.read_csv(filename, sep=';')
        results = defaultdict(lambda: 0)

        for test_type, result in zip(df['Type'], df[target_label]):
            if test_type.strip('\n') != 'grasp_pose':
                continue

            results[result] += 1
            total_trials += 1

        create_summary(outputfile, os.path.basename(filename), results, total_trials)
        print "Analysed", filename
