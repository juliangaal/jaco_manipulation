import pandas as pd
import os

filenames = ['baseline/01/01_baseline_test_recording.csv',
            'baseline/02/02_baseline_test_recording.csv',
            'baseline/combined/combined_baseline_test_recording.csv',
            'anchoring/01/01_anchoring_test_recording.csv',
            'anchoring/02/02_anchoring_test_recording.csv',
            'anchoring/combined/combined_anchoring_test_recording.csv']


def create_summary(f, name, result):
    f.write("---\n" + name)
    for k, v in enumerate(result):
        f.write(k + ": " + v + "%")


for filename in filenames:
    outputfile = os.path.basename(filename)
    f = open(outputfile, 'w')

    df = pd.read_csv(filename, sep=';')
    print df['Result']

    create_summary(f, outputfile, {})
    f.close()
