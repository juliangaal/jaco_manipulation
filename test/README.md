# How to run these tests

# Rule #1
**Always keep a finger on the power button on Jaco**. Movements can be fast from time to time, and error can occur at any time, though very rarely. The first things to break are Jacos fingers...

### 1. client\_grasp\_test.cpp
* *Use Case*: provides simple interface to Jaco arm to test predefined poses and bounding boxes 
* *Requirements*: `jaco_manipulation_server` needs to be running: `roslaunch jaco_manipulation jaco_manipulation`. If you want rviz visualization, use parameter `use_rviz:=true/false`
* *Run*: `rosrun jaco_manipulation client_grasp_test`
* *Evaluation*: your eyes

### 2. client\_test.cpp
* *Use Case*: provides simple interface to Jaco arm to test predefined poses, joint states and goals defined in MoveIt config
* *Requirements*: `jaco_manipulation_server` needs to be running: `roslaunch jaco_manipulation jaco_manipulation`. If you want rviz visualization, use parameter `use_rviz:=true/false`
* *Run*: `rosrun jaco_manipulation client_test`
* *Evaluation*: your eyes

### 3. anchoring_test.cpp
* *Use Case*: continous test of grasping object detected by anchoring system. An object is gripped 100 times and dropped off at random positions
* *Requirements*: [anchoring]() TODO installed, kinect driver installed, caffe installed. Kinect sensor mounted. A couple of ROS nodes needs to be started. Putting them all into one launch file really generated too much output in the terminal, so start seperately
	* generate random drop poses in `<node_root>/scripts/generate` with `python generate_anchoring_poses.py`. Default: 100 drop poses. Change it in the file if needed
	*  `jaco_manipulation_server` needs to be running: `roslaunch jaco_manipulation jaco_manipulation`.
	*  launch anchoring system by typing `roslaunch anchoring anchoring_static_tf.launch`
*  *Run*: launch anchoring test and recorder `roslaunch jaco_manipulation anchoring_test.launch`. The recorder will listen to messages sent by `jaco_manipulation_server` and record them in `<node_root>/scripts/anchoring_test_recording.csv`
*  *Evaluation*: 
	* *Record successful/failed grips on paper!*. Choose 'success' for a successful grip (green points in plot, see below), 'failure' for an unsuccessful grip (red points in plot, see below), and 'kinda' for everything else (yellow points in plot, see below), e.g. grapsed correctly, but not strong enough, or touched object on it;s way to grasp position. Adjust the values in csv column 'Gripped' in `<node_root>/scripts/anchoring_test_recording.csv` in your favorite text editor to reflect the ones you recorded. Default value: 'Default'. **Be very specific to avoid typos. the script to analyse detects 'Default' values, but not typos**
	* You can generate a 3d or 2d plot from the recorded data. Save `<node_root>/scripts/anchoring_test_recording.csv` to a new folder `<node_root>/scripts/results/<some_folder>/<descriptor>_anchoring_test_recording.csv`
		* `<some_folder>`: a folder name of your choice
		* `<descriptor>`: a descriptor of the test. e.g. `01`. **This can not include an underscore! The descriptor is used to generate the plot. E.g. for descriptor `01` and file `01_anchoring_test_recording.csv` will generate a plot named `01_fig.png`**
		* add relative path of the just created file to `anchoring_recordings.txt`
		* run `python analyse_anchoring.py` in `<node_root>/scripts/analyse/`. This generates figures for all tests. 
        * **If you want to analyse individual files, adjust the path in `analyse_anchoring_template.py`**. Plot will be generated in `<node_root>/scripts/results/anchoring/`
		* All figures will be generated to `<node_root>/scripts/results/<some_folder>/<descriptor>_<mode>_fig.png>`, where `<mode>` is either '3d' for the 3d plot option and '2d' for the 3d plot option

### 4. baseline_test.cpp
* *Use Case*: continous test of grasping object. An object is gripped 50 times and dropped off at random positions 50 times as well. Grasping and dropping alternate.
* *Requirements*: `jaco_manipulation_server` needs to be running: `roslaunch jaco_manipulation jaco_manipulation`. If you want rviz visualization, use parameter `use_rviz:=true/false`
*  *Run*: launch anchoring test and recorder `roslaunch jaco_manipulation baseline_test.launch`. The recorder will listen to messages sent by `jaco_manipulation_server` and record them in `<node_root>/scripts/baseline_test_recording.csv`
*  *Evaluation*: You can generate a 3d or 2d plot from the recorded data. Save `<node_root>/scripts/baseline_test_recording.csv` to a new folder `<node_root>/scripts/results/<some_folder>/<descriptor>_baseline_test_recording.csv`
	* `<some_folder>`: a folder name of your choice
	* `<descriptor>`: a descriptor of the test. e.g. `01`. **This can not include an underscore! The descriptor is used to generate the plot. E.g. for descriptor `01` and file `01_baseline_test_recording.csv` will generate a plot named `01_fig.png`**
	* add relative path of the just created file to `baseline_recordings.txt`
	* run `python analyse_baseline.py` in `<node_root>/scripts/analyse/`. This generates figures for all tests. 
	* **If you want to analyse individual files, adjust the path in `analyse_baseline_template.py`**. Plot will be generated in `<node_root>/scripts/results/baseline/`
	* All figures will be generated to `<node_root>/scripts/results/<some_folder>/<descriptor>_<mode>_fig.png>`, where `<mode>` is either '3d' for the 3d plot option and '2d' for the 3d plot option. Red points in the plot correspond to failed attempts, while green points refer to the successful ones.	
 
