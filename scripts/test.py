import os

class Test:
    def __init__(self, filename, labels='empty labels'):
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.filename = self.dir_path + '/' + filename
        self.labels = labels
        self.__setup_log_file()

    def __setup_log_file(self):
        self.file = open(self.filename, "w")
        self.write(self.labels)

    def write(self, line):
        self.file.write(line + '\n')

    def pose_to_string(self, pose):
        return "({x},{y},{z})/({xx},{yy},{zz},{ww})".format(
            x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z,
            xx=pose.pose.orientation.x, yy=pose.pose.orientation.y,
            zz=pose.pose.orientation.z, ww=pose.pose.orientation.w
        )

    def __del__(self):
        self.file.close()
        print "Logged to", self.filename