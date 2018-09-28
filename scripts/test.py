#!/usr/bin/env python3


class Test(object):
    def __init__(self, filename):
        self.filename = filename
        self.__setupLogFile()

    def __setupLogFile(self):
        self.file = open(self.filename, "w")
        self.file.write("Time,CurrentPose,TargetPose,Result")

    def __del__(self):
        self.file.close()
        print("Logged to", self.filename)


class BaseLineTest(Test):
    def __init__(self, filename):
        Test.__init__(self, filename)


test = BaseLineTest("baseline.csv")
