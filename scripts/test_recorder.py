#!/usr/bin/env python
# Copyright (C) 2018  Julian Gaal
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#     You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

import os
from analyse import ResultPlotter


class Test:
    def __init__(self, filename, labels='empty labels', ):
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.filename = self.dir_path + '/' + filename
        self.labels = labels
        self.__setup_log_file()

    def __setup_log_file(self):
        self.file = open(self.filename, "w")
        self.write(self.labels)

    def curr_file(self):
        return self.filename

    def write(self, line):
        self.file.write(line + '\n')

    def __del__(self):
        self.file.close()
        print "Logged to", self.filename
