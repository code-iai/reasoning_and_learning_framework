# Copyright (c) 2018 Institute for Artificial Intelligence - University of Bremen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE


from grasping_position_inference.inference.model import Model


class PositionGrid(object):
    def __init__(self):
        self._model = Model('/home/koralewski/workspace/Learning/auto_learner_docker/models')

    def add_evidences(self, *evidences):
        object_type, grasping_type, robot_face, bottom_face, arm = evidences
        bottom_face = bottom_face.replace(':', '')
        robot_face = robot_face.replace(':', '')

        if arm == 'left':
            arm = 'pr2_left_arm'
        else:
            arm = 'pr2_right_arm'

        self._model.add_predictor(object_type, grasping_type, robot_face, bottom_face, arm)

    def get_grid(self):
        return self._model.get_probability_distribution_for_grid()

    def get_predictor_index(self, x, y):
        index = 0
        for i in range(0, len(self._model.predictors)):
            predictor = self._model.predictors[i]
            print predictor._file_name
            if (predictor._min_x <= x <= predictor._max_x) and (predictor._min_y <= y <= predictor._max_y):
                index = i
                break
        return index

