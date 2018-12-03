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
# SOFTWARE.

from grasping import type as grasping_type, utils


def answer_query(query):
    if query.predicate == 'designator_costmap':
        return grasping_type.get_most_probable('front', 'bottom', 'cup.n.01')
    if query.predicate == 'performing_action':
        if query.parameters[0] == 'FETCHING':
            pose = eval(query.parameters[3])
            transformation_matrix = utils.get_transform_matrix(pose[2], pose[3])
            robot_face, bottom_face = utils.calculate_object_faces(transformation_matrix)
