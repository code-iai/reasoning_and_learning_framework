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
from grasping.position import PositionGrid

_cram_to_word_net_object_ = {'BOWL':'bowl.n.01', 'CUP': 'cup.n.01'}

best_grasping_types = []
best_position_grid = PositionGrid()


def answer_query(query):
    if query.predicate == 'performing_action':
        if query.parameters[0] == 'FETCHING':
            return performing_fetching(query)

        return ''

    elif query.predicate == 'designator_costmap':
        if query.parameters[0] == 'REACHABLE':
            return reachable_costmap(query)

        return ''

    return ''


def performing_fetching(query):
    position_grid = PositionGrid()
    pose = eval(query.parameters[3])
    arm = query.parameters[-1]
    object_type = _cram_to_word_net_object_[query.parameters[1]]

    transformation_matrix = utils.get_transform_matrix(pose[2], pose[3])

    _, bottom_face = utils.calculate_object_faces(transformation_matrix)

    robot_faces = utils.get_possible_robot_faces(bottom_face)

    for robot_face in robot_faces:
        grasping_results = grasping_type.get_storted_list_of_grasping_types_based_on_probability(
            robot_face, bottom_face, object_type)

        best_grasping_types.append(grasping_results)

        grasping_result = grasping_results[0]
        position_grid.add_evidences(object_type, grasping_result, robot_face, bottom_face, arm)

    global best_position_grid
    best_position_grid = position_grid

    return ''


def reachable_costmap(query):
    grid_text = str(best_position_grid.get_grid())
    grid_text = grid_text.replace('\n', ',')
    return "{{\"{}\":{}}}".format(query.parameters[5], grid_text)