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

from grasping import utils
from grasping.position import PositionGrid
from grasping.utils import get_object_robot_translation
import subprocess
import os
import sys
import cram2wordnet.cram2wordnet as cram2wordnet

ARM = 'ARM'
GRASPING_TYPE = 'GRASPING_TYPE'

best_grasping_types = []
best_position_grid = PositionGrid()
fetching_solutions = {}


def answer_query(query):
    if query.predicate == 'performing_action':
        if query.parameters[0] == 'fetching':
            return performing_fetching(query)
        elif query.parameters[0] == 'delivering':
            return performing_delivering(query)
        return '{}'

    elif query.predicate == 'designator_costmap':
        if query.parameters[0] == 'reachable':
            return reachable_costmap(query)

        return '{}'

    elif query.predicate == 'object_type_grasps':
        return object_type_grasps(query)

    return '{}'


def performing_delivering(query):
    position_grid = PositionGrid()
    pose = eval(query.parameters[3])
    object_type = cram2wordnet.map_cram_object_type_to_word_net_instance(query.parameters[1])
    transformation_matrix = utils.get_transform_matrix(pose[2], pose[3])
    _, bottom_face = utils.calculate_object_faces(transformation_matrix)

    arm = fetching_solutions[ARM]
    grasping_result = fetching_solutions[GRASPING_TYPE]
    robot_faces = utils.get_possible_robot_faces(bottom_face)

    print 'DELIVERING'
    print 'Pose:{}'.format(pose)
    print 'Arm:{}'.format(arm)
    print 'Grasping Result:{}'.format(':'+grasping_result.upper())
    print 'Robot Faces:{}'.format(robot_faces)
    print 'Object Type:{}'.format(object_type)
    print 'Bottom face:{}'.format(bottom_face)
    print 'END DELIVERING'

    for robot_face in robot_faces:
        position_grid.add_evidences(object_type, grasping_result.upper(), robot_face, bottom_face, arm)

    global best_position_grid
    best_position_grid = position_grid

    return '{}'


def performing_fetching(query):
    position_grid = PositionGrid()
    pose = eval(query.parameters[3])
    arm = query.parameters[-1]
    fetching_solutions[ARM] = arm
    object_type = cram2wordnet.map_cram_object_type_to_word_net_instance(query.parameters[1])

    transformation_matrix = utils.get_transform_matrix(pose[2], pose[3])

    _, bottom_face = utils.calculate_object_faces(transformation_matrix)

    robot_faces = utils.get_possible_robot_faces(bottom_face)

    for robot_face in robot_faces:
        #grasping_results = grasping_type.get_storted_list_of_grasping_types_based_on_probability(
        #    robot_face, bottom_face, object_type)
        path_to_prac_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hack.py')
        grasping_results = eval(subprocess.check_output(['python', path_to_prac_script , robot_face, bottom_face, object_type, sys.argv[1]]))

        best_grasping_types.append(grasping_results)

        grasping_result = grasping_results[0]

        print 'FETCHING'
        print 'Pose:{}'.format(pose)
        print 'Arm:{}'.format(arm)
        print 'Grasping Result:{}'.format(grasping_result)
        print 'Robot Faces:{}'.format(robot_faces)
        print 'Object Type:{}'.format(object_type)
        print 'Bottom face:{}'.format(bottom_face)
        print 'END FETCHING'
        position_grid.add_evidences(object_type, grasping_result, robot_face, bottom_face, arm)

    global best_position_grid
    best_position_grid = position_grid

    return '{}'


def reachable_costmap(query):
    grid_text = _get_costmap_as_text()
    grid_text = grid_text.replace('\n', ',')
    return "{{\"{}\":{}}}".format(query.parameters[5], grid_text)


def _get_costmap_as_text():
    result = []
    for row in best_position_grid.get_grid():
        row_text = map(str, row)
        result.append(row_text)
    return str(result).replace("'", '')


def object_type_grasps(query):
    pose = eval(query.parameters[2])
    object_robot_translation = get_object_robot_translation(pose[2], pose[3])
    x = object_robot_translation[0]
    y = object_robot_translation[1]

    predictor_index = best_position_grid.get_predictor_index(x,y)
    grasping_result = best_grasping_types[predictor_index]

    grasping_result = map(lambda grasp: grasp.lower(), grasping_result)
    global fetching_solutions
    fetching_solutions[GRASPING_TYPE] = grasping_result[0]
    result = "{{\"{}\":{}}}".format(query.parameters[-1], grasping_result)

    return result.replace("'", '"')