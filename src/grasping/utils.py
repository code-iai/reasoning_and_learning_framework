import tf.transformations as tf
import numpy as np

def calculate_object_faces(robot_to_object_transform):
    object_to_robot_transform = tf.inverse_matrix(robot_to_object_transform)

    quaternion_matrix =  tf.quaternion_matrix(tf.quaternion_from_matrix(object_to_robot_transform))
    robot_negative_x_vector = np.array([-quaternion_matrix[0][0], -quaternion_matrix[1][0], -quaternion_matrix[2][0]])
    robot_negative_z_vector = np.array([-quaternion_matrix[0][2], -quaternion_matrix[1][2], -quaternion_matrix[2][2]])

    return _calculate_vector_face(robot_negative_x_vector), _calculate_vector_face(robot_negative_z_vector)


def get_object_robot_translation(robot_object_translation, robot_object_quaternion):
    transform_matrix = get_transform_matrix(robot_object_translation, robot_object_quaternion)
    object_to_robot_transform = tf.inverse_matrix(transform_matrix)
    return tf.translation_from_matrix(object_to_robot_transform)


def get_transform_matrix(translation, quaternion):
    translation_matrix = tf.translation_matrix(translation)
    quaternion_matrix = tf.quaternion_matrix(quaternion)
    transform_matrix = tf.concatenate_matrices(translation_matrix, quaternion_matrix)

    return transform_matrix


def _calculate_vector_face(robot_vector):
    dimension = abs(robot_vector).argmax()
    value = robot_vector[dimension]

    if dimension == 0:
        if value > 0.:
            return ':FRONT'
        else:
            return ':BACK'
    elif dimension == 1:
        if value > 0.:
            return ':LEFT-SIDE'
        else:
            return ':RIGHT-SIDE'
    else:
        if value > 0.:
            return ':TOP'
        else:
            return ':BOTTOM'


def get_possible_robot_faces(bottom_face):
    possible_robot_faces = [':FRONT', ':BACK', ':LEFT-SIDE', ':RIGHT-SIDE', ':TOP', ':BOTTOM']

    if bottom_face == ':BOTTOM' or bottom_face == ':TOP':
        possible_robot_faces.remove(':BOTTOM')
        possible_robot_faces.remove(':TOP')

    elif bottom_face == ':FRONT' or bottom_face == ':BACK':
        possible_robot_faces.remove(':FRONT')
        possible_robot_faces.remove(':BACK')

    elif bottom_face == ':RIGHT-SIDE' or bottom_face == ':LEFT-SIDE':
        possible_robot_faces.remove(':RIGHT-SIDE')
        possible_robot_faces.remove(':LEFT-SIDE')


    return possible_robot_faces


