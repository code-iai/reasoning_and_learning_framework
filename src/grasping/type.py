from grasping_type_inference.inference import Inference


def get_most_probable(facing_robot, bottom_face, object_type):
    inference = Inference(facing_robot, bottom_face, object_type)
    return inference.get_most_probable_result()

