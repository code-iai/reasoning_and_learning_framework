from grasping_type_inference.inference import Inference
import sys

if __name__ == "__main__":
    facing_robot, bottom_face, object_type = sys.argv[1:]

    inference = Inference(facing_robot, bottom_face, object_type)
    print inference.get_storted_list_of_grasping_types_based_on_probability()