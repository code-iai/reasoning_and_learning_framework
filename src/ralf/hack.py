from grasping_type_inference.grasping_type_mln import set_mln_path
from grasping_type_inference.inference import Inference

import sys

if __name__ == "__main__":
    facing_robot, bottom_face, object_type, mln_path = sys.argv[1:]
    set_mln_path(mln_path+'/mln')
    inference = Inference(facing_robot, bottom_face, object_type)
    print inference.get_storted_list_of_grasping_types_based_on_probability()