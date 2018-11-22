from grasping import type as grasping_type


def answer_query(query):
    if query == 'grasping':
        return grasping_type.get_most_probable('front', 'bottom', 'cup.n.01')
