#! /usr/bin/env python
import rospy
import rosservice
from json_prolog_msgs.srv import *
import actionlib
import reasoning_and_learning_framework.msg as msg

from query.query import Query
from ralf import ralf
import multiprocessing

SERVICE_NAME_SIMPLE_QUERY_ID = 'ralf/simple_query'
SERVICE_NAME_SOLUTION = 'ralf/next_solution'
SERVICE_NAME_FINISH = 'ralf/finish'
NODE_NAME = 'ralf'

service_query_id = None
service_query_solution = None
service_query_finish = None

queries = {}

server = None


class RalfAction(object):
    _feedback = msg.ralfActionFeedback
    _result = msg.ralfResult

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, msg.ralfAction,
                                           execute_cb=self.execute_cb, auto_start=False)
        self.query = Query('first', 0, 'performing_action(fetching, cup, cup-1, [\"base_footprint\", \"cup_1\", [1.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]], [\"map\", \"cup_1\", [1.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]], \"left\").')
        self._as.start()

    def execute_cb(self, goal):
        print 'excute cb'
        # helper variables
        r = rospy.Rate(1)

        #query = Query('first', 0, 'performing_action(fetching, cup, cup-1, [\"base_footprint\", \"cup_1\", [1.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]], [\"map\", \"cup_1\", [1.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]], \"left\").')
        p = multiprocessing.Process(target=ralf.answer_query, args=[self.query])
        p.start()
        p.join()  # this line allows you to wait for processes
        #answer = ralf.answer_query(self.query)

        print 'finished cb'

        #self._result.result = answer
        self._as.set_succeeded(self._result)
        #start_ralf_server()


def call_service_query_id(req):
    query = Query(req.id, req.mode, req.query)
    queries[query.id] = query

    print query._query_text

    # The MLN inference causes that the service dies sometimes without throwing an exception.
    # However until the real cause will be found out, a reboot of the service fixes it.
    if __is_service_running__():
        try:
            __start_service__()
        except Exception as inst:
            print inst

    return PrologQueryResponse(True, '')


def call_service_query_solution(req):
    query = queries.get(req.id, None)

    answer = ralf.answer_query(query)
    #print answerP
    # The MLN inference causes that the service dies sometimes without throwing an exception.
    # However until the real cause will be found out, a reboot of the service fixes it.
    if __is_service_running__():
        try:
            __start_service__()
        except Exception as inst:
            print inst

    return PrologNextSolutionResponse(3, answer)


def call_service_query_finish(req):
    if __is_service_running__():
        try:
            __start_service__()
        except Exception as inst:
            print inst
    return PrologQueryResponse()


def __is_service_running__():
    return '/{}'.format(SERVICE_NAME_SIMPLE_QUERY_ID) not in rosservice.get_service_list() \
           or '/{}'.format(SERVICE_NAME_SOLUTION) not in rosservice.get_service_list() \
           or '/{}'.format(SERVICE_NAME_FINISH) not in rosservice.get_service_list()


def __start_service__():
    print 'Starting services ..'
    if '/{}'.format(SERVICE_NAME_SIMPLE_QUERY_ID) not in rosservice.get_service_list():
        global service_query_id
        service_query_id = rospy.Service(SERVICE_NAME_SIMPLE_QUERY_ID, PrologQuery, call_service_query_id)

    if '/{}'.format(SERVICE_NAME_SOLUTION) not in rosservice.get_service_list():
        global service_query_solution
        service_query_id = rospy.Service(SERVICE_NAME_SOLUTION, PrologNextSolution, call_service_query_solution)

    if '/{}'.format(SERVICE_NAME_FINISH) not in rosservice.get_service_list():
        global service_query_finish
        service_query_finish = rospy.Service(SERVICE_NAME_FINISH, PrologFinish, call_service_query_finish)


def start_ralf_server():
    print 'Starting node ...'
    rospy.init_node(NODE_NAME)
    __start_service__()
    print"Waiting for questions ..."
    rospy.spin()


if __name__ == '__main__':
    start_ralf_server()





