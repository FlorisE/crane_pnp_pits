#! /usr/bin/env python

from rospit.framework import Runner, TestSuite, TestCase, Sensor
from rospit.binary import BinaryConditionEvaluator, BinaryCondition, BinaryMeasurement
from rospit.numeric import BothLimitsEvaluator, BothLimitsCondition, \
                           UpperLimitCondition, UpperLimitEvaluator, \
                           LowerLimitCondition, LowerLimitEvaluator
from rospit.ros import ROSTestCase, ROSInvariant, wait_for_service
from pick_and_placer_msgs.srv import DoPick, DoPickRequest, DoPlace, \
                                     DoPlaceRequest, Initialize, InitializeRequest
from geometry_msgs.msg import PoseStamped
import rospy
import time
import logging
import sys


# def debug_signal_handler(signal, frame):
#     import pdb
#     pdb.set_trace()
#
#
# import signal
#
# signal.signal(signal.SIGINT, debug_signal_handler)


class PresenceSensor(Sensor):
    def __init__(self, topic, node_name, message):
        Sensor.__init__(self)
        self.present = False
        rospy.init_node(node_name)
        rospy.Subscriber(topic, message, self.callback)
        self.data = None

    def callback(self, data):
        self.present = True
        self.data = data

    def sense_internal(self):
        return BinaryMeasurement(self.present)


class BlockPoseSensor(Sensor):
    def __init__(self):
        Sensor.__init__(self)
        topic = "/ar_block_finder/pose"
        self.present = False
        self.pose = None
        rospy.init_node("block_pose_sensor")
        rospy.Subscriber(topic, PoseStamped, self.callback)

    def callback(self, data):
        self.present = True
        self.pose = data

    def sense_internal(self):
        return BinaryMeasurement(self.present)


class Picks(TestCase):
    def __init__(self):
        TestCase.__init__(self, "Picks", wait_for_preconditions=True)
        self.block_sensor = BlockPoseSensor()
        self.picked = False

        block_present_evaluator = BinaryConditionEvaluator(self.block_sensor)
        block_present_condition = BinaryCondition(True)
        self.preconditions = [(block_present_condition, block_present_evaluator)]

        block_picked_pos_evaluator = BinaryConditionEvaluator(
            lambda: self.block_sensor.pose.pose.position.z > 0.1)
        block_picked_pos_condition = BinaryCondition(True)
        block_picked_moveit_evaluator = BinaryConditionEvaluator(lambda: self.picked)
        block_picked_moveit_condition = BinaryCondition(True)
        self.postconditions = [
            (block_picked_pos_condition, block_picked_pos_evaluator),
            (block_picked_moveit_condition, block_picked_moveit_evaluator)]

    def set_up(self):
        service_name = wait_for_service("/pick_and_placer/initialize")
        print("Initializing")
        do_initialize = rospy.ServiceProxy(service_name, Initialize)
        initialize_msg = InitializeRequest()
        do_initialize(initialize_msg)
        print("Initialized")

    def run(self):
        service_name = wait_for_service("/pick_and_placer/do_pick")
        print("Preparing pick")
        do_pick = rospy.ServiceProxy(service_name, DoPick)
        print("Executing pick")
        pick_msg = DoPickRequest()
        pick_msg.pose.x = self.block_sensor.pose.pose.position.x
        pick_msg.pose.y = self.block_sensor.pose.pose.position.y
        pick_response = do_pick(pick_msg)
        self.picked = BinaryMeasurement(pick_response.success)
        time.sleep(4)
        print("Finished executing pick")


class Places(ROSTestCase):
    def __init__(self, depends_on):
        ROSTestCase.__init__(self, "Places", depends_on=depends_on, wait_for_preconditions=True)
        self.block_sensor = BlockPoseSensor()
        self.placed = False

        block_present_evaluator = BinaryConditionEvaluator(self.block_sensor)
        block_present_condition = BinaryCondition(True)
        block_picked_evaluator = BinaryConditionEvaluator(
            lambda: self.block_sensor.pose.pose.position.z > 0.1)
        block_picked_condition = BinaryCondition(True)
        self.preconditions = [
            (block_present_condition, block_present_evaluator),
            (block_picked_condition, block_picked_evaluator)]

        inv_condition = LowerLimitCondition(0.1)
        inv_evaluator = LowerLimitEvaluator(lambda data: data.pose.position.z)
        self.invariants = [
            ROSInvariant(inv_condition, inv_evaluator, "/ar_block_finder/pose", PoseStamped)]

        block_placed_x_condition = BothLimitsCondition(0.07, 0.13)
        block_placed_y_condition = BothLimitsCondition(-0.23, -0.17)
        block_placed_z_condition = UpperLimitCondition(0.08)
        block_placed_x_evaluator = BothLimitsEvaluator(
            lambda: self.block_sensor.pose.pose.position.x)
        block_placed_y_evaluator = BothLimitsEvaluator(
            lambda: self.block_sensor.pose.pose.position.y)
        block_placed_z_evaluator = UpperLimitEvaluator(
            lambda: self.block_sensor.pose.pose.position.z)
        block_placed_moveit_evaluator = BinaryConditionEvaluator(lambda: self.placed)
        block_placed_moveit_condition = BinaryCondition(True)
        self.postconditions = [
                (block_placed_x_condition, block_placed_x_evaluator),
                (block_placed_y_condition, block_placed_y_evaluator),
                (block_placed_z_condition, block_placed_z_evaluator),
                (block_placed_moveit_condition, block_placed_moveit_evaluator)]

    def run(self):
        service_name = wait_for_service("/pick_and_placer/do_place")
        print("Preparing place")
        do_place = rospy.ServiceProxy(service_name, DoPlace)
        print("Executing place")
        place_msg = DoPlaceRequest()
        place_response = do_place(place_msg)
        self.placed = BinaryMeasurement(place_response.success)
        print("Finished executing place")

    def tear_down(self):
        service_name = wait_for_service("/pick_and_placer/initialize")
        do_initialize = rospy.ServiceProxy(service_name, Initialize)
        initialize_msg = InitializeRequest()
        do_initialize(initialize_msg)


def get_test_suite():
    test_suite = TestSuite("Picks and places")
    picks_test_case = Picks()
    places_test_case = Places(depends_on=[picks_test_case])
    test_suite.test_cases.append(picks_test_case)
    test_suite.test_cases.append(places_test_case)
    return test_suite


if __name__ == '__main__':
    # # clear the screen
    # import os
    # clear = lambda: os.system('clear')
    # clear()
    logger = logging.getLogger("test picks and places")
    logger.setLevel(logging.DEBUG)
    logger.addHandler(logging.StreamHandler(sys.stdout))
    runner = Runner(logger)
    runner.run_suite(get_test_suite())
    # raw_input()
