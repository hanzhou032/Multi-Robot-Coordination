#! /usr/bin/env python
import copy
import rospy  # File name without capital
import sys

from sequential_single_item_auction.msg import Bids, SetofTasks, RoundResult, NewTasks, ComTasks, SafetyTasks, LSTMTasks, HypoBids, HypoRoundResult
from std_msgs.msg import String  # Create a topic of current node
from strands_executive_msgs.msg import MdpDomainSpec
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse

import actionlib
from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal

# import threading
from threading import Lock

import csv

class Robot():
    # A class for each robot which incorporates the LSTM generated hypothetical task
    def __init__(self, robot_id):
        # Initialise the nodes for each robot by using the Sequential Single Item Auction schemes
        rospy.init_node("SSIA_"+str(robot_id))
        self.robot_id = robot_id
        print "I AM ROBOT ", rospy.get_namespace()

        # Initialise the biding sections
        self.auction_complete_cond = False
        self.initial_global_tasks = []
        self.current_global_tasks = []
        self.robot_schedule = []
        self.complete_schedule = []
        self.safety_constraint = []
        self.initial_flag = True
        self.hypo_tasks = []
        self.hypo_prob = []
        self.mutex = Lock()
        self.time_complete = 0
        self.time_com_list = []
        self.task_com_list = []

        self.time_nav_arrive_list = []
        self.time_nav_left_list = []
        self.task_nav_list = []

        # Initialise the moving settings
        self.robot_exec_status = False
        self.robot_action_client = actionlib.SimpleActionClient("mdp_plan_exec/execute_policy",
                                                                ExecutePolicyAction)
        # Take care about the namespace
        self.robot_action_client.wait_for_server()
        rospy.loginfo("MDP action client created for " + self.robot_id)

        # Wait until the MDP guarantees the tasks
        rospy.wait_for_service('mdp_plan_exec/get_guarantees_for_co_safe_task')
        rospy.loginfo("finish 'get guarantees' wait for server")
        self.call_MDP_service = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task',
                                                   GetGuaranteesForCoSafeTask)
        self.closest_node = None
        rospy.Subscriber("current_node", String, self.get_waypoint)  # rostopic type robot0/current_node
        rospy.Subscriber("closest_node", String, self.closest_node_cb)  # rostopic type robot0/current_node

        # Initialise the subscriber and publisher settings
        self.source_sot = rospy.Subscriber("/set_of_tasks", SetofTasks, self.assign_tasks)
        # / means no namespace for each one of the robot
        self.pub_bid = rospy.Publisher("/bids_and_tasks", Bids, queue_size=1, latch=True)
        self.source_rr = rospy.Subscriber("/round_result", RoundResult, self.add_task)
        self.source_nt = rospy.Subscriber("/new_tasks", NewTasks, self.reauction)
        self.pub_ct = rospy.Publisher("/com_tasks", ComTasks, queue_size=1, latch=True)
        self.source_st = rospy.Subscriber("/safety_tasks", SafetyTasks, self.assign_safety_tasks)
        self.source_LSTM_tasks = rospy.Subscriber("/LSTM_tasks", LSTMTasks, self.assign_hypo_tasks)

        self.pub_hypo_bid = rospy.Publisher("/hypo_bids", HypoBids, queue_size=1, latch=True)
        self.source_hrr = rospy.Subscriber("/hypo_round_result", HypoRoundResult, self.navigate_hypo_tasks)

        # print "initialised"
        rospy.spin()

    # Subscribe the waypoint message from the current node
    def get_waypoint(self, msg):
        self.current_waypoint = msg.data
        temp_time = rospy.Time.now()

        if msg.data != "none":
            self.last_waypoint = self.current_waypoint[:]  # Last waypoint is no longer useful, be careful.
            self.time_nav_arrive_list.append(temp_time)
            self.task_nav_list.append(msg.data)
        else:
            self.time_nav_left_list.append(temp_time)

        # By fixing this, this time the code is recording every waypoint data the robot navigated
        self.write_instant_nav_data()

        rospy.loginfo("my current waypoint is : %s", self.current_waypoint)
        current_waypoint_spec = self.waypoint_to_spec(self.current_waypoint)

        # delete the finished tasks from the robot_spec_list
        if current_waypoint_spec in self.robot_schedule:
            rospy.loginfo("As I finished a task, I want to delete %s", current_waypoint_spec)
            self.robot_schedule.remove(current_waypoint_spec)
            self.complete_schedule.append(current_waypoint_spec)
            self.time_complete = rospy.Time.now()
            self.time_com_list.append(self.time_complete)
            self.task_com_list.append(self.current_waypoint)
            # print self.time_complete
            # print self.time_com_list
            # self.write_csv()

            if self.robot_schedule == []:
                self.robot_exec_status = False
                rospy.loginfo(self.robot_id + " finished with all goals.")
                self.execute_hypo_tasks()
                # self.write_csv()

    def closest_node_cb(self, msg):
        self.closest_node = msg.data

    # Callback function to the SetofTasks which initialises the task sequence from the robot's point of view
    def assign_tasks(self, msg):  # Instead writing do_bid as the callback function, I change it to assign_tasks().
        # print "receive"
        self.current_global_tasks += msg.tasks
        self.initial_global_tasks += msg.tasks[:]
        rospy.loginfo("received global task is %s", self.current_global_tasks)
        self.do_bid()

    # Bid for each one of the task, the cost is from the MDP service, the expected time.
    def do_bid(self):
        self.best_bid = sys.maxint
        self.best_task = ''
        for task in self.current_global_tasks:
            # time_now = rospy.get_rostime()
            time_now = rospy.Time.now()
            # print time_now

            spec = MdpDomainSpec()
            spec.ltl_task = task
            if self.current_waypoint != "none":
                waypoint = self.current_waypoint  # Obtain the initial waypoint
            else:
                waypoint = self.closest_node

            if self.robot_schedule != []:
                spec.ltl_task = self.list_to_spec(self.robot_schedule) + " & (" + task + ")"
            else:
                spec.ltl_task = task

            rospy.loginfo("my current bid is for achieving %s", spec.ltl_task)
            # print waypoint
            # print "acquiring"
            self.mutex.acquire()
            # print "acquired"
            cost = self.call_MDP_service(copy.copy(waypoint), copy.deepcopy(spec), time_now)  # Receive the expected cost value from the MDP service
            self.mutex.release()
            # print "release"
            #
            # print cost
            # print cost.expected_time.to_sec()
            if cost.expected_time.to_sec() < self.best_bid:
                # print(cost.probability)
                if cost.probability != 0:
                    self.best_bid = cost.expected_time.to_sec()  # probability get from there
                else:
                    self.best_bid = sys.maxint  # to avoid impossible task such as G(!(Waypoint1)) & F(Waypoint1)
                self.best_task = task
        self.gen_my_bid()
        self.pub_bid.publish(self.my_bid)

    # Prepare the bid message
    def gen_my_bid(self):
        self.my_bid = Bids()
        self.my_bid.robot_id = self.robot_id
        self.my_bid.bid = self.best_bid
        self.my_bid.task = self.best_task

    # Callback function when auctioneer allocated the task. It adds the task to the robot schedule and
    # removes it from the global task
    def add_task(self, msg):
        if robot_id == msg.robot_id:  # Checks that the robot is the one specified by the winner task
            if msg.task not in self.robot_schedule:
                if msg.task != self.waypoint_to_spec(self.closest_node):  # this is used to solve the repeating
                    # waypoint problems
                    self.robot_schedule.append(msg.task)  # extend the tasks
                else:
                    closest_node_spec = self.waypoint_to_spec(self.closest_node)  # solve the initial waypoint problem
                    rospy.loginfo("As I finished a task immediately, I want to delete %s", closest_node_spec)
                    self.time_complete = rospy.Time.now()
                    # print self.time_complete
                    self.time_com_list.append(self.time_complete)
                    # print self.time_com_list
                    self.task_com_list.append(self.closest_node)
                    # print self.task_com_list
                    # self.write_csv()
                    self.complete_schedule.append(closest_node_spec)

                    if self.robot_schedule == []:
                        self.robot_exec_status = False
                        rospy.loginfo(self.robot_id + " finished with all goals.")
                        self.execute_hypo_tasks()


            rospy.loginfo("the robot: %s assigned task is %s", self.robot_id, self.robot_schedule)
        if msg.task in self.current_global_tasks:
            self.current_global_tasks.remove(msg.task)
        # rospy.loginfo("remaining current global tasks are:%s", self.current_global_tasks)

        # in the robot part we do not need to consider the completion of the impossible task,
        # and that task is automatically removed from the auctioneer.

        if len(self.current_global_tasks) != 0:
            self.do_bid()  # Sending the bids again if there are any tasks left.
        else:
            # Edit this part to disable repeated goal sending
            self.auction_complete_cond = True
            # self.robot_exec_status = False  # some problem there
            self.move_robot()

    # Transformation from schedule list to machine readable spec
    def list_to_spec(self, robot_schedule):
        new_spec = ''
        for i in range(len(robot_schedule)):
            if i == 0:
                new_spec += "(" + robot_schedule[i] + ")"
            else:
                new_spec += " & (" + robot_schedule[i] + ")"
        return new_spec
    # different rates, different number of robots. /hypothetical different ways.
    # Move robots to the desired target space until the robot has no schedule

    def move_robot(self):
        self.robot_action_client.cancel_all_goals()
        goal = ExecutePolicyGoal()
        if len(self.robot_schedule) != 0:
            # if not self.robot_exec_status:  # later on we can actually delete this condition
            goal.spec.ltl_task = self.list_to_spec(self.robot_schedule)
            self.robot_action_client.send_goal(goal)
            rospy.loginfo("Sent real goal " + goal.spec.ltl_task + " to " + self.robot_id)
            self.robot_exec_status = True
                # self.robot_exec_status = True  # the setup of robot execution status may ba removed later on
        # else:
        #    self.execute_hypo_tasks()

    # Callback function in the case when new task is introduced to each robot
    def reauction(self, msg):
        self.reauction_tasks = msg.tasks
        self.initial_global_tasks += msg.tasks
        # temp_robot_schedule = []
        self.robot_schedule = self.safety_constraint[:]  # problem..fix it
        if self.auction_complete_cond:  # When both of the auction and execution are finished
            if msg.new_flag:
                self.gen_my_ct()
                self.pub_ct.publish(self.my_ct)  # Whenever there is a new task,
                # it tells the auctioneer the complete set of tasks
            else:
                self.current_global_tasks = msg.tasks
                # rospy.loginfo("now the reauctioned global task lists are %s", self.current_global_tasks)
                self.do_bid()
        else:   # When the auctioning process is happening
            self.current_global_tasks = self.initial_global_tasks

    # Commonly used transformation method from waypoint to LTL spec
    def waypoint_to_spec(self, waypoint):
        spec = 'F ("'+waypoint+'")'
        return spec

    # Generate the format of complete set of tasks
    def gen_my_ct(self):
        self.my_ct = ComTasks()
        self.my_ct.robot_id = self.robot_id
        self.my_ct.tasks = self.complete_schedule[:]

    # Callback function used to initialise the safety constraint and solve several corner safety problems
    def assign_safety_tasks(self, msg):
        self.safety_constraint = msg.tasks[:]  # if we want to delete the safety, just an empty.
        self.robot_schedule += msg.tasks[:]
        # rospy.loginfo("received safety task is %s", msg.tasks)
        # print(self.safety_constraint[0][5:-2])
        # print(self.current_waypoint)
        # still need to adjust this part to deal with this problem
        if self.safety_constraint[0][5:-2] == str(self.current_waypoint):
            # print('navigate to the nearest waypoint')
            nearest_waypoint = self.current_waypoint[:]
            nearest_waypoint = 'Waypoint'+str(int(nearest_waypoint[8:]) + 1)  # this is not a reasonable way to do so
            goal = ExecutePolicyGoal()
            goal.spec.ltl_task = self.waypoint_to_spec(nearest_waypoint)
            self.robot_action_client.send_goal(goal)
            # rospy.loginfo("Sent goal " + goal.spec.ltl_task + " to " + self.robot_id)

    # Callback function used to navigate to the hypothetical task
    def assign_hypo_tasks(self, msg):
        self.initial_hypo_tasks = msg.tasks[:]
        self.initial_hypo_prob = msg.prob[:]
        self.do_hypo_bid()
        # self.execute_hypo_tasks()

    def execute_hypo_tasks(self):
        if self.robot_schedule == []:
            if not self.robot_exec_status:
                goal = ExecutePolicyGoal()
                goal.spec.ltl_task = self.list_to_spec(self.hypo_tasks)
                self.robot_action_client.send_goal(goal)
                # self.robot_action_client.cancel_all_goals()
                # we can use | to do the 'or' LTL.
                rospy.loginfo("Sent hypothetical goal: " + goal.spec.ltl_task + " to " + self.robot_id)

    def navigate_hypo_tasks(self, msg):
        if robot_id == msg.robot_id:  # Checks that the robot is the one specified by the winner task
            if msg.task != self.waypoint_to_spec(
                    self.closest_node):  # this is used to solve the repeating waypoint problems
                self.hypo_tasks = [msg.task[:]]
                self.execute_hypo_tasks()
            else:
                closest_node_spec = self.waypoint_to_spec(self.closest_node)  # solve the initial waypoint problem
                rospy.loginfo("I finished a hypo task instantly at %s", closest_node_spec)
        #else:
        #    self.robot_action_client.cancel_all_goals()  # problem there
        #    rospy.loginfo("Hypo task was not allocated to me")


    def do_hypo_bid(self):
        self.best_hypo_bid = sys.maxint
        self.best_hypo_task = ''
        #task = self.hypo_tasks[:]  # we only have one hypothetical task in this case, so that no for loop
        for task in self.initial_hypo_tasks:
            #time_now = rospy.get_rostime()
            time_now = rospy.Time.now()
            # print time_now
            spec = MdpDomainSpec()
            spec.ltl_task = task
            if self.current_waypoint != "none":
                waypoint = self.current_waypoint  # Obtain the initial waypoint
            else:
                waypoint = self.closest_node
            if self.robot_schedule != []:
                spec.ltl_task = self.list_to_spec(self.robot_schedule) + " & (" + task + ")"
            else:
                spec.ltl_task = task
            rospy.loginfo("my current bid for achieving hypo task is %s", spec.ltl_task)
            # print "acquiring"
            self.mutex.acquire()
            cost = self.call_MDP_service(copy.copy(waypoint), copy.deepcopy(spec), time_now)
            self.mutex.release()
            # print "released"
            # Receive the expected cost value from the MDP service
            if cost.expected_time.to_sec() < self.best_hypo_bid:
                # print(cost.probability)
                if cost.probability != 0:
                    self.best_hypo_bid = cost.expected_time.to_sec()  # probability get from there
                else:
                    self.best_hypo_bid = sys.maxint  # to avoid impossible task such as G(!(Waypoint1)) & F(Waypoint1)
                self.best_hypo_task = task
        self.gen_my_hypo_bid()
        self.pub_hypo_bid.publish(self.my_hypo_bid)

    def gen_my_hypo_bid(self):
        self.my_hypo_bid = HypoBids()
        self.my_hypo_bid.robot_id = self.robot_id
        self.my_hypo_bid.bid = self.best_hypo_bid
        self.my_hypo_bid.task = self.best_hypo_task

    def write_csv(self):
        if self.robot_id == "robot_0":
            with open("com_time_list_0.csv", 'wb') as resultFile:
                wr = csv.writer(resultFile, dialect='excel')
                wr.writerow(self.time_com_list)
                wr.writerow(self.task_com_list)
        else:
            with open("com_time_list_1.csv", 'wb') as resultFile:
                wr = csv.writer(resultFile, dialect='excel')
                wr.writerow(self.time_com_list)
                wr.writerow(self.task_com_list)

    def write_instant_nav_data(self):
        if self.robot_id == "robot_0":  # Bear in mind this is not scalable, fix it later
            with open("nav_final_0.csv", 'wb') as resultFile:
                wr = csv.writer(resultFile, dialect='excel')
                wr.writerow(self.time_nav_arrive_list)
                wr.writerow(self.time_nav_left_list)
                wr.writerow(self.task_nav_list)
        else:
            with open("nav_final_1.csv", 'wb') as resultFile:
                wr = csv.writer(resultFile, dialect='excel')
                wr.writerow(self.time_nav_arrive_list)
                wr.writerow(self.time_nav_left_list)
                wr.writerow(self.task_nav_list)
    # not go to the auctioned node.

if __name__ == '__main__':
    filtered_argv = rospy.myargv(argv=sys.argv)
    print filtered_argv
    robot_id = filtered_argv[1]
    robot = Robot(robot_id)
