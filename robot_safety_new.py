#! /usr/bin/env python

import rospy #File name without capital
import sys

from sequential_single_item_auction.msg import Bids, SetofTasks, RoundResult, NewTasks, ComTasks, SafetyTasks
from std_msgs.msg import String  #Create a topic of current node
from strands_executive_msgs.msg import MdpDomainSpec
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse

import actionlib
from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal

class Robot():
    # Set up 4 subsribers(Waypoint, SetofTasks, RoundResult, NewTasks) and one publisher(Bids)
    def __init__(self, robot_id):
        #Initialise the nodes
        rospy.init_node("SSIA_"+str(robot_id))
        self.robot_id=robot_id
        print "I AM ROBOT ", rospy.get_namespace()

        #Initialise the biding sections
        self.auction_complete_cond=False
        self.initial_global_tasks=[]
        self.current_global_tasks=[]
        self.robot_schedule = []
        self.complete_schedule=[]

        self.source_wp = rospy.Subscriber("current_node", String, self.get_waypoint) #rostopic type robot0/current_node
        self.source_sot = rospy.Subscriber("/set_of_tasks", SetofTasks, self.assign_tasks) #/ means no namespace for each one of the robot
        self.pub_bid = rospy.Publisher("/bids_and_tasks", Bids, queue_size=1)
        self.source_rr = rospy.Subscriber("/round_result", RoundResult, self.add_task)

        self.source_nt = rospy.Subscriber("/new_tasks", NewTasks, self.reauction)

        self.pub_ct = rospy.Publisher("/com_tasks", ComTasks, queue_size=1)

        self.source_st=rospy.Subscriber("/safety_tasks",SafetyTasks,self.assign_safety_tasks)

        rospy.wait_for_service('mdp_plan_exec/get_guarantees_for_co_safe_task')
        rospy.loginfo("finish 'get guarantees' wait for server")
        self.call_MDP_service = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task',
                                              GetGuaranteesForCoSafeTask)

        #Initialise the moving settings
        self.robot_exec_status = False
        self.robot_action_client = actionlib.SimpleActionClient("mdp_plan_exec/execute_policy",
                                                                       ExecutePolicyAction) #Take care about the namespace
        self.robot_action_client.wait_for_server()
        rospy.loginfo("MDP action client created for " + self.robot_id)

        rospy.spin()

    def get_waypoint(self,msg):
        self.current_waypoint=msg.data
        if msg.data!="none":
            self.last_waypoint=self.current_waypoint
        rospy.loginfo("my current waypoint is : %s", self.current_waypoint)

        #delete the finished tasks from the robot_spec_list
        # current_waypoint_spec = 'F ("'+self.current_waypoint+'")'
        current_waypoint_spec = self.waypoint_to_spec(self.current_waypoint)

        if current_waypoint_spec in self.robot_schedule:
            rospy.loginfo("As I finished a task, I want to delete %s",current_waypoint_spec)
            self.robot_schedule.remove(current_waypoint_spec)
            self.complete_schedule.append(current_waypoint_spec)
            if self.robot_schedule==[]:
                self.robot_exec_status = False
                rospy.loginfo(self.robot_id + " finished with goal.")

    def assign_tasks(self,msg): # Instead writing do_bid as the callback function, I change it to assign_tasks().
        self.current_global_tasks+=msg.tasks
        self.initial_global_tasks+=msg.tasks[:]
        rospy.loginfo("received global task is %s", self.current_global_tasks)
        self.do_bid()

    def do_bid(self):
        self.best_bid = sys.maxint
        self.best_task=''

        for task in self.current_global_tasks:
            time_now = rospy.get_rostime()
            spec=MdpDomainSpec()
            spec.ltl_task=task
            if self.current_waypoint!="none":
                waypoint = self.current_waypoint  # Obtain the initial waypoint
            else:
                waypoint = self.last_waypoint

            if self.robot_schedule!=[]:
                spec.ltl_task= self.list_to_spec(self.robot_schedule) + " & (" +task +")"
            else:
                spec.ltl_task=task

            rospy.loginfo("my current bid is for achieving %s", spec.ltl_task)

            cost = self.call_MDP_service(waypoint,spec,time_now) # Receive the expected cost value from the MDP service
            if cost.expected_time.to_nsec() < self.best_bid:
                print(cost.probability)
                if cost.probability!=0:
                    self.best_bid = cost.expected_time.to_nsec() #probability get from there
                else:
                    self.best_bid = 999999999999 # to avoid impossible task such as G(!(Waypoint1)) & F(Waypoint1)
                self.best_task = task

        self.gen_my_bid()
        self.pub_bid.publish(self.my_bid)

    def gen_my_bid(self):
        self.my_bid = Bids()
        self.my_bid.robot_id = self.robot_id
        self.my_bid.bid = self.best_bid
        self.my_bid.task = self.best_task
        # rospy.loginfo("my best bid is %s, and my best task is %s",self.my_bid.bid, self.my_bid.task)

    def add_task(self,msg):
        if robot_id == msg.robot_id: # Checks that the robot is the one specified by the winner task
            if msg.task not in self.robot_schedule:
                if msg.task != self.waypoint_to_spec(self.last_waypoint):#this is used to solve the repeating waypoint problems
                    self.robot_schedule.append(msg.task) #extend the tasks
                else:
                    last_waypoint_spec=self.waypoint_to_spec(self.last_waypoint)
                    rospy.loginfo("As I finished a task, I want to delete %s", last_waypoint_spec)
                    self.complete_schedule.append(last_waypoint_spec)
                    if self.current_global_tasks==[]:
                        self.robot_exec_status = False
                        rospy.loginfo(self.robot_id + " finished with goal.")

            rospy.loginfo("the robot: %s assigned task is %s",self.robot_id,self.robot_schedule)

        if msg.task in self.current_global_tasks:
            self.current_global_tasks.remove(msg.task)
        rospy.loginfo("remaining current global tasks are:%s", self.current_global_tasks)

        if len(self.current_global_tasks)!=0:
            self.do_bid() # Sending the bids again if there are any tasks left.
        else:
            #Edit this part to disable repeated goal sending
            self.auction_complete_cond=True
            self.robot_exec_status = False
            self.move_robot()
        # self.robot_exec_status = False
        # self.move_robot()

    def list_to_spec(self, robot_schedule):
        new_spec = ''
        for i in range(len(robot_schedule)):
            if i == 0:
                new_spec += "(" + robot_schedule[i] + ")"
            else:
                new_spec += " & (" + robot_schedule[i] + ")"

        return new_spec

    #do move robots to the desired target space until the robot has no schedule
    def move_robot(self):
        goal = ExecutePolicyGoal()
        if len(self.robot_schedule)!=0:
            if not self.robot_exec_status:
                # temp_robot_schedule = self.robot_schedule
                goal.spec.ltl_task = self.list_to_spec(self.robot_schedule)
                print "\n\nSPEC, ", goal.spec.ltl_task
                self.robot_action_client.send_goal(goal)
                rospy.loginfo("Sent goal " + goal.spec.ltl_task + " to " + self.robot_id)
                self.robot_exec_status = True

    def reauction(self,msg):
        self.reauction_tasks=msg.tasks
        self.initial_global_tasks+=msg.tasks
        self.robot_schedule = self.safety_constraint[:]
        if self.auction_complete_cond: #When both of the auction and execution are finished

            if msg.new_flag==True:
                self.gen_my_ct()
                self.pub_ct.publish(self.my_ct)
            else:
                self.current_global_tasks = msg.tasks
                rospy.loginfo("now the reauctioned global task lists are %s",self.current_global_tasks)
                self.do_bid()
        else:   #When the auctioning process is happenning

            self.current_global_tasks = self.initial_global_tasks

    def waypoint_to_spec(self,waypoint):
        spec = 'F ("'+waypoint+'")'
        return spec

    def gen_my_ct(self):
        self.my_ct=ComTasks()
        self.my_ct.robot_id=self.robot_id
        self.my_ct.tasks=self.complete_schedule[:]

    def assign_safety_tasks(self,msg):
        self.safety_constraint=msg.tasks[:] #if we want to delete the safety, just an empty.
        self.robot_schedule+=msg.tasks[:]
        rospy.loginfo("received safety task is %s", msg.tasks)
        print(self.safety_constraint[0][5:-2])
        print(self.current_waypoint)
        print(type(self.safety_constraint[0][5:-2]))
        print(type(self.current_waypoint))
        print(self.safety_constraint[0][5:-2]==self.current_waypoint)
        if self.safety_constraint[0][5:-2]==str(self.current_waypoint):
            print('navigate to the nearest waypoint')
            nearest_waypoint = self.current_waypoint[:]
            if nearest_waypoint[-2]!='t':
                nearest_waypoint='Waypoint'+str(int(nearest_waypoint[-2:])+1)
            else:
                nearest_waypoint= 'Waypoint'+str(int(nearest_waypoint[-1:]) + 1)

            goal = ExecutePolicyGoal()
            goal.spec.ltl_task = self.waypoint_to_spec(nearest_waypoint)
            print "\n\nSPEC, ", goal.spec.ltl_task
            self.robot_action_client.send_goal(goal)
            rospy.loginfo("Sent goal " + goal.spec.ltl_task + " to " + self.robot_id)

if __name__ == '__main__':
    filtered_argv = rospy.myargv(argv=sys.argv)
    print filtered_argv
    robot_id = filtered_argv[1]
    robot=Robot(robot_id)
