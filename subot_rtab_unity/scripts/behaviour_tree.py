#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import actionlib
import geometry_msgs.msg as geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
import actionlib_msgs.msg as actionlib_msgs
import std_msgs.msg as std_msgs


##############################################################################
# ToBlackboard
##############################################################################

class SetBuffer(py_trees.blackboard.SetBlackboardVariable):
    def __Buffer_To_Goal(self):
        pose_buffer = py_trees.blackboard.Blackboard().get('pose_buffer')
        pose_buffer.header.stamp = rospy.Time.now()
        target_pose  = pose_buffer
        #target_pose.header.stamp = rospy.Time.now()
        return target_pose
    
    def initialise(self):
        self.variable_value = self.__Buffer_To_Goal()
        super(SetBuffer, self).initialise()

##############################################################################
# Actions
##############################################################################
class MoveBaseNav(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get target pose from the blackboard to create an action goal
        """
        self.action_goal = MoveBaseGoal(target_pose=py_trees.blackboard.Blackboard().get("target_pose"))
        super(MoveBaseNav, self).initialise()

    # def update(self):
    #     """
    #     On success, set the resulting path on the blackboard, so ExePath can use it
    #     """
    #     if py_trees.blackboard.Blackboard().get("InRecovery") == False:
    #         status = super(GetPath, self).update()
    #         if status == py_trees.Status.SUCCESS:
    #             py_trees.blackboard.Blackboard().set("path", self.action_client.get_result().path)
    #         return status
    #     return py_trees.Status.FAILURE
    
    # def terminate(self, new_status):
    #     py_trees.blackboard.Blackboard().set("get_path_outcome",self.action_client.get_result())
    #     self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
    #     if self.action_client is not None and self.sent_goal:
    #         motion_state = self.action_client.get_state()
    #         if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
    #            (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
    #             self.action_client.cancel_goal()
    #     self.sent_goal = False
##############################################################################
# Behaviours
##############################################################################

def create_root():
    # Create all behaviours
    bt_root = py_trees.composites.Parallel("MBF BT Demo")
    bt = py_trees.composites.Sequence("BT")

    get_goal = py_trees.composites.Selector("GetGoal")
    get_buffer = py_trees.composites.Selector("GetBuffer")
    apply_buffer = py_trees.composites.Sequence("ApplyBuffer")
    chk_preemp = py_trees.composites.Selector("CheckPreemption")
    fallback = py_trees.composites.Selector("Fallback")
    navigate = py_trees.composites.Sequence("Navigate")
    new_goal = py_trees_ros.subscribers.ToBlackboard(name="NewGoal",
                                                     topic_name="/move_base_simple/goal",
                                                     topic_type=geometry_msgs.PoseStamped,
                                                     blackboard_variables = {'target_pose': None,'pose_buffer': None})
    
    have_goal = py_trees.blackboard.CheckBlackboardVariable(name="HaveGoal?", variable_name="target_pose")
    have_buffer = py_trees.blackboard.CheckBlackboardVariable(name="HaveBuffer?", variable_name="pose_buffer")
    
    clr_goal1 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    clr_buffer = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoalBuffer", variable_name="pose_buffer")
    clr_goal2 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    
    set_buffer = SetBuffer(name="SetBuffer",variable_name='target_pose',variable_value=None)
    set_preempt = py_trees.meta.success_is_running(py_trees.meta.running_is_failure(py_trees_ros.subscribers.ToBlackboard))(name="GotPreemption?",
                                                                    topic_name="/move_base_simple/goal",
                                                                    topic_type=geometry_msgs.PoseStamped,
                                                                    blackboard_variables = {'target_pose': None,'pose_buffer': None})

    cancelling = py_trees.composites.Sequence("Cancelling?")
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name=f"{ns}/cancel_action",
        variable_name="event_cancel_button")
    is_cancel_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Cancel?",
        variable_name='event_cancel_button',
        expected_value=True
    )
    
    move_base = MoveBaseNav(name="MoveBase",
                            action_namespace="/move_base",
                            action_spec=MoveBaseAction,
                            action_goal=MoveBaseActionGoal())


    clr_goal3 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    clr_buffer3 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoalBuffer", variable_name="pose_buffer")

    # Compose tree
    bt_root.add_children([cancel2bb, bt])
    bt.add_children([get_goal, chk_preemp])
    chk_preemp.add_children([cancelling, set_preempt,fallback])
    cancelling.add_children([is_cancel_requested, clr_goal3, clr_buffer3])
    get_goal.add_children([have_goal, get_buffer])
    get_buffer.add_children([apply_buffer,new_goal])
    apply_buffer.add_children([have_buffer,set_buffer])
    navigate.add_children([move_base, clr_goal1,clr_buffer])
    fallback.add_children([navigate, clr_goal2])
    
    #Initialise Blackboard Varibles
    py_trees.blackboard.Blackboard().set("InRecovery",False)
    return bt_root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node("BehaviorTree")
    ns = rospy.get_name() # Node name
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=30):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(500)