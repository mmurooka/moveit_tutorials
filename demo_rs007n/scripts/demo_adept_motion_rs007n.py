#!/usr/bin/env python

import sys
import copy
from math import pi
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class DemoAdeptMotion(object):
  """DemoAdeptMotion"""
  def __init__(self):
    super(DemoAdeptMotion, self).__init__()

    # initialize ROS and MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('demo_adept_motion', anonymous=True)

    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # print information
    # print "=" * 10, " Planning frame: %s" % move_group.get_planning_frame()
    # print "=" * 10, " End effector link: %s" % move_group.get_end_effector_link()
    # print "=" * 10, " Printing robot state"
    # print robot.get_current_state()

    self.robot = robot
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher

  def get_initial_pose(self):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0.15
    pose.position.y = 0.5
    pose.position.z = 0.335
    quat = tf.transformations.quaternion_from_euler(0, pi, pi/2, 'sxyz')
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

  def get_adept_waypoints(self):
    waypoints = []
    wpose = self.get_initial_pose()
    wpose.position.z += 0.025
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x -= 0.3
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    return waypoints

  def go_to_initial_pose(self):
    rospy.loginfo("Send zero joint angle.")
    joint_goal = [0.] * len(self.move_group.get_joints())
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

    rospy.loginfo("Send initial pose.")
    pose_goal = self.get_initial_pose()
    self.move_group.set_pose_target(pose_goal)
    self.move_group.go()
    self.move_group.stop()

  def plan_adept_motion(self):
    rospy.loginfo("Plan adept motion.")
    waypoints = self.get_adept_waypoints()
    (plan, fraction) = self.move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.005,        # eef_step
      0.0)         # jump_threshold

    return plan, fraction

  def apply_time_parametrization(self, plan, algorithm):
    ref_state = self.robot.get_current_state() # I don't know this is OK
    # ref_state = moveit_msgs.msg.RobotState()
    # ref_state.joint_state.name = plan.joint_trajectory.joint_names
    # ref_state.joint_state.position =  plan.joint_trajectory.points[0].positions

    if algorithm == "time_optimal_trajectory_generation":
      plan_retimed = self.move_group.retime_trajectory_totg(
        ref_state, plan,
        velocity_scaling_factor=0.1,
        acceleration_scaling_factor=0.1,
        resample_dt=0.03
      )
    else:
      plan_retimed = self.move_group.retime_trajectory(
        ref_state, plan,
        velocity_scaling_factor=0.1,
        acceleration_scaling_factor=0.1,
        algorithm=algorithm
      )

    print("motion duration before retime (%d points): %s [sec]"
          % (len(plan.joint_trajectory.points), plan.joint_trajectory.points[-1].time_from_start.to_sec()))
    print("motion duration after retime (%d points): %s [sec]"
          % (len(plan_retimed.joint_trajectory.points), plan_retimed.joint_trajectory.points[-1].time_from_start.to_sec()))
    return plan_retimed

  def execute_motion(self, plan):
    rospy.loginfo("Execute adept motion.")
    self.move_group.execute(plan, wait=True)

  def display_motion(self, plan):
    rospy.loginfo("Display adept motion.")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state() # I don't know this is OK
    display_trajectory.trajectory.append(plan)
    self.display_trajectory_publisher.publish(display_trajectory);

  def save_motion_graph(self, plan, pdf_suffix='', pdf_title=''):
    (fig, axes) = plt.subplots(nrows=3,ncols=1,figsize=(10,100))
    fig.suptitle(pdf_title, fontsize=28)

    joint_names = plan.joint_trajectory.joint_names
    joint_num = len(joint_names)
    time = [p.time_from_start.to_sec() for p in plan.joint_trajectory.points]

    axes[0].set_xlabel('time [sec]')
    axes[0].set_ylabel('position [rad]')
    for j in range(joint_num):
      pos = [p.positions[j] for p in plan.joint_trajectory.points]
      axes[0].plot(time, pos, linewidth=1, marker='o', label=joint_names[j])
    axes[0].legend()

    axes[1].set_xlabel('time [sec]')
    axes[1].set_ylabel('velocity [rad/sec]')
    for j in range(joint_num):
      pos = [p.velocities[j] for p in plan.joint_trajectory.points]
      axes[1].plot(time, pos, linewidth=1, marker='o', label=joint_names[j])

    axes[2].set_xlabel('time [sec]')
    axes[2].set_ylabel('acceleration [rad/sec^2]')
    for j in range(joint_num):
      pos = [p.accelerations[j] for p in plan.joint_trajectory.points]
      axes[2].plot(time, pos, linewidth=1, marker='o', label=joint_names[j])

    plt.pause(.001)
    filename='/tmp/demo_adept_motion_rs007n' + pdf_suffix + '.pdf'
    pdf = PdfPages(filename)
    pdf.savefig()
    pdf.close()
    rospy.loginfo('Save graph to %s' % filename)


def main():
  demo = DemoAdeptMotion()
  demo.go_to_initial_pose()
  plan, fraction = demo.plan_adept_motion()
  if fraction == 1.0:
    demo.save_motion_graph(plan, '', 'No Time Parametrization')
    plan_itp = demo.apply_time_parametrization(plan, "iterative_time_parameterization")
    demo.save_motion_graph(plan_itp, '_itp', 'Iterative Time Parametrization')
    plan_isp = demo.apply_time_parametrization(plan, "iterative_spline_parameterization")
    demo.save_motion_graph(plan_isp, '_isp', 'Iterative Spline Parametrization')
    plan_totg = demo.apply_time_parametrization(plan, "time_optimal_trajectory_generation")
    demo.save_motion_graph(plan_totg, '_totg', 'Time Optimal Trajectory Generation (New)')

    demo.execute_motion(plan)
  else:
    rospy.logerr('Plan failed.')

  import IPython
  IPython.embed()


if __name__ == '__main__':
  main()
