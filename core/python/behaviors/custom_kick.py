import memory, pose, commands, cfgstiff, sensors
from task import Task
from state_machine import *
import cfgpose, cfgstiff
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles
from kick_trajectory import trajectory, names

start_frame = -1
class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        self.finish()

  class Kick(Node):
    def run(self):
      global start_frame
      if(start_frame == -1):
        start_frame = vision_frame_info.frame_id

      traj_idx = vision_frame_info.frame_id - start_frame
      if(traj_idx > len(trajectory)):
        start_frame = -1
        self.finish()

      pose = trajectory[traj_idx]
      for i in range(2, core.NUM_JOINTS):
        joint_commands.setJointCommand(i, pose[i])

      #tweak left ankle commands to handle cop dynamics
      cop_x = sensors.getValue(core.LFoot_FSR_CenterOfPressure_X)
      cop_y = sensors.getValue(core.LFoot_FSR_CenterOfPressure_Y)

      kroll = 0.01
      kpitch = 0.01
      droll = -cop_y * kroll
      dpitch = cop_x * kpitch 

      joint_commands.setJointCommand(core.LAnklePitch, pose[core.LAnklePitch] + dpitch)
      joint_commands.setJointCommand(core.LAnkleRoll, pose[core.LAnkleRoll] + droll)

      #send commands and prevent other controllers from operating
      joint_commands.send_body_angles_ = True
      joint_commands.body_angle_time = 1000.0 #todo: figure out what units this is
      walk_request.noWalk()
      kick_request.setNoKick()

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
