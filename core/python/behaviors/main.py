import memory, pose, commands, core, cfgstiff
from memory import *
import mem_objects
from task import Task
from state_machine import *
from math import pi
      
class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      memory.speech.say("ready to play")
      self.finish()

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        memory.speech.say("playing stand complete")
        self.finish()

  class TrackGoal(Node):
    def __init__(self):
      Node.__init__(self)
      self.scan_dir = 1.0
    def run(self):
      goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)
      print "goal distance = " + str(goal.visionDistance)
      turn_threshold = 0.1
      linear_threshold = 50
      target_distance = 2000
      kp_turn = 0.75
      kp_walk = 0.0001
      velocity = 0.5
      if(goal.seen):
        delta = abs(goal.visionBearing - core.joint_values[core.HeadYaw])
        commands.setHeadPan(goal.visionBearing, delta / velocity)

        bodyBearing = goal.visionBearing
        if(abs(bodyBearing) > turn_threshold):
          memory.speech.say("turning!")
          commands.setWalkVelocity(0, 0, kp_turn * bodyBearing)
        else: #body is pointed at the goal's center
          error = goal.visionDistance - target_distance
          if(abs(error) < linear_threshold):
            memory.speech.say("Reached the goal!")
            commands.setWalkVelocity(0, 0, 0)
            #self.finish()
          else:
            commands.setWalkVelocity(kp_walk * error, 0, 0)
            memory.speech.say("walking!")
      else: 
        #need to scan the head to find the goal
        if(self.scan_dir is 0.0):
          self.scan_dir is 1.0
        memory.speech.say("searching!")
        commands.setWalkVelocity(0, 0, 0)
        if(pi/2. - abs(core.joint_values[core.HeadYaw]) > 0.05):
          self.scan_dir = -self.scan_dir
        scan_vel = self.scan_dir*0.5
        commands.setHeadPan(core.joint_values[core.HeadYaw] + scan_vel * 0.1, 0.1)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    track = self.TrackGoal()
    sit = pose.Sit()
    off = self.Off()  
    self.trans(stand, C, track, C, sit, C, off)

