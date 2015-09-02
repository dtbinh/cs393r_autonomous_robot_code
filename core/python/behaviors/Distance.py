import memory, pose, commands, core, cfgstiff
from memory import *
import mem_objects
from task import Task
from state_machine import *
      
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
    def run(self):
      goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)
      print "goal distance = " + str(goal.visionDistance)
      turn_threshold = 0.05
      linear_threshold = 50
      target_distance = 2000
      kp_turn = 0.75
      kp_walk = 0.001
      if(goal.seen):
        if(abs(goal.visionBearing) > turn_threshold):
          memory.speech.say("turning!")
          commands.setWalkVelocity(0, 0, kp_turn * goal.visionBearing)
        else: #body is pointed at the goal's center
          error = goal.visionDistance -target_distance
          if(abs(error) < linear_threshold):
            memory.speech.say("Reached the goal!")
            commands.setWalkVelocity(0, 0, 0)
            #self.finish()
          else:
            commands.setWalkVelocity(kp_walk * error, 0, 0)
            memory.speech.say("walking!")
      else:
        pass
        #need to scan the head to find the goal

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

