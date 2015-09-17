import memory, pose, commands, cfgstiff, math
from task import Task
from state_machine import *

direction = 1.
last_direction_change_time = 0.
class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      self.finish()

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class TurnInPlace(Node):
    def run(self):
      global direction, last_direction_change_time
      turn_amount = 2.0*math.pi
      turn_vel = 0.2
      turn_time = turn_amount / turn_vel
      commands.setWalkVelocity(0, 0, turn_vel * direction)
      if((self.getTime() - last_direction_change_time) > turn_time):
        direction = direction * -1.0
        last_direction_change_time = self.getTime()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    tip = self.TurnInPlace()
    sit = pose.Sit()
    off = self.Off()
    self.trans(stand, C, tip, C, sit, C, off)

