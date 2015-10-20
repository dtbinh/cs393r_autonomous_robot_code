import memory, pose, commands, cfgstiff
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
      if self.getTime() > 2.0:
        memory.speech.say("playing stand complete")
        self.finish()

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.4,0.0,0.1)

  class TurnInPlace(Node):
    def run(self):
      commands.setWalkVelocity(0,0,0.5)

  class GoAround(Node):
    def run(self):
      commands.setWalkVelocity(0,-0.5,0.25)

  class Curve(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0.25)

  class Kick(Node):
    def run(self):
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    walk = self.Walk()
    tip = self.TurnInPlace()
    curve = self.Curve()
    sit = pose.Sit()
    goa = self.GoAround()
    off = self.Off()
    #self.trans(stand, C, walk, T(5.0), curve, T(5.0), sit, C, off)
    self.trans(stand, C, walk, T(40.0), sit, C, off)
    #self.trans(stand, C, walk, T(15.0), self.Stand() , C , self.Kick(), C, self.Stand(), C , sit, C, off)

