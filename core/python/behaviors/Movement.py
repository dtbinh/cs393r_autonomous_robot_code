import memory, pose, commands, cfgstiff, mem_objects, core, random, numpy, math
from task import Task
from state_machine import *
from memory import *
from sets import Set

last_head_time = 0
last_head_pan = 1
last_keeper_direction = 1

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
      global last_head_time, last_head_pan , last_keeper_direction

      commands.setHeadTilt(-14)
      if ((self.getTime() - last_head_time) > 4):
          if(last_head_pan >= 0):
            last_head_pan = -1
          else:
            last_head_pan = 1
          commands.setHeadPan( last_head_pan, 3.5)
          commands.setWalkVelocity(0.2,0.0*last_head_pan,0.05*last_head_pan)
          last_head_time = self.getTime()

      #commands.setWalkVelocity(0.0,0.35,0.04)

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
    global last_head_time
    last_head_time = 0
    stand = self.Stand()
    walk = self.Walk()
    tip = self.TurnInPlace()
    curve = self.Curve()
    sit = pose.Sit()
    goa = self.GoAround()
    off = self.Off()
    #self.trans(stand, C, walk, T(5.0), curve, T(5.0), sit, C, off)
    self.trans(stand, C, walk, T(100.0), sit, C, off)
    #self.trans(stand, C, walk, T(15.0), self.Stand() , C , self.Kick(), C, self.Stand(), C , sit, C, off)

