import memory, pose, commands, cfgstiff
from task import Task
from state_machine import *

kick_flag = 0

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class Kick(Node):
    def run(self):
      global kick_flag;

      if self.getFrames() <= 3 and kick_flag == 0:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
        kick_flag = 1
      #if self.getFrames() > 10 and not memory.kick_request.kick_running_:
      if not memory.kick_request.kick_running_:
        kick_flag = 0
        self.finish()

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Stiff(Node):
    def run(self):
      commands.setStiffness(cfgstiff.kickStand)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Kick(), C , self.Stand() , C)
