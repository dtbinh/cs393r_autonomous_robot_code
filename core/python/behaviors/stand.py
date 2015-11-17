import memory, pose, commands, cfgstiff, cfgpose
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
      #commands.setStiffness(cfgstiff.Zero, 0.3)
      if self.getTime() > 5:
        memory.speech.say("playing stand complete")
        self.finish()

  class Stiff(Node):
    def run(self):
      commands.setStiffness(cfgstiff.kickStand)

  class Off(Node):
    def run(self):
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    self.trans(stand, C, self.Stiff() , T(500) , self.Off())
