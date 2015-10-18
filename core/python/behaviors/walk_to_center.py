import memory, pose, commands, cfgstiff, mem_objects, core, random, numpy
from task import Task
from state_machine import *

class Ready(Task):
  def run(self):
    print "Readying"
    commands.stand()
    if self.getTime() > 5.0:
      self.finish()

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      print "Standing"
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class Walk(Node):
    def run(self):
      print "Spewing"
      sloc = mem_objects.world_objects[core.WO_SELF].loc
      print "I'm at: " + str(sloc)
      commands.setWalkVelocity(0.0,0.0,0.0)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Walk(), C, self.Off())

