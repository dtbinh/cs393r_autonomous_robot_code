import memory, commands, core
from memory import *
import mem_objects
from task import Task
from state_machine import *

class Playing(Task):
  def run(self):
    self.finish()
