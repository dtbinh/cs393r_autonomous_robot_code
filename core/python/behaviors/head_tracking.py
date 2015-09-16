import memory, commands, core
from memory import *
import mem_objects
from task import Task
from state_machine import *

class Playing(Task):
  def run(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    commands.setStiffness()
    feedback_rate = 0.2
    velocity = 0.5
    if(ball.seen):
      delta = abs(ball.visionBearing - core.joint_values[core.HeadYaw])
      commands.setHeadPan(ball.visionBearing, delta / velocity)
      #commands.setHeadPanTilt(ball.visionBearing, ball.visionElevation, delta / velocity)
