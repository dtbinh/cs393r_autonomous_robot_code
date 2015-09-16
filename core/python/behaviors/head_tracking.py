import memory, commands, core
from memory import *
import mem_objects
from task import Task
from state_machine import *
import math

direction = 1.
carrot = 0

class Playing(Task):
  def run(self):
    global carrot, direction
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    commands.setStiffness()
    feedback_rate = 0.2
    velocity = 2.0
    if(ball.seen):
      memory.speech.say("found ball")
      carrot = ball.visionBearing
    else:
      memory.speech.say("searching")
      carrot += math.pi/128.0*direction
      if(carrot > (math.pi/2.0-0.1) and direction > 0.0):
        direction = -1.0
      elif(carrot < -(math.pi/2.0-0.1) and direction < 0.0):
        direction = 1.0

    delta = abs(carrot - core.joint_values[core.HeadYaw])
    commands.setHeadPan(carrot, delta / velocity)
