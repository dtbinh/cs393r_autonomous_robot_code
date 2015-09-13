import memory, commands, core
from memory import *
import mem_objects
from task import Task
from state_machine import *

class Playing(Task):
  def run(self):
    memory.speech.say("Running the correct code now!")
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    commands.setStiffness()
    #while(True):
    feedback_rate = 0.2
    velocity = 0.5
    #if((vision_frame_info.frame_id % 30*feedback_rate) == 0):
    if(ball.seen):
      delta = abs(ball.visionBearing - core.joint_values[core.HeadYaw])
      print "setting head position to " + str(ball.visionBearing)
      commands.setHeadPan(ball.visionBearing, delta / velocity)
    #self.finish()
