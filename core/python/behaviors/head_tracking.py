import memory, pose, commands, core, cfgstiff
from memory import *
import mem_objects
from task import Task
from state_machine import *
import math, numpy

direction = 1.
carrot = 0
seen_ball = False
last_ball_heading = 0
last_ball_time = 0
alpha = 0.6
last_carrot = 0

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
      if self.getTime() > 5.0:
        memory.speech.say("playing stand complete")
        self.finish()

  class TrackGoal(Node):
    def run(self):
      global carrot, direction, seen_ball, last_ball_heading, last_ball_time, alpha, last_carrot
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      commands.setStiffness()
      feedback_rate = 0.2
      velocity = 2.0
      if(ball.seen):
        memory.speech.say("found ball")
        carrot = ball.visionBearing
        if(seen_ball):
          dt = self.getTime() - last_ball_time
          ball_vel = (ball.visionBearing - last_ball_heading) / dt
          direction = numpy.sign(ball_vel)
        else:
          last_carrot = carrot
        seen_ball = True
        last_ball_time = self.getTime()
        last_ball_heading = ball.visionBearing
      else:
        # memory.speech.say("searching")
        carrot += 0.01*direction
        if(carrot > (math.pi/2.0-0.1) and direction > 0.0):
          direction = -1.0
        elif(carrot < -(math.pi/2.0-0.1) and direction < 0.0):
          direction = 1.0

      last_carrot = alpha * last_carrot + (1-alpha) * carrot
      delta = abs(last_carrot - core.joint_values[core.HeadYaw])
      dt = numpy.maximum(0.1, delta / velocity)
      commands.setHeadPan(last_carrot, dt)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    track = self.TrackGoal()
    sit = pose.Sit()
    off = self.Off()  
    self.trans(stand, C, track, C, sit, C, off)
