import core, memory
import pose, commands, cfgstiff
import mem_objects
from task import Task
from state_machine import *
import random
import numpy

class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")
    self.setSubtask(pose.ToPose({ core.LShoulderRoll:80   , core.RShoulderRoll:5   , 
                                  core.LShoulderPitch:-20 , core.RShoulderPitch:-80 }
                                  , 0.5
                                )
                    )


class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
    self.setSubtask(pose.ToPose({ core.LShoulderRoll:5   , core.RShoulderRoll:80   , 
                                  core.LShoulderPitch:-80 , core.RShoulderPitch:-20 }
                                  , 0.5
                                )
                    )
    

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking center")
    self.setSubtask(pose.ToPose({ core.LShoulderRoll:20   , core.RShoulderRoll:20   , 
                                  core.LShoulderPitch:-20 , core.RShoulderPitch:-20 }
                                  , 0.5
                                )
                    )

class Blocker(Node):
  def run(self):
    pose.Sit()
    commands.setHeadPan(bearing, 0.1)
    ball = mem_objects.world_objects[core.WO_BALL]
    bearing = ball.bearing
    x  = ball.loc.x
    y  = ball.loc.y
    d  = ball.distance
    xv = ball.absVel.x
    yv = ball.absVel.y

    if x < 500:
      if xv > -10:
        print "ball moving too slowly"
        return

      print "Ball is close, blocking!\n"
      if ball.bearing > 30 * core.DEG_T_RAD:
        choice = "left"
        print "left\n"
      elif ball.bearing < -30 * core.DEG_T_RAD:
        choice = "right"
        print "right\n"
      else:
        choice = "center"
        print "center\n"
      self.postSignal(choice)

class Playing(LoopingStateMachine):
  class Stand(Node):
    def run(self):
      pose.Sit()
      if self.getTime() > 2.0:
        memory.speech.say("playing stand complete")
        self.finish()

  def setup(self):
    blocker = Blocker()
    blocks = {
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": BlockCenter()
    }
    commands.setStiffness()
    for name in blocks:
      b = blocks[name]
      self.trans(self.Stand(), C, blocker, S(name), b, T(3), blocker)
