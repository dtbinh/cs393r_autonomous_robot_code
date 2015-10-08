import core, memory
import pose, commands, cfgstiff
import mem_objects
from task import Task
from state_machine import *
import random

if_seen_history = [0]*5
if_seen_history_counter = 0
last_x = [0]*3
last_y = [0]*3
last_xv = [0]*3
last_yv = [0]*3
last_bearing = [0]*3
last_distance = [0]*3
last_state_counter = 0
dt = 1.0/30.0
friction = 0.2

class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")
    self.setSubtask(pose.ToPose({ core.LKneePitch:50    , core.RKneePitch:50   ,
                                  core.LHipPitch:-25    , core.RHipPitch:-25   ,
                                  core.LAnklePitch:-25  , core.RAnklePitch:-25 ,
                                  core.LShoulderRoll:80   , core.RShoulderRoll:5   , 
                                  core.LShoulderPitch:-20 , core.RShoulderPitch:-80 }
                                  , 0.5
                                )
                    )


class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
    self.setSubtask(pose.ToPose({ core.LKneePitch:50    , core.RKneePitch:50   ,
                                  core.LHipPitch:-25    , core.RHipPitch:-25   ,
                                  core.LAnklePitch:-25  , core.RAnklePitch:-25 ,
                                  core.LShoulderRoll:5   , core.RShoulderRoll:80   , 
                                  core.LShoulderPitch:-80 , core.RShoulderPitch:-20 }
                                  , 0.5
                                )
                    )
    

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking center")
    self.setSubtask(pose.ToPose({ core.LKneePitch:50    , core.RKneePitch:50   ,
                                  core.LHipPitch:-25    , core.RHipPitch:-25   ,
                                  core.LAnklePitch:-25  , core.RAnklePitch:-25 ,
                                  core.LShoulderRoll:20   , core.RShoulderRoll:20   , 
                                  core.LShoulderPitch:-20 , core.RShoulderPitch:-20 }
                                  , 0.5
                                )
                    )


class Blocker(Node):
  def run(self):
    global if_seen_history , if_seen_history_counter , last_x , last_y , last_xv , last_yv , last_bearing , last_distance , last_state_counter
    global friction, dt

    commands.stand()
    ball = mem_objects.world_objects[core.WO_BALL]
    if_seen_history[if_seen_history_counter] = ball.seen;
    seen_times = sum(if_seen_history)

    x = 0.
    y = 0.
    d = 0.
    bearing = 0.
    xv = 0.
    yv = 0.
    px = 0.
    if(seen_times > 2): # It is actually seen
      #memory.speech.say("oh")
      #print "seen_times = " + str(seen_times) 
      if(ball.seen):
        bearing = ball.bearing
        distance = ball.distance
        x  = ball.loc.x
        y  = ball.loc.y
        d  = ball.distance
        xv = ball.absVel.x
        yv = ball.absVel.y
      else:
        return
        # bearing = 2*last_bearing[0] - last_bearing[1]
        # d  = 2*last_distance[0] - last_distance[1]
        # x  = 2*last_x[0] - last_x[1]
        # y  = 2*last_y[0] - last_y[1]
        # xv = 2*last_xv[0]- last_xv[1]
        # yv = 2*last_yv[0]- last_yv[1]

      if( abs(xv) > 2500 or abs(yv) > 2500) :
        return
      print "========================================================================="
      #print "x = " + str(x) + " y = " + str(y) + " xv = " + str(xv) +" yv = " + str(yv)
      last_av_bearing = sum(last_bearing)/3.0

      last_x[last_state_counter]  = x
      last_y[last_state_counter]  = y
      last_xv[last_state_counter] = xv
      last_yv[last_state_counter] = yv
      last_distance[last_state_counter] = d
      last_bearing[last_state_counter] = bearing
      last_state_counter = (last_state_counter + 1)%3;


      av_x = x
      av_y = y
      av_xa = friction*xv 
      av_ya = friction*yv 
      av_xv = xv
      av_yv = yv
      av_distance = distance
      av_bearing = bearing

      print "Avg : av_x = " + str(av_x) + " av_y = " + str(av_y) + " av_xv = " + str(av_xv) +" av_yv = " + str(av_yv)

      if(av_bearing > 0.75):
        av_bearing = 0.75
      elif(av_bearing < -0.75):
        av_bearing = -0.75

      d_turning = abs(av_bearing - last_av_bearing)/4.0
      if(d_turning < 0.1):
        d_turning = 0.1
      commands.setHeadPan(av_bearing, d_turning)

      #print "!!!!!!!!!!!!!av_x = " + str(av_x) + "!!!!!!!!!!!!!av_y = " + str(av_y)

      # Update x , y , d , xv , yv stable and the way to estimate parameters without seeing the ball
      if(av_xa == 0):
        return
      px = av_x + (av_xv*av_xv/(2*av_xa))

      if(av_xv > -300 or av_xv == 0 or (abs(av_yv)+0.1)/(abs(av_xv)+0.1) > 0.8):
        print(" No!!!!: Vx > 0 or Vx / Vy large , Vx = ") + str(av_xv) + " Vy = " + str(av_yv) + " seen_times = " + str(seen_times)
        return
      elif( px > -350 ):
        print(" No!!!!: Ball too short           seen_times = ")  + str(seen_times)
        return 
      elif( av_distance < 1100 ):
        #print( "Ball is close, blocking! seen_times = " ) + str(seen_times)
        lamda = av_yv / av_xv
        intercept = av_y - lamda*av_x
        hit_goal_line = lamda*(-700) + intercept
        print " Yes!!!: av_yv = " + str(av_yv) + " av_xv = " + str(av_xv) + " hit_goal_line = " + str(hit_goal_line)
        if( hit_goal_line < -125 and hit_goal_line > -500 ):
           choice = "right"
           self.postSignal(choice)
        elif( hit_goal_line >  125 and hit_goal_line < 500):
           choice = "left"
           self.postSignal(choice)
        elif( hit_goal_line > -125 and hit_goal_line < 125):
           choice = "center"
           self.postSignal(choice)
    else: # Think ball is still not seen
      #print( "No Ball\n" )
      commands.setHeadPan( 0 , 0.2 )
      last_x = [0]*3
      last_y = [0]*3
      last_xv = [0]*3
      last_yv = [0]*3
      last_xa = [0]*3
      last_ya = [0]*3
      last_distance =[0]*3
      last_bearing = [0]*3
      #memory.speech.say("oh on")

    if_seen_history_counter = (if_seen_history_counter + 1)%5;

class Playing(LoopingStateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 2.0:
        memory.speech.say("playing stand complete")
        self.finish()

  def setup(self):
    blocker = Blocker()
    blocks = {
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": BlockCenter(),
    }
    for name in blocks:
      b = blocks[name]
      self.trans(self.Stand() , C , blocker, S(name), b, T(5), blocker)
