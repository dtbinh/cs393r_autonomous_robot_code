import core, memory
import pose, commands, cfgstiff
import mem_objects
from task import Task
from state_machine import *
import random


if_seen_history_constant = 5
if_seen_history = [0]*if_seen_history_constant
if_seen_history_counter = 0

last_inf_constant = 2
last_x = [0]*last_inf_constant
last_y = [0]*last_inf_constant
last_xv = [0]*last_inf_constant
last_yv = [0]*last_inf_constant
last_bearing = [0]*last_inf_constant
last_distance = [0]*last_inf_constant
last_state_counter = 0

dt = 1.0/30.0
friction = 0.2

last_head_time = 0
last_head_pan = 1.2

# class BlockLeft(Node):
#   def run(self):
#     UTdebug.log(15, "Blocking left")
#     self.setSubtask(pose.ToPose({ core.LKneePitch:50    , core.RKneePitch:50   ,
#                                   core.LHipPitch:-25    , core.RHipPitch:-25   ,
#                                   core.LAnklePitch:-25  , core.RAnklePitch:-25 ,
#                                   core.LShoulderRoll:80   , core.RShoulderRoll:5   , 
#                                   core.LShoulderPitch:-20 , core.RShoulderPitch:-80 }
#                                   , 0.5
#                                 )
#                     )


# class BlockRight(Node):
#   def run(self):
#     UTdebug.log(15, "Blocking right")
#     self.setSubtask(pose.ToPose({ core.LKneePitch:50    , core.RKneePitch:50   ,
#                                   core.LHipPitch:-25    , core.RHipPitch:-25   ,
#                                   core.LAnklePitch:-25  , core.RAnklePitch:-25 ,
#                                   core.LShoulderRoll:5   , core.RShoulderRoll:80   , 
#                                   core.LShoulderPitch:-80 , core.RShoulderPitch:-20 }
#                                   , 0.5
#                                 )
#                     )
    

# class BlockCenter(Node):
#   def run(self):
#     UTdebug.log(15, "Blocking center")
#     self.setSubtask(pose.ToPose({ core.LKneePitch:50    , core.RKneePitch:50   ,
#                                   core.LHipPitch:-25    , core.RHipPitch:-25   ,
#                                   core.LAnklePitch:-25  , core.RAnklePitch:-25 ,
#                                   core.LShoulderRoll:20   , core.RShoulderRoll:20   , 
#                                   core.LShoulderPitch:-20 , core.RShoulderPitch:-20 }
#                                   , 0.5
#                                 )
#                     )

class GoalieBlock(Node):
  def run(self):
    #UTdebug.log(15, "Blocking center")
    self.setSubtask(pose.ToPose({ 
                                  core.LHipYawPitch: -66 ,
                                  core.LHipRoll: -45 ,
                                  #core.LHipPitch: -48.7775090897009 ,
                                  core.LHipPitch: -40 ,
                                  core.LKneePitch: 125 ,
                                  core.LAnklePitch: -40 ,
                                  core.LAnkleRoll: 9.5 ,
                                  core.RHipYawPitch: -66 ,
                                  core.RHipRoll: -45 ,
                                  #core.RHipPitch: -28.1277571908664 ,
                                  core.RHipPitch: -40 ,
                                  core.RKneePitch: 125 ,
                                  core.RAnklePitch: -40 ,
                                  core.RAnkleRoll: 9.5 ,
                                  core.LShoulderPitch: -85,
                                  core.LShoulderRoll: 28 ,
                                  core.RShoulderPitch: -85 ,
                                  core.RShoulderRoll: 28
                                  }
                                  , 1.5
                                )
                    )

class WalkingLeft(Node):
  def run(self):
    commands.setWalkVelocity(0.0,0.4,-0.04)

class WalkingRight(Node):
  def run(self):
    commands.setWalkVelocity(0.0,-0.4,0.04)

class WalkingCenter(Node):
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)

class Blocker(Node):
  def run(self):
    global if_seen_history_constant , last_inf_constant
    global if_seen_history , if_seen_history_counter , last_x , last_y , last_xv , last_yv , last_bearing , last_distance , last_state_counter
    global friction, dt
    global last_head_time, last_head_pan

    commands.stand()
    commands.setHeadTilt(-15)
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
    if(seen_times > ( if_seen_history_constant/2 )): # It is actually seen
      #print "seen_times = " + str(seen_times) 
      if(ball.seen):
        bearing = ball.visionBearing
        distance = ball.visionDistance
        x  = ball.loc.x
        y  = ball.loc.y
        xv = ball.absVel.x
        yv = ball.absVel.y
      else:
        return
  
      if( abs(xv) > 2500 or abs(yv) > 2500) :
        return

      print "========================================================================="
      #print "x = " + str(x) + " y = " + str(y) + " xv = " + str(xv) +" yv = " + str(yv)
      
      last_av_bearing = sum(last_bearing)/last_inf_constant

      last_x[last_state_counter]  = x
      last_y[last_state_counter]  = y
      last_xv[last_state_counter] = xv
      last_yv[last_state_counter] = yv
      last_distance[last_state_counter] = distance
      last_bearing[last_state_counter] = bearing
      last_state_counter = (last_state_counter + 1)%last_inf_constant

      # av_x = x
      # av_y = y
      # av_xv = xv
      # av_yv = yv
      # av_distance = distance
      # av_bearing = bearing

      av_x = sum(last_x)/last_inf_constant
      av_y = sum(last_y)/last_inf_constant
      av_xv = sum(last_xv)/last_inf_constant
      av_yv = sum(last_yv)/last_inf_constant
      av_distance = sum(last_distance)/last_inf_constant
      av_bearing = sum(last_bearing)/last_inf_constant

      print "Avg : av_x = " + str(av_x) + " av_y = " + str(av_y) + " av_xv = " + str(av_xv) +" av_yv = " + str(av_yv)

      if(av_bearing > 1.2):
        av_bearing = 1.2
      elif(av_bearing < -1.2):
        av_bearing = -1.2

      if(abs(av_xv) > 100 or abs(av_yv) > 100 ):
        d_turning = abs(av_bearing - last_av_bearing)/3.0
        if(d_turning < 0.1):
          d_turning = 0.1
        elif(d_turning > 2):
          d_turning = 2
        commands.setHeadPan(av_bearing, d_turning)
      # else:
      #   if ((self.getTime() - last_head_time) > 2.0):
      #     if(last_head_pan == 1.2):
      #       last_head_pan = -1.2
      #     else:
      #       last_head_pan = 1.2
      #     commands.setHeadPan( last_head_pan, 2.0 )
      #     last_head_time = self.getTime()

      #print "av_x = " + str(av_x) + "\tav_y = " + str(av_y)
      px = av_x + (av_xv*av_xv/(2*1000*friction))

      if(av_xv > -100 or (abs(av_yv)+0.1)/(abs(av_xv)+0.1) > 1):
        print(" No!!!!: Vx > 0 or Vx / Vy large , Vx = ") + str(av_xv) + " Vy = " + str(av_yv) + " seen_times = " + str(seen_times)
        return
      elif( px > -500 ):
        print(" No!!!!: Ball too short px = ") + str(px) + " seen_times = " + str(seen_times)
        return 
      elif( av_distance < 800 ):
        lamda = av_yv / av_xv
        intercept = av_y - lamda*av_x
        hit_goal_line = lamda*(-1200) + intercept

        print " Yes!!!: av_yv = " + str(av_yv) + " av_xv = " + str(av_xv) + " hit_goal_line = " + str(hit_goal_line)
        if( hit_goal_line < -250 ):
           choice = "right"
           self.postSignal(choice)
        elif( hit_goal_line >  250 ):
           choice = "left"
           self.postSignal(choice)
        elif( hit_goal_line > -250 and hit_goal_line < 250):
           choice = "center"
           self.postSignal(choice)

    else: # Think ball is still not seen
      #print( "No Ball\n" )
      commands.setHeadPan( 0 , 0.2 )
      last_x = [0]*last_inf_constant
      last_y = [0]*last_inf_constant
      last_xv = [0]*last_inf_constant
      last_yv = [0]*last_inf_constant
      last_distance =[0]*last_inf_constant
      last_bearing = [0]*last_inf_constant
      #memory.speech.say("oh on")

    if_seen_history_counter = (if_seen_history_counter + 1)%if_seen_history_constant;

class Playing(LoopingStateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 2.0:
        memory.speech.say("Be a man")
        self.finish()

  def setup(self):
    blocker = Blocker()
    goalieblock = GoalieBlock()
    blocks = {
      "left": WalkingLeft(),
      "right": WalkingRight(),
      "center": WalkingCenter(),
    }
    for name in blocks:
      #b = blocks[name]
      self.trans(self.Stand() , C , blocker, S(name), blocks[name] , T(1), goalieblock , T(2) , blocker)
