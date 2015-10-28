import core, memory
import pose, commands, cfgstiff, cfgpose
import mem_objects
from task import Task
from state_machine import *
import random


if_seen_history_constant =9
if_seen_history = [0]*if_seen_history_constant
if_seen_history_counter = 0

last_inf_constant = 2
last_x = [0]*last_inf_constant
last_y = [0]*last_inf_constant
last_xv = [0]*last_inf_constant
last_yv = [0]*last_inf_constant
last_bearing = [0]*last_inf_constant
last_distance = [0]*last_inf_constant
last_av_px = [0]*last_inf_constant
last_state_counter = 0

dt = 1.0/30.0
friction = 0.35

last_head_time = 0
last_head_pan = 1.2
body_turning_flag = 0
block_trigger_flag = 0

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
    self.setSubtask(pose.ToPose({ 
                                  core.LHipYawPitch: -50.7990128575929,
                                  core.LHipRoll: -29.7098065875597,
                                  core.LHipPitch: -36.1211002557756,
                                  core.LKneePitch: 123.397585319279,
                                  core.LAnklePitch: -48.8526839844773,
                                  core.LAnkleRoll: 12.8,
                                  core.RHipYawPitch: -50,
                                  core.RHipRoll: -30.6718114113993,
                                  core.RHipPitch: -37.3563946086858,
                                  core.RKneePitch: 125.072320383009,
                                  core.RAnklePitch: -48.5,
                                  core.RAnkleRoll: 11.4,
                                  core.LShoulderPitch: -79,
                                  core.LShoulderRoll: 25,
                                  core.RShoulderPitch: -79,
                                  core.RShoulderRoll: 26
                                  }
                                  , 1.0
                                )
                    )

class Sitting(Node):
  def run(self):
    self.setSubtask(pose.ToPose({ 
                                  core.HeadYaw: 1.8433177928247,
                                  core.HeadPitch: -22.497878144835,
                                  core.LHipYawPitch: -4.18321199506924,
                                  core.LHipRoll: 0.261268396131328,
                                  core.LHipPitch: -46.5802099129511,
                                  core.LKneePitch: 123.485476193518,
                                  core.LAnklePitch: -70.5794592600033,
                                  core.LAnkleRoll: -0.0902951008275686,
                                  core.RHipYawPitch: -5.18321199506924,
                                  core.RHipRoll: 0.266076849307017,
                                  core.RHipPitch: -47.2002681461832,
                                  core.RKneePitch: 124.72075688605,
                                  core.RAnklePitch: -70.3988690583481,
                                  core.RAnkleRoll: -0.0854866476518796,
                                  core.LShoulderPitch: -92.0202358571845,
                                  core.LShoulderRoll: 2.54645844712083,
                                  core.RShoulderPitch: -92.1129420147891,
                                  core.RShoulderRoll: 2.55126690029652,
                                  }
                                  , 1.5
                                )
                    )

# class GoalieBlock(Node):
#   def run(self):
#     self.setSubtask(pose.PoseSequence(
#       cfgpose.blockright, 1.0,
#       cfgpose.blockright, self.time, 
#       cfgpose.sittingPoseNoArms, 2.0,
#       cfgpose.standingPose, 2.0
#     ))

class Walking(Node):
  def run(self):
    commands.setHeadTilt(-13)
    commands.setWalkVelocity(0.20,0.1,0.04)

class WalkingLeft(Node):
  def run(self):
    commands.setWalkVelocity(0.0,0.5,0.05)

class WalkingRight(Node):
  def run(self):
    commands.setWalkVelocity(0.0,-0.5,0.05)

class WalkingCenter(Node):
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)

#class WalkingandTracking(Node)

class Blocker(Node):
  def run(self):
    global if_seen_history_constant , last_inf_constant
    global if_seen_history , if_seen_history_counter , last_x , last_y , last_xv , last_yv , last_bearing , last_distance , last_state_counter,last_av_px
    global friction, dt
    global last_head_time, last_head_pan, body_turning_flag, block_trigger_flag

    #commands.stand()
    commands.setHeadTilt(-13)
    ball = mem_objects.world_objects[core.WO_BALL]
    if_seen_history[if_seen_history_counter] = ball.seen;
    if_seen_history_counter = (if_seen_history_counter + 1)%if_seen_history_constant;
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
  
      if( abs(xv) > 3000 or abs(yv) > 3000) :
        return

      
      #print "x = " + str(x) + " y = " + str(y) + " xv = " + str(xv) +" yv = " + str(yv)
      
      last_av_bearing = sum(last_bearing)/last_inf_constant

      last_x[last_state_counter]  = x
      last_y[last_state_counter]  = y
      last_xv[last_state_counter] = xv
      last_yv[last_state_counter] = yv
      last_distance[last_state_counter] = distance
      last_bearing[last_state_counter] = bearing
      

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

      px = av_x + (av_xv*abs(av_xv)/(2*1000*friction))
      last_av_px[last_state_counter] = px
      av_px = sum(last_av_px)/last_inf_constant
      last_state_counter = (last_state_counter + 1)%last_inf_constant

      #print "av_vx = " + str(av_xv) + "\tav_vy = " + str(av_yv)
      print("av_px = ") + str(av_px) + " \tseen_times = " + str(seen_times)

      #print "Avg : av_x = " + str(av_x) + " av_y = " + str(av_y) + " av_xv = " + str(av_xv) +" av_yv = " + str(av_yv)

      if(av_bearing > 1.4):
        av_bearing = 1.4
      elif(av_bearing < -1.4):
        av_bearing = -1.4

      d_turning = abs(av_bearing - last_av_bearing)/2.5
      if(d_turning < 0.1):
        d_turning = 0.1
      elif(d_turning > 2):
        d_turning = 2
      commands.setHeadPan(av_bearing, d_turning)

      # print "av_xy = " + str(av_xv) + "\tav_y = " + str(av_yv) + "\tflag = " + str(body_turning_flag) + "\tPAN: " + str(core.joint_values[core.HeadYaw])
      # if( (abs(av_xv) < 300 and abs(av_yv) < 300 ) or body_turning_flag == 1):
      #   head_pan = core.joint_values[core.HeadYaw]
      #   if(head_pan > 0.2):
      #     body_turning_flag = 1
      #     commands.setWalkVelocity(0.0, 0.3, 0.05)
      #   elif(head_pan < -0.2):
      #     body_turning_flag = 1
      #     commands.setWalkVelocity(0.0,-0.3,-0.05)
      #   else:
      #     body_turning_flag = 0

      if(av_xv > -200 or (abs(av_yv)+0.01)/(abs(av_xv)+0.01) > 1):
        print(" No!!!!: Vx > 0 or Vx / Vy large , Vx = ") + str(av_xv) + " Vy = " + str(av_yv) + " seen_times = " + str(seen_times)
        block_trigger_flag = 0
        return
      elif( av_px > -1100 ):
        print(" No!!!!: Ball too short px = ") + str(av_px) + " seen_times = " + str(seen_times)
        block_trigger_flag = 0
        return 
      elif( av_distance < 1200 and av_xv < -100):
        block_trigger_flag = block_trigger_flag + 1
        lamda = av_yv / av_xv
        intercept = av_y - lamda*av_x
        hit_goal_line = lamda*(-1300) + intercept

        if(block_trigger_flag == 2):
          print "========================================================================="
          print " Yes: av_px = " + str(av_px) + " hit_goal_line = " + str(hit_goal_line)
          block_trigger_flag = 0
          if( hit_goal_line < -250 and hit_goal_line > -550):
             choice = "right"
             self.postSignal(choice)
          elif( hit_goal_line >  250 and hit_goal_line < 550):
             choice = "left"
             self.postSignal(choice)
          elif( hit_goal_line > -250 and hit_goal_line < 250):
             choice = "center"
             self.postSignal(choice)

    else: # Think ball is still not seen
      #memory.speech.say("No ball")
      if ((self.getTime() - last_head_time) > 3):
        if(last_head_pan > 0 ):
          last_head_pan = -1.2
        elif(last_head_pan <= 0):
          last_head_pan = 1.2
        commands.setHeadPan( last_head_pan , 2.5 )
        last_head_time = self.getTime()
      last_x = [0]*last_inf_constant
      last_y = [0]*last_inf_constant
      last_xv = [0]*last_inf_constant
      last_yv = [0]*last_inf_constant
      last_distance =[0]*last_inf_constant
      last_bearing = [0]*last_inf_constant


class Playing(LoopingStateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      global last_head_time
      last_head_time = 0
      if self.getTime() > 2.0:
        memory.speech.say("Be a man")
        self.finish()

  def setup(self):
    blocker = Blocker()
    goalieblock = GoalieBlock()
    walk = Walking()
    sit = Sitting()
    blocks = {
      "left": WalkingLeft(),
      "right": WalkingRight(),
      "center": WalkingCenter(),
    }
    for name in blocks:
      #b = blocks[name]
      #self.trans(self.Stand() , C , blocker, S(name), blocks[name] , T(0), goalieblock , T(4.5), sit, T(2) , self.Stand(), C , blocker)
      self.trans(self.Stand() , C , blocker, S(name), blocks[name] , T(0), goalieblock , T(4.5), sit, T(2) , self.Stand(), C , blocker)