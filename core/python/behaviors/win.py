import memory, pose, commands, head, util, cfgpose, cfgstiff, mem_objects, core, random, numpy, math
from task import Task
from state_machine import *
from memory import *
from sets import Set
#from enum import Enum

class Ready(Task):
  def run(self):
    commands.stand()
    if self.getTime() > 3.0:
      self.finish()

#Fields = Enum('Fields', 'A B')
#Modes = Enum('Modes', 'passive attacking defending')
#AttackingStates = Enum('AttackingStates', 'start approach rotate dribble align kick')
#DefendingStates = Enum('DefendingStates', 'start')
class Fields: 
  A=0
  B=1

class Modes:
  passive=0
  attacking=1
  defending=2

class EnemyGoalStates:
  unknown=0
  center=1
  left=2
  right=3

class AttackingStates:
  start=0
  approach=1
  rotate=2
  dribble=3
  align=4
  kick=5

class DefendingStates:
  start=0
  walk_center=1
  walk_left=2
  walk_right=3
  walk=4
  block=5
  sit=6
  block_left=7
  block_right=8

field = Fields.A
mode = Modes.passive
enemy_state = EnemyGoalStates.unknown
current_state = AttackingStates.start
last_mode_change_time = 0
r_goal_dist_filtered = 0.

#attacking globals
rotation_dir = 1.
last_ball_seen_time = 0
next_head_time = 0
kick_start_time = 0
kick_sent = False
recovering_from_kick = False
attack_left = False

#defending globals
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
pose_sent = False
num_times_sent = 0
pose_start_time = 0
defense_time = 0
defense_delay_time = 0
walk_start_time = 0

def tanhController(err, kx, max_cmd):
  return max_cmd * numpy.tanh(kx * err)

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class Win(Node):
    def toPose(self, pose, time):
      global pose_sent, pose_start_time, num_times_sent
      commands.setStiffness()
      times_to_send = 1
      if num_times_sent < times_to_send:
        for i in range(2, core.NUM_JOINTS):
          val = util.getPoseJoint(i, pose, False)
          if val != None:
            joint_commands.setJointCommand(i, val * core.DEG_T_RAD)

        joint_commands.send_body_angles_ = True
        joint_commands.body_angle_time_ = time * 1000.0
        walk_request.noWalk()
        kick_request.setNoKick()
        num_times_sent = num_times_sent+1
        pose_start_time = self.getTime()
        #print "Sending pose at time " + str(pose_start_time) + "! #" + str(num_times_sent)

      if num_times_sent >= times_to_send and ((self.getTime() - pose_start_time) > time):
        #print "DONE!"
        return True
      else:
        return False

    def walk(self, vx, vy, vt):
      commands.setWalkVelocity(vx+0.1, vy+0.0, vt+0.0)

    def stop(self):
      commands.setWalkVelocity(0.,0.,0.)

    def track_ball(self):
      commands.setHeadPan(0, 2.0)
      # global last_ball_seen_time, next_head_time
      # ball = memory.world_objects.getObjPtr(core.WO_BALL)
      # max_vel = 10.0
      # if ball.seen:
      #   last_ball_seen_time = self.getTime()
      #   return
      #   commands.setHeadPan(ball.visionBearing, numpy.abs(core.joint_values[core.HeadYaw] - ball.visionBearing) / max_vel)
      # else:
      #   if((self.getTime() - last_ball_seen_time) > 1.0 and self.getTime() > next_head_time):
      #     target = random.uniform(-1.5, 1.5)
      #     dt = numpy.abs(core.joint_values[core.HeadYaw] - target) / max_vel
      #     commands.setHeadPan(target, dt)
      #     next_head_time = self.getTime() + dt

    def walk_to(self, obj, r_threshold):
      global have_lock, facing_center, at_center
      # memory.speech.say("Walking!")
      sloc = mem_objects.world_objects[robot_state.WO_SELF].loc
      t = -mem_objects.world_objects[robot_state.WO_SELF].orientation
      cx = obj.visionDistance * numpy.cos(obj.visionBearing)
      cy = obj.visionDistance * numpy.sin(obj.visionBearing)
      r = numpy.sqrt(cx*cx+cy*cy)
      t_err = numpy.arctan2(cy, cx)
      #print "local target x,y = " + str(cx) + "," + str(cy)

      if(r < r_threshold):
        #print "Stopping"
        self.stop()
        return True

      x_vel = tanhController(cx, 10.0 / 1000.0, 0.4)
      y_vel = tanhController(cy, 10.0 / 1000.0, 0.4)
      t_vel = 0.0
      #print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      self.walk(x_vel,y_vel,t_vel)
      return False

#DEFENSE============================================================================================
    def defense_start(self):
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir
      global if_seen_history_constant , last_inf_constant
      global if_seen_history , if_seen_history_counter , last_x , last_y , last_xv , last_yv , last_bearing , last_distance , last_state_counter,last_av_px
      global friction, dt
      global last_head_time, last_head_pan, body_turning_flag, block_trigger_flag, pose_sent, num_times_sent

      # commands.stand()
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
        #print("av_px = ") + str(av_px) + " \tseen_times = " + str(seen_times)

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

        #print "av_xy = " + str(av_xv) + "\tav_y = " + str(av_yv) + "\tflag = " + str(body_turning_flag) + "\tPAN: " + str(core.joint_values[core.HeadYaw])
        if( (abs(av_xv) < 300 and abs(av_yv) < 300 ) or body_turning_flag == 1):
          head_pan = core.joint_values[core.HeadYaw]
          if(head_pan > 0.2):
            body_turning_flag = 1
            commands.setWalkVelocity(0.1, 0.4, 0.13)
          elif(head_pan < -0.2):
            body_turning_flag = 1
            commands.setWalkVelocity(0.15,-0.3,-0.05)
          else:
            commands.stand()
            body_turning_flag = 0
          return

        if(av_xv > -200 or (abs(av_yv)+0.01)/(abs(av_xv)+0.01) > 1):
          #print(" No!!!!: Vx > 0 or Vx / Vy large , Vx = ") + str(av_xv) + " Vy = " + str(av_yv) + " seen_times = " + str(seen_times)
          block_trigger_flag = 0
          return
        elif( av_px > -1100 ):
          #print(" No!!!!: Ball too short px = ") + str(av_px) + " seen_times = " + str(seen_times)
          block_trigger_flag = 0
          return 
        elif( av_distance < 1200 and av_xv < -100):
          block_trigger_flag = block_trigger_flag + 1
          lamda = av_yv / av_xv
          intercept = av_y - lamda*av_x
          hit_goal_line = lamda*(-1300) + intercept

          if(block_trigger_flag > 2):
            #print "========================================================================="
            #print "========================================================================="
            #print "========================================================================="
            #print "========================================================================="
            #print "========================================================================="
            #print "========================================================================="
            
            block_trigger_flag = 0
            if( hit_goal_line < -200 and hit_goal_line > -550):
              #print " Left : Yes: av_px = " + str(av_px) + " hit_goal_line = " + str(hit_goal_line)
              pose_sent = False
              num_times_sent = 0
              current_state = DefendingStates.block_right
              #choice = "right"
              #self.postSignal(choice)
            elif( hit_goal_line >  200 and hit_goal_line < 550):
              #print " Right : av_px = " + str(av_px) + " hit_goal_line = " + str(hit_goal_line)
              pose_sent = False
              num_times_sent = 0
              current_state = DefendingStates.block_left
              # choice = "left"
              # self.postSignal(choice)
            elif( hit_goal_line > -200 and hit_goal_line < 200):
              #print " Center: av_px = " + str(av_px) + " hit_goal_line = " + str(hit_goal_line)
              pose_sent = False
              num_times_sent = 0
              current_state = DefendingStates.block
              # choice = "center"
              # self.postSignal(choice)

      else: # Think ball is still not seen
        #memory.speech.say("No ball")
        commands.stand()
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

    def defense_block(self):
      global current_state, DefendingStates, pose_sent, pose_start_time, num_times_sent
      #print "center : ========================================================================="
      if(self.toPose({
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
            )):
        #print "pose time: " + str(pose_start_time)
        #print "current time: " + str(self.getTime())
        if (self.getTime() - pose_start_time) > 4:
          pose_sent = False
          num_times_sent = 0
          pose_start_time = 0
          #print "CHANGING TO SIT"
          current_state = DefendingStates.sit

    def defense_block_left(self):
      global current_state, DefendingStates, pose_sent, pose_start_time, num_times_sent
      #print "left : ========================================================================="
      if(self.toPose({ 
              core.RHipYawPitch : 0,
              core.RHipRoll : 20,
              core.RHipPitch : -32,
              core.RKneePitch : 76,
              core.RAnklePitch : -40,
              core.RAnkleRoll : 17,
              core.LHipYawPitch : 0,
              core.LHipRoll : -34,
              core.LHipPitch : -49,
              core.LKneePitch : 125,
              core.LAnklePitch : -70,
              core.LAnkleRoll : -5,
              core.RShoulderPitch : -97,
              core.RShoulderRoll : 7,
              core.LShoulderPitch : -91,
              core.LShoulderRoll : 45
              }
              , 0.5
            )):
        #print "pose time: " + str(pose_start_time)
        #print "current time: " + str(self.getTime())
        if (self.getTime() - pose_start_time) > 4:
          pose_start_time = 0
          pose_sent = False
          num_times_sent = 0
          #print "CHANGING TO SIT"
          current_state = DefendingStates.sit

    def defense_block_right(self):
      global current_state, DefendingStates, pose_sent, pose_start_time, num_times_sent
      #print "right : ========================================================================="
      if(self.toPose({ 
            core.LHipYawPitch : 0 ,
            core.LHipRoll : 20 ,
            core.LHipPitch : -32 ,
            core.LKneePitch : 76 ,
            core.LAnklePitch : -40 ,
            core.LAnkleRoll : 17 ,
            core.RHipYawPitch : 0 ,
            core.RHipRoll : -34 ,
            core.RHipPitch : -49 ,
            core.RKneePitch : 125 ,
            core.RAnklePitch : -70 ,
            core.RAnkleRoll : -5 ,
            core.LShoulderPitch : -97 ,
            core.LShoulderRoll : 7 ,
            core.RShoulderPitch : -91 ,
            core.RShoulderRoll : 45 
            }
            , 0.5
          )):
        #print "pose time: " + str(pose_start_time)
        #print "current time: " + str(self.getTime())
        if (self.getTime() - pose_start_time) > 4:
          pose_start_time = 0 
          pose_sent = False
          num_times_sent = 0
          #print "CHANGING TO SIT"
          current_state = DefendingStates.sit

    def defense_sit(self):
      global current_state, DefendingStates, pose_sent, pose_start_time, num_times_sent
      if(self.toPose({ 
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
            )):
        #print "pose time: " + str(pose_start_time)
        #print "current time: " + str(self.getTime())
        if (self.getTime() - pose_start_time) > 6.5:
          pose_start_time = 0
          pose_sent = False
          num_times_sent = 0
          #print "CHANGING TO START"
          current_state = DefendingStates.start
        elif (self.getTime() - pose_start_time) > 3.5:
          commands.stand()

    def defense_walk(self):
      global walk_start_time, current_state, DefendingStates
      commands.setHeadTilt(-13)
      commands.setWalkVelocity(0.3,0.0,0.05)
      #print "walk time: " + str(walk_start_time)
      #print "current time: " + str(self.getTime())
      if (self.getTime() - walk_start_time) > 9.0:
        current_state = DefendingStates.start
        # commands.setWalkVelocity(0.0,0.0,0.0)
        commands.stand()

    def defense_walk_left(self):
      commands.setHeadTilt(-13)
      commands.setWalkVelocity(0.0,0.5,0.05)

    def defense_walk_right(self):
      commands.setHeadTilt(-13)
      commands.setWalkVelocity(0.0,-0.5,0.05)

    def defense_walk_center(self):
      commands.setHeadTilt(-13)
      commands.setWalkVelocity(0.0,0.0,0.0)

#ATTACKING=================================================================================
    def attack_start(self):
      commands.setHeadTilt(-10)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_beacon_lm_seen = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK].seen or mem_objects.world_objects[core.WO_BEACON_PINK_BLUE].seen
      o_beacon_rf_seen = mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK].seen or mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW].seen
      o_beacon_rr_seen = mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE].seen or mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW].seen

      rotation_dir = 1. if (not o_beacon_lm_seen and (o_beacon_rf_seen or o_beacon_rr_seen)) else -1.

      if rotation_dir > 0:
        memory.speech.say("Started on the left")
      else:
        memory.speech.say("Started on the right")
        
      if(o_ball.seen): 
        current_state = AttackingStates.approach
      else:
        self.walk(0.0, 0.0, rotation_dir*0.2)

    def attack_approach(self):
      commands.setHeadTilt(-14)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

      if(o_ball.seen): 
        if(self.walk_to(o_ball, 250)):
          current_state = AttackingStates.rotate
          return
      # else:
      #   memory.speech.say("Lost the ball")

    def attack_rotate(self):
      commands.setHeadTilt(-16)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

      if(o_ball.seen and o_goal.seen):
        gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
        if(numpy.abs(gy) <= 200):
          self.stop()
          current_state = AttackingStates.dribble
          return

      bx = o_ball.visionDistance * numpy.cos(o_ball.visionBearing)
      x_vel = tanhController(bx - 250, 10.0, 0.3) # 0.1 #-0.05
      y_vel = 0.4
      t_vel = tanhController(o_ball.visionBearing, 10.0, 0.3) 
      if(rotation_dir > 0):
        y_vel = y_vel * -1.

      self.walk(x_vel, y_vel, t_vel)


    def attack_dribble(self):
      commands.setHeadTilt(-16)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir, r_goal_dist_filtered
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

      if(not o_goal.seen):
        #print "Lost the goal!"
        self.stop()
        current_state = AttackingStates.rotate
        return

      if(not o_ball.seen):
        #print "Lost the ball!"
        self.walk(-0.3, 0.0, 0.0)
        return

      gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
      gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
      bx = o_ball.visionDistance * numpy.cos(o_ball.visionBearing)
      by = o_ball.visionDistance * numpy.sin(o_ball.visionBearing)

      dx_gb = gx - bx
      dy_gb = gy - by
      dt_gb = numpy.arctan2(dy_gb, dx_gb)
      r_goal_ball = numpy.sqrt(dx_gb * dx_gb + dy_gb * dy_gb)
      alpha = 0.3
      r_goal_dist_filtered = alpha*r_goal_ball + (1.0-alpha)*r_goal_dist_filtered
      r_goal_threshold = 750. * numpy.sqrt(2.)
      # r_goal_threshold = 725 * numpy.sqrt(2.)

      #print "gx: " + str(gx)
      #print "gy: " + str(gy)
      #print "bx: " + str(bx)
      #print "by: " + str(by)
      #print "dx_gb: " + str(dx_gb)
      #print "dy_gb: " + str(dy_gb)
      #print "dt_gb: " + str(dt_gb)

      if(r_goal_dist_filtered < r_goal_threshold):
        #print "Stopping with threshold " + str(r_goal_dist_filtered)
        # memory.speech.say("Distance " + str(int(r_goal_dist_filtered)) + ", aligning")
        current_state = AttackingStates.align
        return

      x_vel = 0.3 if numpy.abs(by) < 200 else 0.
      y_vel = tanhController((gy + by)/2.0, 20.0/1000.0, 0.35) 
      t_vel = tanhController(dy_gb, 10.0/1000.0, 0.2) 
      #print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      self.walk(x_vel, y_vel, t_vel)

    def attack_align(self):
      commands.setHeadTilt(-18)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir, kick_sent, attack_left, r_goal_dist_filtered
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]
      
      if(enemy_state is EnemyGoalStates.unknown):
        if(not o_enemy.seen):
          #print "Can't find the enemy!"
          enemy_state = EnemyGoalStates.center
          self.walk(-0.3, 0.0, 0.0)
          return
        if(not o_goal.seen):
          #print "Can't find the goal!"
          self.walk(-0.3, 0.0, 0.0)
          return
        else:
          gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
          gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
          ex = o_enemy.visionDistance * numpy.cos(o_enemy.visionBearing)
          ey = o_enemy.visionDistance * numpy.sin(o_enemy.visionBearing)
          center_threshold = o_goal.radius*0.1
          #print "Threshold: " + str(center_threshold)
          shift = gy - ey
          if(numpy.abs(shift) < center_threshold):
            # attack_left = bool(random.getrandbits(1))
            attack_left = False
            side_string = str("left" if attack_left else "right")
            memory.speech.say("Enemy is in the center, shooting to the " + side_string)
            enemy_state = EnemyGoalStates.center
          elif(shift > 0.):
            memory.speech.say("Enemy is on the right, shooting to the left")
            # attack_left = True
            attack_left = False
            enemy_state = EnemyGoalStates.left
          else:
            memory.speech.say("Enemy is on the left, shooting to the right")
            attack_left = False
            enemy_state = EnemyGoalStates.right

      if(not o_goal.seen or not o_ball.seen):
        #print "Can't find the goal / ball"
        self.walk(-0.3, 0.0, 0.05)
        return


      gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
      gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
      bx = o_ball.visionDistance * numpy.cos(o_ball.visionBearing)
      by = o_ball.visionDistance * numpy.sin(o_ball.visionBearing)
      ex = o_enemy.visionDistance * numpy.cos(o_enemy.visionBearing)
      ey = o_enemy.visionDistance * numpy.sin(o_enemy.visionBearing)
      tx = gx
      ty = gy
      dx_gb = gx - bx
      dy_gb = gy - by
      dt_gb = numpy.arctan2(dy_gb, dx_gb)
      r_goal_ball = numpy.sqrt(dx_gb * dx_gb + dy_gb * dy_gb)
      alpha = 0.3
      r_goal_dist_filtered = alpha*r_goal_ball + (1.0-alpha)*r_goal_dist_filtered

      if(r_goal_dist_filtered > 2000):
        current_state = AttackingStates.dribble
        # memory.speech.say("Distance " + str(int(r_goal_dist_filtered)) + ", going back")
        return

      if(o_enemy.seen):
        if(attack_left):
          enemy_edge = ey + numpy.sqrt(o_enemy.radius)
          goal_edge = gy + o_goal.radius #yes it's supposed to be different
          ty = (enemy_edge+goal_edge)/2.0
        else:
          enemy_edge = ey - numpy.sqrt(o_enemy.radius)
          goal_edge = gy - o_goal.radius #yes it's supposed to be different
          ty = (enemy_edge+goal_edge)/2.0
      else:
        if(attack_left):
          ty += 3.0 * o_goal.radius / 8.
        else:
          ty -= 3.0 * o_goal.radius / 8.

      #print "gx: " + str(gx)
      #print "gy: " + str(gy)
      #print "tx: " + str(tx)
      #print "ty: " + str(ty)
      #print "bx: " + str(bx)
      #print "by: " + str(by)


      threshold = 65
      ball_x_target = 95
      ball_y_target = -100
      goal_y_target = -100

      #override?
      tx = gx
      ty = gy
      if(attack_left):
        ball_y_target += 0
        ty += 250
      else:
        ball_y_target -= 0
        ty -= 250

      if (numpy.abs(bx - ball_x_target) <= threshold) and (numpy.abs(by - ball_y_target) <= threshold) and (numpy.abs(ty - goal_y_target) <= threshold) and (numpy.abs(ty - by) <= threshold):
        kick_sent = False
        current_state = AttackingStates.kick
        return

      x_vel = tanhController(-(ball_x_target - bx), 10.0/1000.0, 0.3) 
      y_vel = tanhController(-(ball_y_target - by), 10.0/1000.0, 0.3) 
      t_vel = tanhController((ty - by), 12.0/1000.0, 0.3) 
      #print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      self.walk(x_vel, y_vel, t_vel)

    def attack_kick(self):
      commands.setHeadTilt(-20)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir, kick_sent, kick_start_time, recovering_from_kick
      if not kick_sent:
        #print "sending kick"
        # memory.speech.say("Kicking")
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
        kick_start_time = self.getTime()
        kick_sent = True
        recovering_from_kick = False
      elif not recovering_from_kick and kick_sent and (self.getTime() - kick_start_time) > 0.5 and not memory.kick_request.kick_running_:
      #   print "kick is done, recovering"
      #   self.walk(-0.2, -0.1, 0.0)
      #   recovering_from_kick = True
      # elif recovering_from_kick and (self.getTime() - kick_start_time) > 7.0:
      #   print "hopefully done recovering"
        kick_sent = False
        recovering_from_kick = False
        self.stop()
        mode = Modes.passive
        current_state = AttackingStates.dribble

    def run(self):
      global walk_start_time, EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir, kick_sent, last_mode_change_time

      #detect mode switches
      hf = sensors.getValue(core.headFront)
      hm = sensors.getValue(core.headMiddle)
      hr = sensors.getValue(core.headRear)
      # print "Head: " + str(hf) + ", " + str(hm) + ", " + str(hr)
      if(mode is not Modes.passive and hf and hm and hr): #need to switch of passive mode
        self.stop()
        memory.speech.say("Passive Mode!")
        mode = Modes.passive
        last_mode_change_time = self.getTime()
      if(mode is not Modes.attacking and hf and hm and not hr and (self.getTime() - last_mode_change_time) > 1.0): #need to switch to attack mode
        memory.speech.say("Attack Mode!")
        mode = Modes.attacking
        enemy_state = EnemyGoalStates.unknown
        kick_sent = False
        r_goal_dist_filtered = 0. 
        current_state = AttackingStates.start
        last_mode_change_time = self.getTime()
      if(mode is not Modes.defending and hm and hr and not hf and (self.getTime() - last_mode_change_time) > 1.0): #need to switch to defense mode
        memory.speech.say("Defense Mode!")
        commands.stand()
        pose_sent = False
        num_times_sent = 0
        pose_start_time = 0
        mode = Modes.defending
        current_state = DefendingStates.walk
        walk_start_time = self.getTime()
        last_mode_change_time = self.getTime()

      #detect kidnapping
      lfl = sensors.getValue(core.fsrLFL)
      lfr = sensors.getValue(core.fsrLFR)
      lrl = sensors.getValue(core.fsrLRL)
      lrr = sensors.getValue(core.fsrLRR)
      rfl = sensors.getValue(core.fsrRFL)
      rfr = sensors.getValue(core.fsrRFR)
      rrl = sensors.getValue(core.fsrRRL)
      rrr = sensors.getValue(core.fsrRRR)
      max_force = numpy.amax([lfl,lfr,lrl,lrr,rfl,rfr,rrl,rrr])
      # print "lfl is " + str(lfl)
      # print "lfr is " + str(lfr)
      # print "lrl is " + str(lrl)
      # print "lrr is " + str(lrr)
      # print "rfl is " + str(rfl)
      # print "rfr is " + str(rfr)
      # print "rrl is " + str(rrl)
      # print "rrr is " + str(rrr)
      # print "max is " + str(max_force)
      # print "tilt " + str((lfl+lrl)/2.-(lfr+lrr)/2.)
      if(numpy.abs(max_force) < 0.10 and mode is not Modes.passive):
        #print("***************************************************")
        #print("***************************************************")
        #print("***************************************************")
        #print("numpy.abs(max_force) = ") + str(numpy.abs(max_force))
        memory.speech.say("Put me down!")
        self.stop()
        mode = Modes.passive

      #execute the appropriate function
      if(mode is Modes.passive):
        #print "passive"
        self.track_ball()
        self.stop()
        return
      else:

        function_map = {}
        if(mode is Modes.attacking):
          #print "Attack mode"
          function_map = {AttackingStates.start:self.attack_start, 
                          AttackingStates.approach:self.attack_approach, 
                          AttackingStates.rotate:self.attack_rotate, 
                          AttackingStates.dribble:self.attack_dribble, 
                          AttackingStates.align:self.attack_align, 
                          AttackingStates.kick:self.attack_kick}
        else:
          #print "Defense mode"
          function_map = {DefendingStates.start:self.defense_start, 
                          DefendingStates.walk_center:self.defense_walk_center, 
                          DefendingStates.walk_left:self.defense_walk_left, 
                          DefendingStates.walk_right:self.defense_walk_right, 
                          DefendingStates.walk:self.defense_walk, 
                          DefendingStates.block:self.defense_block, 
                          DefendingStates.block_left:self.defense_block_left, 
                          DefendingStates.block_right:self.defense_block_right, 
                          DefendingStates.sit:self.defense_sit
                          }
        #print "Current state: " + str(current_state)
        function_map[current_state]()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Win(), C, pose.Sit(), C, self.Off())

