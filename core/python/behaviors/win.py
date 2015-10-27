import memory, pose, commands, cfgstiff, mem_objects, core, random, numpy, math
from task import Task
from state_machine import *
from memory import *
from sets import Set
#from enum import Enum

class Ready(Task):
  def run(self):
    commands.stand()
    if self.getTime() > 5.0:
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

field = Fields.A
mode = Modes.passive
enemy_state = EnemyGoalStates.unknown
current_state = AttackingStates.start
rotation_dir = 1.
last_ball_seen_time = 0
next_head_time = 0
kick_start_time = 0
kick_sent = False

def tanhController(err, kx, max_cmd):
  return max_cmd * numpy.tanh(kx * err)

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class Win(Node):
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
      print "local target x,y = " + str(cx) + "," + str(cy)

      if(r < r_threshold):
        print "Stopping"
        self.stop()
        return True

      x_vel = tanhController(cx, 10.0 / 1000.0, 0.4)
      y_vel = tanhController(cy, 10.0 / 1000.0, 0.4)
      t_vel = 0.0
      print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      self.walk(x_vel,y_vel,t_vel)
      return False

    def defense_start(self):
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

    def attack_start(self):
      commands.setHeadTilt(-14)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_beacon_lm = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK] if field is Fields.A else mem_objects.world_objects[core.WO_BEACON_PINK_BLUE]
      o_beacon_rf = mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK] if field is Fields.A else mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW]
      o_beacon_rr = mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE] if field is Fields.A else mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]

      rotation_dir = 1. if (not o_beacon_lm.seen and (o_beacon_rf.seen or o_beacon_rr.seen)) else -1.

      if rotation_dir > 0:
        memory.speech.say("Started on the left")
      else:
        memory.speech.say("Started on the right")
        
      if(o_ball.seen): 
        current_state = AttackingStates.approach
      else:
        #todo: search for ball
        pass

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
      else:
        memory.speech.say("Lost the ball")

    def attack_rotate(self):
      commands.setHeadTilt(-18)
      
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
      commands.setHeadTilt(-20)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

      if(not o_ball.seen or not o_goal.seen):
        print "Lost the goal or ball!"
        self.walk(-0.1, 0.0, 0.0)
        return

      gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
      gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
      bx = o_ball.visionDistance * numpy.cos(o_ball.visionBearing)
      by = o_ball.visionDistance * numpy.sin(o_ball.visionBearing)

      dx_gb = gx - bx
      dy_gb = gy - by
      dt_gb = numpy.arctan2(dy_gb, dx_gb)
      r_goal_ball = numpy.sqrt(dx_gb * dx_gb + dy_gb * dy_gb)
      r_goal_threshold = 750. * numpy.sqrt(2.)

      print "gx: " + str(gx)
      print "gy: " + str(gy)
      print "bx: " + str(bx)
      print "by: " + str(by)
      print "dx_gb: " + str(dx_gb)
      print "dy_gb: " + str(dy_gb)
      print "dt_gb: " + str(dt_gb)

      if(r_goal_ball < r_goal_threshold):
        print "Stopping with threshold " + str(r_goal_ball)
        current_state = AttackingStates.align
        return

      x_vel = 0.3
      y_vel = tanhController((gy + by)/2.0, 10.0/1000.0, 0.3) 
      t_vel = tanhController(dy_gb, 10.0/1000.0, 0.2) 
      print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      self.walk(x_vel, y_vel, t_vel)

    def attack_align(self):
      commands.setHeadTilt(-20)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir, kick_sent
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]
      
      if(enemy_state is EnemyGoalStates.unknown):
        if(not o_goal.seen or not o_enemy.seen):
          print "Can't find the enemy / goal!"
          self.stop()
          return
        else:
          gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
          gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
          ex = o_enemy.visionDistance * numpy.cos(o_enemy.visionBearing)
          ey = o_enemy.visionDistance * numpy.sin(o_enemy.visionBearing)
          center_threshold = 200.
          shift = gy - ey
          if(numpy.abs(shift) < center_threshold):
            memory.speech.say("Enemy is in the center")
            enemy_state = EnemyGoalStates.center
          elif(shift > 0.):
            memory.speech.say("Enemy is on the right")
            enemy_state = EnemyGoalStates.right
          else:
            memory.speech.say("Enemy is on the left")
            enemy_state = EnemyGoalStates.left

      if(not o_goal.seen or not o_ball.seen):
        print "Can't find the goal / ball"
        return


      gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
      gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
      bx = o_ball.visionDistance * numpy.cos(o_ball.visionBearing)
      by = o_ball.visionDistance * numpy.sin(o_ball.visionBearing)
      tx = gx
      ty = gy

      if(enemy_state is EnemyGoalStates.center):
        ty += o_goal.radius / 4. if bool(random.getrandbits(1)) else -o_goal.radius / 4.
      elif(enemy_state is EnemyGoalStates.right):
        ty += o_goal.radius / 4.
      elif(enemy_state is EnemyGoalStates.left):
        ty -= o_goal.radius / 4.

      print "gx: " + str(gx)
      print "gy: " + str(gy)
      print "tx: " + str(tx)
      print "ty: " + str(ty)
      print "bx: " + str(bx)
      print "by: " + str(by)

      threshold = 50
      ball_x_target = 100
      ball_y_target = -100
      goal_y_target = -100
      if (numpy.abs(bx - ball_x_target) <= threshold) and (numpy.abs(by - ball_y_target) <= threshold) and (numpy.abs(ty - goal_y_target) <= threshold) and (numpy.abs(ty - by) <= threshold): #todo
        kick_sent = False
        current_state = AttackingStates.kick
        return

      x_vel = tanhController(-(ball_x_target - bx), 20.0/1000.0, 0.4) 
      y_vel = tanhController(-(ball_y_target - by), 20.0/1000.0, 0.4) 
      t_vel = tanhController((ty - by), 10.0/1000.0, 0.2) 
      print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      self.walk(x_vel, y_vel, t_vel)

    def attack_kick(self):
      commands.setHeadTilt(-20)
      
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir, kick_sent, kick_start_time
      if not kick_sent:
        print "sending kick"
        memory.speech.say("Kicking")
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
        kick_start_time = self.getTime()
        kick_sent = True
      elif kick_sent and (self.getTime() - kick_start_time) > 0.5 and not memory.kick_request.kick_running_:
        print "kick is done"
        mode = Modes.passive
        current_state = AttackingStates.start

    def run(self):
      global EnemyGoalStates, enemy_state, Modes, mode, states, current_state, Fields, field, rotation_dir, kick_sent

      #detect mode switches
      hf = sensors.getValue(core.headFront)
      hm = sensors.getValue(core.headMiddle)
      hr = sensors.getValue(core.headRear)
      # print "Head: " + str(hf) + ", " + str(hm) + ", " + str(hr)
      if(mode is not Modes.attacking and hf and hm and not hr): #need to switch to attack mode
        memory.speech.say("Attack Mode!")
        mode = Modes.attacking
        kick_sent = False
        current_state = AttackingStates.kick
      if(mode is not Modes.defending and hm and hr and not hf): #need to switch to defense mode
        memory.speech.say("Defense Mode!")
        mode = Modes.defending
        current_state = DefendingStates.start
      if(mode is not Modes.passive and hf and hm and hr): #need to switch of passive mode
        self.stop()
        memory.speech.say("Passive Mode!")
        mode = Modes.passive

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
      if(numpy.abs(max_force) < 0.15 and mode is not Modes.passive):
        memory.speech.say("Put me down!")
        self.stop()
        mode = Modes.passive

      #execute the appropriate function
      if(mode is Modes.passive):
        self.track_ball()
        self.stop()
        return
      else:

        function_map = {}
        if(mode is Modes.attacking):
          print "Attack mode"
          function_map = {AttackingStates.start:self.attack_start, AttackingStates.approach:self.attack_approach, AttackingStates.rotate:self.attack_rotate, AttackingStates.dribble:self.attack_dribble, AttackingStates.align:self.attack_align, AttackingStates.kick:self.attack_kick}
        else:
          print "Defense mode"
          function_map = {DefendingStates.start:self.defense_start}
        print "Current state: " + str(current_state)
        function_map[current_state]()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Win(), C, pose.Sit(), C, self.Off())

