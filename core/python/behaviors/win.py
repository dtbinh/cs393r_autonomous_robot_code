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
current_state = AttackingStates.start
rotation_dir = 1.

def tanhController(x, xd, kx, max_cmd):
  return max_cmd * numpy.tanh(kx * (xd-x))

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class Win(Node):
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
        commands.setWalkVelocity(0., 0., 0.)
        return True

      x_vel = tanhController(cx, 0., 10.0 / 1000.0, 0.4)
      y_vel = tanhController(cy, 0., 10.0 / 1000.0, 0.4)
      t_vel = 0.0
      print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      commands.setWalkVelocity(x_vel, y_vel, t_vel)
      return False

    def defense_start(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

    def attack_start(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_beacon_lm = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK] if field is Fields.A else mem_objects.world_objects[core.WO_BEACON_PINK_BLUE]
      o_beacon_rf = mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK] if field is Fields.A else mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW]
      o_beacon_rr = mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE] if field is Fields.A else mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]

      rotation_dir = 1. if (not o_beacon_lm.seen and (o_beacon_rf.seen or o_beacon_rr.seen)) else -1.

      if(o_ball.seen): 
        current_state = AttackingStates.approach
      else:
        #todo: search for ball
        pass

    def attack_approach(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]
      if(self.walk_to(o_ball, 100)):
        current_state = AttackingStates.rotate
        return

    def attack_rotate(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

      if(o_ball.seen and o_goal.seen):
        commands.setWalkVelocity(0., 0., 0.)
        current_state = AttackingStates.dribble
        return

      x_vel = -0.05
      y_vel = 0.2
      t_vel = tanhController(o_ball.visionBearing, 0., 1.0, 0.2) 
      if(rotation_dir > 0):
        y_vel = y_vel * -1.

      commands.setWalkVelocity(x_vel, y_vel, t_vel)


    def attack_dribble(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

      if(not o_ball.seen or not o_goal.seen):
        print "Lost the goal or ball!"
        return

      gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
      gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
      bx = o_ball.visionDistance * numpy.cos(o_ball.visionBearing)
      by = o_ball.visionDistance * numpy.sin(o_ball.visionBearing)

      dx_gb = gx - bx
      dy_gb = gy - by
      dt_gb = numpy.arctan2(dy_gb, dx_gb)
      r_goal_ball = numpy.sqrt(dx_gb * dx_gb + dy_gb * dy_gb)
      r_goal_threshold = 900. * numpy.sqrt(2.)

      if(r_goal_ball < r_goal_threshold):
        current_state = AttackingStates.align
        return

      x_vel = 0.1
      y_vel = tanhController(dy_gb, 0., 1.0/1000.0, 0.1) 
      t_vel = tanhController(dy_gb, 0., 1.0/1000.0, 0.05) 

    def attack_align(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir
      o_self = mem_objects.world_objects[robot_state.WO_SELF]
      o_ball = mem_objects.world_objects[core.WO_BALL]
      o_goal = mem_objects.world_objects[core.WO_OPP_GOAL]
      o_enemy = mem_objects.world_objects[core.WO_OPPONENT1]

      if(not o_ball.seen or not o_goal.seen or not o_enemy.seen):
        print "Lost the goal, enemy, or ball!"
        return

      gx = o_goal.visionDistance * numpy.cos(o_goal.visionBearing)
      gy = o_goal.visionDistance * numpy.sin(o_goal.visionBearing)
      ex = o_enemy.visionDistance * numpy.cos(o_enemy.visionBearing)
      ey = o_enemy.visionDistance * numpy.sin(o_enemy.visionBearing)
      tx = gx
      ty = gy

      center_threshold = 200.
      shift = gy - ey
      if(numpy.abs(shift) < center_threshold):
        memory.speech.say("Enemy is in the center")
        ty += o_goal.radius / 4. if bool(random.getrandbits(1)) else -o_goal.radius / 4.
      elif(shift > 0.):
        memory.speech.say("Enemy is on the right")
        ty += o_goal.radius / 4.
      else:
        memory.speech.say("Enemy is on the left")
        ty -= o_goal.radius / 4.

      #todo: actually align

      if(True): #todo
        current_state = AttackingStates.kick
        return

    def attack_kick(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        current_state = AttackingStates.start
        return

    def run(self):
      global Modes, mode, states, current_state, Fields, field, rotation_dir

      #detect mode switches
      hf = sensors.getValue(core.headFront)
      hm = sensors.getValue(core.headMiddle)
      hr = sensors.getValue(core.headRear)
      print "Head: " + str(hf) + ", " + str(hm) + ", " + str(hr)
      if(mode is not Modes.attacking and hf and hm and not hr): #need to switch to attack mode
        memory.speech.say("Attack Mode!")
        mode = Modes.attacking
        current_state = AttackingStates.start
      if(mode is not Modes.defending and hm and hr and not hf): #need to switch to defense mode
        memory.speech.say("Defense Mode!")
        mode = Modes.defending
        current_state = DefendingStates.start
      if(mode is not Modes.passive and hf and hm and hr): #need to switch of passive mode
        commands.setWalkVelocity(0., 0., 0.)
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
        mode = Modes.passive

      #execute the appropriate function
      if(mode is Modes.passive):
        return
      else:
        function_map = {}
        if(mode is Modes.attacking):
          function_map = {AttackingStates.start:self.attack_start(), AttackingStates.approach:self.attack_approach(), AttackingStates.dribble:self.attack_dribble(), AttackingStates.align:self.attack_align(), AttackingStates.kick:self.attack_kick()}
        else:
          function_map = {DefendingStates.start:self.defense_start()}
        function_map.get(current_state)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Win(), C, pose.Sit(), C, self.Off())

