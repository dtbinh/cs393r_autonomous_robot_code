import memory, pose, commands, cfgstiff, mem_objects, core, random, numpy, math
from task import Task
from state_machine import *
from memory import *
from sets import Set

direction = 1.
last_direction_change_time = 0.

have_lock = False
facing_center = False
at_center = False
num_beacons_required = 2
last_head_time = 0
last_frame_time = 0

beacons_seen = Set()

class Ready(Task):
  def run(self):
    print "Readying"
    commands.stand()
    if self.getTime() > 5.0:
      self.finish()

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      print "Standing"
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class WalkToCenter(Node):
    def search(self):
      global direction, last_direction_change_time, have_lock, facing_center, at_center, beacons_seen
      # memory.speech.say("Searching!")

      for i in xrange(core.WO_BEACON_BLUE_YELLOW, core.WO_BEACON_YELLOW_PINK):
        if(mem_objects.world_objects[i].seen):
          beacons_seen.add(i)
          print "saw beacon " + str(i)
      if(len(beacons_seen) >= num_beacons_required):
        # memory.speech.say("Done")
        have_lock = True
        return

      turn_amount = 2.0*math.pi
      turn_vel = 0.2
      turn_time = turn_amount / turn_vel
      commands.setWalkVelocity(0, 0, turn_vel * direction)
      if((self.getTime() - last_direction_change_time) > turn_time):
        direction = direction * -1.0
        last_direction_change_time = self.getTime()

    def turn(self):
      # memory.speech.say("Turning!")
      global have_lock, facing_center, at_center
      sloc = mem_objects.world_objects[robot_state.WO_SELF].loc
      t = -mem_objects.world_objects[robot_state.WO_SELF].orientation
      cx = (-sloc.x) * numpy.cos(t) - (-sloc.y) * numpy.sin(t)
      cy = (-sloc.x) * numpy.sin(t) + (-sloc.y) * numpy.cos(t)
      print "global x,y,t = " + str(sloc.x) + "," + str(sloc.y) + "," + str(t)
      print "local center x,y = " + str(cx) + "," + str(cy)
      t_err = numpy.arctan2(cy, cx)
      print "t_err = " + str(t_err)

      if numpy.abs(t_err) < 0.05:
        facing_center = True
        return
      else:
        Kt = 1.0
        vt_max = 0.25
        t_vel = vt_max * numpy.tanh(Kt * t_err);
        print "t_vel = " + str(t_vel)
        commands.setWalkVelocity(0.0, 0.0, t_vel)

    def walk(self):
      global have_lock, facing_center, at_center
      # memory.speech.say("Walking!")
      sloc = mem_objects.world_objects[robot_state.WO_SELF].loc
      t = -mem_objects.world_objects[robot_state.WO_SELF].orientation
      cx = (-sloc.x) * numpy.cos(t) - (-sloc.y) * numpy.sin(t)
      cy = (-sloc.x) * numpy.sin(t) + (-sloc.y) * numpy.cos(t)
      t_err = numpy.arctan2(cy, cx)
      print "global x,y,t = " + str(sloc.x) + "," + str(sloc.y) + "," + str(t)
      print "local center x,y = " + str(cx) + "," + str(cy)

      Kx = 5.0
      vx_max = 0.35
      x_vel = vx_max * numpy.tanh(Kx * cx / 1000.0)
      Ky = 5.0
      vy_max = 0.4
      y_vel = vy_max * numpy.tanh(Ky * cy / 1000.0)
      t_vel = 0.0
      if ((x_vel * x_vel + y_vel * y_vel) < 0.1):
        t_vel = 0.1
      print "vel x,y,t = " + str(x_vel) + "," + str(y_vel) + "," + str(t_vel)
      commands.setWalkVelocity(x_vel, y_vel, t_vel)

    def kidnapped(self):
      global direction, last_direction_change_time, have_lock, facing_center, at_center, beacons_seen
      memory.speech.say("Put me down!")
      have_lock = False
      facing_center = False
      beacons_seen = Set()

    def run(self):
      global have_lock, facing_center, last_head_time
      if ((self.getTime() - last_head_time) > 2.0):
        commands.setHeadPan(random.uniform(-1.5, 1.5), 2.0)
        commands.setHeadTilt(-15)
        last_head_time = self.getTime()

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

      if(numpy.abs(max_force) < 0.15):
        self.kidnapped()
        return

      if not have_lock and not facing_center:
        self.search()
      elif have_lock and not facing_center:
        self.turn()
      elif have_lock and facing_center:
        self.walk()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.WalkToCenter(), C, self.Off())

