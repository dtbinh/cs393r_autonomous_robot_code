import core, memory
import pose, commands, cfgstiff, cfgpose
import mem_objects
from task import Task
from state_machine import *


if_seen_history_constant =7
if_seen_history = [0]*if_seen_history_constant
if_seen_history_counter = 0

last_inf_constant = 2
last_x = [0]*last_inf_constant
last_y = [0]*last_inf_constant
last_bearing = [0]*last_inf_constant
last_distance = [0]*last_inf_constant
last_inf_counter = 0

last_head_time = 0
last_head_pan = 1.2


class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        self.finish()

  class Kick(Node):
    def run(self):
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class Tracking(Node):
    def run(self):
      global if_seen_history_constant , if_seen_history , if_seen_history_counter
      global last_inf_constant , last_inf_counter, last_x, last_y, last_bearing, last_distance
      global last_head_time, last_head_pan

      ball = mem_objects.world_objects[core.WO_BALL]
      if_seen_history[if_seen_history_counter] = ball.seen;
      if_seen_history_counter = (if_seen_history_counter + 1)%if_seen_history_constant
      seen_times = sum(if_seen_history)


      x = 0.
      y = 0.
      bearing = 0.
      distance = 0.
      
      if(seen_times > ( if_seen_history_constant/2 )): # It is actually seen
        if(ball.seen):
          x  = ball.loc.x
          y  = ball.loc.y
          bearing = ball.visionBearing
          distance = ball.visionDistance
        else:
          return

        last_av_bearing = sum(last_bearing)/last_inf_constant

        last_x[last_inf_counter]  = x
        last_y[last_inf_counter]  = y
        last_bearing[last_inf_counter] = bearing
        last_distance[last_inf_counter] = distance

        av_x = sum(last_x)/last_inf_constant
        av_y = sum(last_y)/last_inf_constant
        av_bearing = sum(last_bearing)/last_inf_constant
        av_distance = sum(last_distance)/last_inf_constant

        last_inf_counter = (last_inf_counter + 1)%last_inf_constant
        
        if(av_bearing > 1.4):
          av_bearing = 1.4
        elif(av_bearing < -1.4):
          av_bearing = -1.4

        d_turning = abs(av_bearing - last_av_bearing)
        if(d_turning < 0.1):
          d_turning = 0.1
        elif(d_turning > 2):
          d_turning = 2
        commands.setHeadPan(av_bearing, d_turning)
        
      else:
        if ((self.getTime() - last_head_time) > 3.5):
          if(last_head_pan > 0 ):
            last_head_pan = -1.2
          elif(last_head_pan <= 0):
            last_head_pan = 1.2
          commands.setHeadPan( last_head_pan , 3 )
          last_head_time = self.getTime()
        last_x = [0]*last_inf_constant
        last_y = [0]*last_inf_constant
        last_bearing = [0]*last_inf_constant
        last_distance =[0]*last_inf_constant

  class Stiff(Node):
    def run(self):
      commands.setStiffness(cfgstiff.kickStand)

  class Off(Node):
    def run(self):
      #commands.setStiffness(cfgstiff.kickStand)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Kick(), C , self.Tracking(), T(500), self.Stiff() , T(500) , pose.Sit(), C, self.Off())
