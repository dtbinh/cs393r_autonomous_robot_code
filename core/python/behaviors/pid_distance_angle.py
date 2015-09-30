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

#parameters
e_distance_set = 160
e_distance_k0 = 0.0004
e_distance_k1 = 0.003
e_distance_k2 = 0

e_angle_x_set_close = 192 # x_goal_indel when close
e_angle_x_set_far = 160 # x_goal_indel when far away
e_angle_x_threshold = 15
e_angle_k0 = 0.003
e_angle_k1 = 0
e_angle_k2 = 0

PID_walk_velocity = 0
last_PID_walk_velocity_0 = 0
last_PID_walk_velocity_1 = 0
last_PID_walk_velocity_2 = 0
PID_walk_velocity_list = [0]*100
PID_walk_velocity_counter = 0

PID_angular_velocity = 0
last_PID_angular_velocity_0 = 0
last_PID_angular_velocity_1 = 0
last_PID_angular_velocity_2 = 0
PID_angular_velocity_list = [0]*100

Approaching_state = 0 # dis>450 : 0 ;  350<dis<450 find goal: 1 ; ready to kick: 2;

ball_if_seen = [0]*40
ball_if_seen_counter = 0
ball_if_find_flag = 0


#ball_kick_pixel_x_set = 192;
#ball_kick_pixel_y_set = 170;
#ball_kick_pixel_x_threshold = 15;
#ball_kick_pixel_y_threshold = 10;


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

  class PID_walk(Node):
    def run(self):
      global e_distance_set, e_distance_k0, e_distance_k1, e_distance_k2
      global e_angle_x_set_close, e_angle_x_set_far, e_angle_x_threshold, e_angle_k0, e_angle_k1, e_angle_k2
      global last_PID_walk_velocity_0, last_PID_walk_velocity_1, last_PID_walk_velocity_2, PID_walk_velocity
      global last_PID_angular_velocity_0, last_PID_angular_velocity_1, last_PID_angular_velocity_2 , PID_angular_velocity
      global Approaching_state, ball_if_seen, ball_if_seen_counter, ball_if_find_flag

      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      if(ball.seen):
        memory.speech.say("oh")
        ball_distance = ball.visionDistance
        ball_image_x = ball.imageCenterX
        ball_image_y = ball.imageCenterY

        ball_if_seen[ball_if_seen_counter] = 1
        seen_times = sum(ball_if_seen)
        
        if( seen_times >= 20 ):
          ball_if_find_flag = 1
          if(ball_distance < 150 and ball_image_x > 177 and ball_image_x < 207): #stop and prepare kick
            commands.setWalkVelocity(0,0,0)
            self.finish()
          elif((ball_distance > 400 or ball_distance < 300 or Approaching_state == 2) and Approaching_state != 1): #direct walking area
            last_PID_walk_velocity_2 = last_PID_walk_velocity_1
            last_PID_walk_velocity_1 = last_PID_walk_velocity_0
            #last_PID_walk_velocity_0 = e_distance_k0 * (ball_distance - e_distance_set) #use advanced method maybe
            last_PID_walk_velocity_0 = e_distance_k0 * (ball_distance - e_distance_set) + e_distance_k1 * sum(PID_walk_velocity_list)
            PID_walk_velocity = 0.33*( last_PID_walk_velocity_0 + last_PID_walk_velocity_1 + last_PID_walk_velocity_2 )
            if( PID_walk_velocity > 0.55):
              PID_walk_velocity = 0.55
            if( PID_walk_velocity < 0.35):
              PID_walk_velocity = 0.35

            if(ball_distance > 400 and Approaching_state == 0):
              e_angle_x_set = e_angle_x_set_far
              Approaching_state = 0
              memory.speech.say("one")
            else:
              e_angle_x_set = e_angle_x_set_close
              #Approaching_state = 2
              memory.speech.say("four")

            last_PID_angular_velocity_2 = last_PID_angular_velocity_1
            last_PID_angular_velocity_1 = last_PID_angular_velocity_0
            last_PID_angular_velocity_0 = -e_angle_k0 * (ball_image_x - e_angle_x_set)
            PID_angular_velocity = 0.33*( last_PID_angular_velocity_0 + last_PID_angular_velocity_1 + last_PID_angular_velocity_2)
            print "seen_times = " + str(seen_times) +  " PID_walk_velocity = "  + str(PID_walk_velocity) + " PID_angular_velocity is \n"  +  str(PID_angular_velocity)
            if( PID_angular_velocity > 0.3 ):
              PID_angular_velocity = 0.3
            elif( PID_angular_velocity < -0.3):
              PID_angular_velocity = -0.3

            commands.setWalkVelocity(PID_walk_velocity, 0 , PID_angular_velocity)
            #print "walking velocity = " , PID_walk_velocity
          else: #finding the goal
            Approaching_state = 1
            goal = memory.world_objects.getObjPtr(core.WO_OPP_GOAL)
            if(goal.seen):
              if(goal.imageCenterX < 175 and goal.imageCenterX > 145):
                Approaching_state = 2
                commands.setWalkVelocity( PID_walk_velocity , 0, PID_angular_velocity )
              elif(goal.imageCenterX >= 175): # Using P control here
                commands.setWalkVelocity(0 , 0.5 , -0.12 )
                memory.speech.say("three")
              elif(goal.imageCenterX <= 145):
                commands.setWalkVelocity(0 , -0.5 ,0.12 )
                memory.speech.say("three")
            else:
                commands.setWalkVelocity(0 , 0.5 , -0.12 )
                memory.speech.say("two")
        else:
          ball_if_find_flag = 0
          Approaching_state = 0
          commands.setWalkVelocity(0 , 0 , 0.2)
      else: #finding_ball
        ball_if_seen[ball_if_seen_counter] = 0
        seen_times = sum(ball_if_seen)
        print "seen_times = " , seen_times

        if( seen_times < 20 ):
          memory.speech.say("finding")
          ball_if_find_flag = 0
          Approaching_state = 0
          commands.setWalkVelocity(0 , 0 , 0.2)
        else:
          ball_if_find_flag = 1
          commands.setWalkVelocity( PID_walk_velocity , 0, PID_angular_velocity )

      ball_if_seen_counter = (ball_if_seen_counter + 1)%40;      
        #Approaching_state = 0

  class Kick(Node):
    def run(self):
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class TrackGoal(Node):
    def run(self):
      global carrot, direction, seen_ball, last_ball_heading, last_ball_time, alpha, last_carrot, ball
      commands.setStiffness()
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
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
    pid_walk = self.PID_walk()
    self.trans(stand, C, pid_walk, C, self.Kick() , C , sit, C, off)
