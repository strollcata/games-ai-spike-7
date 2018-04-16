'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games by Clinton Woodward cwoodward@swin.edu.au

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform
from path import Path

AGENT_MODES = {
    KEY._1: 'seek',
    KEY._2: 'arrive_slow',
    KEY._3: 'arrive_normal',
    KEY._4: 'arrive_fast',
    KEY._5: 'flee',
    KEY._6: 'follow_path',
    KEY._7: 'wander',
    KEY._8: 'alignment',
    KEY._9: 'cohesion',
    KEY._0: 'separation',
}


class Agent(object):

    # NOTE: Class Object (not *instance*) variables!
    DECELERATION_SPEEDS = {
        'slow': 0.9,
        'normal': 0.5,
        'fast': 0.1
    }

    def __init__(self, world=None, scale=30.0, mass=1.0, mode='seek'):
        # keep a reference to the world object
        self.world = world
        self.mode = mode
        # where am i and where am i going? random
        dir = radians(random()*360)
        self.pos = Vector2D(randrange(world.cx), randrange(world.cy))
        self.vel = Vector2D()
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.scale = Vector2D(scale, scale)  # easy scaling of agent size
        self.force = Vector2D()  # current steering force
        self.acceeration = Vector2D()  # current steering force
        self.mass = mass
        # limits?
        self.max_speed = 20.0 * scale
        self.max_force = 500.0
        # data for drawing this agent
        self.color = 'ORANGE'
        self.vehicle_shape = [
            Point2D(-1.0,  0.6),
            Point2D( 1.0,  0.0),
            Point2D(-1.0, -0.6)
        ]
        self.path = Path()
        self.randomise_path()
        self.waypoint_threshold = 50.0
        self.group_threshold = 250.0
        self.show_info = False
        self.wander_target = Vector2D(1, 0)
        self.base_wander_dist = 1.0
        self.base_wander_radius = 1.0
        self.base_wander_jitter = 10.0
        self.wander_dist = self.base_wander_dist * scale
        self.wander_radius = self.base_wander_radius * scale
        self.wander_jitter = self.base_wander_jitter * scale
        self.bRadius = scale
        self.neighbours = []
        self.weighted_sum = False
        self.active_modes = []
        self.sum_weighting = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.agent_number = len(self.world.agents)
        self.flee_distance = 100
        self.wander_parameter = 'dist'

    def randomise_path(self):
        cx = self.world.cx
        cy = self.world.cy
        margin = min(cx, cy) * (1/6)
        pathpoints = randrange(2, 10)
        self.path.create_random_path(pathpoints, (0+margin), (0+margin), (cx-margin), (cy-margin))

    def calculate(self, delta):
        # reset the steering force
        self.wander_dist = self.base_wander_dist * self.bRadius
        self.wander_radius = self.base_wander_radius * self.bRadius
        self.wander_jitter = self.base_wander_jitter * self.bRadius
        mode = self.mode
        sum_weighting = self.sum_weighting
        if ((self.weighted_sum) and (len(self.active_modes) > 0)):
            force = Vector2D()
            if 'seek' in self.active_modes:
                force += (self.seek(self.world.target) * sum_weighting[0])
            if 'arrive_slow' in self.active_modes:
                force += (self.arrive(self.world.target, 'slow') * sum_weighting[1])
            if 'arrive_normal' in self.active_modes:
                force += (self.arrive(self.world.target, 'normal') * sum_weighting[2])
            if 'arrive_fast' in self.active_modes:
                force += (self.arrive(self.world.target, 'fast') * sum_weighting[3])
            if 'flee' in self.active_modes:
                force += (self.flee(self.world.target) * sum_weighting[4])
            if 'follow_path' in self.active_modes:
                force += (self.follow_path() * sum_weighting[5])
            if 'wander' in self.active_modes:
                force += (self.wander(delta) * sum_weighting[6])
            if 'alignment' in self.active_modes:
                force += (self.alignment(delta) * sum_weighting[7])
            if 'cohesion' in self.active_modes:
                force += (self.cohesion(delta) * sum_weighting[8])
            if 'separation' in self.active_modes:
                force += (self.separation(delta) * sum_weighting[9])
        elif mode == 'seek':
            force = self.seek(self.world.target)
        elif mode == 'arrive_slow':
            force = self.arrive(self.world.target, 'slow')
        elif mode == 'arrive_normal':
            force = self.arrive(self.world.target, 'normal')
        elif mode == 'arrive_fast':
            force = self.arrive(self.world.target, 'fast')
        elif mode == 'flee':
            force = self.flee(self.world.target)
        elif mode == 'follow_path':
            force = self.follow_path()
        elif mode == 'wander':
            force = self.wander(delta)
        elif mode == 'alignment':
            force = self.alignment(delta)
        elif mode == 'cohesion':
            force = self.cohesion(delta)
        elif mode == 'separation':
           force = self.separation(delta)
        force.x /= self.mass
        force.y /= self.mass
        self.force = force
        return force

    def weight_sums(self):
        my_modes = self.active_modes
        if len(my_modes) > 0:
            old_weighting = []
            for weight in self.sum_weighting:
                old_weighting.append(weight)
            for mode in my_modes:
                self.sum_weighting[0] = my_modes.count('seek')/len(my_modes)
                self.sum_weighting[1] = my_modes.count('arrive_slow')/len(my_modes)
                self.sum_weighting[2] = my_modes.count('arrive_normal')/len(my_modes)
                self.sum_weighting[3] = my_modes.count('arrive_fast')/len(my_modes)
                self.sum_weighting[4] = my_modes.count('flee')/len(my_modes)
                self.sum_weighting[5] = my_modes.count('follow_path')/len(my_modes)
                self.sum_weighting[6] = my_modes.count('wander')/len(my_modes)
                self.sum_weighting[7] = my_modes.count('alignment')/len(my_modes)
                self.sum_weighting[8] = my_modes.count('cohesion')/len(my_modes)
                self.sum_weighting[9] = my_modes.count('separation')/len(my_modes)
            if self.sum_weighting != old_weighting:
                print("Agent %d's current behaviour weighting is: " % (self.agent_number), end = "")
                print("Seek: %f; Slow Arrive: %f; Normal Arrive: %f; Fast Arrive: %f; Flee: %f; Follow Path: %f; Wander: %f; Alignment: %f; Cohesion: %f; Separation: %f." % (self.sum_weighting[0], self.sum_weighting[1], self.sum_weighting[2], self.sum_weighting[3], self.sum_weighting[4], self.sum_weighting[5], self.sum_weighting[6], self.sum_weighting[7], self.sum_weighting[8], self.sum_weighting[9]))

    def update(self, delta):
        ''' update vehicle position and orientation '''
        if self.weighted_sum:
            self.weight_sums()
        self.neighbourhood()
        force = self.calculate(delta)
        force.truncate(self.max_force)
        # new velocity
        self.vel += force * delta
        # check for limits of new velocity
        self.vel.truncate(self.max_speed)
        # update position
        self.pos += self.vel * delta
        # update heading is non-zero velocity (moving)
        if self.vel.length_sq() > 0.00000001:
            self.heading = self.vel.get_normalised()
            self.side = self.heading.perp()
        # treat world as continuous space - wrap new position if needed
        self.world.wrap_around(self.pos)

    def render(self, color=None):
        ''' Draw the triangle agent with color'''
        if ((self.mode == 'follow_path') or ('follow_path' in self.active_modes)):
            self.path.render()
        egi.set_pen_color(name=self.color)
        pts = self.world.transform_points(self.vehicle_shape, self.pos,
                                          self.heading, self.side, self.scale)

        if ((self.mode == 'wander') or ('wander' in self.active_modes)):
            wnd_pos = Vector2D(self.wander_dist, 0)
            wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            egi.green_pen()
            egi.circle(wld_pos, self.wander_radius)
            egi.red_pen()
            wnd_pos = (self.wander_target + Vector2D(self.wander_dist, 0))
            wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            egi.circle(wld_pos, 3)
        # draw it!
        egi.closed_shape(pts)

        if self.show_info:
            s = 0.5 # <-- scaling factor
            # force
            egi.red_pen()
            egi.line_with_arrow(self.pos, self.pos + self.force * s, 5)
            # velocity
            egi.grey_pen()
            egi.line_with_arrow(self.pos, self.pos + self.vel * s, 5)
            # net (desired) change
            egi.white_pen()
            egi.line_with_arrow(self.pos+self.vel * s, self.pos+ (self.force+self.vel) * s, 5)
            egi.line_with_arrow(self.pos, self.pos+ (self.force+self.vel) * s, 5)

    def speed(self):
        return self.vel.length()

    def neighbourhood(self):
        self.neighbours = []
        agents = self.world.agents
        for agent in agents:
            if agent.pos != self.pos:
                to_agent = agent.pos - self.pos
                dist = to_agent.length()
                if dist <= self.group_threshold:
                    self.neighbours.append(agent)

    def parameter_shift(self, op):
        mode = self.mode
        modes = self.active_modes
        if ((mode == 'flee') or ('flee' in modes)):
            if op == 'up':
                self.flee_distance += 50
            elif op == 'down':
                self.flee_distance = max(0, self.flee_distance - 50)
        if ((mode == 'follow_path') or ('follow_path' in modes)):
            if op == 'up':
                self.waypoint_threshold += 25
            elif op == 'down':
                self.waypoint_threshold = max(0, self.waypoint_threshold - 25)
        if ((mode == 'wander') or ('wander' in modes)):
            wan_param = self.wander_parameter
            if wan_param == 'dist':
                if op == 'up':
                    self.base_wander_dist += 0.5
                elif op == 'down':
                    self.base_wander_dist = max(0, self.base_wander_dist - 0.5)
            elif wan_param == 'radius':
                if op == 'up':
                    self.base_wander_radius += 0.5
                elif op == 'down':
                    self.base_wander_radius = max(0, self.base_wander_radius - 0.5)
            elif wan_param == 'jitter':
                if op == 'up':
                    self.base_wander_jitter += 5
                elif op == 'down':
                    self.base_wander_jitter = max(0, self.base_wander_jitter - 5)

    #--------------------------------------------------------------------------

    def seek(self, target_pos):
        ''' move towards target position '''
        desired_vel = (target_pos - self.pos).normalise() * self.max_speed
        return (desired_vel - self.vel)

    def flee(self, hunter_pos):
        ''' move away from hunter position '''
        flee_dist = self.flee_distance
        if ((self.pos.x < hunter_pos.x + flee_dist) and (self.pos.x > hunter_pos.x - flee_dist)):
            if ((self.pos.y < hunter_pos.y + flee_dist) and (self.pos.y > hunter_pos.y - flee_dist)):
                desired_vel = (self.pos - hunter_pos).normalise() * self.max_speed
                return (desired_vel - self.vel)
        return Vector2D()

    def arrive(self, target_pos, speed):
        ''' this behaviour is similar to seek() but it attempts to arrive at
            the target position with a zero velocity'''
        decel_rate = self.DECELERATION_SPEEDS[speed]
        to_target = target_pos - self.pos
        dist = to_target.length()
        if dist > 0:
            # calculate the speed required to reach the target given the
            # desired deceleration rate
            speed = dist / decel_rate
            # make sure the velocity does not exceed the max
            speed = min(speed, self.max_speed)
            # from here proceed just like Seek except we don't need to
            # normalize the to_target vector because we have already gone to the
            # trouble of calculating its length for dist.
            desired_vel = to_target * (speed / dist)
            return (desired_vel - self.vel)
        return Vector2D(0, 0)

    def pursuit(self, evader):
        ''' this behaviour predicts where an agent will be in time T and seeks
            towards that point to intercept it. '''
## OPTIONAL EXTRA... pursuit (you'll need something to pursue!)
        return Vector2D()

    def follow_path(self):
        to_point = self.path.current_pt() - self.pos
        dist = to_point.length()
        if self.path.is_finished():
            decel_rate = self.DECELERATION_SPEEDS['normal']
            if dist > 0:
                # calculate the speed required to reach the target given the
                # desired deceleration rate
                speed = dist / decel_rate
                # make sure the velocity does not exceed the max
                speed = min(speed, self.max_speed)
                # from here proceed just like Seek except we don't need to
                # normalize the to_target vector because we have already gone to the
                # trouble of calculating its length for dist.
                desired_vel = to_point * (speed / dist)
                return (desired_vel - self.vel)
            return Vector2D(0, 0)
        else:
            if dist < self.waypoint_threshold:
                self.path.inc_current_pt()
            desired_vel = to_point.normalise() * self.max_speed
            return (desired_vel - self.vel)
                
    def wander(self, delta):
        wt = self.wander_target
        jitter_tts = self.wander_jitter * delta
        wt += Vector2D(uniform(-1, 1) * jitter_tts, uniform(-1, 1) * jitter_tts)
        wt.normalise()
        wt *= self.wander_radius
        target = wt + Vector2D(self.wander_dist, 0)
        wld_target = self.world.transform_point(target, self.pos, self.heading, self.side)
        return self.seek(wld_target)

    def alignment(self, delta):
        new_force = Vector2D()
        blank_force = Vector2D()
        for agent in self.neighbours:
            if new_force == blank_force:
                new_force = agent.force
            else:
                new_force += agent.force
        if len(self.neighbours) > 0:
            new_force /= len(self.neighbours)
        else:
            return self.wander(delta)
        return new_force

    def cohesion(self, delta):
        centre_mass = Vector2D()
        steering_force = Vector2D()
        for agent in self.neighbours:
            centre_mass += agent.pos
        if len(self.neighbours) > 0:
            centre_mass /= len(self.neighbours)
            steering_force = self.seek(centre_mass)
            return steering_force
        else:
            return self.wander(delta)

    def separation(self, delta):
        centre_mass = Vector2D()
        steering_force = Vector2D()
        for agent in self.neighbours:
            centre_mass += agent.pos
        if len(self.neighbours) > 0:
            centre_mass /= len(self.neighbours)
            steering_force = self.flee(centre_mass)
            return steering_force
        else:
            return self.wander(delta)
