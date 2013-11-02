import numpy as np
from scipy.stats import norm
from utils import Pose
import utils
from collections import namedtuple
import raycasting

Particle = namedtuple('Particle', 'x, y, theta, w')

class ParticleFilter(object):
    """PARTICLE FILTER YAY"""
    def __init__(self, n, start_pose, data, scale=1, theta_scale=np.pi/4):
        self.n = n
        self.data = data
        self.start_pose = start_pose
        self.scale = scale
        self.theta_scale = theta_scale
        particles_x = np.random.normal(start_pose.x, scale, n)
        particles_y = np.random.normal(start_pose.y, scale, n)
        particles_theta = np.random.normal(start_pose.theta,theta_scale, n)
        self.particles = [Particle(x, y, z, 1./n) for x, y, z
                            in zip(particles_x, particles_y,
                                   particles_theta)]

    def get_x(self):
        return [pose.x for pose in self.particles]

    def get_y(self):
        return [pose.y for pose in self.particles]

    def get_theta(self):
        return [pose.theta for pose in self.particles]

    def gauss(self):
        xy = np.random.normal(0, self.scale, 2)
        theta = np.random.normal(0, self.theta_scale, 1)
        weight = (0,)
        return np.append(xy, [theta, weight])

    def weighted_choice(self, choices):
       total = sum(p.w for p in choices)
       r = np.random.uniform(0, total)
       upto = 0
       for p in choices:
            if upto + p.w > r:
                return p
            upto += p.w
       assert False, "Shouldn't get here"

    def update(self, dt):
        particles = [self.weighted_choice(self.particles)
            for i in range(len(self.particles))]
        new_particles = [
            utils.update_pose(p, self.data.wheel_speeds, dt, noisy=True)
            for p in self.particles]
        for p in new_particles:
            exp_readings = raycasting.exp_readings_for_pose(p, self.data.distance_thresholds)
            # probability = sum(norm.pdf(pr, r, 1)
            #                   for pr, r in zip(exp_readings,
            #                                 self.data.sensor_values))
            # print exp_readings

        self.particles = new_particles

