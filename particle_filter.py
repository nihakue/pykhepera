import numpy as np
from utils import Pose
import utils
from collections import namedtuple

Particle = namedtuple('Particle', 'x, y, theta, w')

class ParticleFilter(object):
    """PARTICLE FILTER YAY"""
    def __init__(self, n, start_pose, data, scale=10):
        theta_scale = np.pi/4
        self.n = n
        self.data = data
        self.start_pose = start_pose
        particles_x = np.random.normal(start_pose.x, scale, n)
        particles_y = np.random.normal(start_pose.y, scale, n)
        particles_theta = np.random.normal(start_pose.theta,theta_scale , n)
        self.particles = [Particle(x, y, z, .1) for x, y, z in zip(particles_x,
                                                           particles_y,
                                                           particles_theta)]

    def get_x(self):
        return [pose.x for pose in self.particles]

    def get_y(self):
        return [pose.y for pose in self.particles]

    def get_theta(self):
        return [pose.theta for pose in self.particles]

    def normal(self, position):
        pass

    def update(self, dt):
        new_particles = [utils.update_pose(p, self.data.wheel_speeds, dt) for p in self.particles]
        self.particles = new_particles
