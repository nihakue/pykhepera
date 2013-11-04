import numpy as np
from scipy.stats import norm
from utils import Pose
from utils import Particle
import utils
from collections import namedtuple
import raycasting


class ParticleFilter(object):
    """PARTICLE FILTER YAY"""
    def __init__(self, n, start_pose, data, scale=1, theta_scale=np.pi/16):
        self.n = n
        self.data = data
        self.start_pose = start_pose
        self.scale = scale
        self.theta_scale = theta_scale
        # self.particles = self.random_particles()
        self.particles = self.rand_gaussian_particles(start_pose, n, 1./n)
        self.likliest = self.particles[0]
        self.least_likely = self.particles[0]

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
        particles = self.sample_particles(len(self.particles))
        #Move sampled particles based on a motor control
        new_particles = [
            utils.update_pose(p, self.data.wheel_speeds, dt, noisy=True)
            for p in particles
            if p.x < 1469 and p.x > 0
            and p.y < 962 and p.y > 0]
        self.likliest = new_particles[0] #not likliest yet
        self.least_likely = new_particles[0] #not least likely yet
        new_particles.append(self.data.pose)
        if len(new_particles) < len(self.particles):
            shortage = len(self.particles) - len(new_particles)
            new_particles += (self.rand_gaussian_particles(self.likliest,
                                 shortage, self.likliest.w))
        eta = 0
        #Calculate the sensor probabilities (weights) for each particle
        for p in new_particles:
            exp_readings = raycasting.exp_readings_for_pose(p, self.data.distance_thresholds)
            # print 'exp readings', exp_readings
            # print 'acutal readings', self.data.sensor_values
            weight = sum(norm.pdf(pr, r, 100)
                              for pr, r in zip(exp_readings,
                                            self.data.sensor_values))
            eta += weight
            p.w = weight
        #Normalize weights
        print 'eta: ', eta
        if eta > 0:
            total = 0
            for p in new_particles:
                p.w = p.w/eta
                if p.w > self.likliest.w:
                    self.likliest = p #now likliest
                if p.w < self.least_likely.w:
                    self.least_likely = p
                total += p.w
        else:
            self.particles = self.random_particles()
            return

        if len(new_particles) > len(self.particles):
            new_particles.remove(self.least_likely)

        self.particles = new_particles

    def sample_particles(self, num):
        return [self.weighted_choice(self.particles)
            for i in xrange(num)]

    def likely_pose(self):
        particles = tuple((p.x, p.y, p.theta) for p in self.particles)
        xs, ys, thetas = zip(*particles)
        mean_x = np.mean(xs)
        mean_y = np.mean(ys)
        mean_theta = np.mean(thetas)
        return Pose(mean_x, mean_y, mean_theta)

    def random_particles(self):
        tup = ((np.random.uniform(0, 1469),
               np.random.uniform(0,962),
               np.random.uniform(0, 2*np.pi))
                for i in xrange(self.n))
        particles = [Particle(x, y, theta, 1./self.n)
            for (x, y, theta) in tup]
        return particles

    def rand_gaussian_particles(self, loc, num, weight):
        particles_x = np.random.normal(loc.x, self.scale, num)
        particles_y = np.random.normal(loc.y, self.scale, num)
        particles_theta = np.random.normal(loc.theta, self.theta_scale, num)
        return [Particle(x, y, z, weight) for x, y, z
                            in zip(particles_x, particles_y,
                                   particles_theta)]
