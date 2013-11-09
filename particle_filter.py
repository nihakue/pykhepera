import numpy as np
from scipy.stats import norm
from utils import Pose
from utils import Particle
import utils
import time
from multiprocessing import Pool
from collections import namedtuple
import raycasting
import itertools
import pickle


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
        with open('raycasting_table_bin.data', 'rb') as td:
            print 'loading giant sensor table...'
            self.table_data = pickle.loads(td.read())

    def get_x(self):
        return [pose.x for pose in self.particles]

    def get_y(self):
        return [pose.y for pose in self.particles]

    def get_theta(self):
        return [pose.theta for pose in self.particles]

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
        t0 = time.time()
        particles = self.sample_particles(len(self.particles))
        #Move sampled particles based on a motor control
        new_particles = [
            utils.update_pose(p, self.data.wheel_speeds, dt, noisy=True)
            for p in particles
            if p.x < 1469 and p.x > 0
            and p.y < 962 and p.y > 0]
        # odo_pose = self.data.pose
        # odo_pose.w = 1./len(new_particles)*10
        # new_particles.append(odo_pose)
        if len(new_particles) < self.n:
            shortage = self.n - len(new_particles)
            likliest_particle = self.most_likely()
            new_particles += (self.rand_gaussian_particles(likliest_particle,
                                 shortage, likliest_particle.w))
        eta = 0
        #Calculate the sensor probabilities (weights) for each particle
        #Use a pool of workers to utilize multiple cores
        # exp_readings = self.pool.map(raycasting.exp_readings_for_pose_star, itertools.izip(new_particles, itertools.repeat(self.data.distance_thresholds)))
        # if len(exp_readings) != len(new_particles):
        #     assert False, "Array of expected readings must have the same size as the array of new particles. exp_readings: %d new_particles: %d" % (len(exp_readings), len(new_particles))

        #Sum sensor probabilites (assumption is that they are independent)
        for p in new_particles:
            print p
            weight = self.probability_sum(4, self.table_data[p])
            eta += weight
            p.w = weight

        #Normalize weights
        print 'eta: ', eta
        new_particles = self.normalize(eta, new_particles)

        while len(new_particles) > len(self.particles):
            new_particles.remove(self.least_likely(new_particles))

        self.particles = new_particles
        duration = time.time() - t0
        # print 'update took %.4f seconds' % duration

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

    def random_particles(self, quantity):
        tup = ((np.random.uniform(0, 1469),
               np.random.uniform(0,962),
               np.random.uniform(0, 2*np.pi))
                for i in xrange(quantity))
        particles = [Particle(x, y, theta, self.least_likely(self.particles).w)
            for (x, y, theta) in tup]
        return particles

    def rand_gaussian_particles(self, loc, num, weight):
        particles_x = np.random.normal(loc.x, self.scale, num)
        particles_y = np.random.normal(loc.y, self.scale, num)
        particles_theta = np.random.normal(loc.theta, self.theta_scale, num)
        return [Particle(x, y, z, weight) for x, y, z
                            in zip(particles_x, particles_y,
                                   particles_theta)]

    def normalize(self, eta, particles):
        if eta > 0:
            for p in particles:
                p.w = p.w/eta
                if p.w > self.likliest.w:
                    self.likliest = p #now likliest
            return particles
        else:
            self.particles = self.random_particles(self.n)
            return

    def least_likely(self, particles):
        return min(particles, key=weight_key)

    def most_likely(self, particles=None):
        if not particles:
            particles = self.particles
        return max(particles, key=weight_key)

    def probability_sum(self, scale, exp_reading):
        prob_sum = sum(norm.pdf(pr, r, 60)
                              for pr, r in zip(exp_reading,
                                            self.data.sensor_values))
        # print 'probability of: %s when reading: %s' % (exp_reading, self.data.sensor_values)
        # print prob_sum
        return prob_sum

    def tear_down(self):
        self.pool.terminate()
        self.pool.join()

def weight_key(particle):
    return particle.w
