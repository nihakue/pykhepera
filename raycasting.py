from scipy import misc
from collections import namedtuple
from collections import OrderedDict
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.mlab import find as m_find
import utils

__arena = np.array([])
__lidar = OrderedDict(
             [('port', np.pi/2),
             ('port_quarter', np.pi/4),
             ('port_bow', 0),
             ('stbd_bow', 0),
             ('stbd_quarter', -np.pi/4),
             ('stbd', -np.pi/2),
             ('stbd_stern', np.pi),
             ('port_stern', np.pi)])


def load_arena():
    global __arena
    if not __arena.any():
        __arena = np.flipud(misc.imread('arena.bmp')).T


def to_IR(distance):
    pass


def exp_readings_for_pose(pose, thresholds, ir_range=80,
                          radius=26.5, plot=False):
    if type(thresholds) is list:
        thresholds_s = thresholds
    else:
        sorted_keys = thresholds.keys()
        sorted_keys.sort()
        thresholds_s = [thresholds[key] for key in sorted_keys
                       if 'sensor' in key]
    exp_ds = exp_distances_for_pose(pose, ir_range, radius, plot)
    readings = [utils.estimated_reading(d, t)
                for d, t in zip(exp_ds, thresholds_s)]
    return readings


def exp_distances_for_pose(pose, ir_range=80, radius=26.5, plot=False):
    '''accepts a pose containing x, y, and phi(heading),
    uses raycasting to determine the nearest obstacles, and returns
    the distance to those obstacles.
    optional lidar ir_range and robot radius'''
    global __arena
    global __lidar

    load_arena()
    if plot:
        plt.ion()
        plt.imshow(__arena, origin='lower')
        plt.xlim(-5, np.size(__arena, axis=1))
        plt.ylim(-5, np.size(__arena, axis=0))

    lidar_range = [40, 40, 40, 40, 40, 40, 40, 40]
    i = 0
    dist = 999
    for laser, theta in __lidar.iteritems():
        phi = pose.theta + theta
        offset = phi
        bow_stern_offset = 14.5

        if 'bow' in laser:
            if 'stbd' in laser:
                offset += np.deg2rad(-bow_stern_offset)
            else:
                offset += np.deg2rad(bow_stern_offset)
        if 'stern' in laser:
            if 'stbd' in laser:
                offset += np.deg2rad(bow_stern_offset)
            else:
                offset += np.deg2rad(-bow_stern_offset)

        r = np.linspace(0, ir_range, 10)
        x_offset = radius*np.cos(offset)
        y_offset = radius*np.sin(offset)
        x = pose.x + x_offset + (r*np.cos(phi))
        y = pose.y + y_offset + (r*np.sin(phi))
        # remove out of bounds points
        temp = []
        it = np.nditer(x, flags=['f_index'])
        while not it.finished:
            if (x[it.index] > np.size(__arena, axis=0)
                or y[it.index] > np.size(__arena, axis=1)
                or x[it.index] <= 0 or y[it.index] <= 0):
                temp.append(it.index)
            it.iternext()
        x[temp] = []
        y[temp] = []
        if plot:
            plt.plot(x, y, 'r')
        #computing intersections
        xint = np.round(x)
        yint = np.round(y)
        #Correcting zero map indexing
        xint2 = [1 if a==0 else a for a in xint]
        yint2 = [1 if b==0 else b for b in yint]
        b = [__arena[xint2[j],yint2[j]]
            for j in xrange(len(xint2))
            if xint2[j] < __arena.shape[0]
            and yint2[j] < __arena.shape[1]]
        indices = m_find(np.ravel(b) == 1)
        if indices.any():
            xb = x[indices[0]]
            yb = y[indices[0]]
            if plot:
                plt.plot(xb, yb, 'g*')
                robot_circle = plt.Circle((pose.x, pose.y),
                                          radius=radius, color='g')
                plt.gca().add_artist(robot_circle)
            dist = np.sqrt((pose.x - xb)**2 + (pose.y - yb)**2) - radius
            #update the ir_range dictionary
            lidar_range[i] = dist

        i += 1
    return lidar_range

if __name__ == '__main__':
    load_arena()
