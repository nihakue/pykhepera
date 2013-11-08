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


def load_arena(res='mm'):
    global __arena
    if not __arena.any():
        if res is 'mm':
            arena = 'arena.bmp'
        elif res is 'cm':
            arena = 'arena_cm.bmp'
        __arena = np.flipud(misc.imread(arena)).T


def get_arena(res):
    if res is 'mm':
        arena = 'arena.bmp'
    elif res is 'cm':
        arena = 'arena_cm.bmp'
    return np.flipud(misc.imread(arena)).T


def to_IR(distance):
    pass

def exp_readings_for_pose_star(argv):
    return exp_readings_for_pose(*argv)

def exp_readings_for_pose(pose, thresholds, ir_range=80,
                          radius=27.5, plot=False, res='cm'):
    if type(thresholds) is list:
        thresholds_s = thresholds
    else:
        sorted_keys = thresholds.keys()
        sorted_keys.sort()
        thresholds_s = [thresholds[key] for key in sorted_keys
                       if 'sensor' in key]
    exp_ds = exp_distances_for_pose(pose, ir_range, radius, plot, res)
    readings = [utils.estimated_reading(d, t)
                for d, t in zip(exp_ds, thresholds_s)]
    return readings


def exp_distances_for_pose(pose, ir_range=61, radius=26.5, plot=False, res='cm'):
    '''accepts a pose containing x, y, and phi(heading),
    uses raycasting to determine the nearest obstacles, and returns
    the distance to those obstacles.
    optional lidar ir_range and robot radius'''
    global __lidar
    if res is 'cm':
        ir_range = ir_range/10
        radius = radius/10

    arena = get_arena(res)

    if plot:
        plt.ion()
        plt.imshow(__arena, origin='lower')
        plt.xlim(-5, np.size(__arena, axis=1))
        plt.ylim(-5, np.size(__arena, axis=0))

    if type(pose) is tuple:
        Pose = namedtuple('Pose', 'x, y, theta')
        pose = Pose(pose[0], pose[1], pose[2])

    #TODO: Set this to the max threshold value
    lidar_range = [40, 40, 40, 40, 40, 40, 40, 40]
    i = 0
    dist = 999
    for laser, theta in __lidar.iteritems():
        phi = pose.theta + theta
        offset = phi
        bow_stern_offset = 14.5
        if res is 'cm':
            bow_stern_offset = bow_stern_offset/10

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

        r = np.linspace(0, ir_range, ir_range)
        x_offset = radius*np.cos(offset)
        y_offset = radius*np.sin(offset)
        x = pose.x + x_offset + (r*np.cos(phi))
        y = pose.y + y_offset + (r*np.sin(phi))
        # remove out of bounds points
        temp = []
        it = np.nditer(x, flags=['f_index'])
        while not it.finished:
            if (x[it.index] > np.size(arena, axis=0)
                or y[it.index] > np.size(arena, axis=1)
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
        b = [arena[xint2[j],yint2[j]]
            for j in xrange(len(xint2))
            if xint2[j] < arena.shape[0]
            and yint2[j] < arena.shape[1]]
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


def generate_table():
    import data
    import pickle
    from itertools import product
    from itertools import izip
    from itertools import repeat
    from multiprocessing import Pool
    from collections import namedtuple
    arena = get_arena('cm')
    d = data.Data()
    d.load_calibration()
    p = Pool(processes=16)
    print 'creating a big ass table of data...'
    possible_poses = [utils.Pose(x, y, theta)
                        for (x, y, theta) in product(xrange(arena.shape[0]),
                                                     xrange(arena.shape[1]),
                                                     utils.pirange())]
    print 'created a table with: %d entries' % len(possible_poses)
    print 'calculating a bunch of reading values...'
    exp_readings = p.map(
                         exp_readings_for_pose_star,
                         izip(possible_poses,
                         repeat(d.distance_thresholds)))
    # table = dict((utils.Pose(x, y, z), exp_readings_for_pose(utils.Pose(x, y, z), d.distance_thresholds))
    #              for (x, y, z) in product(range(10), range(10), utils.pirange()))
    table = dict(izip(possible_poses, exp_readings))
    with open('raycasting_table_bin.data', 'w') as raycasting_table:
	pickler = pickle.Pickler(raycasting_table, protocol=-1)
	pickler.dump(table)


if __name__ == '__main__':
    generate_table()

