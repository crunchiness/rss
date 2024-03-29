import os
import numpy as np
import datetime

from robot.state.map import Y_MAX, X_MAX

from robot.state.utils import at_orientation

from robot.body.sensors import Sensors
import time

def make_file_path(path=None):
    base_path = os.path.dirname(os.path.dirname(os.path.realpath('__file__')))
    base_path += '/student'
    path = base_path + '/log/' if path is None else base_path + '/' + path
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def angle_between(p1, p2):
    """Returns the angle in radians between two points"""
    vec = np.array(p2) - np.array(p1)
    if vec[0] == 0.:
        return 0.
    angle = np.arctan(vec[1] / vec[0])
    if vec[0] < 0 and vec[1] < 0:
        return angle + 0.5 * np.pi
    return angle

def euclidean_distance(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def find_shortest_path(nodes, start, end, path=None):
    if path is None:
        path = [start]
    else:
        path = path + [start]
    if start == end:
        return path
    if start not in nodes:
        return None
    shortest_path = None
    for node in nodes[start]['connects']:
        if node not in path:
            new_path = find_shortest_path(nodes, node, end, path)
            if new_path:
                if not shortest_path or len(new_path) < len(shortest_path):
                    shortest_path = new_path
    return shortest_path

def get_nearest_node(nodes, x, y):
    """
    :return: tuple (nearest node name, distance to closest node)
    """
    min_dist = 999999999
    min_dist_node = ''
    a = np.array((x, y))
    for key in nodes.keys():
        dist = euclidean_distance(a, (nodes[key]['x'], nodes[key]['y']))
        if dist < min_dist:
            min_dist = dist
            min_dist_node = key
    return min_dist_node, min_dist

def get_path_to_room(nodes, x, y, destination_room):
    """
    :return: list of actual nodes (not keys) that are MILESTONES in our path
    """
    nearest_node_key, distance = get_nearest_node(nodes, x, y)
    possible_destination_nodes = []
    for key in nodes.keys():
        if nodes[key]['room'] == destination_room and not nodes[key]['ambiguous']:
            possible_destination_nodes.append(key)

    # get shortest paths to unambiguous nodes in destination_room
    paths = [find_shortest_path(nodes, nearest_node_key, dst) for dst in possible_destination_nodes]

    # find the shortest of them
    path = reduce(lambda a, b: a if len(a) < len(b) else b, paths[1:], paths[0])

    return map(lambda node_key: nodes[node_key], path)

def reached_milestone(milestone, x, y, tolerance=5):
    """
    :param milestone: node
    :param x:
    :param y:
    :param tolerance: distance in cm that we allow robot to be out of place
    :return:
    """
    return euclidean_distance((x, y), (milestone['x'], milestone['y'])) < tolerance


def orientate2(milestone, x, y, orientation):
    """
    :param milestone: node
    :param x:
    :param y:
    :param orientation:
    :return: angle by which to turn to be facing the milestone
    """
    v1 = np.array((x, y))
    v2 = np.array((milestone['x'], milestone['y']))
    angle = angle_between(v1, v2) + 0.5 * np.pi - orientation
    simplified = - (2 * np.pi - angle) if angle > np.pi else angle
    return simplified

def orientate(milestone, x, y, orientation):
    """
    :param milestone: node
    :param x:
    :param y:
    :param orientation:
    :return: angle by which to turn to be facing the milestone
    """
    v1 = np.array((float(x), float(y)))
    v2 = np.array((float(milestone['x']), float(milestone['y'])))
    vprime = v2-v1
    vprime = vprime/np.linalg.norm(vprime)
    current_vector = at_orientation(np.array([0., 1.]), orientation)
    output = np.arctan2(vprime[1],vprime[0]) - np.arctan2(current_vector[1],current_vector[0])
    return -output


from robot.state.map import NODES

assert get_nearest_node(NODES, 280, 155) == ('F1', 0.0)
assert get_nearest_node(NODES, 283, 159) == ('F1', 5.)

# test_1 = (orientate({'x': 5, 'y': 5}, 0, 5, -91. * np.pi / 180.) / np.pi) * 180.
# expect_1 = 179.
# assert test_1 == expect_1, 'Got {0}, expected {1}'.format(test_1, expect_1)
#
# test_2 = (orientate({'x': 5, 'y': 5}, 0, 5, -90. * np.pi / 180.) / np.pi) * 180.
# expect_2 = -180.
# assert test_2 == expect_2, 'Got {0}, expected {1}'.format(test_2, expect_2)
#
# test_3 = orientate({'x': 0, 'y': 0}, 5, 5, np.pi) * 180. / np.pi
# expect_3 = -45.
# assert test_3 == expect_3, 'Got {0}, expected {1}'.format(test_3, expect_3)

# test_4 = orientate({'x': 0, 'y': 0}, 0, 0, 0.5 * np.pi)
# expect_4 = 0.
# assert test_4 == expect_4, 'Got {0}, expected {1}'.format(test_4, expect_4)


# not used
def determine_room(x, y):
    # TODO: add obstacles
    if not 0 <= x < X_MAX:
        raise Exception("X coordinate doesn't make sense: {0}".format(x))
    if not 0 <= y < Y_MAX:
        raise Exception("Y coordinate doesn't make sense: {0}".format(y))

    if y < 132:
        # A or B
        if x < 143:
            return 'A'
        else:
            return 'B'
    elif y < 231-46:
        # C or B
        if x < 143:
            return 'C'
        else:
            return 'B'
    elif y < 231 - 46 + 49:
        # C, C2 or B
        if x < 143:
            return 'C'
        elif x < 143 + 34:
            return 'C2'
        else:
            return 'B'
    elif y < 132 + 162:
        # C or D
        if x < 143:
            return 'C'
        else:
            return 'D'
    elif y < 132 + 162 + 46:
        # E, C1 or D
        if x < 93:
            return 'E'
        elif x < 93 + 50:
            return 'C1'
        else:
            return 'D'
    elif y < 231 + 162:
        # E or D
        if x < 177:
            return 'E'
        else:
            return 'D'
    else:
        # E or F
        if x < 177:
            return 'E'
        else:
            return 'F'


def log(text):
    print (u'{0} ::: {1}'.format(datetime.datetime.now().strftime('%H:%M:%S'), text))


def collect_front_IR_and_sonar_measurements(io, ok):
    sensors = Sensors(io)

    i = 0
    base_path = os.path.dirname(os.path.dirname(__file__))
    path = base_path + '/log/collect_front_IR_and_sonar_measurements{}.csv'
    while os.path.exists(path.format(i)):
        i += 1
    path = path.format(i)

    log(path)

    f = open(path, 'w')
    while ok():
        text = str(sensors.get_irs_and_sonar())
        log(text)
        f.write(text)
        f.write('\n')
        time.sleep(0.01)
    f.close()