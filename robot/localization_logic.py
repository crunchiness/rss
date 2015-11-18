import time

import numpy as np

from body.sensors import SensorRunningAverage
import utils
from robot.state.map import NODES

from robot.utils import log

from robot.body.motors import HALL_ANGLE, HALL_PERIMETER

DESTINATION_ROOM = 'B'

# When this is reached we are sure enough of our location
LOCALISATION_CONF = 0.6

# If we fall down to this level, start exploring again
LOCALISATION_CONF_BREAK = 0.4

# Max step size in cm when were executing a plan
MAX_STEP_SIZE = 5.


class RoomBelief:
    """Room belief that's based on vision data alone
    """
    def __init__(self, history_size=5, decay=1.5):
        self.history_size = history_size
        self.decay = decay
        self.observations = history_size * [None]  # list of sets of pieces seen
        self.pieces = {
            'green': {'rooms': ['B', 'C', 'F'], 'prob': 0.9},
            'orange': {'rooms': ['B'], 'prob': 0.3},
            'blue': {'rooms': ['A', 'D', 'E'], 'prob': 0.8},
            'white': {'rooms': ['C'], 'prob': 0.3},
            'yellow': {'rooms': ['B', 'D'], 'prob': 0.8},
            'black': {'rooms': ['A', 'D', 'E', 'F'], 'prob': 0.3},  # including bases
            'red': {'rooms': ['E'], 'prob': 0.8}
        }

    def update_belief(self, vision_belief):
        seen = set([])
        for key in vision_belief.keys():
            if vision_belief[key]:
                seen.add(key)
        self.observations = self.observations[1:] + [seen]

    def get_belief(self, basic_start=False):
        """
        :param basic_start: if set to true, must return A or F
        :return:
        """
        # won't actually add up to one, but it's ok
        room_probabilities = {
            'A': 0.,
            'B': 0.,
            'C': 0.,
            'D': 0.,
            'E': 0.,
            'F': 0.
        }
        for i in xrange(self.history_size):
            observation = self.observations[self.history_size - 1 - i]
            for color in observation:
                rooms = self.pieces[color]['rooms']
                prob = self.pieces[color]['prob']
                for room in rooms:
                    room_probabilities[room] += prob ** (self.decay ** i)
        max_prob = 0.
        max_prob_room = 'A'
        for key in room_probabilities:
            if basic_start:
                if room_probabilities[key] > max_prob and key in ['A', 'F']:
                    max_prob = room_probabilities[key]
                    max_prob_room = key
            else:
                if room_probabilities[key] > max_prob:
                    max_prob = room_probabilities[key]
                    max_prob_room = key

        return max_prob_room


def wander(sensors, particles, motors, left_ir, right_ir, sonar, state, vision):
    """Loop for when we are unsure of our location at all
    """

    log('Starting wandering mode')

    #TODO HACK
    x, y, o = particles.get_position_by_max_weight()
    xy_conf = 0.3
    while xy_conf < LOCALISATION_CONF:

        # localization - driving around avoiding obstacles
        front_ir_reading = sensors.get_ir_left()
        right_ir_reading = sensors.get_ir_right()
        sonar_reading = sensors.get_sonar()

        # Update position via particle filter
        measurements = {
            'IR_left': front_ir_reading if front_ir_reading is not None else 0,
            'IR_right': right_ir_reading if right_ir_reading is not None else 0,
            'sonar': sonar_reading if sonar_reading is not None else 0,
        }

        log('Measurement predictions: {}'
        .format(particles.measurement_prediction_explicit(np.array([x, y]), o)))

        particles.sense(measurements)
        particles.resample()
        x, y, o = particles.get_position_by_max_weight()

        if front_ir_reading > 30 and right_ir_reading > 30:
            # Move forwards 16.5 cm
            log('Going {}cm forward'.format(3*HALL_PERIMETER))
            motors.go_forward(3*HALL_PERIMETER)
            particles.forward(3*HALL_PERIMETER)

            #state['room_belief'].update_belief(vision.belief)

            # we are sure enough, go back to high level plan execution
            # if xy_conf >= LOCALISATION_CONF:
            #     log('Gained confidence while wandering, going into travelling mode')
            #     state['mode'] = 'travelling'
            #     return

        if front_ir_reading <= 30:
            log('Turning {} angles left'.format(2*HALL_ANGLE))
            motors.turn_by(-2*HALL_ANGLE)
            particles.rotate(-2*HALL_ANGLE * np.pi / 180.)
        elif right_ir_reading <= 30:
            log('Turning {} angles right'.format(2*HALL_ANGLE))
            motors.turn_by(2*HALL_ANGLE)
            particles.rotate(2*HALL_ANGLE * np.pi / 180.)

def travel(sensors, particles, motors, state, vision):
    """Loop for when we know what where we are, aka loop for travelling
    """
    # TODO: add obstacle avoidance

    log('Starting travel to room ' + DESTINATION_ROOM)

    x, y, o, xy_conf = particles.get_position_by_weighted_average()
    path_to_room = utils.get_path_to_room(NODES, x, y, DESTINATION_ROOM)

    for milestone in path_to_room:
        while not utils.reached_milestone(milestone, x, y):

            # orientate towards the goal
            turn_angle = utils.orientate(milestone, x, y, o)
            motors.turn_by(turn_angle, radians=True)
            particles.rotate(turn_angle)

            # go a small step forwards
            distance = utils.euclidean_distance((milestone['x'], milestone['y']), (x, y))
            if distance > MAX_STEP_SIZE:
                motors.go_forward(distance)
                particles.forward(distance)

            # keep sensing the world
            state['room_belief'].update_belief(vision.belief)

            front_ir_reading = sensors.get_ir_left()
            right_ir_reading = sensors.get_ir_right()
            particles.sense({
                'IR_left': front_ir_reading,
                'IR_right': right_ir_reading
            })
            x, y, o, xy_conf = particles.get_position_by_weighted_average()
            particles.resample()
            if xy_conf < LOCALISATION_CONF_BREAK:
                # we got lost, go back to localization
                log('Got lost in travel mode while looking for ' + DESTINATION_ROOM)
                log('Going into wandering mode')
                state['mode'] = 'wandering'
                return


def look_around(motors, sensors, front_ir, right_ir, sonar, particles, state, vision):

    log('Starting look_around')

    multiple = 2.0

    n = int(360/(multiple*HALL_ANGLE))
    for i in xrange(n):
        motors.turn_by((multiple*HALL_ANGLE))
        particles.rotate((multiple*HALL_ANGLE)/180. * np.pi)
        front_ir_reading = sensors.get_ir_left()
        right_ir_reading = sensors.get_ir_right()
        sonar_reading = sensors.get_sonar()

        measurements = {
            'IR_left': front_ir_reading if front_ir_reading is not None else 0,
            'IR_right': right_ir_reading if right_ir_reading is not None else 0,
            'sonar': sonar_reading if sonar_reading is not None else 0,
        }

        log('Measurements: {}'
        .format(particles.measurement_prediction_explicit(np.array([x, y]), o)))

        particles.sense(measurements)
        particles.resample()
        x, y, o = particles.get_position_by_weighted_average()
        particles.get_position_by_max_weight()
        #state['room_belief'].update_belief(vision.belief)

    #start_room = state['room_belief'].get_belief(basic_start=False)

    # log('Start room identified as ' + start_room)
    #
    # f = open('start_room.txt', 'w')
    # f.write(start_room)
    # f.close()
    state['mode'] = 'wandering'


def wander_and_travel(sensors, particles, motors, vision):
    """Robot logic for milestone 1
       It has two states:
        * 'wandering' - driving around avoiding obstacles (until particle filtering cannot provide a location)
        * 'travelling' - when we are sure of our location, go to the destination room (keep localizing with the filter)
       Robot may fall back to wandering if we get lost.
    """

    log('Starting wander_and_travel')

    state = {
        'mode': 'wandering',
        'room_belief': RoomBelief()
    }
    front_ir = SensorRunningAverage()
    right_ir = SensorRunningAverage()
    sonar = SensorRunningAverage()

    x, y, o = particles.get_position_by_weighted_average()
    particles.get_position_by_max_weight()
    log('Measurement prediction at weighted average: {}'
        .format(particles.measurement_prediction_explicit(np.array([x, y]), o)))

    while True:
        if state['mode'] == 'starting':
            log('Looking around')
            look_around(motors, sensors, front_ir, right_ir, sonar, particles, state, vision)
        elif state['mode'] == 'wandering':
            log('Wandering')
            wander(sensors, particles, motors, front_ir, right_ir, sonar, state, vision)
        elif state['mode'] == 'travelling':
            log('Travelling')
            travel(sensors, particles, motors, state, vision)
        else:
            raise Exception('Unknown state {0}'.format(state['mode']))


def perform_basic_milestone(sensors, motors):
    # Detect the room
    # n = 12
    # for i in xrange(n):
    #     motors.turn_by(360 / n)
    #     time.sleep(0.5)

    log('Starting perform_basic_milestone')

    start_room = 'A' if DESTINATION_ROOM in ['B', 'C'] else 'F'

    log('Start room identified as ' + start_room)

    f = open('start_room.txt', 'w')
    f.write(start_room)
    f.close()

    # Go to destination
    value = sensors.get_ir_left()
    total_distance = 0
    while total_distance < 170:
        while value < 70:
            motors.turn_by(10)
            value = sensors.get_ir_left()
        motors.go_forward(10)
        total_distance += 10
        value = sensors.get_ir_left()
        time.sleep(0.3)
    if DESTINATION_ROOM in ['E', 'B']:
        # right
        motors.turn_by(90)
    elif DESTINATION_ROOM in ['C', 'D']:
        motors.turn_by(-90)
        # left
    value = sensors.get_ir_left()
    while value > 20:
        motors.go_forward(10)
        value = sensors.get_ir_left()

    # Stop this madness
    while True:
        pass
