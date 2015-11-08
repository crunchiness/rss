import time

import numpy as np

from body.sensors import SensorRunningAverage
import utils
from robot.state.map import NODES

from robot.utils import log

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


def wander(sensors, particles, motors, front_ir, right_ir, state, vision):
    """Loop for when we are unsure of our location at all
    """

    log('Starting wandering mode')

    x, y, o, xy_conf = particles.get_position_by_weighted_average()
    while xy_conf < LOCALISATION_CONF:

        # localization - driving around avoiding obstacles
        front_avg = front_ir.get_avg()
        right_avg = right_ir.get_avg()
        while front_avg > 20 and right_avg > 15:

            # Move forwards 10 cm
            motors.go_forward(10)
            particles.forward(10)

            state['room_belief'].update_belief(vision.belief)

            # Update position via particle filter
            front_ir_reading = sensors.get_ir_front()
            right_ir_reading = sensors.get_ir_right()
            front_ir.add_value(front_ir_reading)
            right_ir.add_value(right_ir_reading)
            front_avg = front_ir.get_avg()
            right_avg = right_ir.get_avg()
            particles.sense({
                'IR_left': front_ir_reading if front_ir_reading is not None else 0,
                'IR_right': right_ir_reading if right_ir_reading is not None else 0,
            })
            x, y, o, xy_conf = particles.get_position_by_weighted_average()
            particles.resample()

            # we are sure enough, go back to high level plan execution
            if xy_conf >= LOCALISATION_CONF:
                log('Gained confidence while wandering, going into travelling mode')
                state['mode'] = 'travelling'
                return

            # Update running average
            front_ir.add_value(front_ir_reading)
            right_ir.add_value(right_ir_reading)

        if front_avg <= 15:
            motors.turn_by(30)
            particles.rotate(30. * np.pi / 180.)
        elif right_avg <= 15:
            motors.turn_by(-30)
            particles.rotate(-30. * np.pi / 180.)


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

            front_ir_reading = sensors.get_ir_front()
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


def look_around(motors, sensors, front_ir, right_ir, particles, state, vision):

    log('Starting look_around')

    n = 12
    for i in xrange(n):
        motors.turn_by(360 / n)
        particles.rotate(2 * np.pi / n)
        front_ir_reading = sensors.get_ir_front()
        right_ir_reading = sensors.get_ir_right()
        front_ir.add_value(front_ir_reading)
        right_ir.add_value(right_ir_reading)
        particles.sense({
            'IR_left': front_ir_reading if front_ir_reading is not None else 0,
            'IR_right': right_ir_reading if right_ir_reading is not None else 0,
        })
        particles.resample()
        state['room_belief'].update_belief(vision.belief)

    start_room = state['room_belief'].get_belief(basic_start=False)

    log('Start room identified as ' + start_room)

    f = open('start_room.txt', 'w')
    f.write(start_room)
    f.close()
    state['mode'] = 'travelling'


def wander_and_travel(sensors, particles, motors, vision):
    """Robot logic for milestone 1
       It has two states:
        * 'wandering' - driving around avoiding obstacles (until particle filtering cannot provide a location)
        * 'travelling' - when we are sure of our location, go to the destination room (keep localizing with the filter)
       Robot may fall back to wandering if we get lost.
    """

    log('Starting wander_and_travel')

    state = {
        'mode': 'starting',
        'room_belief': RoomBelief()
    }
    front_ir = SensorRunningAverage()
    right_ir = SensorRunningAverage()

    while True:
        if state['mode'] == 'starting':
            print 'Looking around'
            look_around(motors, sensors, front_ir, right_ir, particles, state, vision)
        elif state['mode'] == 'wandering':
            print 'Wandering'
            wander(sensors, particles, motors, front_ir, right_ir, state, vision)
        elif state['mode'] == 'travelling':
            print 'Travelling'
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
    value = sensors.get_ir_front()
    total_distance = 0
    while total_distance < 170:
        while value < 70:
            motors.turn_by(10)
            value = sensors.get_ir_front()
        motors.go_forward(10)
        total_distance += 10
        value = sensors.get_ir_front()
        time.sleep(0.3)
    if DESTINATION_ROOM in ['E', 'B']:
        # right
        motors.turn_by(90)
    elif DESTINATION_ROOM in ['C', 'D']:
        motors.turn_by(-90)
        # left
    value = sensors.get_ir_front()
    while value > 20:
        motors.go_forward(10)
        value = sensors.get_ir_front()

    # Stop this madness
    while True:
        pass
