from body.sensors import SensorRunningAverage
import utils
from state.map import NODES

DESTINATION_ROOM = 'D'

# When this is reached we are sure enough of our location
LOCALISATION_CONF = 0.6

# If we fall down to this level, start exploring again
LOCALISATION_CONF_BREAK = 0.4

# Max step size in cm when were executing a plan
MAX_STEP_SIZE = 5.

# TODO: incorporate vision!!


def wander(sensors, particles, motors, front_ir, right_ir, state):
    """Loop for when we are unsure of our location at all
    """
    x, y, o, xy_conf = particles.get_position_by_weight()
    while xy_conf < LOCALISATION_CONF:

        # localization - driving around avoiding obstacles
        front_avg = front_ir.get_avg()
        right_avg = right_ir.get_avg()
        while front_avg > 15 and right_avg > 15:

            # Move forwards 10 cm
            motors.go_forward(10)
            particles.forward(10)

            # Update position via particle filter
            front_ir_reading = sensors.get_ir_front()
            right_ir_reading = sensors.get_ir_right()
            front_avg = front_ir.get_avg()
            right_avg = right_ir.get_avg()
            particles.sense({
                'front_ir': front_ir_reading if front_ir_reading is not None else 0,
                'right_ir': right_ir_reading if right_ir_reading is not None else 0,
            })
            x, y, o, xy_conf = particles.get_position_by_weight()

            # we are sure enough, go back to high level plan execution
            if xy_conf >= LOCALISATION_CONF:
                state['state'] = 'travelling'
                return

            # Update running average
            front_ir.add_value(front_ir_reading)
            right_ir.add_value(right_ir_reading)

        if front_avg <= 15:
            motors.turn_by(30)
        elif right_avg <= 15:
            motors.turn_by(-30)


def travel(sensors, particles, motors, state):
    """Loop for when we know what where we are, aka loop for travelling
    """
    # TODO: add obstacle avoidance
    x, y, o, xy_conf = particles.get_position_by_weight()
    path_to_room = utils.get_path_to_room(NODES, x, y, DESTINATION_ROOM)

    for milestone in path_to_room:
        while not utils.reached_milestone(milestone, x, y):

            # orientate towards the goal
            turn_angle = utils.orientate(milestone, x, y, o)
            motors.turn_by(turn_angle)

            # go a small step forwards
            distance = utils.euclidean_distance((milestone['x'], milestone['y']), (x, y))
            if distance > MAX_STEP_SIZE:
                motors.go_forward(distance)

            # keep sensing the world
            front_ir_reading = sensors.get_ir_front()
            right_ir_reading = sensors.get_ir_right()
            particles.sense({
                'front_ir': front_ir_reading,
                'right_ir': right_ir_reading
            })
            x, y, o, xy_conf = particles.get_position_by_weight()
            if xy_conf < LOCALISATION_CONF_BREAK:
                # we got lost, go back to localization
                state['state'] = 'wandering'
                return


def wander_and_travel(sensors, particles, motors):
    """Robot logic for milestone 1
       It has two states:
        * 'wandering' - driving around avoiding obstacles (until particle filtering cannot provide a location)
        * 'travelling' - when we are sure of our location, go to the destination room (keep localizing with the filter)
       Robot may fall back to wandering if we get lost.
    """
    state = {'state': 'wandering'}

    front_ir = SensorRunningAverage()
    right_ir = SensorRunningAverage()

    while True:
        if state['state'] == 'wandering':
            wander(sensors, particles, motors, front_ir, right_ir, state)
        elif state['state'] == 'travelling':
            travel(sensors, particles, motors, state)
        else:
            raise Exception('Unknown state {0}'.format(state['state']))
