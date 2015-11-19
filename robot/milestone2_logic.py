from robot.utils import log

TARGET_RESOURCE = 'mario'


def S0_check_box():
    # S0_check_box: check if we see a box
    #     if we see a box:
    #         go to S1_approach_box
    #     else
    #         go to S2_scan
    pass


def S1_approach_box():
    # S1_approach_box: approaching the box
    # if path is clear
    #     go towards the box (20 cm at a time)
    # else
    #     go to S5_just_go
    # if we can't confirm
    #      go backwards 10 cm
    #      go to S0_check_box
    # if we can confirm, still in a distance
    #      try to orientate according to vision
    #      go to S1_approach_box
    # if we can confirm, and close enough (need more details)
    #      go to S3_determine_and_pick
    pass


def S2_scan():
    # S2_scan:
    # turn left 60 degrees
    # while not 60 degrees right from initial OR not target_resource found
    #    look for resource (and save sensor measurements)
    #    if target_resource found
    #        go to S3_determine_and_pick
    #    turn 15 degrees right
    # if nothing
    #     S5_just_go
    # if something
    #    go to S4_decide_route
    pass


def S3_determine_and_pick():
    #  S3_determine_and_pick: pick up the box if the one
    # wiggle around until box in the middle, reject if wrong (need more details)
    # go forwards like 20 cm OR until we detect the box inside with that clicky sensor
    # if no clicky sensor,
    #     go back 20 cm
    #     go to S2_scan
    pass


def S4_decide_route():
    #  S4_decide_route: decide where to go based on vision scan (assuming turning by angle is accurate enough)
    # we've made a scan and there's some distant objects detected
    # select the biggest one, which is not in the wall
    # turn back towards it
    # if it's still detectable
    #     go to S1_approach_box
    # if not detectable
    #     go forwards 5 cm
    #     if detectable
    #     if not detectable
    #         go back 5 cm, select another target if we had multiple, if not
    #         go to S2_scan
    pass


def S5_just_go():
    # S5_just_go: just go
    # if path is clear
    #     go forwards 20cm
    # if not too close to wall
    #     turn right 40 degrees?
    # else
    #     go back 10cm? and try turning right again
    # go to S2_scan
    pass


def milestone2(sensors, particles, motors, left_ir, right_ir, sonar, state, vision):
    state = {
        'mode': 'S0_check_box'
    }
    while True:
        if state['mode'] == 'S0_check_box':
            log('S0_check_box')
            S0_check_box()
        elif state['mode'] == 'S1_approach_box':
            log('S1_approach_box')
            S1_approach_box()
        elif state['mode'] == 'S2_scan':
            log('S2_scan')
            S2_scan()
        elif state['mode'] == 'S3_determine_and_pick':
            log('S3_determine_and_pick')
            S3_determine_and_pick()
        elif state['mode'] == 'S4_decide_route':
            log('S4_decide_route')
            S4_decide_route()
        elif state['mode'] == 'S5_just_go':
            log('S5_just_go')
            S5_just_go()
        else:
            raise Exception('Unknown state {0}'.format(state['mode']))
