TARGET_RESOURCE = 'mario'


def milestone2(sensors, particles, motors, left_ir, right_ir, sonar, state, vision):
    # we know where we are, do we know orientation?
    # while no resource detected, keep some empty space in front (need more details)
    # look around, ideally don't look at the wall
    #
    # ONE: finding resource
    # if we see a box in a distance
    #     go towards the box (20 cm at a time)
    #     if we can't confirm
    #          go backwards 10 cm
    #          go to ONE
    #     if we can confirm, still in a distance
    #          try to orientate according to vision
    #          go to ONE
    #     if we can confirm, and close enough (need
    #          go to TWO
    #
    # if no box to be seen, do a scan
    #   SCAN:
    #     turn left 60 degrees
    #     while not 60 degrees right from initial OR not target_resource found
    #         look for resource (and save sensor measurements)
    #         if target_resource found
    #             go to TWO
    #         turn 15 degrees right
    #     if nothing
    #         drive forwards, or turn right if obstacle (need more details)
    #     if something
    #         go to THREE
    #
    # TWO: pick up the box if the one
    #     wiggle around until box in the middle, reject if wrong (need more details)
    #     go forwards like 20 cm OR until we detect the box inside with that clicky sensor
    #     if no clicky sensor,
    #         go back 20 cm and do SCAN
    #
    # THREE: decide where to go based on vision scan (assuming turning by angle is accurate enough)
    #     we've made a scan and there's some distant objects detected
    #     select the biggest one, which is not in the wall
    #     turn back towards it
    #     if it's still detectable
    #         go to ONE
    #     if not detectable
    #         go forwards 5 cm
    #         if detectable
    #         if not detectable
    #             go back 5 cm, select another target if we had multiple, if not
    #             go to ONE
    #
    #
    pass
