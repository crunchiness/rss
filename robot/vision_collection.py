from robot.body.sensors import SensorRunningAverage


def vision_collection(sensors, motors):

    front_ir = SensorRunningAverage()
    right_ir = SensorRunningAverage()

    front_avg = front_ir.get_avg()
    right_avg = right_ir.get_avg()

    while front_avg > 15 and right_avg > 15:

        # Move forwards 10 cm
        motors.go_forward(10)

        front_ir_reading = sensors.get_ir_front()
        right_ir_reading = sensors.get_ir_right()
        front_avg = front_ir.get_avg()
        right_avg = right_ir.get_avg()

        # Update running average
        front_ir.add_value(front_ir_reading)
        right_ir.add_value(right_ir_reading)

    if front_avg <= 15:
        motors.turn_by(30)
    elif right_avg <= 15:
        motors.turn_by(-30)
