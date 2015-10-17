"""
Port mapping

0: left IR sensor
1: right IR sensor
"""


def ir_input_to_distance(voltage):
    value = 4800. / (voltage - 20.)
    return value if value > 0 else None


class Sensors:
    def __init__(self, io):
        self.analogue = 10 * [None]
        self.io = io

    def get_analogue(self):
        self.analogue = self.io.getSensors()

    def get_ir_left_raw(self):
        """
        :rtype : float
        """
        self.get_analogue()
        return self.analogue[0]

    def get_ir_left(self):
        return ir_input_to_distance(self.get_ir_left_raw())

    def get_ir_right_raw(self):
        """
        :rtype : float
        """
        self.get_analogue()
        return self.analogue[1]

    def get_ir_right(self):
        return ir_input_to_distance(self.get_ir_right_raw())


class SensorRunningAverage:
    # TODO: values should expire if new are not added
    def __init__(self, number_of_values=5, init_value=20):
        self.number_of_values = number_of_values
        self.values = number_of_values * [init_value]

    def add_value(self, new_value):
        """Dismisses oldest, adds new one at the end
        """
        self.values = self.values[1:] + [new_value]

    def get_avg(self):
        """Arithmetic average
        """
        return sum(self.values) / self.number_of_values
