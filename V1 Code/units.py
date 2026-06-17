class Speed:
    @staticmethod
    def in_mm_per_second(value):
        return value

    @staticmethod
    def in_cm_per_second(value):
        return value / 10

    @staticmethod
    def in_m_per_second(value):
        return value / 1000

class Direction:
    FORWARDS = "FORWARDS"
    BACKWARDS = "BACKWARDS"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    CLOCKWISE = "CLOCKWISE"
    COUNTERCLOCKWISE = "COUNTERCLOCKWISE"

    @staticmethod
    def angle(degrees):
        return degrees