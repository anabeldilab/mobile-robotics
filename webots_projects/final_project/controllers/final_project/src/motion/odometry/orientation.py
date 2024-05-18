class Orientation:
    ORIENTATIONS = {
        "N": 0,
        "W": 90,
        "S": 180,
        "E": 270,
    }

    @staticmethod
    def get_orientation_from_cardinal(cardinal):
        """
        Get the orientation in degrees from the cardinal direction.

        Parameters:
        - cardinal: The cardinal direction ('N', 'E', 'S', 'W').

        Returns:
        - int: The orientation in degrees.
        """
        return Orientation.ORIENTATIONS.get(cardinal, None)

    @staticmethod
    def get_cardinal_from_orientation(orientation):
        """
        Get the cardinal direction from the orientation in degrees.

        Parameters:
        - orientation: The orientation in degrees.

        Returns:
        - str: The cardinal direction ('N', 'E', 'S', 'W').
        """
        for cardinal, angle in Orientation.ORIENTATIONS.items():
            if angle == orientation:
                return cardinal
        return None

    @staticmethod
    def get_target_orientation(current_cardinal, direction):
        """
        Get the target orientation after turning 90 degrees.

        Parameters:
        - current_cardinal: The current cardinal direction.
        - direction: The direction of the turn ('left' or 'right').

        Returns:
        - str: The target cardinal direction.
        """
        turn_mapping = {
            "N": {"left": "W", "right": "E"},
            "E": {"left": "N", "right": "S"},
            "S": {"left": "E", "right": "W"},
            "W": {"left": "S", "right": "N"},
        }
        return turn_mapping[current_cardinal][direction]