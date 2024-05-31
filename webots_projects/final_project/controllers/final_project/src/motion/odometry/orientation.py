class Orientation:
    ORIENTATIONS = {
        "N": 0,
        "NW": 45,
        "W": 90,
        "SW": 135,
        "S": 180,
        "SE": 225,
        "E": 270,
        "NE": 315,
    }

    @staticmethod
    def get_orientation_from_cardinal(cardinal):
        """
        Get the orientation in degrees from the cardinal direction.

        Parameters:
        - cardinal: The cardinal direction ('N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE').

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
        - str: The cardinal direction ('N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE').
        """
        for cardinal, angle in Orientation.ORIENTATIONS.items():
            if angle == orientation:
                return cardinal
        return None

    @staticmethod
    def get_target_orientation(current_cardinal, direction = "left", angle = 90):
        """
        Get the target cardinal direction after a turn.

        Parameters:
        - current_cardinal: The current cardinal direction.
        - direction: The direction of the turn ('left' or 'right').

        Returns:
        - str: The target cardinal direction.
        """
        current_orientation = Orientation.get_orientation_from_cardinal(current_cardinal)
        print(f"current_orientation: {current_orientation}, current_cardinal: {current_cardinal}")
        if direction == "left":
            target_orientation = current_orientation + angle
        else:
            target_orientation = current_orientation - angle

        target_orientation = target_orientation % 360

        print(f"target_orientation: {target_orientation}, target_cardinal: {Orientation.get_cardinal_from_orientation(target_orientation)}")
        
        return Orientation.get_cardinal_from_orientation(target_orientation)
    
