class Orientation:
    ORIENTATIONS = {
        "N": [[0, -1], [1, 0]],
        "E": [[1, 0], [0, 1]],
        "S": [[0, 1], [-1, 0]],
        "W": [[-1, 0], [0, -1]],
    }

    @staticmethod
    def get_orientation(khepera_node):
        """
        Get the orientation of the khepera_node based on its orientation matrix.

        Parameters:
        - khepera_node: The node object representing the Khepera robot.

        Returns:
        - str: The cardinal orientation of the node ('N', 'E', 'S', 'W').
        """
        orientation_matrix = khepera_node.getOrientation()
        simplified_orientation = [
            [round(orientation_matrix[0]), round(orientation_matrix[1])],
            [round(orientation_matrix[3]), round(orientation_matrix[4])],
        ]
        for cardinal, matrix in Orientation.ORIENTATIONS.items():
            if matrix == simplified_orientation:
                return cardinal
        raise ValueError("Current orientation does not match any known cardinal direction.")

    @staticmethod
    def get_target_orientation(khepera_node, direction="left"):
        """
        Get the target orientation of the khepera_node based on a given turn direction (left or right).

        Parameters:
        - khepera_node: The node object representing the Khepera robot.
        - direction (str): The turn direction ('left' or 'right'). Defaults to 'left'.

        Returns:
        - list of list of int: The target orientation matrix for the given turn direction.
        """
        current_cardinal = Orientation.get_orientation(khepera_node)
        turn_mapping = {
            "N": {"left": Orientation.ORIENTATIONS["W"], "right": Orientation.ORIENTATIONS["E"]},
            "E": {"left": Orientation.ORIENTATIONS["N"], "right": Orientation.ORIENTATIONS["S"]},
            "S": {"left": Orientation.ORIENTATIONS["E"], "right": Orientation.ORIENTATIONS["W"]},
            "W": {"left": Orientation.ORIENTATIONS["S"], "right": Orientation.ORIENTATIONS["N"]},
        }
        return turn_mapping[current_cardinal][direction]