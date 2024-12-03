#Takes the frontier points and checks in the map which features are close to it within a 10 m range.

import math  # Import at the beginning of your script


def check_visible_objects_from_centroid_simple(centroid_x, centroid_y):
    import math

    # Define the forward direction (e.g., along positive X-axis)
    forward_angle = 0.0  # radians

    # Field of view from -75 to +75 degrees (total 150 degrees)
    fov_angle = math.radians(150)  # Convert degrees to radians

    # Sensor range
    sensor_range = 10.0  # meters

    # List of known objects
    objects = [(4, -1), (-2, -4), (-5, 5)]

    # List to store visible objects
    visible_objects = []

    for obj in objects:
        obj_x, obj_y = obj

        # Compute distance to the object
        dx = obj_x - centroid_x
        dy = obj_y - centroid_y
        distance = math.hypot(dx, dy)

        if distance > sensor_range:
            continue  # Object is out of range

        # Compute angle to the object relative to forward direction
        angle_to_object = math.atan2(dy, dx)
        angle_diff = (angle_to_object - forward_angle + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Check if angle is within ±75 degrees (half of FOV angle)
        if abs(angle_diff) <= fov_angle / 2:
            visible_objects.append(obj)

    return visible_objects
