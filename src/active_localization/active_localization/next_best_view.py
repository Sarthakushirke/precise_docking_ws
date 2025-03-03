#Takes the frontier points and checks in the map which features are close to it within a 10 m range.

import math  # Import at the beginning of your script


def check_visible_objects_from_centroid_simple(centroid_x, centroid_y):
    

    # Define the forward direction (e.g., along positive X-axis)
    forward_angle = 0.0  # radians

    # Field of view from -75 to +75 degrees (total 150 degrees)
    fov_angle = math.radians(170)  # Convert degrees to radians

    # Sensor Camera range
    sensor_range = 30.0  # meters (camera range)

    # List of known objects
    # objects = [(3.0, 4.0),(-6.0, 5.0),(-3.0,-4.0)]

    # List of known objects extra_features.sdf
    objects = [
                (5.0, -4.0),
                (3.0, 2.0),
                (5.0,7.0),
                (-1.0,7.0),
                (-3.0,-5.0),
                (-11.0,5.0),
                (-13.0,0.0) ] #Last one is the goal location
    

    #Robot  simulation
    # objects = [
    #             (1.0, -0.8),
    #             (0.6, 0.4),
    #             (1.0,1.4),
    #             (-0.2,1.4),
    #             (-0.6,-1.0),
    #             (-2.2,1.0),
            
            
    #         ]

    #Robot 
    # objects = [
    #             (3.6, -1.2),
    #             (3.2, 0.0),
    #             (3.5, 0.8),
    #             (2.5, 0.4),
    #             (2.1, -1.5),
    #             (0.5,0.3)
            
    #         ]

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
