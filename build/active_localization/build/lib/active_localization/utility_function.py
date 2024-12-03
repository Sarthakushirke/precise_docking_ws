#This will give the best frontier the bots needs to visit on the path to the goal and the next best view 
#Example:- Suppose we have 3 frontier. We will check from which frontier the path goes to the goal and which frontier gives the 
#next best view that would be selected.

            # visible_objects_per_centroid = {}
            # information_gain_per_centroid = {}
            # utility_per_centroid = {}

        #     visible_objects = check_visible_objects_from_centroid_simple(centroid, resolution, originX, originY)

        #     visible_objects_per_centroid[group_id] = visible_objects

        #     # Compute Information Gain
        #     information_gain = len(visible_objects)
        #     information_gain_per_centroid[group_id] = information_gain


        #     # Compute Utility
        #     weight_info_gain = 1.0
        #     weight_path_length = 1.0
        #     # utility_value = (weight_info_gain * information_gain) - (weight_path_length * path_length)
        #     # utility_per_centroid[group_id] = utility_value


        # # Find the best centroid based on utility
        # if utility_per_centroid:
        #     best_centroid_id = max(utility_per_centroid, key=utility_per_centroid.get)
        #     best_centroid = centroids[best_centroid_id]
        #     self.get_logger().info(f"Best centroid: {best_centroid} with utility: {utility_per_centroid[best_centroid_id]}")

        #     # Convert centroid grid indices to world coordinates
        #     best_centroid_world = (
        #         originX + best_centroid[1] * resolution + resolution / 2,
        #         originY + best_centroid[0] * resolution + resolution / 2
        #     )

        #     # You can now publish a goal or plan to move to `best_centroid_world`
        # else:
        #     self.get_logger().warn("No valid centroids found.")


        # For debugging or logging
        # for group_id, visible_objs in visible_objects_per_centroid.items():
        #     self.get_logger().info(f"Centroid {group_id} can see objects: {visible_objs}")




