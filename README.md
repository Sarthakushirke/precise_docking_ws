[Graduation_project.pdf](https://github.com/user-attachments/files/19734537/Graduation_project.pdf)
# Utility-Based Frontier Exploration for  Active Localization!

# Abstract
Localization is a fundamental challenge in mobile robotics, especially when the robot’s initial pose on a given map is unknown. Traditional methods like particle filters often require a dense representation of the belief space, which can be computationally expensive and may fail in environments with sparse or ambiguous features.

To address these limitations, this thesis explores active localization, leveraging observed local features through camera detection to efficiently generate and refine hypotheses about the robot’s global pose. In this approach, each hypothesis corresponds to a potential robot pose and is associated with a point-cloud scan from which frontiers are extracted. These frontiers minimize the robot’s actions and help extract the information gained while considering the shortest path to the goal. A utility function evaluates the information gain and the shortest path to the goal from each frontier, helping the robot determine the optimal action to take. By visiting frontiers, the robot not only gathers new information to validate and prune hypotheses, but also navigates toward areas of dense features, improving localization
confidence. 

This active localization strategy ensures efficient exploration while progressively converging on a precise pose without forgetting to reach a target location. Once localization is sufficiently accurate, the robot transitions seamlessly to goal-directed planning. This thesis demonstrates how integrating active localization with decision-making improves localization and helps the robot reach the targets.

We demonstrate the effectiveness of the framework by validating it with a fully actuated robot equipped with a 360° laser range scanner and RGB camera in the simulation environment using ROS and Gazebo, and extend it to initial hardware experiments using a ROSbot 2 pro robot.
