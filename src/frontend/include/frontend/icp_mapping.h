#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include "frontend/matplotlibcpp.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "pointmatcher/PointMatcher.h"

namespace plt = matplotlibcpp;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class MapBuilder {
public:
    MapBuilder() {}

    // Function to update the map with transformed cloud (directly add to global map)
    void updateGlobalMap(const DP &transformed_cloud) {
        for (int i = 0; i < transformed_cloud.features.cols(); ++i) {
            float x = transformed_cloud.features(0, i);
            float y = transformed_cloud.features(1, i);
            global_map.push_back(std::make_pair(x, y));
        }
    }

    // Function to plot the global map using matplotlibcpp
    void plotGlobalMap() {
        if (!global_map.empty()) {
            std::vector<float> x, y;

            // Extract the points from global_map
            for (auto &point : global_map) {
                x.push_back(point.first);
                y.push_back(point.second);
            }

            // Plot the map
            plt::scatter(x, y, 1.0, {{"color", "black"}});
            plt::title("2D Global Map");
            plt::xlabel("X");
            plt::ylabel("Y");
            plt::grid(true);
            plt::pause(0.01);  // Pause to allow live updates
        }
    }

private:
    std::vector<std::pair<float, float>> global_map;  // Global map storing (x, y) points
};

#endif
