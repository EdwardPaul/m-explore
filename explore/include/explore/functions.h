#pragma once
#include "math.h"
#include "state.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

float distance(const Xplore::State& first, const Xplore::State& second) {
    return pow((first.x() - second.x()) * (first.x() - second.x()) + 
               (first.y() - second.y()) * (first.y() - second.y()), 0.5);
}

bool isInsideRadius(const Xplore::State& first, const Xplore::State& second, 
                     const float radius) {
    return distance(first, second) < radius;
}

bool isObstacle(const Xplore::State& state, const nav_msgs::OccupancyGrid& grid) {
    int x = (int)round(state.x());
    int y = (int)round(state.y());
    if (x < grid.info.width && y < grid.info.height) {
        return grid.data[x + y * grid.info.width] == 100;
    }
    return false;
}

bool isFree(const Xplore::State& state, const nav_msgs::OccupancyGrid& grid) {
    int x = (int)round(state.x());
    int y = (int)round(state.y());
    if (x < grid.info.width && y < grid.info.height) {
        return grid.data[x + y * grid.info.width] == 0;
    }
    return false;
}

bool isFree(const Xplore::State& state, const costmap_2d::Costmap2D* costmap) {
    int x = (int)round(state.x());
    int y = (int)round(state.y());
    int width = costmap->getSizeInCellsX();
    int height = costmap->getSizeInCellsY();
    if (x < width && y < height) {
        auto cost = costmap->getCost(x, y);
        return cost == costmap_2d::FREE_SPACE;
    }
    return false;
}

bool isNearUnknown(const Xplore::State& state, const nav_msgs::OccupancyGrid& grid) {
    int x = (int)round(state.x());
    int y = (int)round(state.y());
    for (int i = x - 3; i < x + 3; ++i) {
        for (int j = y - 3; j < y + 3; ++j) {
            if (i < grid.info.width && j < grid.info.height) {
                if (grid.data[i + j * grid.info.width] == -1) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool isNearUnknown(const Xplore::State& state, const costmap_2d::Costmap2D* costmap) {
    int x = (int)round(state.x());
    int y = (int)round(state.y());
    int width = costmap->getSizeInCellsX();
    int height = costmap->getSizeInCellsY();
    for (int i = x - 3; i < x + 3; ++i) {
        for (int j = y - 3; j < y + 3; ++j) {
            if (i < width && j < height) {
                auto cost = costmap->getCost(x, y);
                if (cost == costmap_2d::NO_INFORMATION) {
                    return true;
                }
            }
        }
    }
    return false;
}