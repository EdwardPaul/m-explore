#pragma once
#include <iostream>
#include <random>
#include <vector>
#include <cstdlib>
#include <list>
#include <set>
#include <unistd.h>

#include "state.h"
#include "frontier.h"
#include "functions.h"
#include "tree.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "visualization_msgs/Marker.h"
#include <costmap_2d/costmap_2d.h>

namespace Xplore {

class RRT {
public:
    RRT(const State& curr_state, const int num_iterations, const float step_size)
    : curr_state_(curr_state), num_iterations_(num_iterations), step_size_(step_size) {
        tree_ = new Tree(curr_state);
        
        line_viz_pub = nh.advertise<visualization_msgs::Marker>("vertices", 10);
    }

    RRT(const State& curr_state, const int num_iterations, const float step_size, 
        costmap_2d::Costmap2D* costmap)
    : curr_state_(curr_state), num_iterations_(num_iterations), step_size_(step_size),
      costmap_(costmap) {
        tree_ = new Tree(curr_state);
        
        line_viz_pub = nh.advertise<visualization_msgs::Marker>("vertices", 10);
    }

    ~RRT() {
        // TO DO:
        // Delete Tree 
        frontiers_.clear();
    }

    void setGrid(const nav_msgs::OccupancyGrid grid) {
        grid_ = grid;
    }

    void setState(const State& state) {
        curr_state_ = state;
    }

    void resetTree() {
        delete tree_;
        frontiers_.clear();
        // tree_->deleteNodes();
    }

    void getFrontiers() {
        resetTree();
        tree_ = new Tree(curr_state_);
        for (int i = 0; i < num_iterations_; i++) {
            Node* curr_node = getRandomNode();
            if (curr_node) {
                Node* nearest = tree_->getNearestNode(curr_node);
                if (isInsideRadius(curr_node->state(), nearest->state(), step_size_)) {
                    if (isFree(curr_node->state(), costmap_)) {
                        tree_->addNode(nearest, curr_node);
                        // visualizeNode(nearest, curr_node);
                        if (isNearUnknown(curr_node->state(), costmap_)) {
                            Frontier new_frontier;
                            new_frontier.setState(curr_node->state());
                            computeUtility(new_frontier);
                            buildPath(new_frontier, curr_node);
                            frontiers_.insert(new_frontier);
                        }
                    }
                } else {
                    Node* steer_node = tree_->getSteerNode(curr_node, nearest, step_size_);
                    if (isFree(steer_node->state(), costmap_)) {
                        tree_->addNode(nearest, steer_node);
                        // visualizeNode(nearest, steer_node);
                        if (isNearUnknown(steer_node->state(), costmap_)) {
                            Frontier new_frontier;
                            new_frontier.setState(steer_node->state());
                            computeUtility(new_frontier);
                            buildPath(new_frontier, steer_node);
                            frontiers_.insert(new_frontier);
                        }
                    }
                }
            }
        }
        ROS_WARN("Number of frontiers: %d", frontiers_.size());
    }

    void visualizeNode(const Node* nearest, const Node* new_node) {
        line.points.clear();
        line.header.frame_id = grid_.header.frame_id;
        line.header.stamp = ros::Time::now();
        line.header.seq = 0;
        line.ns = "markers";
        line.id =1;
        line.type=line.LINE_LIST;
        line.action = line.ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x =  0.5;
        line.scale.y= 0.5;
        line.color.r =9.0/255.0;
        line.color.g= 91.0/255.0;
        line.color.b =236.0/255.0;
        line.color.a = 1.0;
        line.lifetime = ros::Duration();
        geometry_msgs::Point p_nearest, p_new;
        State state_nearest, state_new;
        state_nearest = nearest->state();
        state_new = new_node->state();
        p_nearest.x = state_nearest.x();
        p_nearest.y = state_nearest.y();
        p_nearest.z = 0;
        p_new.x = state_new.x();
        p_new.y = state_new.y();
        p_new.z = 0;
        line.points.push_back(p_nearest);
        line.points.push_back(p_new);
        // sleep(2);


        line_viz_pub.publish(line);

    }

    void computeUtility(Frontier& frontier) {
        // TO DO: 
        // Different utility functions
        float utility = -distance(curr_state_, frontier.state());
        frontier.setUtility(utility);
    }

    void buildPath(Frontier& frontier, Node* curr_node) {
        Node *last = curr_node;
        while (last != NULL) {
            tree_->addNodeToPath(last);
            last = last->parent();
        }
        frontier.setPath(tree_->path());
    }

    Node* getRandomNode() {
        int width = costmap_->getSizeInCellsX();
        int height = costmap_->getSizeInCellsY();
        State new_state(rand() % (width + 1), rand() % (height + 1));
        Node* ret = new Node(new_state);
        return ret;
    }

    size_t frontiersSize() {
        return frontiers_.size();
    }

    void chooseTarget() {
        if (!frontiers_.empty()) {
            target_ = *begin(frontiers_);
        }
    }

    std::vector<Node*> getPathToTarget() const {
        if (target_.path().size()) {
            return target_.path();
        }
        return {};
    }

    std::set<Frontier> frontiers() const {
        return frontiers_;
    }

    Frontier target() const {
        return target_;
    }

private:
    State curr_state_;
    Tree* tree_;
    int num_iterations_;
    float step_size_;
    std::set<Frontier> frontiers_;
    Frontier target_;

    nav_msgs::OccupancyGrid grid_;
    costmap_2d::Costmap2D* costmap_;
    visualization_msgs::Marker line;

    ros::Publisher line_viz_pub;
    ros::NodeHandle nh;
};
}