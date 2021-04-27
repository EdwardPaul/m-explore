#pragma once
#include <iostream>
#include <random>
#include <vector>
#include <cstdlib>
#include <list>
#include <set>

#include "state.h"
#include "frontier.h"
#include "functions.h"

namespace Xplore {

class Node {
public:
    Node () {}
    Node(const State& state, Node* parent = nullptr) 
        : parent_(parent), state_(state) {
        if (parent_) {
            parent_->addChild(this);
        }
    }

    State state() const {
        return state_;
    }

    Node* parent() const {
        return parent_;
    }

    void addChild(Node* child) {
        children_.push_back(child);
    }

private:
    State state_;
    std::vector<Node*> children_;
    Node* parent_;
};

class Tree {
public:
    Tree(const State& start_state) {
        root_ = new Node(start_state);
        last_node_ = root_;
        nodes_.push_back(root_);
        start_state_ = start_state;
    }
    ~Tree() {}

    void addNodeToPath(Node* node) {
        path_.push_back(node);
    }

    // void get_frontiers() {
    //     for (int i = 0; i < max_iterations_; i++) {
    //         Node* curr_node = getRandomNode();
    //         if (curr_node) {
    //             if (curr_node->state().isFree(grid_)) {
    //                 if (curr_node->state().isNearUnknown(grid_)) {
    //                     frontiers.push_back(curr_node);
    //                 }
    //             }
    //             // Node* curr_nearest = nearest(curr_node->state());
    //             // if (curr_node->distance(curr_nearest) > step_size_) {
    //                 // State steer_state = steer(curr_node, curr_nearest);
    //                 // ROS_WARN("STEER TO CURRENT (%d, %d) is (%d, %d)", curr_node->state().x(), curr_node->state().y(), steer_state.x(), steer_state.y());
    //                 // if (steer_state.isFree(grid_)) {
    //                     // Node* new_tree_node = new Node(steer_state, curr_nearest);
    //                     // addNode(curr_nearest, new_tree_node);
    //                 // }
    //             // } else {
    //                 // if (curr_node->state().isFree(grid_)) {
    //                 //     Node* new_tree_node = new Node(curr_node->state(), curr_nearest);
    //                 //     addNode(curr_nearest, new_tree_node);
    //                 // }
    //             // }
    //         }
    //         // if (reachedGoal()) {
    //         //     std::cout << "Goal reached" << std::endl;
    //         //     cout << "Number of nodes: " << nodes.size() << endl;
    //         //     return;
    //         // } 
    //     }
    //     cout << frontiers.size() << endl;
    // }

    Node* getNearestNode(const Node* node) {
        Node* nearest = nullptr;
        float min_dist = 1e9;
        for (auto it = begin(nodes_); it != end(nodes_); it++) {
            float dist = distance(node->state(), (*it)->state());
            if (dist < min_dist) {
                min_dist = dist;
                nearest = *it;
            }
        }
        return nearest;
    }

    Node* getSteerNode(const Node* curr, const Node* nearest, float step_size) {
        State to = curr->state();
        State from = nearest->state();
        State intermediate = from - to;
        intermediate.normalize();
        State result_state{from.x() + intermediate.x() * step_size, from.y() + intermediate.y() * step_size};
        Node* result = new Node(result_state);
        return result;
    }

    void addNode(Node* nearest, Node* new_node) {
        nearest->addChild(new_node);
        nodes_.push_back(new_node);
        // ROS_WARN("NEW NODE (x,y) : (%d, %d)", new_node->state().x(), new_node->state().y());
        last_node_ = new_node;
    }

    std::vector<Node*> path() const {
        return path_;
    }

    void deleteNodes() {
        auto it = begin(nodes_);
        while (it != end(nodes_)) {
            delete *it;
            it++;
        }
        // root_ = new Node(start_state_);
        // last_node_ = root_;
        // nodes_.push_back(root_);
        // nodes_.resize(0); 
        // auto it = begin(nodes_);
        // while (it != end(nodes)) {
        //     delete *it;
        //     it++;
        // } 
        
        // path_.resize(0);
    }
    std::vector<Node*> nodes_;
private:
    State start_state_;
    Node* root_;
    Node* last_node_;
    
    std::vector<Node*> path_;
};

}
