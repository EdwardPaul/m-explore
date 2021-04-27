#pragma once
#include "state.h"
#include "tree.h"
#include <queue>

namespace Xplore {

class Frontier {
public:
    Frontier() = default;
    Frontier(const State& state, const std::vector<Node*>& path, const float utility)
    : state_(state), path_(path), utility_(utility) {}

    ~Frontier() {
        // TO DO: 
        // delete queue
    }

    float utility() const {
        return utility_;
    }

    State state() const {
        return state_;
    }

    std::vector<Node*> path() const {
        return path_;
    }

    void setUtility(float utility) {
        utility_ = utility;
    } 

    void setPath(const std::vector<Node*>& path) {
        path_ = path;
    }

    void setState(const State& state) {
        state_ = state;
    }

private:
    State state_;
    std::vector<Node*> path_;
    float utility_;
};

bool operator<(const Frontier& first, const Frontier& second) {
    return first.utility() > second.utility();
}

}