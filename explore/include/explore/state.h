#pragma once

namespace Xplore {
    
class State {
public:

    State() = default;
    State(const float x, const float y) 
    : x_(x), y_(y) {}
    ~State(){}

    float x() const {
        return x_;
    }

    float y() const {
        return y_;
    }

    float norm() const {
        return pow(x_*x_ + y_*y_, 0.5);
    }

    void normalize() {
        x_ = x_ / norm();
        y_ = y_ / norm(); 
    }

private:
    float x_;
    float y_;
};

State operator-(const State& lhs, const State& rhs) {
        return State(lhs.x() - rhs.x(), lhs.y() - rhs.y());
}

State operator+(const State& lhs, const State& rhs) {
        return State(lhs.x() + rhs.x(), lhs.y() + rhs.y());
}
};