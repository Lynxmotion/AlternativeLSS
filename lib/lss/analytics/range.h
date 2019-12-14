//
// Created by guru on 12/13/19.
//

#pragma once

#include <limits>

class Range {
public:
    int current;
    int target;
    int min;
    int max;

    inline Range() : current(0), target(0), min(std::numeric_limits<int>::min()), max(std::numeric_limits<int>::max()) {}

    int delta() const { return target - current; }

    inline void limits(int _min, int _max) { min = _min; max = _max; }
};


