#pragma once

#include <vector>
#include "pathing/path/path_segment.hpp"
#include "pathing/path/path_point.hpp"
#include "pathing/geometry/pose.hpp"

namespace pathing::path {

class Path {

private:
    std::vector<PathSegment> segments;

public:
    Path();

    void addSegment(PathSegment segment);

    size_t size() const;

    const PathSegment& getSegment(size_t index) const;
    
    PathPoint getClosestPoint(const pathing::geometry::Pose& pose) const;

    PathPoint getLookaheadPoint(
        const pathing::geometry::Pose& pose,
        double lookaheadDistance
    ) const;
};

}