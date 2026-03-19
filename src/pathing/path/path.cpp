#include "pathing/path/path.hpp"
#include <cmath>

using namespace pathing::geometry;

namespace pathing::path {

Path::Path() {}

void Path::addSegment(PathSegment segment)
{
    segments.push_back(std::move(segment));
}

size_t Path::size() const
{
    return segments.size();
}

const PathSegment& Path::getSegment(size_t index) const
{
    return segments[index];
}

PathPoint Path::getClosestPoint(const Pose& pose) const
{
    double bestDist = 1e9;
    PathPoint bestPoint;

    const int samples = 20;

    for (const auto& segment : segments) {
        for (int i = 0; i <= samples; i++) {
            double t = static_cast<double>(i) / samples;

            Vector2 pos = segment.getPoint(t);

            double dx = pos.x - pose.x;
            double dy = pos.y - pose.y;

            double dist = dx * dx + dy * dy;

            if (dist < bestDist) {
                bestDist = dist;

                bestPoint = PathPoint(pos, 0.0, 0.0);
            }
        }
    }

    return bestPoint;
}

PathPoint Path::getLookaheadPoint(
    const Pose& pose,
    double lookaheadDistance
) const
{
    const int samples = 50;

    for (const auto& segment : segments) {
        for (int i = 0; i <= samples; i++) {
            double t = static_cast<double>(i) / samples;

            Vector2 pos = segment.getPoint(t);

            double dx = pos.x - pose.x;
            double dy = pos.y - pose.y;

            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= lookaheadDistance) {
                return PathPoint(pos, 0.0, 0.0);
            }
        }
    }

    if (!segments.empty()) {
        Vector2 pos = segments.back().getPoint(1.0);
        return PathPoint(pos, 0.0, 0.0);
    }

    return PathPoint();
}

}