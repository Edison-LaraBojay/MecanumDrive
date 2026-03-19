#pragma once

#include <vector>
#include "pathing/path/path.hpp"

namespace pathing::path {

class PathChain {
private:
    std::vector<Path> paths;

public:
    PathChain() = default;

    void add(const Path& path);

    Path& get(size_t index);

    size_t size() const;
};

}