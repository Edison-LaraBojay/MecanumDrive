#include "pathing/path/path_chain.hpp"

namespace pathing::path {

void PathChain::add(const Path& path) {
    paths.push_back(path);
}

Path& PathChain::get(size_t index) {
    return paths.at(index);
}

size_t PathChain::size() const {
    return paths.size();
}

}