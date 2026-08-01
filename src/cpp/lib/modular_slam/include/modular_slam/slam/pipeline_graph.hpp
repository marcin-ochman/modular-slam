#ifndef MODULAR_SLAM_PIPELINE_GRAPH_HPP
#define MODULAR_SLAM_PIPELINE_GRAPH_HPP

#include "modular_slam/slam/module_spec.hpp"

namespace mslam
{

struct ModuleNode
{
    std::size_t moduleIndex = 0;
    ModuleSpec spec;
};

struct PipelineGraph
{
    std::vector<ModuleNode> nodes;

    // edges[a] = modules that depend on a
    std::vector<std::vector<std::size_t>> edges;

    // reverseEdges[b] = modules that b depends on
    std::vector<std::vector<std::size_t>> reverseEdges;

    // topological levels; modules in the same level may be parallelized
    std::vector<std::vector<std::size_t>> levels;
};

} // namespace mslam

#endif // MODULAR_SLAM_PIPELINE_GRAPH_HPP
