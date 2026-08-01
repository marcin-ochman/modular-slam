#include "modular_slam/slam/slam_builder.hpp"

#include <algorithm>
#include <expected>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mslam::detail
{

struct ProducerInfo
{
    std::size_t moduleIndex = 0;
    OutputSpec output;
};

inline bool contains(const std::vector<std::size_t>& values, std::size_t value)
{
    return std::ranges::find(values, value) != values.end();
}

inline Status checkSameType(TypeToken expected, TypeToken actual, std::string_view slotName,
                            std::string_view moduleName)
{
    if(expected != actual)
    {
        return std::unexpected(Error::typeMismatch(slotName, moduleName));
    }

    return {};
}

inline Status registerAvailableSlot(std::unordered_map<SlotId, OutputSpec>& availableSlots, const OutputSpec& slot)
{
    auto it = availableSlots.find(slot.id);

    if(it == availableSlots.end())
    {
        availableSlots.emplace(slot.id, slot);
        return {};
    }

    if(it->second.type != slot.type)
    {
        return std::unexpected(Error::slotConflict(slot.name, ""));
    }

    return {};
}

inline Status registerGlobalSlotType(std::unordered_map<SlotId, OutputSpec>& globalSlots, const OutputSpec& slot,
                                     std::string_view moduleName = {})
{
    auto it = globalSlots.find(slot.id);

    if(it == globalSlots.end())
    {
        globalSlots.emplace(slot.id, slot);
        return {};
    }

    if(it->second.type != slot.type)
    {
        return std::unexpected(Error::slotConflict(slot.name, moduleName));
    }

    return {};
}

inline Status registerGlobalSlotType(std::unordered_map<SlotId, OutputSpec>& globalSlots, const InputSpec& slot,
                                     std::string_view moduleName = {})
{
    OutputSpec asOutput{.id = slot.id, .name = slot.name, .type = slot.type, .mergePolicy = MergePolicy::SingleWriter};

    return registerGlobalSlotType(globalSlots, asOutput, moduleName);
}

inline void addEdge(PipelineGraph& graph, std::size_t producer, std::size_t consumer)
{
    auto& outEdges = graph.edges[producer];

    if(!contains(outEdges, consumer))
    {
        outEdges.push_back(consumer);
    }

    auto& inEdges = graph.reverseEdges[consumer];

    if(!contains(inEdges, producer))
    {
        inEdges.push_back(producer);
    }
}

inline Expected<std::vector<std::vector<std::size_t>>> computeLevels(const PipelineGraph& graph)
{
    const std::size_t nodeCount = graph.nodes.size();

    std::vector<int> indegree(nodeCount, 0);

    for(std::size_t i = 0; i < nodeCount; ++i)
    {
        indegree[i] = static_cast<int>(graph.reverseEdges[i].size());
    }

    std::vector<std::size_t> ready;
    ready.reserve(nodeCount);

    for(std::size_t i = 0; i < nodeCount; ++i)
    {
        if(indegree[i] == 0)
        {
            ready.push_back(i);
        }
    }

    std::vector<std::vector<std::size_t>> levels;
    std::size_t visited = 0;

    while(!ready.empty())
    {
        std::vector<std::size_t> level = std::move(ready);
        ready.clear();

        std::sort(level.begin(), level.end(),
                  [&](std::size_t a, std::size_t b)
                  {
                      const auto& specA = graph.nodes[a].spec;
                      const auto& specB = graph.nodes[b].spec;

                      if(specA.execution.priority != specB.execution.priority)
                      {
                          return specA.execution.priority > specB.execution.priority;
                      }

                      return specA.name < specB.name;
                  });

        visited += level.size();

        for(std::size_t moduleIndex : level)
        {
            for(std::size_t dependent : graph.edges[moduleIndex])
            {
                --indegree[dependent];

                if(indegree[dependent] == 0)
                {
                    ready.push_back(dependent);
                }
            }
        }

        levels.push_back(std::move(level));
    }

    if(visited != nodeCount)
    {
        return std::unexpected(Error::dependencyCycle());
    }

    return levels;
}

Expected<PipelineGraph> buildPipelineGraph(const std::vector<std::unique_ptr<Module>>& modules,
                                           const std::vector<OutputSpec>& externalInputs,
                                           const std::vector<OutputSpec>& initialStateSlots)
{
    PipelineGraph graph;

    const std::size_t moduleCount = modules.size();

    graph.nodes.reserve(moduleCount);
    graph.edges.resize(moduleCount);
    graph.reverseEdges.resize(moduleCount);

    std::unordered_map<SlotId, OutputSpec> availableSlots;
    std::unordered_map<SlotId, OutputSpec> globalSlotTypes;
    std::unordered_map<SlotId, std::vector<ProducerInfo>> producers;
    std::unordered_set<std::string> moduleNames;

    for(const auto& externalInput : externalInputs)
    {
        auto status = registerAvailableSlot(availableSlots, externalInput);
        if(!status)
        {
            return std::unexpected(status.error());
        }

        status = registerGlobalSlotType(globalSlotTypes, externalInput);
        if(!status)
        {
            return std::unexpected(status.error());
        }
    }

    for(const auto& stateSlot : initialStateSlots)
    {
        auto status = registerAvailableSlot(availableSlots, stateSlot);
        if(!status)
        {
            return std::unexpected(status.error());
        }

        status = registerGlobalSlotType(globalSlotTypes, stateSlot);
        if(!status)
        {
            return std::unexpected(status.error());
        }
    }

    for(std::size_t i = 0; i < moduleCount; ++i)
    {
        if(!modules[i])
        {
            return std::unexpected(Error::invalidPipeline("Null module in pipeline"));
        }

        ModuleSpec spec = modules[i]->spec();

        if(spec.name.empty())
        {
            return std::unexpected(Error::invalidPipeline("Module has empty name"));
        }

        if(moduleNames.contains(spec.name))
        {
            return std::unexpected(Error::invalidPipeline("Duplicate module name: " + spec.name));
        }

        moduleNames.insert(spec.name);

        for(const auto& input : spec.inputs)
        {
            auto status = registerGlobalSlotType(globalSlotTypes, input, spec.name);

            if(!status)
            {
                return std::unexpected(status.error());
            }
        }

        std::unordered_set<SlotId> moduleOutputIds;

        for(const auto& output : spec.outputs)
        {
            auto status = registerGlobalSlotType(globalSlotTypes, output, spec.name);

            if(!status)
            {
                return std::unexpected(status.error());
            }

            if(moduleOutputIds.contains(output.id))
            {
                return std::unexpected(Error::invalidPipeline(
                    "Module '" + spec.name +
                    "' declares the same output slot more than once: " + std::string(output.name)));
            }

            moduleOutputIds.insert(output.id);

            if(output.mergePolicy != MergePolicy::SingleWriter)
            {
                return std::unexpected(
                    Error::invalidPipeline("Only SingleWriter outputs are supported by this graph builder"));
            }

            auto& slotProducers = producers[output.id];

            if(!slotProducers.empty())
            {
                return std::unexpected(Error::slotConflict(output.name, spec.name));
            }

            slotProducers.push_back(ProducerInfo{.moduleIndex = i, .output = output});
        }

        graph.nodes.push_back(ModuleNode{.moduleIndex = i, .spec = std::move(spec)});
    }

    for(std::size_t consumerIndex = 0; consumerIndex < moduleCount; ++consumerIndex)
    {
        const auto& consumerSpec = graph.nodes[consumerIndex].spec;

        for(const auto& input : consumerSpec.inputs)
        {
            bool inputSatisfiedByProducer = false;

            auto producerIt = producers.find(input.id);

            if(producerIt != producers.end())
            {
                for(const auto& producer : producerIt->second)
                {
                    auto status = checkSameType(input.type, producer.output.type, input.name, consumerSpec.name);

                    if(!status)
                    {
                        return std::unexpected(status.error());
                    }

                    if(producer.moduleIndex == consumerIndex)
                    {
                        continue;
                    }

                    addEdge(graph, producer.moduleIndex, consumerIndex);

                    inputSatisfiedByProducer = true;
                }
            }

            if(inputSatisfiedByProducer)
            {
                continue;
            }

            auto availableIt = availableSlots.find(input.id);

            if(availableIt != availableSlots.end())
            {
                auto status = checkSameType(input.type, availableIt->second.type, input.name, consumerSpec.name);

                if(!status)
                {
                    return std::unexpected(status.error());
                }

                continue;
            }

            if(input.policy == InputPolicy::Required)
            {
                auto error = Error::missingSlot(input.name);
                error.moduleName = consumerSpec.name;
                return std::unexpected(std::move(error));
            }
        }
    }

    auto levels = computeLevels(graph);

    if(!levels)
    {
        return std::unexpected(levels.error());
    }

    graph.levels = std::move(*levels);

    return graph;
}

} // namespace mslam::detail
