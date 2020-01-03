#ifndef PLUGIN_LOADER_HPP_
#define PLUGIN_LOADER_HPP_
#include <boost/dll/import.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

namespace mslam
{

namespace bf = boost::filesystem;
namespace dll = boost::dll;

template <typename BlockType>
using BlockFactoryCreator = BlockType*();

template <typename BlockType>
using BlockFactoryCreatorBoostFunction = boost::function<BlockFactoryCreator<BlockType>>;

template <typename BlockType>
BlockFactoryCreatorBoostFunction<BlockType> loadFactoryMethod(bf::path path_to_shared_library,
                                                              const std::string& factoryFunctionName)
{
    return dll::import_alias<BlockFactoryCreator<BlockType>>(path_to_shared_library, factoryFunctionName,
                                                             dll::load_mode::append_decorations);
}

} // namespace mslam

#endif /* PLUGIN_LOADER_HPP_ */
