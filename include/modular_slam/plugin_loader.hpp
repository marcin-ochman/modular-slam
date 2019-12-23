#ifndef PLUGIN_LOADER_HPP_
#define PLUGIN_LOADER_HPP_


namespace mslam {

  template <typename BlockType>
  using BlockFactoryMethodType = BlockType* ();

  template <typename BlockType>
  BlockFactoryMethodType loadFactoryMethod(const std::string& factoryFunctionName)
  {
    return dll::import<BlockFactoryMethodType<BlockType>>(path_to_shared_library, std::string(factoryFunctionName));
  }

}

#endif /* PLUGIN_LOADER_HPP_ */
