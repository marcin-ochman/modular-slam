#define CATCH_CONFIG_MAIN

#include <boost/dll/alias.hpp>
#include <catch2/catch.hpp>

std::unique_ptr<int> intFactory()
{
    return nullptr;
}

BOOST_DLL_ALIAS(intFactory, dummyPluginFactory)
