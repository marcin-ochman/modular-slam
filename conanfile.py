#!/usr/bin/env python3

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps

class ModularSlamRecipe(ConanFile):
    name = "modular_slam"
    version = "0.0.1"

    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": True, "fPIC": True}

    def requirements(self):
        self.tool_requires("cmake/3.30.0")
        self.requires("catch2/3.11.0")
       
    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure(variables={
            "CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
        })
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
