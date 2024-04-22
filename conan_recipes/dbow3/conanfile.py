import os
from conan import ConanFile
from conan.tools.scm import Git
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.files import patch, get

class Dbow3Recipe(ConanFile):
    name = "dbow3"
    version = "c5ae539abddcef43ef64fa130555e2d521098369"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": True, "fPIC": True}
    exports_sources = "*.patch"

    def source(self):
        get(self, "https://github.com/rmsalinas/DBow3/archive/refs/heads/master.zip", strip_root=True)
        patch_file = os.path.join(self.export_sources_folder, "dbow3.patch")
        patch(self, patch_file=patch_file)

    def requirements(self):
        self.requires("opencv/4.8.1")

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
        variables={
            "BUILD_UTILS":"OFF"
        }

        cmake.configure(variables=variables)
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["DBoW3"]
