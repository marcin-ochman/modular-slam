from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps

class ModularSlamRecipe(ConanFile):
    name = "modular_slam"
    version = "0.0.1"
    # package_type = "library"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": True, "fPIC": True}

    # Sources are located in the same place as this recipe, copy them to the recipe
    # exports_sources = "CMakeLists.txt", "src/*", "include/*", "config/*", "tests/*", "benchmarks/*", "apps/*"

    def requirements(self):
        self.tool_requires("cmake/3.29.7")
        self.requires("boost/1.84.0")
        self.requires("catch2/3.5.4")
        self.requires("nlohmann_json/3.11.3")
        self.requires("trompeloeil/47")
        self.requires("dbow3/c5ae539abddcef43ef64fa130555e2d521098369")
        self.requires("eigen/3.4.0")
        self.requires("ceres-solver/2.2.0")
        self.requires("spdlog/1.13.0")
        self.requires("glm/cci.20230113")
        self.requires("opencv/4.8.1", force=True)
        self.requires("qt/6.6.3", force=True)
        # self.requires("qt/6.7.3", force=True)
        # self.requires("libpng/1.6.43", override=True)
        # self.requires("ffmpeg/6.1", override=True)
        # self.requires("xkbcommon/1.6.0", override=True)
        # self.requires("librealsense/2.53.1")
        # self.requires("libxpm/3.5.17", override=True)

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

        self.options["opencv/*"].with_qt = True
        self.options["opencv/*"].with_wayland = False

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
