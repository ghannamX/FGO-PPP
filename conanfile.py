from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, cmake_layout


class FgoPppConan(ConanFile):
    name = "fgo-ppp"
    version = "0.1.0"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

    def requirements(self):
        # Linear algebra — core dependency, shared with GTSAM
        self.requires("eigen/3.4.0")

        # Logging
        self.requires("spdlog/1.14.1")

        # String formatting (used by spdlog and directly)
        self.requires("fmt/10.2.1")

        # Unit testing
        self.requires("gtest/1.14.0")

        # --- Phase 3+ (uncomment when needed) ---
        # Lie groups SO3/SE3 for IMU pre-integration
        # self.requires("sophus/1.22.10")
        #
        # GTSAM is not on ConanCenter — use CMake FetchContent instead (see CMakeLists.txt)

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()
