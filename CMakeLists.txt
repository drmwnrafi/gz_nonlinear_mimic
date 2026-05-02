cmake_minimum_required(VERSION 3.10)
project(non_linear_mimic)

# ✅ CORRECT: gz-sim8 has NO "system" component - just use the base package
find_package(gz-sim8 REQUIRED)

# ✅ CORRECT: gz-plugin2 DOES have a "register" component
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Create shared library
add_library(NonLinearMimic SHARED 
  src/NonLinearMimic.cc
)

# ✅ Link against correct targets (no ::system suffix)
target_link_libraries(NonLinearMimic
  gz-sim8::core          # ✅ Core Gazebo Sim library
  gz-plugin2::register   # ✅ Plugin registration macros
)

# Include directories (optional, if you have headers)
target_include_directories(NonLinearMimic PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Ensure RPATH is set for runtime plugin loading
set_target_properties(NonLinearMimic PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install to lib folder
install(TARGETS NonLinearMimic
  DESTINATION lib
)