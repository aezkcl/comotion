# Examples

This repository keeps example code separate from standalone benchmark
workloads.

## Library Use

```bash
cmake --build build --target comotion_library_smoke
./build/examples/comotion_library_smoke
```

[examples/library_smoke.cpp](examples/library_smoke.cpp) demonstrates the
smallest expected downstream pattern: link against `comotion`, create a
`MultiRobotProblem`, add a `FlyingSphere`, and call the collision checker.

## Installed Package Use

The `examples/` directory can also be configured as a standalone downstream
project after CoMotion has been installed:

```bash
cmake --install build --prefix /path/to/comotion-install
cmake -S examples -B /tmp/comotion-examples-build \
  -DCMAKE_PREFIX_PATH=/path/to/comotion-install
cmake --build /tmp/comotion-examples-build
/tmp/comotion-examples-build/comotion_library_smoke
```

For another downstream project, point CMake at the same install prefix:

```bash
cmake -S downstream -B downstream/build \
  -DCMAKE_PREFIX_PATH=/path/to/comotion-install
```

The downstream target links to the installed CoMotion CMake package, currently
exported as `comotion`:

```cmake
find_package(comotion REQUIRED)
target_link_libraries(my_target PRIVATE comotion::comotion)
```

## Standalone Workloads

The standalone workload executables live in `apps/` and build under
`build/apps/`:

```bash
cmake --build build --target mobile_robot_2d_crossing
cmake --build build --target planar_manipulator_cross
cmake --build build --target panda_cage
cmake --build build --target panda_flat
```

For repeated runs, use the three public benchmark runners documented in
[BENCHMARKS.md](BENCHMARKS.md).
