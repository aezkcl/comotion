# Resources

This directory contains the robot descriptions used by the public CoMotion apps
and benchmark workloads.

Included resource families:

- Panda
- Planar3

The `*_spherized.urdf` files provide conservative sphere decompositions used by
the sphere, VAMP, and FCL collision backends in the public benchmark workloads.
The public source tree intentionally ships only the minimal Panda and Planar3
URDF/SRDF files required by the benchmark runners.

Large unused robot families, problem archives, demo media, heightfields, and
legacy conversion helpers were removed from the public source tree. The public
CoMotion benchmark entry points are the three runners documented in
[../BENCHMARKS.md](../BENCHMARKS.md).

## Provenance And Release Status

The table below records the release status visible from this repository.

| Resource | Contents | Release status |
| --- | --- | --- |
| `planar3/` | Synthetic planar manipulator URDF, SRDF, and sphere model | Local synthetic model. |
| `panda/` | Panda SRDF and spherized collision model | Derived benchmark model. No Panda mesh assets are redistributed. Verify upstream attribution before release if replacing these files with external robot assets. |
