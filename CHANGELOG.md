# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versions follow [PEP 440](https://peps.python.org/pep-0440/): the first three
segments track the bundled OctoMap C++ library version (currently 1.8.0), and a
fourth segment counts binding revisions on that upstream.

This history was backfilled from git and PyPI release records. Releases through
1.8.0.post12 used a `.postN` suffix and predate git tags, so each is dated by its
PyPI upload.

## [1.8.0.13] - 2026-06-20

The first release in over six years: a full build and tooling modernization plus
several long-standing binding fixes.

### Added
- `dynamicEDT_getDistanceAndClosestObstacle` binding for the Euclidean distance transform (#37).

### Fixed
- `castRay` now fills the `end` coordinate whether or not the ray hits an occupied voxel (#36).
- OctoMap file paths accept `str`, not only `bytes` (#38).
- Iterator crash caused by the name-mangled `__is_end` / `__is_acceseable` attributes (#23).
- Compilation on modern compilers, by pinning the OctoMap build to C++14 (#24).

### Changed
- Build migrated to scikit-build-core, producing a statically linked extension (#40).
- OctoMap C++ is fetched and pinned via CMake FetchContent instead of a git submodule (#41).
- Binary wheels (CPython 3.9-3.13; manylinux x86_64/aarch64, macOS arm64) and an sdist are built with cibuildwheel (#49).
- PyPI publishing runs on GitHub Release via Trusted Publishing, with no stored API token (#52).
- Adopted uv for the lockfile, dev dependency group, and install/CI flow (#42).
- Migrated linting and formatting from black and flake8 to ruff (#46).
- Test suite converted from unittest to pytest and run from the repository root (#43).
- README rewritten with a Python quick start; relative image URLs are rewritten at build time via hatch-fancy-pypi-readme, retiring github2pypi (#44, #53).
- Example dependencies moved from a published extra to a dependency group, with the GUI stack and Python pinned to keep them runnable (#42, #45).
- CI tests the lower and upper supported Python bounds, 3.9 and 3.14 (#47).

### Removed
- Python 2 and Python 3.5 support, alongside repository housekeeping and an added LICENSE file (#35).

## [1.8.0.post12] - 2019-07-03

### Removed
- `bbx_min` and `bbx_max` arguments from `extractPointCloud`.

## [1.8.0.post11] - 2019-07-02

### Fixed
- Output dimension of `extractPointCloud`.

## [1.8.0.post10] - 2019-06-29

### Added
- `bbx_min` and `bbx_max` arguments to `extractPointCloud`.

## [1.8.0.post9] - 2019-06-21

### Changed
- Reverted linking against static libraries.

### Removed
- Unused `opengl_camera_transform`.

## [1.8.0.post8] - 2019-06-21

### Changed
- Linked against static libraries.

## [1.8.0.post7] - 2019-06-21

### Fixed
- Further handling of empty point clouds.

## [1.8.0.post6] - 2019-06-21

### Fixed
- Zero-sized point cloud handling in `extractPointCloud`.

## [1.8.0.post5] - 2019-06-11

### Changed
- Maintenance re-release; no functional changes.

## [1.8.0.post4] - 2019-05-08

### Added
- `README.md` to the source distribution manifest.

## [1.8.0.post3] - 2019-05-08

### Added
- `extractPointCloud`, plus visualization helpers and example scripts.

### Changed
- `getMetric*` methods return `numpy.ndarray`.

## [1.8.0.post2] - 2019-04-16

### Changed
- README and `setup.py` packaging metadata.

## [1.8.0.post1] - 2019-04-16

### Added
- Initial PyPI release of the OctoMap Python binding (bundling OctoMap 1.8.0):
  OcTree read/write, node and tree iterators, `OcTreeKey`, ray casting, and a
  dynamicEDT3D distance field.
