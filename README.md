<h1 align="center">octomap-python</h1>
<h4 align="center">Python binding of <a href="https://github.com/OctoMap/octomap">the OctoMap library</a>.</h4>

<div align="center">
  <a href="https://pypi.python.org/pypi/octomap-python"><img src="https://img.shields.io/pypi/v/octomap-python.svg"></a>
  <a href="https://pypi.org/project/octomap-python"><img src="https://img.shields.io/pypi/pyversions/octomap-python.svg"></a>
  <a href="https://github.com/wkentaro/octomap-python/actions"><img src="https://github.com/wkentaro/octomap-python/workflows/ci/badge.svg"></a>
</div>


## Installation

```bash
uv add octomap-python
```


## Example

```bash
git clone --recursive https://github.com/wkentaro/octomap-python.git && cd octomap-python
uv sync --group examples

cd examples
uv run python insertPointCloud.py
```

<img src="examples/.readme/insertPointCloud.jpg" height="200px" />


## Release

Releases are published to PyPI by the `publish` workflow, which fires when a
GitHub Release is published. It builds the wheel matrix + sdist and uploads via
[PyPI Trusted Publishing](https://docs.pypi.org/trusted-publishers/) (OIDC, no
stored token).

To cut a release:

1. Bump `version` in `pyproject.toml` to match the new tag and commit it.
2. Create a GitHub Release with tag `vX.Y.Z` (e.g. `v1.8.0.post13`).
3. The `publish` workflow runs and uploads the built distributions to PyPI.


## Acknowledgement

This is a fork of [neka-nat/python-octomap](https://github.com/neka-nat/python-octomap).
