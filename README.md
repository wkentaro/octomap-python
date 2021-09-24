<h1 align="center">octomap-python</h1>
<h4 align="center">Python binding of <a href="https://github.com/OctoMap/octomap">the OctoMap library</a>.</h4>

<div align="center">
  <a href="https://pypi.python.org/pypi/octomap-python"><img src="https://img.shields.io/pypi/v/octomap-python.svg"></a>
  <a href="https://pypi.org/project/octomap-python"><img src="https://img.shields.io/pypi/pyversions/octomap-python.svg"></a>
  <a href="https://github.com/wkentaro/octomap-python/actions"><img src="https://github.com/wkentaro/octomap-python/workflows/ci/badge.svg"></a>
</div>


## Installation

```bash
pip install octomap-python
```


## Example

```bash
git clone --recursive https://github.com/wkentaro/octomap-python.git && cd octomap-python
pip install -e '.[example]'

cd examples
python insertPointCloud.py
```

<img src="examples/.readme/insertPointCloud.jpg" height="200px" />


## Acknowledgement

This is a fork of [neka-nat/python-octomap](https://github.com/neka-nat/python-octomap).
