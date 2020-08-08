# octomap-python

[![PyPI Version](https://img.shields.io/pypi/v/octomap-python.svg)](https://pypi.python.org/pypi/octomap-python)
[![Python Versions](https://img.shields.io/pypi/pyversions/octomap-python.svg)](https://pypi.org/project/octomap-python)
[![Build Status](https://travis-ci.com/wkentaro/octomap-python.svg?branch=master)](https://travis-ci.com/wkentaro/octomap-python)


Python binding of [the OctoMap library](https://github.com/OctoMap/octomap).


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
