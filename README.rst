python-octomap
=========

.. image:: https://travis-ci.org/neka-nat/python-octomap.svg?branch=master
    :target: https://travis-ci.org/neka-nat/python-octomap

About
-----
This library is a Python binding of Octomap library.

Build
-----
Building **python-octomap requires Octomap and DynamicEDT3D libraries**.
When building, you can specify their location with the --include-dirs
and --library-dirs command line options:


    $ python setup.py build_ext --include-dirs /path/to/includes --library-dirs /path/to/libraries

If you use ubuntu 18.04, you can install the necessary libraries from apt:

    $ sudo apt-get install liboctomap-dev libdynamicedt3d-dev

    $ python setup.py build

If you use ubuntu 18.04 and ROS melodic, you can also use the following command:

    $ sudo apt-get install ros-melodic-octomap ros-melodic-dynamic-edt-3d

    $ source /opt/ros/melodic/setup.bash

    $ python setup.py build


Install
-------
You can install python-octomap using the normal distutils install command:

    $ python setup.py install


View octomap
------------
You can also show octomap using "example/octomap_viewer.py".
This viewer needs "mayavi".

    $ python example1.py # Create octomap file, "test.bt"

    $ python octomap_viewer.py -f test.bt -s

.. image:: example/image.png
