python-octomap
=========

About
-----
This library is a Python binding of Octomap library.

Build
-----
Building **python-octomap requires Octomap headers and libraries**.
When building, you can specify their location with the --include-dirs
and --library-dirs command line options:

    $ python setup.py build_ext --include-dirs /path/to/includes --library-dirs /path/to/libraries

If you use ubuntu 12.04 and install ros-octomap at /opt directory, you can use the following command:

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
