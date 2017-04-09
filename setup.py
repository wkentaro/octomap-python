import os
import sys
from distutils.core import Extension, setup
from Cython.Distutils import build_ext

platform_supported = False
for prefix in ['darwin', 'linux', 'bsd']:
    if prefix in sys.platform:
        platform_supported = True
        include_dirs = [
            '/usr/include',
            '/usr/local/include',
        ]
        lib_dirs = [
            '/usr/lib',
            '/usr/local/lib',
        ]
        if 'ROS_ROOT' in os.environ:
            include_dirs += [os.path.join(os.environ['ROS_ROOT'], '../../include')]
            lib_dirs += [os.path.join(os.environ['ROS_ROOT'], '../../lib')]
        if 'CPATH' in os.environ:
            include_dirs += os.environ['CPATH'].split(':')
        if 'LD_LIBRARY_PATH' in os.environ:
            lib_dirs += os.environ['LD_LIBRARY_PATH'].split(':')
        break

if sys.platform == "win32":
    platform_supported = False

if not platform_supported:
    raise NotImplementedError(sys.platform)

setup(
    name="octomap",
    version="0.5",
    license = "BSD",
    packages=["octomap"],
    ext_modules=[Extension(
        "octomap",
        ["octomap/octomap.pyx"],
        include_dirs = include_dirs,
        library_dirs = lib_dirs,
        libraries=[
                "dynamicedt3d",
                "octomap",
                "octomath"
                ],
        language="c++")],
    cmdclass={'build_ext': build_ext},
    )
