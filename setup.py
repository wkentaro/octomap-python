
from distutils.core import Extension, setup
from Cython.Distutils import build_ext

setup(
    name="octomap",
    version="0.1",
    license = "BSD",
    packages=["octomap"],
    ext_modules=[Extension(
        "octomap",
        ["octomap/octomap.pyx"],
        libraries=[
                "octomap",
                "octomath"
                ],
        language="c++")],
    cmdclass={'build_ext': build_ext},
    )
