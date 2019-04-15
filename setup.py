from distutils.core import Extension
from distutils.core import setup

from Cython.Distutils import build_ext


setup(
    name='octomap',
    version='0.5',
    license='BSD',
    packages=['octomap'],
    ext_modules=[
        Extension(
            'octomap',
            ['octomap/octomap.pyx'],
            libraries=[
                'dynamicedt3d',
                'octomap',
                'octomath'
            ],
            language='c++',
        )
    ],
    cmdclass={'build_ext': build_ext},
)
