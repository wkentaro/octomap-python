from Cython.Distutils import build_ext
from setuptools import Extension
import skbuild


def main():
    ext_modules = [
        Extension(
            'octomap',
            ['octomap/octomap.pyx'],
            include_dirs=[
                'src/octomap/octomap/include',
                'src/octomap/dynamicEDT3D/include',
            ],
            library_dirs=[
                'src/octomap/lib',
            ],
            libraries=[
                'dynamicedt3d',
                'octomap',
                'octomath',
            ],
            language='c++',
        )
    ]

    skbuild.setup(
        name='octomap',
        version='0.5',
        license='BSD',
        ext_modules=ext_modules,
        cmdclass={'build_ext': build_ext},
        cmake_args=[],
        cmake_source_dir='src/octomap',
    )


if __name__ == '__main__':
    main()
