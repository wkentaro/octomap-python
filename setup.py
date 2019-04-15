import subprocess
import sys


# https://github.com/skvark/opencv-python/blob/master/setup.py
def install_packages(*requirements):
    # No more convenient way until PEP 518 is implemented;
    # setuptools only handles eggs
    subprocess.check_call(
        [sys.executable, "-m", "pip", "install"] + list(requirements)
    )


# https://github.com/skvark/opencv-python/blob/master/setup.py
def get_or_install(name, version=None):
    """ If a package is already installed, build against it. If not, install"""
    # Do not import 3rd-party modules into the current process
    import json
    js_packages = json.loads(
        subprocess.check_output(
            [sys.executable, "-m", "pip", "list", "--format", "json"]
        ).decode('ascii'))  # valid names & versions are ASCII as per PEP 440
    try:
        [package] = (
            package for package in js_packages if package['name'] == name
        )
    except ValueError:
        install_packages("%s==%s" % (name, version) if version else name)
        return version
    else:
        return package['version']


def main():
    get_or_install('numpy')
    get_or_install('Cython')
    get_or_install('setuptools')
    get_or_install('scikit-build')

    from Cython.Distutils import build_ext
    from setuptools import Extension
    import skbuild

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
