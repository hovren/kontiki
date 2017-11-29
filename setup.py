import os
import re
import sys
import sysconfig
import platform
import subprocess
from pathlib import Path

from distutils.version import LooseVersion
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
from setuptools.command.test import test as TestCommand


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: " +
                ", ".join(e.name for e in self.extensions))

        # Find the directory above the built package
        # I.e. if the extension outputs are /path/to/build/package/subpack/mod
        # then this returns /path/to/build/
        if len(self.get_outputs()) > 1:
            base_outputdir = os.path.commonpath(self.get_outputs())
        else:
            base_outputdir = os.path.dirname(self.get_outputs()[0])

        base_outputdir = os.path.abspath(base_outputdir)

        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + base_outputdir,
            '-DPYTHON_EXECUTABLE=' + sys.executable
        ]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        build_args += ['--', '-j2']

        self.build_args = build_args

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''),
            self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        # CMakeLists.txt is in the same directory as this setup.py file
        cmake_list_dir = os.path.abspath(os.path.dirname(__file__))
        print('-'*10, 'Running CMake prepare', '-'*40)
        subprocess.check_call(['cmake', cmake_list_dir] + cmake_args,
                              cwd=self.build_temp, env=env)

        print('-'*10, 'Building extensions', '-'*40)
        print('Extensions:', self.extensions)
        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name)))


        target = ext.name.split("/")[-1]
        cmake_cmd = ['cmake', '--build', '.', '--target', target] + self.build_args
        subprocess.check_call(cmake_cmd,
                              cwd=self.build_temp)
        print()  # Add an empty line for cleaner output
        print('DEBUGDEBUGDEBUG')
        print('extdir:', self.get_ext_fullpath(ext.name))
        print('command:', ' '.join(cmake_cmd))
        print('build_args', self.build_args)
        print('extdir', extdir)

def root_extension(modname):
    return CMakeExtension('taser.' + modname)

def trajectory_extension(modname):
    return root_extension('trajectories.' + modname)

ext_modules = [
    root_extension('dummy'),
    trajectory_extension('_constant_trajectory'),
]


setup(
    name='taser',
    version='1.0',
    author='Hannes Ovrén',
    author_email='hannes.ovren@liu.se',
    description='Trajectory and structure estimation',
    long_description='',
    #packages=['taser'],
    pacakges = find_packages(),
    ext_modules=ext_modules,
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)