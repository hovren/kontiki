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
    def __init__(self, name):
        Extension.__init__(self, name, sources=[])


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: " +
                ", ".join(e.name for e in self.extensions))

        build_directory = os.path.abspath(self.build_temp)

        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + build_directory,
            '-DPYTHON_EXECUTABLE=' + sys.executable
        ]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]

        # Assuming Makefiles
        build_args += ['--', '-j2']

        self.build_args = build_args

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''),
            self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        # CMakeLists.txt is in the same directory as this setup.py file
        #cmake_list_dir = os.path.abspath(os.path.dirname(__file__))
        cmake_list_dir = (Path(__file__).parents[0] / '../').resolve()
        print('CMAKE LIST DIR', cmake_list_dir)
        print('-'*10, 'Running CMake prepare', '-'*40)
        subprocess.check_call(['cmake', cmake_list_dir] + cmake_args,
                              cwd=self.build_temp, env=env)

        print('-'*10, 'Building extensions', '-'*40)
        cmake_cmd = ['cmake', '--build', '.'] + self.build_args
        subprocess.check_call(cmake_cmd,
                              cwd=self.build_temp)

        # Move from build temp to final position
        for ext in self.extensions:
            self.move_output(ext)

    def move_output(self, ext):
        build_temp = Path(self.build_temp).resolve()
        dest_path = Path(self.get_ext_fullpath(ext.name)).resolve()
        source_path = build_temp / self.get_ext_filename(ext.name)
        dest_directory = dest_path.parents[0]
        dest_directory.mkdir(parents=True, exist_ok=True)
        self.copy_file(source_path, dest_path)


class KontikiExtension(CMakeExtension):
    def __init__(self, module):
        super().__init__('kontiki.' + module)


class TrajectoryExtension(KontikiExtension):
    def __init__(self, module):
        super().__init__('trajectories.' + module)


class MeasurementExtension(KontikiExtension):
    def __init__(self, module):
        super().__init__('measurements.' + module)


class SensorExtension(KontikiExtension):
    def __init__(self, module):
        super().__init__('sensors.' + module)


ext_modules = [
#    KontikiExtension('dummy'),
    KontikiExtension('sfm'),
    KontikiExtension('_trajectory_estimator'),
    KontikiExtension('_ceres'),
    TrajectoryExtension('_linear_trajectory'),
    TrajectoryExtension('_uniform_r3_spline_trajectory'),
    TrajectoryExtension('_uniform_so3_spline_trajectory'),
    TrajectoryExtension('_uniform_se3_spline_trajectory'),
    TrajectoryExtension('_split_trajectory'),
    SensorExtension('_pinhole_camera'),
    SensorExtension('_atan_camera'),
    SensorExtension('_basic_imu'),
    SensorExtension('_constant_bias_imu'),
    MeasurementExtension('_position_measurement'),
    MeasurementExtension('_static_rscamera_measurement'),
    MeasurementExtension('_lifting_rscamera_measurement'),
    MeasurementExtension('_newton_rscamera_measurement'),
    MeasurementExtension('_accelerometer_measurement'),
    MeasurementExtension('_gyroscope_measurement'),
]


setup(
    name='kontiki',
    version='1.0',
    author='Hannes Ovrén',
    author_email='hannes.ovren@liu.se',
    description='Continuous-time toolkit for trajectory estimation',
    long_description='',
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)