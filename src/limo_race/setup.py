from glob import glob

from setuptools import setup

PKG = 'limo_race'
NODES = ['mapper', 'map_saver', 'map_server', 'raceline_generator', 'global_path', 'localizer',
         'local_planner', 'controller', 'go', 'bringup']
TOOLS = ['check_sensors', 'check_hose', 'calib_speed', 'calib_steer', 'teleop_key', 'odom_tf', 'viz_relay']

setup(
    name=PKG,
    version='1.0.0',
    packages=[PKG],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG]),
        ('share/' + PKG, ['package.xml']),
        ('share/' + PKG + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='LIMO 실차 레이싱 노드',
    license='Apache-2.0',
    entry_points={'console_scripts': [f'{n} = {PKG}.{n}:main' for n in NODES]
                  + [f'{t} = {PKG}.tools:{t}' for t in TOOLS]},
)
