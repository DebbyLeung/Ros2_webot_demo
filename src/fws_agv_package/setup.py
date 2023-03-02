from setuptools import setup

package_name = 'fws_agv_package'
data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])]
data_files.append(('share/' + package_name + '/launch', ['launch/world_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world_agv.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/fws_agv.urdf']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='debby',
    maintainer_email='stebbyleung@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fws_robot_driver = fws_agv_package.fws_robot_driver:main',
            'teleop_twist_keyboard = fws_agv_package.teleop_twist_keyboard:main'
        ],
    },
)
