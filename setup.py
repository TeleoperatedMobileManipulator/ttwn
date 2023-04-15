from setuptools import setup

package_name = 'robotarm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tawanpc',
    maintainer_email='tawanpc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           "networkbridge = robotarm_pkg.networkbridge:main",
           "convert_degrees = robotarm_pkg.convert_degrees:main",
           "inverse = robotarm_pkg.inverse:main",
           "pid = robotarm_pkg.pid:main",
           "motor = robotarm_pkg.motor:main",
           "sensor = robotarm_pkg.sensor:main",
           "videocommu = robotarm_pkg.videocommu:main",
           "servo = robotarm_pkg.servo:main",
           "forward = robotarm_pkg.forward:main"
           
        ],
    },
)
