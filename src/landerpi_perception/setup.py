from setuptools import setup
from glob import glob
package_name = 'landerpi_perception'
setup(name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
                ('share/' + package_name + '/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='LanderPi maintainers', maintainer_email='maintainer@example.com',
    description='LanderPi task foundation', license='Apache-2.0',
    entry_points={'console_scripts': ['perception_server = landerpi_perception.perception_server:main']})
