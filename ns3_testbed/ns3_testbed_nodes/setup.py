from setuptools import find_packages
from setuptools import setup

package_name = 'ns3_testbed_nodes'

setup(
    name=package_name,
    version='0.6.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
#    py_modules=['ns3_testbed_nodes.set_nns'],
    install_requires=['setuptools'],
    zip_safe=True,
    author='your name',
    author_email='you@yours.com',
    maintainer='your name',
    maintainer_email='you@yours.com',
    keywords=['ROS'],
    classifiers=[
        'Programming Language :: Python'
    ],
    description=(
        'ns-3 testbed nodes.'
    ),
    license='your license',
    entry_points={
        'console_scripts': [
            'gs = ns3_testbed_nodes.gs:main',
            'r = ns3_testbed_nodes.r:main'
        ],
    },
)
