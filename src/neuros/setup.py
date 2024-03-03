from setuptools import setup

package_name = 'neuros'

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
    maintainer='lperry',
    maintainer_email='leeperryis@hotmail.com',
    description='The generic NeuROS transport node',
    license='MIT License. Copyright (c) 2023 Lee Perry',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = neuros.node:main'
        ],
    },
)
