from setuptools import setup

package_name = 'autonofly_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nada',
    maintainer_email='nada@example.com',
    description='SLAM package for drone navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fastslam_node = autonofly_slam.fastslam_node:main',
        ],
    },
)