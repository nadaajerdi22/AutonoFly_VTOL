from setuptools import setup

package_name = 'autonofly_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nada',
    maintainer_email='nadaajerdi2022@gmail.com',
    description='Navigation autonome: RRT',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_node = autonofly_nav.rrt_node:main',
            'talker = autonofly_nav.talker:main',
        ],
    },
)
