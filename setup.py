from setuptools import find_packages, setup

package_name = 'ros2_ibus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'flySkyiBus @ git+https://github.com/GamerHegi64/FlySky-Ibus.git'
    ],
    zip_safe=True,
    maintainer='jabax',
    maintainer_email='jaime.bravo.algaba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ibus_node = ros2_ibus.ibus_node:main',
            'ibus_console_viewer_node = ros2_ibus.ibus_console_viewer:main'
        ],
    },
)
