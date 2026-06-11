from setuptools import find_packages, setup

package_name = 'turtlebot4_custom_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matrix',
    maintainer_email='lwendlan@umd.edu',
    description='Any Nodes/Libraries Created in Python',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_to_node = turtlebot4_custom_py.nav_to_node:main',
            'command_control = turtlebot4_custom_py.command_control:main',
            'nav_patrol_loop = turtlebot4_custom_py.nav_patrol_loop:main',
            'location_mapper = turtlebot4_custom_py.llm_location_mapper:main',
            'location_mapper_eval = turtlebot4_custom_py.location_mapper_eval:main',
            'llm_navigation = turtlebot4_custom_py.llm_navigation_node:main',
            'patrol_with_llm = turtlebot4_custom_py.patrol_with_llm_node:main',
            'yolo_detection = turtlebot4_custom_py.yolo_detection_node:main',
            'survey_locations = turtlebot4_custom_py.survey_locations:main',
            'merge_maps = turtlebot4_custom_py.merge_maps:main',
        ],
    },
)
