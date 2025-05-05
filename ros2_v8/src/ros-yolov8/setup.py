from setuptools import find_packages, setup

package_name = 'ros_yolo'

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
    maintainer='aspera',
    maintainer_email='aspera@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capturer = ros_yolo.camera_capture_function:main',
            'detector1 = ros_yolo.target_detect_function:main',
            'detector = ros_yolo.detect:main',
            'pub = ros_yolo.pub_id:main',
            'nub = ros_yolo.number:main',
            'detector_text = ros_yolo.detect_text:main',
            'save = ros_yolo.save:main',
            'record = ros_yolo.record:main',
            'text_pub = ros_yolo.text_pub:main'
        ],
    },
)
