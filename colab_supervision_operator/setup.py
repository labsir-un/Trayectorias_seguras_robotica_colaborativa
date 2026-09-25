from setuptools import find_packages, setup

package_name = 'colab_supervision_operator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/face_landmarker.task']),
    ],
    install_requires=['setuptools', 
                    'numpy',
                    'opencv-python',
                    'ultralytics',
                    'mediapipe'],
    zip_safe=True,
    maintainer='jsdaleman',
    maintainer_email='jdaleman@unal.edu.co',
    description='package for colab supervision operator, supervision of states in robotics cells',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "operator_fatigue = colab_supervision_operator.nodes.operator_fatigue:main",
            "camera = colab_supervision_operator.nodes.camera:main",
            "robot_operator_interaction = colab_supervision_operator.nodes.robot_operator_interaction:main",
            "realsense_operator_xyz_test = colab_supervision_operator.nodes.realsense_operator_xyz_test:main",
        ],
    },
)
