from setuptools import find_packages, setup

package_name = 'hand_follow'

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
    maintainer='dominik',
    maintainer_email='dominik.urbaniak@upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'real_hand_follow_ur3e = hand_follow.real_hand_follow_ur3e:main',
        'a2_hand_detection = hand_follow.a2_hand_detection:main',
        ],
    },
)
