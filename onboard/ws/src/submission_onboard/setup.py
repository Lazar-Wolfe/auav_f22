from setuptools import setup

package_name = 'submission_onboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kishan, Chaitanya, Satyam',
    maintainer_email='kishan.jesalpura@gmail.com, c28gaur@gmail.com, satyamk1102@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'setpoint_try = submission_onboard.TrajectorySetPoint_publisher:main',
            'car_fake = submission_onboard.Sample_traj_car:main',
            'transformation_node = submission_onboard.get3dcoord:main',
            'detector_node = submission_onboard.redros:main',
            'car_detector_node = submission_onboard.car_detector:main',
        ],
    },
)
