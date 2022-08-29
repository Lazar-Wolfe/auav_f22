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
    maintainer='kishan',
    maintainer_email='kishan.jesalpura@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj_node = submission_onboard.TrajectoryBezier_publisher:main',
            'transformation_node = submission_onboard.get3dcoord:main',
            'detector_node = submission_onboard.redros:main',
        ],
    },
)
