from setuptools import find_packages, setup

package_name = 'arm_pi_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fotis Rousogiannis',
    maintainer_email='115790877+FotisRousogiannis@users.noreply.github.com',
    description=('Raspberry Pi driver of the arm: AS5600 encoders to joint_states, '
                 'servos through PCA9685 (ADR-0021).'),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'driver = arm_pi_driver.node:main',
        ],
    },
)
