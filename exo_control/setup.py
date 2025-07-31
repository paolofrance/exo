from setuptools import setup

package_name = 'exo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Exoskeleton control package with C++ and Python nodes.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'two_boards_exo_control = exo_control.two_boards_exo_control:main',
            'admittance_controller = exo_control.admittance_controller:main',
            'stoop_trajs = exo_control.stoop_trajs:main',
        ],
    },
)
