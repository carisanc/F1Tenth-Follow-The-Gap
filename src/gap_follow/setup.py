from setuptools import setup

package_name = 'gap_follow'

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
    maintainer='Carolina',
    maintainer_email='carolina@email.com',
    description='Follow The Gap',
    license='MIT',
    entry_points={
        'console_scripts': [
            'reactive_gap_follower = gap_follow.reactive_gap_follow:main',
        ],
    },
)
