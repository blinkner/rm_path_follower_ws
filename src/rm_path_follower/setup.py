from setuptools import find_packages, setup
from glob import glob

setup(
    name='rm_path_follower',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rm_path_follower']),
        ('share/rm_path_follower', ['package.xml'])
    ] + [('share/rm_path_follower/' + d, glob(d + '/*')) for d in ['launch', 'config', 'worlds', 'rviz']],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adriano',
    maintainer_email='maintainer@example.com',
    description='Aula 04: teleop, sensores e navegacao reativa',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_extractor = rm_path_follower.map_extractor:main',
            'planner_node = rm_path_follower.planner_node:main',
            'controller_node = rm_path_follower.controller_node:main',
            'plot_resultados = rm_path_follower.utils.plot_resultados:main',
        ],
    },
)