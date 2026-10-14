from glob import glob
from setuptools import setup

name = 'underwater_inspection'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                ('share/' + name, ['package.xml']),
                ('share/' + name + '/launch', glob('launch/*.py')),
                ('share/' + name + '/config', glob('config/*.yaml')),
                ('share/' + name + '/scenarios', glob('scenarios/*.scn'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='sencangWei', maintainer_email='sencangWei@users.noreply.github.com',
    description='Vision-guided underwater inspection benchmark', license='GPL-3.0-or-later',
    entry_points={'console_scripts': ['mission = underwater_inspection.mission:main',
                                     'viewer = underwater_inspection.viewer:main']})
