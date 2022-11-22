import os
from glob import glob
from setuptools import setup

package_name = 'urscript_picker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.script'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsvm',
    maintainer_email='tsvetomila.mihaylova@gmail.com',
    description='URScript client',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urscript_picker = urscript_picker.urscript_picker:main'
        ],
    },
)
