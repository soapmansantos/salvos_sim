from setuptools import setup
import os
from glob import glob

package_name = 'salvos_gz'

def files(patterns):
    out = []
    for p in patterns:
        out.extend(glob(p, recursive=True))
    return [f for f in out if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=['salvos_gz'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),

        (f'share/{package_name}/launch', files(['launch/*.py'])),

        (f'share/{package_name}/config', files(['config/*.yaml'])),

        (f'share/{package_name}/worlds', files(['worlds/*'])),

        (f'share/{package_name}/models/urdf', files(['models/urdf/*', 'models/urdf/*.xacro'])),

        (f'share/{package_name}/models/hermes', files([
            'models/hermes/model.sdf',
            'models/hermes/model.config',
        ])),

        (f'share/{package_name}/models/hermes/meshes', files([
            'models/hermes/meshes/*.stl',
            'models/hermes/meshes/*.dae',
            'models/hermes/meshes/*.obj',
            'models/hermes/meshes/*.mtl',
            'models/hermes/meshes/*.png',
            'models/hermes/meshes/*.jpg',
            'models/hermes/meshes/*.jpeg',
            'models/hermes/meshes/*.tga',
            'models/hermes/meshes/*.bmp',
        ])),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santos',
    maintainer_email='santosgarciavalls@gmail.com',
    description='Simulation of Hermes drone',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
                    'pose_bridge = salvos_gz.pose_bridge:main',
    ]},
)
