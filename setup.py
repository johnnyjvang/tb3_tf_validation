from setuptools import find_packages, setup

package_name = 'tb3_tf_validation'

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
    maintainer='jvang',
    maintainer_email='johnnyjvang@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_tree_check = tb3_tf_validation.tf_tree_check:main',
            'tf_static_check = tb3_tf_validation.tf_static_check:main',
            'tf_dynamic_check = tb3_tf_validation.tf_dynamic_check:main',
        ],
    },
)
