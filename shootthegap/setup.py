from setuptools import setup

package_name = 'shootthegap'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='precious',
    maintainer_email='akashokkumar300@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = shootthegap.gap_shooter:main',
            'disparity_extender = shootthegap.disparity_extender:main'
        ],
    },
)
