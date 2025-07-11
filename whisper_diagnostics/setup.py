from setuptools import setup

package_name = 'whisper_diagnostics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Diagnostics node for whisper pipeline testing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_diagnostics_node = whisper_diagnostics.test_diagnostics_node:main',
        ],
    },
)
