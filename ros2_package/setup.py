from setuptools import setup

package_name = 'optical_flow_fusion'

setup(
    name=package_name,
    version='2.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    
    description=(
        'Dual H-Flow optical flow sensor fusion and attitude-mode position '
        'hold controller for PX4 v1.14.3 in GPS-denied indoor environments. '
        'Runs on Raspberry Pi 4 via uXRCE-DDS bridge.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Splits /fmu/out/sensor_optical_flow by device_id → two clean topics
            'dmux_node       = optical_flow_fusion.scripts.dmux_node:main',
            # Quality-weighted fusion → /fmu/in/fused_optical_flow → PX4 EKF2
            'fusion_node     = optical_flow_fusion.scripts.fusion_node:main',
            # Attitude-mode position hold controller - heartbeat + setpoints
            'controller_node = optical_flow_fusion.scripts.controller_node:main',
            # Logs all pipeline signals to CSV in ~/uav_logs/ at 10 Hz
            'logger_node     = optical_flow_fusion.scripts.logger_node:main',
            # Live rich terminal dashboard - system health at 4 Hz
            'dashboard_node  = optical_flow_fusion.scripts.dashboard_node:main',
        ],
    },
)
