from setuptools import setup
pkg = 'hospital_delivery'
setup(
    name=pkg, version='0.1.0', packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', ['launch/agent.launch.py']),
        ('share/' + pkg + '/config', ['config/agent.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'order_manager = hospital_delivery.order_manager:main',
        'task_planner = hospital_delivery.task_planner:main',
        't_agent = hospital_delivery.t_agent:main',
        'crowd_monitor = hospital_delivery.crowd_monitor:main',
        'nav_executor = hospital_delivery.nav_executor:main',
        'safety_supervisor = hospital_delivery.safety_supervisor:main',
        'bedside_verifier = hospital_delivery.bedside_verifier:main',
    ]},
)
