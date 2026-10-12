from setuptools import setup

setup(
    name="lostlink_planner",
    version="0.2.0",
    packages=["lostlink_planner"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/lostlink_planner"]),
        ("share/lostlink_planner", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kareena Mueenuddin",
    description="Lost-Link SAR Stage 2 planner",
    license="MIT",
    entry_points={"console_scripts": ["avoid_mission = lostlink_planner.avoid_mission_node:main"]},
)
