from setuptools import find_packages, setup

setup(
    name="drone_api",
    version="0.0.1",
    description="Python bridge for Drone API through Genomix",
    author="Selvakumar H S",
    author_email="selvakumar.h-s@laas.com",
    url="",
    packages=find_packages(include=["drone_api", "drone_api.*", "up_demo", "up_demo.*"]),
    install_requires=[],
    license="MIT",
    extras_require={"unified_planning": ["unified_planning", "up_aries"]},
    tests_require=["pytest"],
    # package_data={"exampleproject": ["data/schema.json"]},
)
