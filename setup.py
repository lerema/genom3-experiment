# Copyright 2022 Selvakumar H S, LAAS-CNRS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages, setup

# Read requirements.txt
# with open("requirements.txt", "r", encoding="utf-8") as f:
#     requirements = f.read().splitlines()
requirements = []

setup(
    name="drone_api",
    version="0.0.1",
    description="Python bridge for Drone API through Genomix. \
        An experimental integration of Genom framework with Unified Planning framework.",
    author="Selvakumar H S, LAAS-CNRS",
    author_email="selvakumar.h-s@laas.com",
    url="https://github.com/lerema/genom3-experiment.git",
    requires=["up_esb"],
    packages=find_packages(
        include=["drone_api", "drone_api.*", "up_demo", "up_demo.*"]
    ),
    install_requires=[],
    license="MIT",
    extras_require={"unified_planning": ["unified_planning", "up_aries"]},
    tests_require=["pytest"],
    # package_data={"exampleproject": ["data/schema.json"]},
)
