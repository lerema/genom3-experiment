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

"""Util functions and classes for the drone_api package."""
import logging
import os


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


def setup_logging(file_name: str):
    """Setup logging to file."""
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../logs")

    file_name = os.path.basename(file_name).split("/")[-1].split(".")[0]

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    if os.path.exists(os.path.join(log_dir, f"{file_name}.log")):
        os.remove(os.path.join(log_dir, f"{file_name}.log"))

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s %(name)-12s %(levelname)-8s %(message)s",
        datefmt="%m-%d %H:%M",
        filename=os.path.join(log_dir, f"{file_name}.log"),
        filemode="w",
    )
