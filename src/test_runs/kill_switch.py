#!/usr/bin/env python
from drone_api.connect import Connector


c = Connector(drone_id=1)
c.stop()
