from pprint import pprint
from copy import deepcopy
import cv2
import numpy as np

from drone_api.connect import Connector

c = Connector(drone_id=1)

d435 = c.components["d435"].component

d435.set_fps(30)
d435.disconnect()
d435.connect("832112070817")

arucotag = c.components["arucotag"].component
# arucotag.set_length(0.2)
# arucotag.output_frame(0)
arucotag.add_marker("10")
arucotag.connect_port("frame", "d4351/frame/raw")
# arucotag.connect_port("drone", "pom1/frame/robot")
arucotag.connect_port("intrinsics", "d4351/intrinsics")
arucotag.connect_port("extrinsics", "d4351/extrinsics")

# camviz = c.components["camviz"]
# # camviz.connect_port("frame", "d4351/frame/raw")
# # camviz.add_pixel_display("tag1", "d4351")
# camviz.show(ratio=2)
# camviz.add_camera("d4351/frame/raw", ack=True)

first = True
while True:
    try:
        image_buffer = d435.frame("raw")
    except:
        print("Unable to fetch image")
        continue

    if image_buffer["frame"]["pixels"] == []:
        print("No image")
        continue

    if first:
        # Share the configurations
        data = deepcopy(image_buffer["frame"])
        data.pop("pixels")
        pprint(data)
        first = False

    # Decode image
    image_data = np.array(image_buffer["frame"]["pixels"], dtype=np.uint8)
    image_data = np.reshape(image_data, (480, 640, 3))

    if arucotag.pose("10"):
        print(arucotag.pose("10"))

    cv2.imshow("image", image_data)
    cv2.waitKey(1)
