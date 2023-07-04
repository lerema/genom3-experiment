import cv2
import numpy as np

from drone_api.connect import Connector

c = Connector(drone_id=1)

d435 = c.components["d435"].component
camviz = c.components["camviz"]

d435.set_fps(30)
d435.disconnect()
d435.connect("832112070817")

# arucotag = c.components["arucotag"].component
# arucotag.set_length(0.2)
# arucotag.output_frame(0)
# arucotag.connect_port("frame", "d4351/frame/raw")
# arucotag.connect_port("drone", "pom1/frame/robot")
# arucotag.connect_port("intrinsics", "d4351/intrinsics")
# arucotag.connect_port("extrinsics", "d4351/extrinsics")

# camviz.connect_port("frame", "d4351/frame/raw")
# camviz.add_pixel_display("tag1", "d4351")
camviz.show(ratio=2)
camviz.add_camera("d4351/frame/raw", ack=True)

while True:
    try:
        image_buffer = d435.frame("raw")
    except:
        print("Unable to fetch image")
        continue

    if image_buffer["frame"]["pixels"] == []:
        print("No image")
        continue

    # Decode image
    image_data = np.array(image_buffer["frame"]["pixels"], dtype=np.uint8)
    image_data = np.reshape(image_data, (1280, 720))

    cv2.imshow("image", image_data)
    cv2.waitKey(1)
