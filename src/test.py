"""Test threading issue."""

from drone_api.connect import Connector
import threading


def main():
    """Main function"""
    try:
        c = Connector()
    except Exception as e:
        raise Exception("Failed to connect to the drone") from e

    # Start the connection and take off
    c.start()
    
    def move():
        while True:
            c.components["maneuver"].component.goto({
                    "x": 10.0,
                    "y": 0.0,
                    "z": 1,
                    "yaw": 0,
                    "duration": 0,
                    'ack': True,
                })
    
    def get_pose():
        while True:
            print(c.components["maneuver"].component.get_reference()["reference"]["pos"])
    
    thread_1 = threading.Thread(target=move)
    thread_2 = threading.Thread(target=get_pose)
    
    thread_1.start()
    thread_2.start()
    
    thread_1.join()
    thread_2.join()

    # wait until keypress
    input("Press Enter to exit...")

    c.stop()
    
if __name__ == "__main__":
    main()