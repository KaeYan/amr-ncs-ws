import sys
from time import sleep
import uuid
import rclpy

from rmf_door_msgs.msg import DoorRequest

def main(argv=sys.argv):

    rclpy.init(args=argv)
    node = rclpy.create_node('door_request_publisher')

    # FILL IN FOO AND BAR HERE
    publisher = node.create_publisher(DoorRequest, '/door_requests', 10)
    sleep(0.5)

    door_name = str(sys.argv[1])
    requested_id = str(sys.argv[2])
    door_mode = int(sys.argv[3])
    # uint32 MODE_CLOSED=0
    # uint32 MODE_MOVING=1
    # uint32 MODE_OPEN=2
    # uint32 MODE_OFFLINE=3
    # uint32 MODE_UNKNOWN=4   

    msg = DoorRequest()
    msg.requester_id = requested_id
    msg.door_name = door_name
    msg.requested_mode.value = door_mode

    for _ in range(5):
        publisher.publish(msg)
        sleep(0.5)

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
