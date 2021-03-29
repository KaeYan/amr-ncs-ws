import sys
from time import sleep
import uuid
import rclpy

from rmf_lift_msgs.msg import LiftRequest

def main(argv=sys.argv):

    rclpy.init(args=argv)
    node = rclpy.create_node('lift_request_publisher')

    # FILL IN FOO AND BAR HERE
    publisher = node.create_publisher(LiftRequest, '/lift_requests', 10)
    sleep(0.5)

    lift_name = str(sys.argv[1])
    session_id = str(sys.argv[2])
    request_type = int(sys.argv[3])
    # uint8 REQUEST_END_SESSION=0
    # uint8 REQUEST_AGV_MODE=1
    # uint8 REQUEST_HUMAN_MODE=2
    destination_floor = str(sys.argv[4])
    door_state = int(sys.argv[5])
    # uint8 DOOR_CLOSED=0
    # uint8 DOOR_OPEN=2

    msg = LiftRequest()
    msg.lift_name = lift_name
    msg.session_id = session_id
    msg.request_type = request_type
    msg.destination_floor = destination_floor
    msg.door_state = door_state

    for _ in range(5):
        publisher.publish(msg)
        sleep(0.5)

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

