import sys
from time import sleep
import uuid
import rclpy

from rmf_task_msgs.msg import Loop

def main(argv=sys.argv):

    rclpy.init(args=argv)
    node = rclpy.create_node('loop_request_publisher')

    # FILL IN FOO AND BAR HERE
    publisher = node.create_publisher(Loop, '/loop_requests', 10)
    sleep(0.5)

    start = str(sys.argv[1])
    finish = str(sys.argv[2])
    robot_type = str(sys.argv[3])
    num_loops = int(sys.argv[4])
    task_id = 'loop#' + str(uuid.uuid1())

    msg = Loop()
    msg.task_id = task_id
    msg.robot_type = robot_type
    msg.num_loops = num_loops
    msg.start_name = start
    msg.finish_name = finish

    for _ in range(5):
        publisher.publish(msg)
        sleep(0.5)

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)