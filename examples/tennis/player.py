import time

from std_msgs.msg import Int8

from nodes.hooks import neuros_initialise, neuros_receive

@neuros_initialise
def stretch_out(node):
    node.get_logger().info(f"{node.get_name()} is ready to play!")
    if node.get_config().get_node_parameter("first_to_serve"):
        msg = Int8()
        msg.data = 1
        node.get_connection_output("ball").send(msg)

@neuros_receive("ball")
def whack_the_ball(node):
    ball = node.get_connection_input("ball").latest()
    node.get_logger().info(f"whack {ball.data}")
    time.sleep(0.5)
    msg = Int8()
    msg.data = ball.data + 1
    node.get_connection_output("ball").send(msg)
