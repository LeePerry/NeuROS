import time

from std_msgs.msg import Int8

from nodes.hooks import neuros_initialise, neuros_receive

@neuros_initialise
def warm_up(node):
    node.get_logger().info(f"{node.get_name()} has entered the game")
    if node.get_config().get_node_parameter("first_to_serve"):
        msg = Int8()
        msg.data = 1
        node.send("ball", msg)

@neuros_receive("ball")
def hit_the_ball(node, input):
    time.sleep(0.25)
    node.get_logger().info(f"Whack {input.data}")
    msg = Int8()
    msg.data = input.data + 1
    node.send("ball", msg)
