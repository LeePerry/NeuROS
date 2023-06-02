import time

from nodes.hooks import neuros_initialise, neuros_receive

@neuros_initialise
def arrive_at_centre_court(node):
    node.get_logger().info(f"{node.get_name()} is ready to play!")
    if node.get_config().get_parameter("first_to_serve"):
        serve = node.get_outgoing("ball")
        ball = serve.create_packet()
        ball.data = 1
        serve.send(ball)

@neuros_receive("ball")
def whack_the_ball(node):
    time.sleep(0.5)
    ball = node.get_incoming("ball").latest()
    node.get_logger().info(f"whack {ball.data}")
    ball.data += 1
    node.get_outgoing("ball").send(ball)
