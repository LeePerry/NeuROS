from nodes.decorators import neuros_initialise, neuros_connection

@neuros_initialise
def warm_up(node):
    print(f"{node.name} has entered the game")

@neuros_connection("ball")
def hit_the_ball(node, input):
    print(f"[{input}] {node.name} hit the ball")
    output = input + 1
    node.send("ball", output)
