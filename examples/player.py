
@neuros_initialise
def warm_up(context):
    print(f"{context.node_name} has entered the game")

@neuros_input("ball")
def hit_the_ball(context, message):
    print(f"[{message}] {context.node_name} hit the ball")
    context.ouput("ball", message + 1)
