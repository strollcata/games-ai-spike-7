'''Autonomous Agent Movement: Seek, Arrive and Flee

Created for COS30002 AI for Games, Lab 05
By Clinton Woodward cwoodward@swin.edu.au

'''
from graphics import egi, KEY
from pyglet import window, clock
from pyglet.gl import *

from vector2d import Vector2D
from world import World
from agent import Agent, AGENT_MODES  # Agent with seek, arrive, flee and pursuit


def on_mouse_press(x, y, button, modifiers):
    if button == 1:  # left
        world.target = Vector2D(x, y)


def on_key_press(symbol, modifiers):
    if symbol == KEY.P:
        world.paused = not world.paused
    elif symbol == KEY.A:
        world.agents.append(Agent(world))
    elif symbol == KEY.W:
        for myagent in world.agents:
            myagent.max_speed += 1000
    elif symbol == KEY.S:
        for myagent in world.agents:
            myagent.max_speed = max(0, myagent.max_speed - 1000)
    elif symbol == KEY.H:
        for myagent in world.agents:
            myagent.mass += 0.1
    elif symbol == KEY.L:
        for myagent in world.agents:
            myagent.mass = max(0.1, myagent.mass - 0.1)
    elif symbol == KEY.R:
        for myagent in world.agents:
            myagent.randomise_path()
    elif symbol == KEY.I:
        for agent in world.agents:
            agent.show_info = not agent.show_info
    elif symbol == KEY.B:
        for agent in world.agents:
            agent.scale.x += 5
            agent.scale.y += 5
    elif symbol == KEY.T:
        for agent in world.agents:
            agent.scale.x = max(5, agent.scale.x - 5)
            agent.scale.y = max(5, agent.scale.y - 5)
    elif symbol == KEY.D:
        for agent in world.agents:
            agent.weighted_sum = not agent.weighted_sum
            if not agent.weighted_sum:
                agent.active_modes = []
    elif symbol == KEY.U:
        for agent in world.agents:
            agent.parameter_shift("up")
    elif symbol == KEY.J:
        for agent in world.agents:
            agent.parameter_shift("down")
    elif symbol == KEY.G:
        for agent in world.agents:
            if agent.wander_parameter == 'dist':
                agent.wander_parameter = 'radius'
            elif agent.wander_parameter == 'radius':
                agent.wander_parameter = 'jitter'
            elif agent.wander_parameter == 'jitter':
                agent.wander_parameter = 'dist'
    elif symbol in AGENT_MODES:
        for agent_num in range(len(world.agents)):
            agent = world.agents[agent_num]
            if agent.weighted_sum:
                agent.active_modes.append(AGENT_MODES[symbol])
            else:
                agent.mode = AGENT_MODES[symbol]



def on_resize(cx, cy):
    world.cx = cx
    world.cy = cy

if __name__ == '__main__':

    # create a pyglet window and set glOptions
    win = window.Window(width=500, height=500, vsync=True, resizable=True)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    # needed so that egi knows where to draw
    egi.InitWithPyglet(win)
    # prep the fps display
    fps_display = clock.ClockDisplay()
    # register key and mouse event handlers
    win.push_handlers(on_key_press)
    win.push_handlers(on_mouse_press)
    win.push_handlers(on_resize)

    # create a world for agents
    world = World(500, 500)
    # add one agent
    world.agents.append(Agent(world))
    # unpause the world ready for movement
    print("Modifier controls: A to add an agent, W to increase max speed, S to decrease max speed, H to add mass, L to take away mass, B to increase scale, T to decrease scale, I to show direction info, and R to generate a new path in Follow Path mode.")
    print("Mode controls: 1 for Seek, 2 for Slow Arrive, 3 for Normal Arrive, 4 for Fast Arrive, 5 for Flee, 6 for Follow Path, 7 for Wander, 8 for Alignment, 9 for Cohesion, 0 for Separation, and D to determine steering by weighted sum.")
    print("Mode parameter controls: U to shift parameter up, J to shift parameter down, G to change which wander parameter will be shifted.")
    print("Changeable mode parameters include: flee distance in flee; waypoint threshold in follow path; and wander distance, radius and jitter.")
    print("Selected wander parameter for change starts at distance. Pressing G cycles through distance, radius and jitter in that order.")
    print("A mode's parameter can only be changed if that mode is currently active, either on its own or as part of a weighted-sum combination. Changing parameters while using weighted-sum combinations changes the parameters of all applicable and active modes simultaneously.")
    world.paused = False

    while not win.has_exit:
        win.dispatch_events()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # show nice FPS bottom right (default)
        delta = clock.tick()
        world.update(delta)
        world.render()
        fps_display.draw()
        # swap the double buffer
        win.flip()

