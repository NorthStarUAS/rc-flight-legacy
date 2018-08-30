# a simple locally connected (pc) joystick interface

have_pygame = False
have_joystick = False
num_axes = 0
num_buttons = 0
num_hats = 0
axes = []
buttons = []
hats = []

try:
    import pygame
    have_pygame = True
except:
    print("pygame import failed, joystick inactive")

if have_pygame:
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        have_joystick = True
        print("Detected a joystick")
    else:
        print("no joysticks found")
        
if have_joystick:
    j = pygame.joystick.Joystick(0)
    j.init()
    num_axes = j.get_numaxes()
    axes = [0.0] * num_axes
    num_buttons = j.get_numbuttons()
    buttons = [0] * num_buttons
    num_hats = j.get_numhats()
    hats = [0] * num_hats

def update():
    if not have_joystick:
        return None
    pygame.event.pump()
    for i in range(num_axes):
        axes[i] = j.get_axis(i)
    for i in range(num_buttons):
        buttons[i] = j.get_button(i)
    for i in range(num_hats):
        hats[i] = j.get_hat(i)
    print(axes, buttons, hats)
    return axes

    
