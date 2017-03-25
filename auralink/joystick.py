# a simple locally connected (pc) joystick interface

have_pygame = False
have_joystick = False
num_axes = 0
axes = []

try:
    import pygame
    have_pygame = True
except:
    print "pygame import failed, joystick inactive"

if have_pygame:
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        have_joystick = True
    else:
        print "no joysticks found"
        
if have_joystick:
    j = pygame.joystick.Joystick(0)
    j.init()
    num_axes = j.get_numaxes()
    axes = [0.0] * num_axes

def update():
    if not have_joystick:
        return None
    pygame.event.pump()
    for i in range(num_axes):
        axes[i] = j.get_axis(i)
    return axes

    
