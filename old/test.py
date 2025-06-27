import course
import lt
import graphic
from draw import *
import sys
import numpy as np

# simulation
viewer = graphic.Viewer(scale=500)



def event_handler(key, type, args):
    if type == 'DOWN':
        if key == 'q':
            sys.exit()
        elif key == 'w':
            mode = 'manual-vel'
            ref_vel_forward = 0.06*2
            ref_pos = None
        elif key == 'a':
            mode = 'manual-vel'
            ref_vel_rotate = 0.2*4
            ref_pos = None
        elif key == 'd':
            mode = 'manual-vel'
            ref_vel_rotate = -0.2*4
            ref_pos = None
        elif key == 's':
            stepFlag = True
            runFlag = True
        elif key == 'r':
            runFlag ^= True
        elif key == 'c':
            mode = 'auto'
        elif key == 'p':
            model.debug()
            print("ref_pos=", ref_pos)
        elif key == 'LB':
            ref_pos = args
            mode = 'manual-pos'
        elif key == 'b':
            model.pop()
            controller.samples.pop(-1)
    else:
        ref_vel_forward = 0
        ref_vel_rotate = 0



while True:
    viewer.clear()
    model.observe(course.sample)
    viewer.draw(draw_course(course))
    viewer.draw(draw_model(model))
    viewer.draw(draw_controller(controller))
    viewer.handle_event(event_handler)

    controller.add_sample()
    controller.check_corner()

    if runFlag:
        if mode == 'auto':
            #ref_pos, ref_vel, ref_acc = controller.simple(0.06, dt*100)
            #ref_vel_forward, ref_vel_rotate = controller.pos2vel(ref_pos, ref_vel, ref_acc, dt)
            ref_vel_forward = 0.20
            ref_vel_rotate = controller.pi(ref_vel_forward)
        elif mode == 'manual-pos':
            ref_vel = np.array([0, 0])
            ref_acc = np.array([0, 0])
            ref_vel_forward, ref_vel_rotate = controller.pos2vel(ref_pos, ref_vel, ref_acc, dt)
        model.step(ref_vel_forward, ref_vel_rotate, dt)
        #model.debug()
    if stepFlag:
        runFlag = False
        stepFlag = False

    if ref_pos is not None:
        viewer.draw(graphic.draw_circle_cmd(ref_pos, 0.005, color=(130,190,255)))
    viewer.flush(30)


