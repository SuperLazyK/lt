import graphic
import geometry
import numpy as np
#import course

def draw_line_segment(s, color=None):
    pa = s.p0 + geometry.normal(s.v0) * s.w2
    pb = s.p0 - geometry.normal(s.v0) * s.w2
    pc = s.p1 - geometry.normal(s.v0) * s.w2
    pd = s.p1 + geometry.normal(s.v0) * s.w2
    return graphic.draw_lineseg_cmd(pa, pb, color=color) + graphic.draw_lineseg_cmd(pb, pc, color=color) + graphic.draw_lineseg_cmd(pc, pd, color=color) + graphic.draw_lineseg_cmd(pd, pa, color=color)

def draw_arc_segment(s, color=None):
    pa = s.p0 + geometry.normal(s.v0) * s.w2
    pb = s.p0 - geometry.normal(s.v0) * s.w2
    pc = s.p1 - geometry.normal(s.v1) * s.w2
    pd = s.p1 + geometry.normal(s.v1) * s.w2
    if s.ccw:
        th0 = s.th0
        th1 = s.th1
    else:
        th0 = s.th1
        th1 = s.th0

    return graphic.draw_arcseg_cmd(s.c, s.r - s.w2, th0, th1, color=color) + graphic.draw_arcseg_cmd(s.c, s.r + s.w2, th0, th1, color=color) + graphic.draw_lineseg_cmd(pa, pb, color=color) + graphic.draw_lineseg_cmd(pc, pd, color=color)


def draw_segment(s):
    if geometry.is_line_segment(s):
        return draw_line_segment(s)
    elif geometry.is_arc_segment(s):
        return draw_arc_segment(s)

def draw_segments(segments):
    dos = []
    for s in segments:
        dos = dos + draw_segment(s)
    return dos

def draw_course(course):
    dos = draw_segments(course.segments)
    for o in course.marks:
        dos = dos + graphic.draw_circle_cmd(o.c, o.r, width=0, color=(180, 180, 180))
    return dos

def draw_course_designer(course):
    dos = draw_course(course)
    if course.current_seg is not None:
        if geometry.is_line_segment(course.current_seg):
            dos = dos + draw_line_segment(course.current_seg, color=(0, 255, 0))
        elif geometry.is_arc_segment(course.current_seg):
            dos = dos + draw_arc_segment(course.current_seg, color=(0, 0, 0))
    return dos

#def draw_model(model):
#
#    ret = graphic.draw_eqtri_cmd(np.array((model.x, model.y)), model.r, model.th, color=(122, 122, 255))
#
#    for sen, ob in zip(model.line_sensor, model.line_sensor_val):
#        ret = ret + graphic.draw_circle_cmd(sen, model.lss/2, color=(255, 0, 0) if ob else (122, 200, 40))
#    ret = ret + graphic.draw_circle_cmd(model.corner_sensor, model.lss/2, color=(255, 0, 0) if model.corner_sensor_val else (122, 200, 40))
#    ret = ret + graphic.draw_circle_cmd(model.goal_sensor, model.lss/2, color=(255, 0, 0) if model.goal_sensor_val else (122, 200, 40))
#
#    return ret
#
#def draw_controller(controller):
#    ret = []
#    if len(controller.samples) <= 1:
#        return []
#    #ret = ret + graphic.draw_lineseg_cmd(controller.samples[-1], controller.samples[-1] + 500*controller.line_dir, color=(200,100, 100))
#    #ret = ret + graphic.draw_circle_cmd(np.array([model.x, model.y]), 10*controller.r, color=(200,100, 100), width=1)
#    return ret
#
