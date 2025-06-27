import numpy as np
import sys
import yaml
from pprint import pprint
import geometry

###############################################
# segment: (start point, start direction, end point, end direction)
#  - line: start point, end point
#  - arc: center, r, th-start, th-end
#  - sample()
#  - curvature is not used
#
# segment estimator : (start point, start direction, curvature)
#  - length is unknown : infinite
#  - variance of curvature: reset to inf for first estimation
#  - update_curvature() at edge detection
#
# course : collection of line/arc segment
#  - add segment()
#  - save/load()
#
# course designer :
#
# stakeholder:
#  - designer: yaml-io
#  - drawer: center, radius
#  - simulator: sensing
#  - mapper(estimator) : loop closing (length)
#     line/arc should be treated in the same way
#      curvature changes continuously
#      (carvature = 1/R, length) <=> delta_theta = length / R
#  - runner
#
###############################################

def gen_simple_course(r):
    seg = geometry.ArcSegment(np.zeros(2), r, np.array([0, -r]), np.array([1, 0]), 0, np.array([0, -r]), np.array([1, 0]), 2*np.pi - 0.01, True, 0.019/2)
    return LTCourse([seg])

class LTCourse():
    def __init__(self, segments, marks = []):
        self.segments = segments
        self.marks = marks

    # IR sensor is on/off
    def sample(self, points):
        ob = []
        for p in points:
            ret = False
            for seg in self.segments:
                ret = seg.sample(p)
                if ret:
                    break
            if not ret:
                for mark in self.marks:
                    ret = check_on_circle(p, mark.c, mark.r)
                    if ret:
                        break
            ob.append(ret)
        return ob

    def to_dict(self):
        return {"segments": [s.to_dict() for s in self.segments], "marks": [s.to_dict() for s in self.marks]}


def gen_course_dict(d):
     ret = LTCourse(d['segments'], d['marks'])
     #assert np.linalg.norm(ret.segments[0]["start"] - ret.segments[-1]["end"]) < 0.001, "course is not closed"
     return ret


def gen_course_yaml(filepath):
    with open(filepath, 'r') as f:
        d = yaml.safe_load(f)
        return gen_coutse_dict(d)


def save_course_yaml(filepath, c):
    with open(filepath, 'w') as f:
        yaml.dump(c.to_dict(), f, default_flow_style=False, allow_unicode=True)


class LTCourseDesigner():
    def __init__(self, lw = 0.019, cm=0.02, mark_d=0.06):
        self.lw2 = lw/2.
        self.cm2 = cm/2.
        self.mark_d = mark_d
        self.history = []
        self.redo_buf = []
        self.set_segments([])

    def update_marks(self):
        self.marks = [geometry.gen_circle_d_r_aside_line(self.mark_d, self.cm2, s.p0, s.v0) for s in self.segments]

    def cancel(self):
        self.current_seg = None
        self.current_pos = None

    def set_segments(self, segs):
        self.segments = segs
        self.current_seg = None
        self.current_pos = None
        self.update_marks()

    def save(self, filepath):
        c = LTCourse(self.segments, self.marks)
        save_course_yaml(filepath, c)

    def undo(self):
        if len(self.history) > 0:
            self.redo_buf.append(self.segments.copy())
            self.set_segments(self.history.pop())

    def redo(self):
        if len(self.redo_buf) > 0:
            self.set_segments(self.redo_buf.pop())

    def clear(self):
        self.set_segments([])

    def append_segment(self):
        if self.current_seg is not None:
            self.history.append(self.segments.copy())
            #print(vars(self.current_seg))
            self.segments.append(self.current_seg)
            self.update_marks()
        self.current_pos = None
        self.current_seg = None

    def set_start_point(self, point):
        if len(self.segments) == 0:
            self.current_pos = point
        else:
            self.current_pos = self.segments[-1].p1
        self.current_seg = None


    def set_current_line_seg(self, point):
        if self.current_pos is None:
            print("line needs currnet_pos")
            return

        # todo merge line seg
        if len(self.segments) > 0 and geometry.is_line_segment(self.segments[-1]):
            #print("line needs a previoue arc")
            return

        seg = geometry.gen_line_segment_p0_p1(self.current_pos, point, self.lw2)
        if seg is None:
            print("too short line")
            return

        self.current_seg = seg

    def set_current_arc_seg(self, point):
        if self.current_pos is None:
            return

        if len(self.segments) == 0:
            self.set_current_line_seg(point)
            return

        if geometry.check_parallel(self.current_pos, self.segments[-1].v1, point, epsilon=np.pi/180 * 20):
            d = np.dot(point - self.current_pos, self.segments[-1].v1)
            if d <= 0:
                return
            point = self.current_pos + d * self.segments[-1].v1
            self.set_current_line_seg(point)
            return

        arc = geometry.gen_arc_segment(self.current_pos, self.segments[-1].v1, point, self.lw2)

        if arc is not None:
            self.current_seg = arc

    def try_close_loop_with_arc_line(self):
        if len(self.segments) < 2:
            return

        if self.current_pos is None:
            self.current_pos = self.segments[-1].p1

        point = geometry.close_loop_point(self.segments[-1].p1, self.segments[-1].v1, self.segments[0].p0, self.segments[0].v0, self.lw2)

        if point is None:
            print("fail to close")
            return

        self.set_current_arc_seg(point)

    def close_loop(self):
        self.segments[0].p0 = self.segments[-1].p1
        self.offset_center()

    def offset_center(self):
        c = np.zeros(2)
        for seg in self.segments:
            c = c + seg.p0
        c = c / len(self.segments)

        for seg in self.segments:
            seg.translation(-c)

        for seg in self.marks:
            seg.translation(-c)

        if self.current_pos is not None:
            self.current_pos = self.current_pos - c

        if self.current_seg is not None:
            self.current_seg.transition(-c)

if __name__ == '__main__':
    #def ndarray_representer(dumper: yaml.Dumper, array: np.ndarray) -> yaml.Node:
    #    return dumper.represent_list(array.tolist())

    #yaml.add_representer(np.ndarray, ndarray_representer)

    import graphic
    import draw
    viewer = graphic.Viewer(scale=500)
    cd = LTCourseDesigner()

    def event_handler(key, type, args):
        if type == 'DOWN':
            if key == 'q':
                sys.exit()
            elif key == 's':
                cd.save("course.yaml")
            elif key == 'u':
                cd.undo()
            elif key == 'r':
                cd.redo()
            elif key == 'd':
                cd.clear()
            elif key == 'c':
                cd.offset_center()
            elif key == 'LB':
                if args["shift"]:
                    cd.try_close_loop_with_arc_line()
                else:
                    cd.set_start_point(args["pos"])
            elif key == ';':
                viewer.scale = viewer.scale * 2
            elif key == '-':
                viewer.scale = viewer.scale * 0.5
        elif type == 'UP':
            if key == 'LB':
                cd.append_segment()
                if args["shift"]:
                    cd.close_loop()
                    cd.save("course.yaml")
            elif key == 'RB':
                cd.cancel()
        elif type == 'MOVE':
            if args["shift"]:
                cd.try_close_loop_with_arc_line()
            else:
                cd.set_current_arc_seg(args["pos"])

    viewer.handle_event(event_handler)


    while True:
        viewer.clear()
        viewer.handle_event(event_handler)
        viewer.draw(draw.draw_course_designer(cd))
        viewer.flush(30)




