import numpy as np
import sys
import yaml
from pprint import pprint

def check_on_line(pos, start, end, d):
    vecl = end - start
    l = np.linalg.norm(vecl)
    v = vecl / l
    vecp = pos - start
    lv = vecp @ v
    if lv < 0 or lv > l:
        return False
    ld = np.linalg.norm(vecp - lv * v)
    return ld <= d

def check_on_arc(pos, origin, r, d, start_vec, end_vec):
    v = pos - origin
    x = np.linalg.norm(v)
    #print(np.cross(start_vec, v), np.cross(end_vec, v))
    #print("x,d,r,", r-d, x, r+d, r-d < x and x < r+d)
    if np.cross(start_vec, v) >= 0 and np.cross(end_vec, v) <= 0:
        return r-d < x and x < r+d
    return False

def noramalize(v):
    l = np.linalg.norm(v)
    return v/l

def calc_intersect(p0, v0, p1, v1):
    p0x = p0[0]
    p0y = p0[1]
    p1x = p1[0]
    p1y = p1[1]
    v0x = v0[0]
    v0y = v0[1]
    v1x = v1[0]
    v1y = v1[1]
    d = v0x*v1y-v0y*v1x
    if abs(d) < 0.01:
        return None
    x = (p1x*v0x*v1y+((p0y-p1y)*v0x-p0x*v0y)*v1x)/d
    y = (((p1x-p0x)*v0y+p0y*v0x)*v1y-p1y*v0y*v1x)/d
    return np.array([x, y])


def check_on_circle(pos, origin, r):
    x = np.linalg.norm(pos - origin)
    return x <= r

def circle_pos_vec_to_dir_vec(v, ccw=True):
    ret = np.array([-v[1], v[0]])
    l = np.linalg.norm(ret)
    if ccw:
        return ret / l
    else:
        return -ret / l

class Course():
    # list of (carvature = 1/R, length) <=> delta_theta = length / R
    def __init__(self, lw = 0.019, cm=0.02, mark_d=0.06):
        self.lw2 = lw/2.
        self.cm2 = cm/2.
        self.mark_d = mark_d
        self.segments = []
        self.popped = []
        self.marks = []
        self.current_seg = None
        self.current_pos = None

    def load(self, filepath):
        with open(filepath, 'r') as f:
            d = yaml.safe_load(f)
            self.lw2 = d['lw2']
            self.cm2 = d['cm2']
            self.mark_d = d['md']
            self.segments = d['segments']
            self.marks = d['marks']
            self.current_seg = None

    def save(self, filepath):
        d = {}
        d['lw2'] = self.lw2
        d['cm2'] = self.cm2
        d['md'] = self.mark_d
        d['segments'] = self.segments
        d['marks'] = self.marks
        with open(filepath, 'w') as f:
            yaml.dump(d, f, default_flow_style=False, allow_unicode=True)

    def draw_course(self):
        dos = []
        for o in self.segments:
            if o['type'] == 'arcseg':
                dos = dos + graphic.draw_arcseg_cmd(o['origin'], o['r'], o['start-th'], o['end-th'])
            elif o['type'] == 'lineseg':
                dos = dos + graphic.draw_lineseg_cmd(o['start'], o['end'])
        if self.current_seg is not None:
            o = self.current_seg
            if o['type'] == 'arcseg':
                dos = dos + graphic.draw_arcseg_cmd(o['origin'], o['r'], o['start-th'], o['end-th'], color=(255, 200, 200))
            elif o['type'] == 'lineseg':
                dos = dos + graphic.draw_lineseg_cmd(o['start'], o['end'], color=(200, 200, 250))
        for o in self.marks:
            dos = dos + graphic.draw_circle_cmd(o['origin'], o['r'], width=0, color=(180, 180, 180))
        return dos

    def push(self):
        if self.current_seg is not None:
            self.segments.append(self.current_seg)
            self.current_seg = None

    def undo(self):
        if len(self.segments) > 0:
            self.popped.append(self.segments.pop())

    def redo(self):
        if len(self.popped) > 0:
            self.segments.append(self.popped.pop())

    def debug(self):
        print("segments")
        pprint(self.segments)
        print("current_seg")
        pprint(self.current_seg)
        print("current_pos")
        pprint(self.current_pos)

    def clear(self):
        self.segments = []
        self.current_seg = None
        self.current_pos = None

    def set_start_point(self, point):
        if len(self.segments) == 0:
            self.current_pos = point
        else:
            self.current_pos = self.segments[-1]["end"]
        self.current_seg = None

    def try_add_line_segment(self, point):
        if self.current_pos is None:
            #print("line needs currnet_pos")
            return
        if len(self.segments) > 0 and self.segments[-1]['type'] == "lineseg":
            print("line needs a previoue curve")
            return
        direction = point - self.current_pos
        l = np.linalg.norm(direction)
        if l < 0.01:
            return
        self.current_seg = {"type":"lineseg", "start":self.current_pos, "end":point, "start-dir": direction/l, "end-dir": direction/l}

    def try_add_curve_segment(self, point):
        if self.current_pos is None:
            #print("curve needs currnet_pos")
            return
        if len(self.segments) == 0:
            self.try_add_line_segment(point)
            return
        x0 = self.current_pos[0]
        y0 = self.current_pos[1]
        x1 = point[0]
        y1 = point[1]
        dx = self.segments[-1]["end-dir"][0] #dx0
        dy = self.segments[-1]["end-dir"][1] #dy0
        start_dir = np.array([dx, dy])

        d = 2*(dx*(y1-y0)-dy*(x1-x0))

        if abs(d) < 0.02:
            self.try_add_line_segment(point)
            return

        cx = -(dy*y1**2+((-2*dy*y0)-2*dx*x0)*y1+dy*y0**2+2*dx*x0*y0+dy*x1**2-dy*x0**2)/d
        cy = (dx*y1**2-dx*y0**2+(2*dy*x0-2*dy*x1)*y0+dx*x1**2-2*dx*x0*x1+dx*x0**2)/d
        r = (np.sqrt(dy**2+dx**2)*(y1**2-2*y0*y1+y0**2+x1**2-2*x0*x1+x0**2))/abs(d)

        self.current_seg = self.gen_arc_segment(x0, y0, x1, y1, cx, cy, r, start_dir)


    def gen_arc_segment(self, x0, y0, x1, y1, cx, cy, r, start_dir):
        p0 = np.array([x0 - cx, y0 - cy])
        p1 = np.array([x1 - cx, y1 - cy])
        if np.cross(p0, start_dir) >= 0:
            dir1 = circle_pos_vec_to_dir_vec(p1)
            return {"type":"arcseg", "origin": np.array([cx, cy]), "r": r, "start":self.current_pos, "end":np.array([x1, y1]), "start-dir": start_dir, "end-dir": dir1,
                                "start-th" : np.arctan2(p0[1], p0[0]),
                                "end-th" : np.arctan2(p1[1], p1[0]),
                                }
        else:
            dir1 = circle_pos_vec_to_dir_vec(p1, ccw=False)
            return {"type":"arcseg", "origin": np.array([cx, cy]), "r": r, "start":self.current_pos, "end":np.array([x1, y1]), "start-dir": start_dir, "end-dir": dir1,
                                "start-th" : np.arctan2(p1[1], p1[0]),
                                "end-th" : np.arctan2(p0[1], p0[0]),
                                }

    def append_segment(self):
        if self.current_seg is not None:
            self.segments.append(self.current_seg)
        self.current_pos = None
        self.current_seg = None

    def try_close_loop_with_arc_line(self):
        if len(self.segments) < 2:
            return

        p1 = self.segments[0]["start"]
        v1 = self.segments[0]["start-dir"]
        p0 = self.segments[-1]["end"]
        v0 = self.segments[-1]["end-dir"]
        po = calc_intersect(p0, v0, p1, v1)
        if po is None:
            return

        # directional traverse between tangent point of 2 circles should be enabled
        if (p0 - po) @ (v0) > 0:
            point = po - np.linalg.norm(po - p0) * v1
        else:
            point = po + np.linalg.norm(po - p0) * v1

        # check point is "extension" of the original line

        self.try_add_curve_segment(point)

    def close_loop(self):
        self.segments[0]["start"] = self.segments[-1]["end"]

    def sample(self, points):
        ob = []
        for p in points:
            ret = False
            for seg in self.segments:
                if seg['type'] == 'lineseg':
                    ret = check_on_line(p, seg['start'], seg['end'], self.lw2)
                elif seg['type'] == 'arcseg':
                    ret = check_on_arc(p, seg['origin'], seg['r'], self.lw2, seg['start-dir'], seg['end-dir'])
                if ret:
                    break
            if not ret:
                for mark in self.marks:
                    ret = check_on_circle(p, mark['origin'], mark['r'])
                    if ret:
                        break
            ob.append(ret)

        return ob


if __name__ == '__main__':
    import graphic
    viewer = graphic.Viewer(scale=500)
    course = Course()

    def event_handler(key, type, args):
        if type == 'DOWN':
            if key == 'q':
                sys.exit()
            elif key == 's':
                course.save("course.yaml")
            #elif key == 'l':
            #    course.save("course.yaml")
            elif key == 'p':
                course.debug()
            elif key == 'u':
                course.undo()
            elif key == 'r':
                course.redo()
            elif key == 'LB':
                course.set_start_point(args["pos"])
            elif key == '1':
                viewer.scale = viewer.scale * 2
            elif key == '2':
                viewer.scale = viewer.scale * 0.5

        elif type == 'UP':
            if key == 'LB':
                if args["shift"]:
                    course.append_segment()
                    course.close_loop()
                else:
                    course.append_segment()
        elif type == 'MOVE':
            if args["shift"]:
                course.try_close_loop_with_arc_line()
            else:
                course.try_add_curve_segment(args["pos"])

    viewer.handle_event(event_handler)


    while True:
        viewer.clear()
        viewer.handle_event(event_handler)
        viewer.draw(course.draw_course())
        viewer.flush(30)

