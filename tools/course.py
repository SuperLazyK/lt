import math
import sys

def lineseg(d):
    return (0, d)

def arcseg(r, dth):
    a = 1.0/r
    l = math.radians(dth)/a
    return (1.0/r, l)


def gen_course():
    segments = []
    segments.append(lineseg(0.5))
    segments.append(arcseg(0.5, 180))
    segments.append(lineseg(1))
    segments.append(arcseg(0.8, 180))
    segments.append(lineseg(0.5))
    segments.append(arcseg(0.3, 90))
    segments.append(lineseg(0.5))
    segments.append(arcseg(0.3, 90))
    segments.append(lineseg(0.5))
    segments.append(arcseg(0.25, 180))
    segments.append(lineseg(0.5))
    return segments


class Course():
    # 2-1 コースの走行面は黒色とし、コースは、幅1.9cm の白色のラインで示された周回コースである。ラインの全長は60m以下とする。
    # 2-2 ラインは、直線と円弧の組合せにより構成される。ラインは交差することがある。
    # 2-3 ラインを構成する円弧の曲率半径は、ラインの中心を基準に10cm以上とする。また、曲率変化点間の距離は10cm以上とする。
    # 2-4 ラインが交差するとき、交差の角度は90度±5度とする。(図1 参照) ラインが交差する点の前後10cmは、ラインは直線とする。
    # list of (carvature = 1/R, length) <=> delta_theta = length / R
    def __init__(self, segments=gen_course()):
        self.segments = segments

        segments = []
        marks = []
        mark_d = 0.05
        x0 = 0
        y0 = 0
        th0 = 0
        for seg in self.segments:
            a = seg[0]
            l = seg[1]
            if a == 0: # line segment
                x1 = x0 + l * math.cos(th0)
                y1 = y0 + l * math.sin(th0)
                th1 = th0
                segments.append({"type":"lineseg", "start":(x0,y0), "end":(x1, y1)})
            else: # curve
                r = 1.0 / a
                assert abs(r) >= 0.1
                dth = l * a
                th1 = th0 + dth
                # center
                if a > 0:
                    cx = x0 - r * math.sin(th0)
                    cy = y0 + r * math.cos(th0)
                    x1 = cx + r * math.sin(th1)
                    y1 = cy - r * math.cos(th1)
                else:
                    cx = x0 + r * math.sin(th0)
                    cy = y0 - r * math.cos(th0)
                    x1 = cx + r * math.sin(th1)
                    y1 = cy - r * math.cos(th1)
                segments.append({"type":"arcseg", "origin":(cx,cy), "r":r, "start":th0-math.pi/2, "end":th1-math.pi/2})
                marks.append({"type":"circle", "origin":((mark_d * cx + (r-mark_d) * x0) / r, (mark_d * cy + (r-mark_d) * y0) / r), "r":mark_d/2})
                marks.append({"type":"circle", "origin":((mark_d * cx + (r-mark_d) * x1) / r, (mark_d * cy + (r-mark_d) * y1) / r), "r":mark_d/2})
            x0 = x1
            y0 = y1
            th0 = th1

        self.segments = segments
        self.marks = marks


if __name__ == '__main__':
    import graphic
    viewer = graphic.Viewer(scale=200)
    segs = Course()

    #print(segs.segments)
    #print(segs.marks)
    dos = []
    dos = dos + graphic.draw_geos(segs.segments)
    dos = dos + graphic.draw_geos(segs.marks)

    def event_handler(key, type, shifted):
        if key == 'q':
            sys.exit()

    while True:
        viewer.handle_event(event_handler)
        viewer.clear()
        viewer.draw(dos)
        #viewer.draw_horizon(0)
        viewer.flush(30)


