import numpy as np

EPSILON_LEN = 0.001
EPSILON_RAD = 0.001



def enough_length(v):
    l = np.linalg.norm(v)
    return l > EPSILON_LEN

def normalize(v):
    l = np.linalg.norm(v)
    return v / l

def normal(v, ccw=True):
    ret = np.array([-v[1], v[0]])
    l = np.linalg.norm(ret)
    if ccw:
        return ret / l
    else:
        return -ret / l


def check_parallel(p0, v0, p1, epsilon=EPSILON_RAD):
    return abs(np.cross(v0, normalize(p1-p0))) < epsilon


def check_on_line(pos, start, end, epsilon=EPSILON_LEN):
    vecl = end - start
    l = np.linalg.norm(vecl)
    v = vecl / l
    vecp = pos - start
    lv = vecp @ v
    if lv < 0 or lv > l:
        return False
    ld = np.linalg.norm(vecp - lv * v)
    return ld <= epsilon


class LineSegment():
    def __init__(self, p0, p1, v, w2):
        self.p0 = p0
        self.p1 = p1
        self.v0 = v # direction
        self.v1 = v # direction
        self.w2 = w2 # half of line width

    def sample(self, pos):
        return check_on_line(pos, self.p0, self.p1, self.w2)

    def translation(self, pos):
        self.p0 = self.p0 + pos
        self.p1 = self.p1 + pos

    def to_dict(self):
        return {"type":"lineseg", "p0":self.p0.tolist(), "p1":self.p1.tolist(), "v": self.v0.tolist(), "w2": float(self.w2)}


def gen_line_segment_p0_p1(p0, p1, w2 = 0.019):
    if not enough_length(p1-p0):
        return None
    return LineSegment(p0, p1, normalize(p1-p0), w2)


def gen_line_segment_dict(d):
    return LineSegment(np.array(d["p0"]), np.array(d["p1"]), np.array(d["v"]), d["w2"])


def calc_intersect_of_2_lines(p0, v0, p1, v1, epsilon=EPSILON_RAD):
    p0x = p0[0]
    p0y = p0[1]
    p1x = p1[0]
    p1y = p1[1]
    v0x = v0[0]
    v0y = v0[1]
    v1x = v1[0]
    v1y = v1[1]
    d = v0x*v1y-v0y*v1x
    if abs(d) < epsilon:
        return None
    x = (p1x*v0x*v1y+((p0y-p1y)*v0x-p0x*v0y)*v1x)/d
    y = (((p1x-p0x)*v0y+p0y*v0x)*v1y-p1y*v0y*v1x)/d
    return np.array([x, y])


# hassle to handle 2 pi periodic things. using slerp is better?
# check exist t in [0,1]. s.t. c0 ^ t c1 ^(1-t) = c
#  (c0 / c1)^t = c / c1
#  angle function return log with (-pi, pi]
#  check if angle(c / c1) is between 0 and angle(c0 / c1)
#  note:
#    cross product can tell only left/right half plane of the vector
#    uneasy to handle more than 180 dec rotation
def check_on_arc(pos, origin, r, th0, th1, ccw, epsilon):
    assert False, "not implemented"
    v = pos - origin
    d = np.linalg.norm(v)
    th = np.arctan2(v[1], v[0])

    # check in circle
    if abs(d - r) > epsilon:
        return False

    if ccw and th0 > th1:
        th0 = th0 + 2 * np.pi
    elif not ccw and th0 < th1:
        th1 = th1 + 2 * np.pi

    return th0 <= th and th <= th1



class ArcSegment():
    def __init__(self, c, r, p0, v0, th0, p1, v1, th1, ccw, w2):
        self.c = c
        self.r = r
        self.p0 = p0
        self.v0 = v0
        self.th0 = th0
        self.p1 = p1
        self.v1 = v1
        self.th1 = th1
        self.ccw = ccw
        self.w2 = w2

    def sample(self, pos):
        return check_on_arc(pos, self.c, self.r, self.th0, self.th1, self.ccw, self.w2)

    def to_dict(self):
        return {"type":"arcseg", "c":self.c.tolist(), "r":float(self.r),
                "p0":self.p0.tolist(), "v0": self.v0.tolist(), "th0": float(self.th0),
                "p1":self.p1.tolist(), "v1": self.v1.tolist(), "th0": float(self.th1),
                "ccw":bool(self.ccw), "w2": float(self.w2)}

    def translation(self, pos):
        self.p0 = self.p0 + pos
        self.p1 = self.p1 + pos
        self.c = self.c + pos


def gen_arc_segment_dict(d):
    return ArcSegment(np.array(d["c"]), d["r"], np.array(d["p0"]), np.array(d["v0"]), np.array(d["p1"]), np.array(d["v1"]), np.array(d["ccw"]), d["w2"])


# start point, start direction, end point
def gen_arc_segment(p0, v0, p1, w2=0.019, epsilon = EPSILON_RAD): # epsilon radian

    if not enough_length(p1-p0):
        return None

    if check_parallel(p0, v0, p1, epsilon):
        return None

    x0 = p0[0]
    y0 = p0[1]
    x1 = p1[0]
    y1 = p1[1]
    dx = v0[0]
    dy = v0[1]
    vx = x1-x0
    vy = y1-y0
    d = 2*(dx*vy-dy*vx)

    cx = -(dy*y1**2+((-2*dy*y0)-2*dx*x0)*y1+dy*y0**2+2*dx*x0*y0+dy*x1**2-dy*x0**2)/d
    cy = (dx*y1**2-dx*y0**2+(2*dy*x0-2*dy*x1)*y0+dx*x1**2-2*dx*x0*x1+dx*x0**2)/d
    r = (np.sqrt(dy**2+dx**2)*(y1**2-2*y0*y1+y0**2+x1**2-2*x0*x1+x0**2))/abs(d)

    center = np.array([cx, cy])
    cp0 = np.array([x0 - cx, y0 - cy])
    cp1 = np.array([x1 - cx, y1 - cy])

    # p1 is in left/right half plane of p0-v0
    ccw = np.cross(v0, p1-p0) >= 0 # ???
    v1 = normal(cp1, ccw)

    th0 = np.arctan2(cp0[1], cp0[0])
    th1 = np.arctan2(cp1[1], cp1[0])

    return ArcSegment(center, r, p0, v0, th0, p1, v1, th1, ccw, w2)


# note: tangent line of end point of the arc is same as start line
# p1 v1 : start line segment
def close_loop_point(p0, v0, p1, v1, w2):
        # step1 calc end of arc
        # intersection of 2 tangent line of the given circle
        po = calc_intersect_of_2_lines(p0, v0, p1, v1)
        if po is None:
            return None

        # check point is "extension" of the original line
        # directional traverse between tangent point of 2 circles should be enabled
        if (p0 - po) @ (v0) > 0:
            point = po - np.linalg.norm(po - p0) * v1
        else:
            point = po + np.linalg.norm(po - p0) * v1

        return point


def check_in_circle(pos, center, r):
    x = np.linalg.norm(pos - center)
    return x <= r

class Circle():
    def __init__(self, c, r):
        self.c = c
        self.r = r

    def sample(self, pos):
        return check_in_circle(self.c, self.r)

    def to_dict(self):
        return {"type":"circle", "c":self.c.tolist(), "r": float(self.r)}

    def translation(self, pos):
        self.c = self.c + pos

def gen_circle_dict(d):
    return ArcSegment(np.array(d["c"]), d["r"])


# start point, start direction, end point
def gen_circle(c, r): # epsilon radian
    return Circle(c, r)

def gen_circle_d_r_aside_line(d, r, p, v, ccw=True):
    c = p - normal(v, ccw) * d
    return gen_circle(c, r)

def is_arc_segment(s):
    return isinstance(s, ArcSegment)

def is_line_segment(s):
    return isinstance(s, LineSegment)

if __name__ == '__main__':

    p0 = np.array([0.32,  0.512])
    v0 = np.array([-0.07417526,  0.99724522])
    p1 = np.array([-0.4963522,  -0.29448918])
    print(check_parallel(p0, v0, p1))




