import numpy as np

def normalize(v):
    l2 = np.linalg.norm(v)
    assert l2 > 0.0001
    return v/l2

# perfect vel tracking assumption
class LTModel():
    # driveing-dir: x
    # left-dir : y
    def __init__(self, wr = 0.01, d=0.1, r=0.05, lss=0.019/2, cm=0.06):
        self.wr = wr # wheel radius
        self.d = d # wheel distance
        self.r = r # body size
        self.lss = lss # line sensor span
        self.cm = cm # corner marker
        self.sensors_local = np.array([
            [r, -3 * lss + lss/2, 1],
            [r, -2 * lss + lss/2, 1],
            [r, -1 * lss + lss/2, 1],
            [r,  0 * lss + lss/2, 1],
            [r,  1 * lss + lss/2, 1],
            [r,  2 * lss + lss/2, 1],
            [0,  cm, 1],
            [0,  -cm, 1]
            ])
        self.clear()

    def clear(self):
        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dy = 0
        self.dth = 0
        self.update_sensor_pos()
        self.history = []

    def step_u(self, omega_l, omega_r, dt):
        omega_l, omega_r = u[0], u[1]
        vl = self.wr * omega_l
        vr = self.wr * omega_r
        v = (vl + vr) / 2
        new_dx = v * np.cos(self.th)
        new_dy = v * np.sin(self.th)
        new_dth = (vr - v) / self.d
        self.x = self.x + (new_dx + self.dx)/2*dt
        self.y = self.y + (new_dy + self.dy)/2*dt
        self.th = self.th + (new_dth + self.dth)/2*dt
        self.dx = new_dx
        self.dy = new_dy
        self.dth = new_dth
        self.update_sensor_pos()

    def step(self, ref_vel_forward, ref_vel_rotate, dt):
        new_dx = ref_vel_forward * np.cos(self.th)
        new_dy = ref_vel_forward * np.sin(self.th)
        new_dth = ref_vel_rotate 
        self.x = self.x + (new_dx + self.dx)/2*dt
        self.y = self.y + (new_dy + self.dy)/2*dt
        self.th = self.th + (new_dth + self.dth)/2*dt
        self.dx = new_dx
        self.dy = new_dy
        self.dth = new_dth
        self.update_sensor_pos()
        self.history.append([self.x, self.y, self.th, self.dx, self.dy, self.dth, self.line_sensor, self.corner_sensor])

    def pop(self):
        [self.x, self.y, self.th, self.dx, self.dy, self.dth, self.line_sensor, self.corner_sensor] = self.history.pop(-1)

    def update_sensor_pos(self):
        c = np.cos(self.th)
        s = np.sin(self.th)
        R = np.array([
            [ c, -s, self.x],
            [ s, c, self.y],
            [ 0, 0, 1]
            ])
        sensors = (R @ self.sensors_local.T).T
        self.line_sensor = sensors[:-2,:-1]
        self.corner_sensor = sensors[-2,:-1]
        self.goal_sensor = sensors[-1,:-1]

    def observe(self, f):
        self.line_sensor_val = f(self.line_sensor)
        self.corner_sensor_val = f([self.corner_sensor])[0]
        self.goal_sensor_val = f([self.goal_sensor])[0]

    def debug(self):
        print("model.x=", self.x)
        print("model.y=", self.y)
        print("model.th=", self.th)
        print("model.dx=", self.dx)
        print("model.dy=", self.dy)
        print("model.dth=", self.dth)

    def draw_model(self, model):

        ret = graphic.draw_eqtri_cmd(np.array((model.x, model.y)), model.r, model.th, color=(122, 122, 255))

        for sen, ob in zip(model.line_sensor, model.line_sensor_val):
            ret = ret + graphic.draw_circle_cmd(sen, model.lss/2, color=(255, 0, 0) if ob else (122, 200, 40))
        ret = ret + graphic.draw_circle_cmd(model.corner_sensor, model.lss/2, color=(255, 0, 0) if model.corner_sensor_val else (122, 200, 40))
        ret = ret + graphic.draw_circle_cmd(model.goal_sensor, model.lss/2, color=(255, 0, 0) if model.goal_sensor_val else (122, 200, 40))

        return ret


class LTController():
    def __init__(self, model, ref_vel = 0.06):
        self.model = model
        self.samples = []
        self.segment = [0] # index of samples
        self.xi = 0.06
        self.already_reset = False
        self.I = 0

    def check_corner(self):
        if self.model.corner_sensor_val and not self.already_reset:
            self.already_reset = True
            self.segment.append(len(self.samples))
        elif not self.model.corner_sensor_val and self.already_reset:
            self.already_reset = False

    def add_sample(self):
        sens = []
        for i,v in enumerate(self.model.line_sensor_val):
            if v:
                sens.append(self.model.line_sensor[i])
        if len(sens) < 2:
            print("WARNING!! no line is detected!!")
            #self.model.debug()
            return False

        new_pos = (sens[0] + sens[-1])/2

        if len(self.samples) == 0:
            self.samples.append(new_pos)
        else:
            delta = new_pos - self.samples[-1]
            dv = np.linalg.norm(delta)
            if dv < 0.005:
                return False
            self.samples.append(new_pos)
        return True

    # ref is always 0. Kd = 0
    def pi(self, ref_vel_forward, Kp=10, Ki=1.15):

        if self.already_reset:
            self.I = 0

        o = np.array([self.model.x, self.model.y])
        sample = self.samples[-1]
        target_dir = normalize(sample - o)
        current_dir = np.array([np.cos(self.model.th), np.sin(self.model.th)])
        err = np.cross(current_dir, target_dir)
        err = np.arcsin(err)
        self.I = self.I + err

        return ref_vel_forward * (Kp * err + Ki * self.I)


    def simple(self, ref_vel_forward, dt):
        self.r = ref_vel_forward * dt
        o = np.array([self.model.x, self.model.y])

        if len(self.samples) <= 1:
            ref_vel = np.array([np.cos(self.model.th), np.sin(self.model.th)]) * ref_vel_forward
            ref_pos = o + ref_vel * dt
            ref_acc = np.array([0, 0])
        else:
            print("debug----")
            print("o", o)
            p1 = self.samples[-2]
            p2 = self.samples[-1]
            print(self.samples)
            print("p1", p1)
            print("p2", p2)

            line_dir = (p2 - p1) / np.linalg.norm(p2-p1) # vec of line
            self.line_dir = line_dir
            print("line_dir", line_dir)
            c = line_dir[0]
            s = line_dir[1]

            vert_dir = np.array([-s, c]) # vec from origin to line
            if np.dot(vert_dir, p1 - o) < 0:
                vert_dir = -vert_dir
            print("vert_dir", vert_dir)

            pc = (p1 + p2)/2
            d = s * pc[0] - c * pc[1]
            print("d", d)
            # estimated line: - s x + c y + d = 0

            dist_vert_dir = -s * o[0] + c * o[1] + d
            print("dist_vert_dir", dist_vert_dir)
            assert self.r > dist_vert_dir
            dist_line_dir = (self.r*self.r - dist_vert_dir * dist_vert_dir) ** 0.5
            print("dist_line_dir", dist_line_dir)

            ref_pos = o + vert_dir * dist_vert_dir + line_dir * dist_line_dir
            print("ref_pos", ref_pos)
            ref_vel = line_dir * ref_vel_forward
            ref_pos = self.samples[-1]
            ref_acc = np.array([0, 0])

        return ref_pos, ref_vel, ref_acc


    def pos2vel(self, ref_pos, ref_vel, ref_acc, dt, Kp=1000, Kv=10):
        pos = np.array([self.model.x, self.model.y])
        vel = np.array([self.model.dx, self.model.dy])
        u = ref_acc + Kp * (ref_pos - pos) + Kv * (ref_vel - vel)
        ux = u[0]
        uy = u[1]
        c = np.cos(self.model.th)
        s = np.sin(self.model.th)
        uv = self.xi + (ux * c + uy * s) * dt
        uw = (uy * c - ux * s) / uv
        print("uv", uv, np.clip(uv, -0.4, 0.4))
        print("uw", uw, np.clip(uw, -2.8, 2.8))
        uv = np.clip(uv, -0.4, 0.4)
        uw = np.clip(uw, -2.8, 2.8)
        self.xi = uv
        if abs(self.xi) < 0.01:
            return 0, 0
        else:
            return uv, uw

    def draw_controller(self, controller):
        ret = []
        if len(controller.samples) <= 1:
            return []
        #ret = ret + graphic.draw_lineseg_cmd(controller.samples[-1], controller.samples[-1] + 500*controller.line_dir, color=(200,100, 100))
        #ret = ret + graphic.draw_circle_cmd(np.array([model.x, model.y]), 10*controller.r, color=(200,100, 100), width=1)
        return ret







