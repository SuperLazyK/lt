import numpy as np

# communication between controller and world with sensor / actuator

# sensor values
# simulator write, controller read
class LTSensor:
    def __init__(self):
        self.imu = None
        self.irs = None # IR sensors
        self.ms = None # magnetic sensor

    def read(self):
        return self.imu, self.irs, self.ms

    def write(self, imu, irs, ms):
        self.imu = imu,
        self.irs = irs,
        self.ms = msr


# control values
# controller write, simulator read
class LTActuator:
    def __init__(self):
        self.trans_speed = 0
        self.rotate_speed = 0

    def read(self):
        return self.trans_speed, self.rotate_speed

    def write(self, trans_speed, rotate_speed):
        self.trans_speed = trans_speed
        self.rotate_speed = rotate_speed


# sensor position calculation from model state
# est: controller read/write
# true: simulator read/write
# direction type is complex
class LTModel():
    def __init__(self, wr = 0.01, d=0.1, r=0.05, lss=0.019/2, cm=0.06):
        self.wr = wr # wheel radius
        self.d = d # wheel distance
        self.r = r # body size
        self.lss = lss # line sensor span
        self.cm = cm # corner marker, start/end marker distance
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
        self.q = 0 + 0j
        self.dx = 0
        self.dy = 0
        self.update_sensor_pos()

    def get_state(self):
        return self.x, self.y , self.q, self.dx, self.dy

    def set_state(self, x, y, q, dx, dy):
        self.x = x
        self.y = y
        self.q = q / abs(q) # normalize just in case
        self.dx = dx
        self.dy = dy
        self.update_sensor_pos()

    def get_sensor_pos(self):
        return self.sensor_pos

    def update_sensor_pos(self):
        c = self.q.real
        s = self.q.imag
        R = np.array([
            [ c, -s, self.x],
            [ s, c, self.y],
            [ 0, 0, 1]
            ])
        self.sensor_pos = (R @ self.sensors_local.T).T
        #self.line_sensor = sensors[:-2,:]
        #self.corner_sensor = sensors[-2,:]
        #self.goal_sensor = sensors[-1,:]


# simulation of motor + world (true state)
# sampling noise for bias, slip, sensing, delay, etc..
class LTSimulator:
    def __init__(self, actuator, sensor, true_map):
       self.random_seed = 0
       # true state
       self.model = LTModel()
       #self.ab = np.zeros(2)  # bias of accelerometer
       #self.wb = 0  # bias of gyro
       self.actuator = actuator
       self.sensor = sensor
       self.map = true_map

    def step(self, dt):
        # add control_noise
        # add control delay
        # sampling bias noise
        # sampling slip
        pass


class LTController:
    def __init__(self, actuator, sensor, default_speed):
       self.actuator = actuator
       self.sensor = sensor
       self.xi = default_speed # current transition velocity

    #def update_observer(self):
    #    # get sensor values
    #    # step observer

    #def calc_ref_pos(self):
    #    # calc reference line from map and current pos
    #    # pursuit style

    def step(dt):
        pass
        # update observer
        # get current estimate position/vel
        # check corner mark
        # update Map
        # calc ref pos
        # calc ref vel from ref pos (PI control with xi)



## Localization
#class EKF:
#    def __init__(self, sensor):
#        self.sensor = sensor
#        NominalState()
#        ErrorState()
#
#    def step(observed):
#        pass
#
## Mapping
## update map with observation and estimated pos
## NOTE: correct position in EKF
#class Mapper:
#    def update():
#        pass
#
#

if __name__ == '__main__':

    default_speed = 0.06
    dt = 0.01

    import course
    sensor = LTSensor()
    actuator = LTActuator()
    true_map = course.gen_simple_course(0.5)
    simulator = LTSimulator(actuator, sensor, true_map)
    controller = LTController(actuator, sensor, default_speed)

    #while True:
    #    # update sensor output
    #    simulator.observe() 

    #    # update actuator input
    #    controller.step()

    #    # update true state
    #    simulator.step(dt)

    #    #draw(simulator)
    #    #draw(controller)
