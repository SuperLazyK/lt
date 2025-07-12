from math import sin, cos
import numpy as np

#-------------------------------------------
# state
#-------------------------------------------

# u/w -> langtitude/lattitude
# omega yaw
# L/R  wheel L/R


#-------------------------------------------
# block
#-------------------------------------------

def plant_car(tauL, tauR, x, y, th, vu, vw, omega, Ddvu, Ddomega, dt):

    dvu = (M*R*c*omega**2 + tauL + tauR)/(M*R) + Ddvu
    ddth = (-L2*tauL + L2*tauR - M*R*c*omega*vu)/(J*R + M*R*c**2) + Ddomega
    dvw = c * ddth

    vx =  vu * cos(th) - vw * sin(th)
    vy =  vu * sin(th) + vw * cos(th)

    x = x + vx/2*dt
    y = y + vy/2*dt
    th = th + omega/2*dt

    vu = vu + dvu/2*dt
    vw = vw + dvw/2*dt
    omega = omega + ddth/2*dt

    return x, y, th, vu, vw, omega, dvu, dvw, ddth


def plant_motor(V, dphi, tau, dt):
    dtau = (-Kb*Kt*dphi + Kt*V - Rm*tau)/Lm
    tau = tau + dtau/2 * dt
    return tau


def kinematics_v_omega2dphi(vu, omega):
    dphiL = (-L2*omega - L2*omega + 2*vu)/(2*R)
    dphiR = (L2*omega + L2*omega + 2*vu)/(2*R)
    return dphiL, dphiR

# round noise
def measure_enc(vu, omega):
    return kinematics_v_omega2dphi(vu, omega)


# noise
def measure_IMU(dvu, dvw, dth, ddth):
    dvu_IMU = dvu - ddth * rw_IMU + dth**2 * ru_IMU
    dvw_IMU = dvw + ddth * ru_IMU + dth**2 * rw_IMU
    ddth_IMU = ddth
    return dvu_IMU, dvw_IMU, ddth_IMU


# basically, delta ref is regarded as 0 for stability
def controller_PD_motor(ref_vu, ref_omega, dvu, ddth):
    K1s = Ku*Lm*M*R/(2*Kt)
    K2s = -J*Kw*Lm*R/(Kt*L)
    K3s = Ku*Lm*M*R/(2*Kt)
    K4s = J*Kw*Lm*R/(Kt*L)

    K1ns = (2*Kb*Kt*Ku + Ku*M*R**2*Rm)/(2*Kt*R)
    K2ns = (-2*J*Kw*R**2*Rm - Kb*Kt*Kw*L**2)/(2*Kt*L*R)
    K3ns = (2*Kb*Kt*Ku + Ku*M*R**2*Rm)/(2*Kt*R)
    K4ns = (2*J*Kw*R**2*Rm + Kb*Kt*Kw*L**2)/(2*Kt*L*R)

    VL = K1s * dvu + K1ns * ref_vu + K2s * ddth + K2ns * ref_omega
    VR = K3s * dvu + K3ns * ref_vu + K4s * ddth + K4ns * ref_omega

    return VL, VR


def nonlinear_term_compensator(vu, omega, dvu, ddth):
    d1 = c * omega **2
    d2 = c * vu * omega
    dd1 = 2 * c * omega * ddth
    dd2 = c * (dvu * omega + vu * ddth)
    VLc = L2*M*R*(-Lm*dd1 - Rm * d1)/(Kt*(L)) + (-M*R*(Lm*dd2 + Rm*d2)/(Kt*(L)))
    VRc = L2*M*R*(-Lm*dd1 - Rm * d1)/ (Kt*(L)) * d1 + M*R*(Lm*dd2 + Rm*d2)/(Kt*(L))
    return VLc, VRc


# open loop
def openloop(ref_vu, ref_omega, x, y, th, vu, vw, omega, tauL, tauR, dvu, dvw, ddth, Ddvu, Ddomega, dt):

    VL, VR = controller_PD_motor(ref_vu, ref_omega, dvu, ddth)
    cVL, cVR = nonlinear_term_compensator(vu, omega, dvu, ddth)
    dphiL, dphiR = kinematics_v_omega2dphi(vu, omega)

    tauL = plant_motor(VL + cVL, dphiL, tauL, dt)
    tauR = plant_motor(VR + cVR, dphiR, tauR, dt)
    x, y, th, vu, vw, omega, dvu, dvw, ddth = plant_car(tauL, tauR, x, y, th, vu, vw, omega, Ddvu, Ddomega, dt)

    return x, y, th, vu, vw, omega, tauL, tauR, dvu, dvw, ddth



if __name__ == "__main__":
    import matplotlib.pyplot as plt

    def plot(xs, ys):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(xs, ys, "-", c="Blue", linewidth=1, alpha=1)
        plt.show()



    # car pose
    x = 0
    y = 0
    th = 0
    # car vel
    vu = 0
    vw = 0
    omega = 0
    # wheel
    tauL = 0 # current
    tauR = 0

    dt = 0.01
    dvu = 0
    dvw = 0
    ddth = 0

    t = 0

    history = []
    for i in range(1000):
        ref_vu = 1
        ref_omega = 0
        Ddvu = 0
        Ddomega = 0
        x, y, th, vu, vw, omega, tauL, tauR, dvu, dvw, ddth = openloop(ref_vu, ref_omega, x, y, th, vu, vw, omega, tauL, tauR, dvu, dvw, ddth, Ddvu, Ddomega, dt)
        t = t + dt
        d = {"t":t,"x":x,"y":y,"th":th,"vu":vu,"vw":vw,"omega":omega,"tauL":tauL,"tauR":tauR}
        history.append(d)
    plot([d["t"] for d in history], [d["vu"] for d in history])

