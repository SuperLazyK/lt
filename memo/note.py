from sympy import symbols, Eq, solve, cos, sin, Matrix, simplify, shape, eye, lambdify, expand, fraction, Poly
import sys
import numpy as np

def printMat(M, name="M"):
    for i in range(shape(M)[0]):
        for j in range(shape(M)[1]):
            print(f"{name}[{i},{j}] = {M[i,j]}")

def sp2np(M):
    return np.array(M).astype(np.float64)

# https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/Lecture_13/20191104%20-%20ETH%20-%2001%20-%20Modeling.pdf
# https://docs.duckietown.com/daffy/instructor-manual/resources/slides/models/03-full-lecture-modeling.html
#
#             C  (cog)
#             ^
#             |
#             c
#             |
#             v
# WL <--LL--> A <--LR--> WR
#
# NOTE: c != 0 causes non-linear term
# NOTE: assume LR = LL = L2

L2 = symbols("L2") # L / 2
Ts = symbols("Ts") # time constant for lead compensasor of PD controller
M, Jw, J, R, tauL, tauR, dtauL, dtauR, sL, sR, LL, LR, c = symbols('M Jw J R tauL tauR dtauL dtauR sL sR LL LR c')
LL = L2
LR = L2
vu,vw,vL,vR,wL,wR = symbols("vu vw vL vR wL wR") # vu,vw is velocity at C
dvu,dvw,dvL,dvR,dwL,dwR = symbols("dvu dvw dvL dvR dwL dwR")
th, dth, ddth = symbols("th omega domega")
au, aw = symbols("au aw")
vxA, vyA, vuA = symbols("vxA vyA vuA") # velocity at A
Rm,Lm,Kb,Kt,V,tau,dtau,i,di,dphi = symbols("Rm Lm Kb Kt V tau dtau i di dphi") #motor rotor
VL, VR  = symbols("VL VR")
Ku, Kw = symbols("Ku Kw")

constant = {
    M: 0.1, # car mass [kg]
    J: 0.0001, # car inertia [kg m^2]
    L2: 0.03, #[m]
    c: 0.01, #[m]
    R: 0.01, # wheel radius [m]
    Rm: 0.1, # motor resistance [ohm]
    Lm: 0.1, # motor Inductance [H]
    Kt: 0.01, # [Nm/V]
    Kb: 0.01, # [V / (rad / sec)]
    Ku: 100,
    Kw: 100,
    Ts: 0.001
    }

s = symbols("s")

FuL,FuR = symbols('FuL FuR')
FwL,FwR = symbols('FwL FwR')
FwLR = symbols('FwLR')

# dynamics
eq_du = Eq(M * au, FuL + FuR)
eq_dw = Eq(M * aw, FwLR)
eq_dth = Eq( J * ddth,  LR*FuR - LL*FuL - c * (FwLR))

eq_intu = Eq(au, dvu - vw * dth)
eq_intw = Eq(aw, dvw + vu * dth)

eq_noskid1 = Eq(vw, c * dth)
eq_noskid2 = Eq(dvw, c * ddth)

eqs_dyn = [eq_du, eq_dw, eq_dth, eq_intu, eq_intw, eq_noskid1, eq_noskid2]
solve_target_dyn = [dvu, dvw, ddth, au, aw, vw, FwLR]
sol = solve(eqs_dyn, solve_target_dyn)

#print(f"{dvu} = {sol[dvu]}")
#print(f"{dvw} = {sol[dvw]}")
#print(f"{ddth} = {sol[ddth]}")

#---------------------------------------------
# slip rate s = 0 + zero wheel inertia
#---------------------------------------------
# encoder value should match IMU in this case
eq_wheelL = Eq(tauL, FuL * R)
eq_wheelR = Eq(tauR, FuR * R)
#sol = solve(eqs_dyn + [eq_wheelL, eq_wheelR], solve_target_dyn + [FuL, FuR])
#print("")
#print("dynamics")
#print(f"{dvu} = {sol[dvu]}")
#print(f"{dvw} = {sol[dvw]}")
#print(f"{ddth} = {sol[ddth]}")


#----------------
# kinematics
#----------------

# control wheel velocity to control C velocity
# vel C <-> vel A (in inertia frame)
# vw != 0 but vwA == 0 (no latitude slip!!)
# vw is caused by only rotation
eq_ACx_kin = Eq(vxA, vu * cos(th) - (vw - c * dth) * sin(th))
eq_ACy_kin = Eq(vyA, vu * sin(th) - (vw - c * dth) * cos(th))
eq_sL_noslip = Eq(wL * R, vL)
eq_sR_noslip = Eq(wR * R, vR)
eq_vuA_kin = Eq(vuA, (vL + vR) / 2)
eq_omega_kin = Eq(dth, (vR - vL) / (LL + LR))
eq_x_kin = Eq(vuA*cos(th), vxA)
eq_y_kin = Eq(vuA*sin(th), vyA)

# forward
sol = solve([eq_ACx_kin, eq_ACy_kin, eq_sL_noslip, eq_sR_noslip, eq_vuA_kin, eq_omega_kin, eq_x_kin, eq_y_kin],
            [vxA, vyA, vu, vw, dth, vuA, vL, vR])

solinv = solve([Eq(vu, sol[vu]), Eq(dth, sol[dth])],
            [wL, wR])

# vu/omega -> wL/wR
def f_inverse_kinematics(params=constant):
    f = lambdify([vu, dth], Matrix([solinv[wL], solinv[wR]]).subs(params))
    return f

#--------------
# motor dynamics
#--------------

eqMV = Eq(V, Rm * i + Lm * di + Kb * dphi)
eqT = Eq(tau, Kt * i)
eqdT = Eq(dtau, Kt * di)

sol = solve([eqMV, eqT,eqdT], [i, di,dtau])

#--------------------
#plant state equation
#--------------------
sol = solve(eqs_dyn + [eq_wheelL, eq_wheelR], solve_target_dyn + [FuL, FuR])
eqdvu = Eq(dvu, sol[dvu])
eqdomega = Eq(ddth, sol[ddth])
eqmL = Eq(dtauL, (-Kb*Kt*dwL + Kt*VL - Rm*tauL)/Lm)
eqmR = Eq(dtauR, (-Kb*Kt*dwR + Kt*VR - Rm*tauR)/Lm)
eq_dsL_noslip = Eq(dwL * R, dvL)
eq_dsR_noslip = Eq(dwR * R, dvR)
eq_dvu_kin = Eq(dvu, (dvL + dvR) / 2)
eq_domega_kin = Eq(ddth, (dvR - dvL) / (LL + LR))

#eq_omega_kin = Eq(dth, (vR - vL) / (LL + LR))
sol = solve([eqdvu, eqdomega, eqmL, eqmR, eq_dsL_noslip, eq_dsR_noslip, eq_dvu_kin, eq_domega_kin], [dvu, ddth, dwL, dwR, dvL, dvR, dtauL, dtauR])

plant_ss = Matrix([sol[ddth], sol[dvu], sol[dtauL], sol[dtauR]])

def f_plant_ss(params=constant):
    expr = plant_ss.subs(params)
    f = lambdify([vu, dth, tauL, tauR], expr)
    return f


#-----------------------
# linearlize
#-----------------------

# regard c*omega**2 and c*vu * omega as external state disturbance

d1, d2 = symbols("d1 d2")
disturbance_sub = {c * dth **2: d1, c * vu * dth : d2}
disturbance = {v: k for k, v in disturbance_sub.items()}
disturbance_diff = {d1: 2 * c * dth * ddth, d2: c * dvu * dth + c * vu * ddth}

plant_ss_dist = plant_ss.subs(disturbance_sub)
# x' = A x + B u + W d
# controllability
# W
A = simplify(plant_ss_dist.jacobian(Matrix([vu, dth, tauL, tauR])))
B = simplify(plant_ss_dist.jacobian(Matrix([VL, VR])))
Wss = simplify(plant_ss_dist.jacobian(Matrix([d1, d2])))
C = Matrix([[1, 0, 0, 0], [0, 1, 0, 0]])

def f_plant_ss_dist(params=constant):
    return sp2np(A.subs(params)),sp2np(B.subs(params)),sp2np(C.subs(params)),sp2np(Wss.subs(params))


#-----------------------
# non-linear compensation
#-----------------------

# note that P has 1/s. s * PlantTF is 1st order lag
# Y = C (sI - A)^-1 (B U + Wss D)
# P = C (sI - A)^-1 B
# W = C (sI - A)^-1 Wss
P = C * (s* eye(4) - A).inv() * B
W = C * (s* eye(4) - A).inv() * Wss

def f_plant_tf_dist(params=constant):
    return P.subs(params), W.subs(params)

# Y = P (U + Comp) + W D
# to cancel W D
Comp = -P.inv() * W
#print(expand(Comp[0]).coeff(s,0))
Comp_ss = Comp.subs(s, 0)*Matrix([disturbance[d1], disturbance[d2]]) + Comp.diff(s) * Matrix([disturbance_diff[d1], disturbance_diff[d2]])

def f_non_linear_compensator(params=constant):
    f = lambdify([vu, dth, dvu, ddth], Comp_ss.subs(params))
    return f

#-------------------------------
# PD controller for linear system
#-------------------------------
# input ref vu/dth -> output VL/VR
# plant has 1/s. No need of PID

K1, K2, K3, K4 = symbols("K1, K2, K3, K4")
# Y = C^-1 (sI - A) ^-1 (B (K Y + deltaU)+ D W)
# if P K is 1 / (T*s) , convergence should be ok
K = Matrix([[K1, K2],[K3, K4]])
KsP = s * P * K
eqKP1 = Eq(KsP[0, 1], 0)
eqKP2 = Eq(KsP[1, 0], 0)
eqKP3 = Eq(KsP[0, 0], Ku / (Ts * s + 1))
eqKP4 = Eq(KsP[1, 1], Kw / (Ts * s + 1))
sol = solve([eqKP1, eqKP2, eqKP3, eqKP4], [K1, K2, K3, K4])
Ksol = K.subs(sol)
evu_ref, edth_ref = symbols("evu_ref eomega_ref")

KE_ss = Ksol.subs(s, 0)*Matrix([evu_ref, edth_ref]) + Ksol.diff(s) * Matrix([dvu, ddth])
#print(KE_ss.subs(constant))

# lead compensator is necessary?
def f_pd_controller(params=constant):
    f = lambdify([evu_ref, edth_ref, dvu, ddth], KE_ss.subs(params))
    return f


def f_pd_controller_tf(params=constant):
    return Ksol.subs(params)

#-----------------------------------
# test
#-----------------------------------

def sptfM2ctss(M):
    nums = []
    dens = []
    for i in range(shape(M)[0]):
        row_nums = []
        row_dens = []
        for j in range(shape(M)[1]):
            num, den = fraction(M[i,j])
            row_nums.append(sp2np(Poly(num, s).all_coeffs()).tolist())
            row_dens.append(sp2np(Poly(den, s).all_coeffs()).tolist())
        nums.append(row_nums)
        dens.append(row_dens)
    return ct.tf2ss(nums, dens)



if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import control as ct
    mP,mWss = f_plant_tf_dist()
    ssP = sptfM2ctss(mP)
    mK = f_pd_controller_tf()
    ssK = sptfM2ctss(mK)
    L = ct.series(ssK, ssP) # same as sptfM2ctss(simplify(mP * mK))
    # MIMO tf feedback is not implemented
    Lc = ct.feedback(L, np.eye(2))
    print(Lc.noutputs)
    print(Lc.ninputs)
    t, y = ct.step_response(Lc)
    print(y.shape)
    i = 1
    j = 1
    plt.plot(t, y[i][j], label=f"Output {i}{j}")

    plt.xlabel("Time [s]")
    plt.ylabel("Output")
    plt.title("Step Response")
    plt.grid(True)
    plt.show()





