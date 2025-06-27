from sympy import symbols, integrate, Eq, solve, cos, sin, Matrix, simplify, shape, eye
import sys


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
M, Jw, J, R, tauL, tauR, dtauL, dtauR, sL, sR, LL, LR, c = symbols('M Jw J R tauL tauR dtauL dtauR sL sR LL LR c')
dsL, dsR = symbols('dsL dsR')
vu,vw,vL,vR,wL,wR = symbols("vu vw vL vR wL wR") # vu,vw is velocity at C
dvu,dvw,dvL,dvR,dwL,dwR = symbols("vu' vw' vL' vR' wL' wR'")
th, dth, ddth = symbols("th omega domega")
au, aw = symbols("au aw")
vxA, vyA, vuA = symbols("vxA vyA vuA") # velocity at A
Rm,Lm,Kb,Kt,V,tau,dtau,i,di,dphi = symbols("Rm Lm Kb Kt V tau dtau i di dphi") #motor rotor
VL, VR  = symbols("VL VR")

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
sol = solve(eqs_dyn + [eq_wheelL, eq_wheelR], solve_target_dyn + [FuL, FuR])
print("")
print("dynamics")
print(f"{dvu} = {sol[dvu]}")
print(f"{dvw} = {sol[dvw]}")
print(f"{ddth} = {sol[ddth]}")


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
print("")
print("foward kinematics (input -> output)")
print(f"{vu} = {sol[vu]}")
print(f"{vw} = {sol[vw]}")
print(f"{dth} = {sol[dth]}")
# inverse
solinv = solve([Eq(vu, sol[vu]), Eq(dth, sol[dth])],
            [wL, wR])
print("")
print("inverse kinematics (for controller/encoder simulation: output -> input) ")
print(f"{wL} = {solinv[wL]}")
print(f"{wR} = {solinv[wR]}")


#--------------
# motor dynamics
#--------------

eqMV = Eq(V, Rm * i + Lm * di + Kb * dphi)
eqT = Eq(tau, Kt * i)
eqdT = Eq(dtau, Kt * di)

sol = solve([eqMV, eqT,eqdT], [i, di,dtau])
print("")
print("motor dynamics")
print(f"{dtau} = {sol[dtau]}")

#--------------
# whole dynamics(non-linear)
#--------------
sol = solve(eqs_dyn + [eq_wheelL, eq_wheelR], solve_target_dyn + [FuL, FuR])
eqdvu = Eq(dvu, sol[dvu])
eqdomega = Eq(ddth, sol[ddth])
eqmL = Eq(dtauL, (-Kb*Kt*dwL + Kt*VL - Rm*tauL)/Lm)
eqmR = Eq(dtauR, (-Kb*Kt*dwR + Kt*VR - Rm*tauR)/Lm)
eq_dsL_noslip = Eq(dwL * R, dvL)
eq_dsR_noslip = Eq(dwR * R, dvR)
eq_dvu_kin = Eq(dvu, (dvL + dvR) / 2)
eq_domega_kin = Eq(ddth, (dvR - dvL) / (LL + LR))

d1, d2 = symbols("d1 d2")

#eq_omega_kin = Eq(dth, (vR - vL) / (LL + LR))
sol = solve([eqdvu, eqdomega, eqmL, eqmR, eq_dsL_noslip, eq_dsR_noslip, eq_dvu_kin, eq_domega_kin], [dvu, ddth, dwL, dwR, dvL, dvR, dtauL, dtauR])
print(sol)
print("")
print("whole dynamics: non-linear")
print("regard c*omega**2, c*vu * omega as external output disturbance")
dtauLrhs = sol[dtauL].subs({c * dth **2: d1, c * vu * dth : d2})
dtauRrhs = sol[dtauR].subs({c * dth **2: d1, c * vu * dth : d2})
dvurhs = sol[dvu].subs({c * dth **2: d1, c * vu * dth : d2})
ddthrhs = sol[ddth].subs({c * dth **2: d1, c * vu * dth : d2})
print()
print(f"{dtauL} = {dtauLrhs}")
print(f"{dtauR} = {dtauLrhs}")
print(f"{dvu} = {dvurhs}")
print(f"{ddth} = {ddthrhs}")

# x' = A x + B u + V d
A = simplify(Matrix([dvurhs, ddthrhs, dtauLrhs, dtauRrhs]).jacobian(Matrix([vu, dth, tauL, tauR])))
B = simplify(Matrix([dvurhs, ddthrhs, dtauLrhs, dtauRrhs]).jacobian(Matrix([VL, VR])))
Wss = simplify(Matrix([dvurhs, ddthrhs, dtauLrhs, dtauRrhs]).jacobian(Matrix([d1, d2])))
C = Matrix([[1, 0, 0, 0], [0, 1, 0, 0]])
# X = (sI - A)^-1 (B U + WD)
# Y = C (sI - A)^-1 (B U + WD)
# deltaU/D = -B^+ Wss
dU = simplify( -(B.T * B).inv()* B.T * Wss)

def printMat(M, name):
    for i in range(shape(M)[0]):
        for j in range(shape(M)[1]):
            print(f"{name}[{i},{j}] = {M[i,j]}")

printMat(A, "A")
printMat(B, "B")
printMat(Wss, "W")
printMat(C, "C")
print("compensator ss")
printMat(dU, "dU")

# Y = C (sI - A)^-1 (B U + WD)
print()
P0 = C * (s* eye(4) - A).inv() * B
W0 = C * (s* eye(4) - A).inv() * Wss

eqd1 = Eq(s * tauL, dtauLrhs)
eqd2 = Eq(s * tauR, dtauRrhs)
eqd3 = Eq(s * vu, dvurhs)
eqd4 = Eq(s * dth, ddthrhs)
sol = solve([eqd1, eqd2, eqd3, eqd4], [vu, dth, tauL, tauR])

print(f"vu(s) = {sol[vu]}")
print(f"omega(s) = {sol[dth]}")
print("")
P = simplify(Matrix([sol[vu], sol[dth]]).jacobian(Matrix([VL, VR])))
W = simplify(Matrix([sol[vu], sol[dth]]).jacobian(Matrix([d1, d2])))
#print(simplify(P - P0)) # assert 0
#print(simplify(W - W0)) # assert 0
#print()
#for i in range(shape(P)[0]):
#    for j in range(shape(P)[1]):
#        print(f"P[{i},{j}] = {P[i,j]}")
#print()
#for i in range(shape(W)[0]):
#    for j in range(shape(W)[1]):
#        print(f"W[{i},{j}] = {W[i,j]}")
# P (U + deltaU) + W D = Y
# P U + WD = Y
# <=>
# P deltaU + WD = 0
# deltaU/D = - P^-1 W

print("compensator tf")
print(simplify(-P.inv() * W))
sys.exit(0)

print(f"disturbance of domega: {simplify((sol[dth] - sol[dth].subs({d1:0,d2:0,VL:1,VR:0}) * VL - sol[dth].subs({d1:0,d2:0,VL:0,VR:1}) * VR))}")

#--------------
# whole dynamics (simple model for controller) c==0, LR=LL=L/2
#--------------
sol = solve(eqs_dyn + [eq_wheelL, eq_wheelR], solve_target_dyn + [FuL, FuR])
eqdvu = Eq(dvu, sol[dvu].subs(c,0))
eqdomega = Eq(ddth, sol[ddth].subs(c,0))
eqmL = Eq(dtauL, (-Kb*Kt*dwL + Kt*VL - Rm*tauL)/Lm)
eqmR = Eq(dtauR, (-Kb*Kt*dwR + Kt*VR - Rm*tauR)/Lm)
eq_dsL_noslip = Eq(dwL * R, dvL)
eq_dsR_noslip = Eq(dwR * R, dvR)
eq_dvu_kin = Eq(dvu, (dvL + dvR) / 2)
eq_domega_kin = Eq(ddth, (dvR - dvL) / (LL + LR))

#eq_omega_kin = Eq(dth, (vR - vL) / (LL + LR))
sol = solve([eqdvu, eqdomega, eqmL, eqmR, eq_dsL_noslip, eq_dsR_noslip, eq_dvu_kin, eq_domega_kin], [dvu, ddth, dwL, dwR, dvL, dvR, dtauL, dtauR])
print(sol)
print("")
print("motor dynamics (simple)")
print(f"{dtauL} = {sol[dtauL]}")
print(f"{dtauR} = {sol[dtauR]}")
print(f"{dvu} = {sol[dvu]}")
print(f"{ddth} = {sol[ddth]}")

A = Matrix([sol[dtauL], sol[dtauR], sol[dvu], sol[ddth]]).jacobian(Matrix([tauL, tauR, vu, dth]))
B = Matrix([sol[dtauL], sol[dtauR], sol[dvu], sol[ddth]]).jacobian(Matrix([VL, VR]))

print("")
print("total plant (simple) x' = A x + B u")
print(A[0,0])
for i in range(4):
    for j in range(4):
        print(f"A[{i},{j}] = {A[i,j]}")
for i in range(4):
    for j in range(2):
        print(f"B[{i},{j}] = {B[i,j]}")


print("")
print("total plant (simple + transfer function style)")

eqP1 = Eq(s * tauL, sol[dtauL])
eqP2 = Eq(s * tauR, sol[dtauR])
eqP3 = Eq(s * vu, sol[dvu])
eqP4 = Eq(s * dth, sol[ddth])

sol = solve([eqP1, eqP2, eqP3, eqP4], [tauL, tauR, vu, dth])
print(f"vu = {sol[vu]}")
print(f"dth = {sol[dth]}")


K1, K2, K3, K4 = symbols("K1, K2, K3, K4")
Ku, Kw = symbols("Ku Kw")
L = symbols("L")
K = Matrix([[K1, K2],[K3, K4]])
sP = s*Matrix([sol[vu], sol[dth]]).jacobian(Matrix([VL, VR]))

def config(expr):
    return simplify(expr.subs({M:0.1,J:1,LL:0.03,LR:0.03,c:0,R:0.01,Rm:0.1,Lm:0.1,Kt:0.01,Kb:0.01}))

for i in range(2):
    for j in range(2):
        print(f"sP[{i},{j}] = {config(sP[i,j])}")

KsP = sP * K
eqKP1 = Eq(KsP[0, 1], 0)
eqKP2 = Eq(KsP[1, 0], 0)
eqKP3 = Eq(KsP[0, 0], Ku)
eqKP4 = Eq(KsP[1, 1], Kw)
print("")
sol = solve([eqKP1.subs(LR,L/2).subs(LL,L/2), eqKP2.subs(LR,L/2).subs(LL,L/2), eqKP3.subs(LR,L/2).subs(LL,L/2), eqKP4.subs(LR,L/2).subs(LL,L/2)], [K1, K2, K3, K4])
print(f"K1s = {sol[K1].diff(s)}")
print(f"K2s = {sol[K2].diff(s)}")
print(f"K3s = {sol[K3].diff(s)}")
print(f"K4s = {sol[K4].diff(s)}")

print(f"K1ns = {sol[K1].subs(s,0)}")
print(f"K2ns = {sol[K2].subs(s,0)}")
print(f"K3ns = {sol[K3].subs(s,0)}")
print(f"K4ns = {sol[K4].subs(s,0)}")

