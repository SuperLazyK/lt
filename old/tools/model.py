#--------------------------------------
# Error State Kalman Filter (ESKF)
#-------------------------------------
# sensor : IMU + encoder + LineCenterDetector
# input : acc(x,y) + 
#

import numpy as np
from collections import namedtuple
from sympy import lambdify, diff, symbols, Matrix, Function
from sympy import sin, cos, tan, exp, log, sinh, cosh, tanh, diff, sqrt, Symbol, symbols, Matrix
import sys

#-----------------
# symbols
#-----------------

t_ = Symbol('t')

# p : position (global coordinate)
# v : velocity (global coordinate)
# a : acceleration (???? coordinate)
# th : direction (???? coordinate)
# omega : acceleration (???? coordinate)

ptx_, pty_, tht_, vtx_, vty_, atx_, aty_, omegat_ = functions('pt.x, pt.y, tht, vt.x, vt.y, at.x, at.y, omegat') # true state
#abtx_, abty_, omegabt_ = Symbol('abt.x, abt.y, omegabt') # true sensor bias (no need to use gravity vector)
#
#amx_, amy_, omegam_ = Symbol('am.x, am.y, omegam') # measured sensor value
#anx_, any_, omegan_ = Symbol('an.x, an.y, omegan') # measured noise value
#awx_, awy_, omegaw_ = Symbol('aw.x, aw.y, omegaw') # bias noise value (accumlated)
#
#pdx_, pdy_, thd_, vdx_, vdy_  = Symbol('pd.x, pd.y, thd, vd.x, vd.y') # error state
#abdx_, abdy_, omegabd_ = Symbol('abd.x, abd.y, omegabd') # error state for sensor bias
#
## vectors
#pt_ = Matrix([[ptx_, pty_]])
#vt_ = Matrix([[vtx_, vty_]])
#at_ = Matrix([[atx_, aty_]])

## input
#
#x = Matrix([[x+x1, x+x2, x+x3]])
#
#B = A * Transpose(A)
#
#        v_ = Matrix(symbols('v[0:%d]' %(m))) # slcak var symbols for dummy wait
#
#        #box constraint
#        C_ = Matrix([u_[i] ** 2 + v_[i] ** 2 - max_u[i] ** 2 for i in range(m)])
#        r_ = Matrix(symbols('r[0:%d]' %(m))) # laglan-multiplier symbols for rho
#        #dCdu_ = C_.jacobian(u_)
#        #dCdv_ = C_.jacobian(v_)
#        H_ = L_ + l_.dot(f_)
#        H_ = H_ + r_.dot(C_)
#        H_ = H_ - Matrix(dw).dot(v_)
#        H_ = L_ + l_.dot(f_) + r_.dot(C_) - Matrix(dw).dot(v_)
#        dHdx_ = Matrix([H_]).jacobian(x_)
#        #dHdu_ = Matrix([H_]).jacobian(u_)
#        dHduvr_ = Matrix([H_]).jacobian(Matrix([u_, v_, r_]))
#        dphidx_ = phi_.jacobian(x_)
#
#        # lambdify
#        self.lambd_dphidx = lambdify([x_], dphidx_)
#        self.lambd_dHduvr = lambdify([x_, l_, u_, v_, r_], dHduvr_)
#        self.lambd_dHdx = lambdify([x_, l_, u_], dHdx_) # v, r is not used
#        self.lambd_f = lambdify([x_, u_], f_)
#        #self.lambd_C = lambdify([u_,v_], C_)
#        self.dphidx = lambda x: self.lambd_dphidx(x)[0]
#        self.dHduvr = lambda x,l,uvr: self.lambd_dHduvr(x,l,uvr[0:m], uvr[m:2*m], uvr[2*m:3*m])[0]
#        self.dHdx   = lambda x,l,u: self.lambd_dHdx(x,l,u)[0]
#        self.f      = lambda x,u: self.lambd_f(x,u)[:,0]
#
#    def step(self, t, x, u, dt):
#        return rk4(self.f, t, x, u, dt)
#
#
