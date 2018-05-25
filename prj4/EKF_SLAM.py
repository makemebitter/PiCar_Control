import numpy as np
from math import *


def cov2elli(x,P,n,NP):
    """ function [X,Y] = cov2elli(x,P,n,NP)
    COV2ELLI Ellipse contour from Gaussian mean and covariances matrix.
    [X,Y] = COV2ELLI(X0,P) returns X and Y coordinates of the contour of
    the 1?sigma ellipse of the Gaussian defined by mean X0 and covariances
    matrix P. The contour is defined by 16 points, thus both X and Y are
    16-vectors.
    
    [X,Y] = COV2ELLI(X0,P,n,NP) returns the n?sigma ellipse and defines the
    contour with NP points instead of the default 16 points.
    
    The ellipse can be plotted in a 2D graphic by just creating a line 
    with 'line(X,Y)' or 'plot(X,Y)'. """
    alpha = 2*pi/NP*np.array(range(NP+1)) # NP angle intervals for one turn
    circle = np.vstack([np.cos(alpha),np.sin(alpha)]) # the unit circle
    # SVD method, P = R*D*R' = R*d*d*R'
    R, D, Vh = np.linalg.svd(P)

    d = np.diag(np.sqrt(D))
    # n?sigma ellipse <? rotated 1?sigma ellipse <? aligned 1?sigma ellipse <? unit circle
    ellip = n * R .dot (d) .dot(circle)
    # output ready for plotting (X and Y line vectors)
    # print ellip
    X = x[0,0]+ellip[0:1,:]
    Y = x[1,0]+ellip[1:2,:]

    return X,Y

def QR_real_scan (p,p_i,name_dis_bearing):
    """simulates the behavior of SCAN but with actual sensor
    function [y, Y_p] = scan (p)
    SCAN perform a range-and-bearing measure of a 2D point.
    
    In:
    p : point in sensor frame p = [p x  p y]
    Out:
    y : measurement y = [range  bearing]
    Y_p: Jacobian wrt p"""
#     p=W[:,p_i:p_i+1]
    px = p[0,0]
    py = p[1,0]
    y=name_dis_bearing[p_i]
    Y_p = np.vstack([
    np.hstack([px/sqrt(px**2+py**2) , py/sqrt(px**2+py**2)]),
    np.hstack([-py/(px**2*(py**2/px**2 + 1)), 1/(px*(py**2/px**2 + 1))]) ])
    return y,Y_p

def scan (p):
    """function [y, Y_p] = scan (p)
    SCAN perform a range-and-bearing measure of a 2D point.
    
    In:
    p : point in sensor frame p = [p x  p y]
    Out:
    y : measurement y = [range  bearing]
    Y_p: Jacobian wrt p"""

    px = p[0,0]
    py = p[1,0]
    d = sqrt(px**2+py**2)
    a = atan2(py,px)
    y = np.vstack([d,a])
    Y_p = np.vstack([
    np.hstack([px/sqrt(px**2+py**2) , py/sqrt(px**2+py**2)]),
    np.hstack([-py/(px**2*(py**2/px**2 + 1)), 1/(px*(py**2/px**2 + 1))]) ])
    return y,Y_p

def invScan(y):
    """function [p, P_y] = invScan(y)
    INVSCAN Backproject a range-and-bearing measure into a 2D point.
    In:
    y : range-and-bearing measurement y = [range  bearing]
    Out:
    p : point in sensor frame p = [p x  p y]
    P y: Jacobian wrt y"""
    d = y[0,0]
    a = y[1,0]
    px = d*cos(a)
    py = d*sin(a)
    p = np.vstack([px,py])
    P_y = np.vstack([np.hstack([cos(a) , -d*sin(a)]), np.hstack([sin(a) , d*cos(a)]) ])
    return p, P_y
def fromFrame(F, pf):
    #     [pw, PW_f, PW_pf] = fromFrame(F, pf)
        # FROMFRAME Transform a point PF from local frame F to the global frame.
        # In:
        # F : reference frame F = [f x  f y  f alpha]
        # pf: point in frame F pf = [pf x  pf y]
        # Out:
        # pw: point in global frame
        # PW_f: Jacobian wrt F
        # PW_pf: Jacobian wrt pf

    t = F[0:2]
    a = F[2,0]
    R = np.array([[cos(a),-sin(a)] , [sin(a),cos(a)]])


    pw = np.dot(R,pf) + np.tile(t, (1,pf.shape[1])) # Allow for multiple points
    px = pf[0,0]
    py = pf[1,0]
    PW_f = np.array([[ 1, 0, -py*cos(a)-px*sin(a)],[0,1,px*cos(a)-py*sin(a)]])
    PW_pf = R


    return pw, PW_f, PW_pf
def toFrame(F , p):
    # function [pf, PF_f, PF_p] = toFrame(F , p)
    # # TOFRAME transform point P from global frame to frame F
    # #
    # # In:
    # # F : reference frame F = [f x  f y  f alpha]
    # # p : point in global frame p = [p x  p y]
    # # Out:
    # # pf: point in frame F
    # # PF f: Jacobian wrt F
    # # PF p: Jacobian wrt p


    t = F[0:2]
    a = F[2,0]
    R = np.array([[cos(a),-sin(a)] , [sin(a),cos(a)]])
    pf = np.dot(R.T , (p - t))
     # Jacobians requested
    px = p[0,0]
    py = p[1,0]
    x = t[0,0]
    y = t[1,0]
    
    PF_f =np.array([[ -cos(a), -sin(a), cos(a)*(py - y) - sin(a)*(px - x)],[sin(a), -cos(a), - cos(a)*(px - x) - sin(a)*(py - y)]])

    PF_p = R.T
    return pf, PF_f, PF_p

def move(r, u, n):
# [ro, RO_r, RO_n] = move(r, u, n)
# MOVE Robot motion, with separated control and perturbation inputs.
#
# In:
# r: robot pose r = [x  y  alpha]
# u: control signal u = [d x  d alpha]
# n: perturbation, additive to control signal
# Out:
# ro: updated robot pose
# RO r: Jacobian d(ro) / d(r)
# RO n: Jacobian d(ro) / d(n)

# r=R
# u=U
# n=np.zeros((2,1))
# print r,u,n

    a = r[2,0]

    dx = u[0,0] + n[0,0]
    da = u[1,0] + n[1,0]
    ao = a + da
    if ao > pi:
        ao = ao - 2*pi
    if ao < -pi:
        ao = ao + 2*pi
    # build position increment dp=[dxdy], from control signal dx
    dp = np.vstack([dx,0])
    to, TO_r, TO_dt = fromFrame(r, dp)
    AO_a = 1
    AO_da = 1
    RO_r = np.vstack([TO_r,np.array([0,0,AO_a])])
    #     RO_r = [TO_r  0 0 AO_a]
    RO_n = np.vstack([np.hstack([TO_dt[:,[0]],np.zeros((2,1))]),np.array([0,AO_da])])
    #     RO_n = [TO_dt(:,1) np.zeros(2,1)  0 AO da]
    ro = np.vstack([to,ao])
    #     ro = [toao]
    return ro,RO_r, RO_n

def QR_real_observe(r,p,p_i,name_dis_bearing):
    """simulates the behavior of OBSERVE but use actual sensor measurement
    function [y, Y_r, Y_p] = observe(r, p)
    OBSERVE Transform a point P to robot frame and take a
    range-and-bearing measurement.
    
    In:
    r : robot frame r = [r_x  r_y  r_alpha]
    p : point in global frame p = [p_x  p_y]
    Out:
    y: range-and-bearing measurement
    Y_r: Jacobian wrt r
    Y_p: Jacobian wrt p"""
    pr, PR_r, PR_p = toFrame(r, p)
    y,Y_pr=QR_real_scan(pr,p_i,name_dis_bearing)
    Y_r = np.dot(Y_pr, PR_r)
    Y_p = np.dot(Y_pr, PR_p)
    return y, Y_r, Y_p
    
def observe(r, p):
    """function [y, Y_r, Y_p] = observe(r, p)
    OBSERVE Transform a point P to robot frame and take a
    range-and-bearing measurement.
    
    In:
    r : robot frame r = [r_x  r_y  r_alpha]
    p : point in global frame p = [p_x  p_y]
    Out:
    y: range-and-bearing measurement
    Y_r: Jacobian wrt r
    Y_p: Jacobian wrt p"""

    pr, PR_r, PR_p = toFrame(r, p)
    y, Y_pr = scan(pr)
    # The chain rule!
    Y_r = np.dot(Y_pr, PR_r)
    Y_p = np.dot(Y_pr, PR_p)
    return y, Y_r, Y_p

def invObserve(r, y):
    # function [p, P_r, P_y] = invObserve(r, y)
    # INVOBSERVE Backproject a range-and-bearing measurement and transform
    # to map frame.
    #
    # In:
    # r : robot frame r = [r x  r y  r alpha]
    # y : measurement y = [range  bearing]
    # Out:
    # p : point in sensor frame
    # P_r: Jacobian wrt r
    # P_y: Jacobian wrt y


    P_r, PR_y = invScan(y)
    p, P_r, P_pr = fromFrame(r, P_r)
    # here the chain rule !
    P_y = P_pr .dot(PR_y)
    return p, P_r, P_y