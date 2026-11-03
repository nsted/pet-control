"""
Coupled static gravity model of the hanging robot (seven joints, held by PD springs).

Geometry from Nick's Fusion export (cm, world frame at the straight hanging pose): joint axes and points on them, and the
centre of each module's mass, which sits ON its motor axis (module k's mass is on axis k). Mass is 2.2 kg over 8 modules,
0.275 kg per module (Nick). The robot is held by the head, so gravity points along the straight chain.

``equilibrium(cmd, m, kp)`` solves, for a vector of commanded joint offsets, the static joint angles and the motor torques
when every joint is held by ``tau = kp * (cmd - phi)``, with all the geometric coupling between joints (the alternating
axes mean rotating one joint loads its neighbours). With no free parameters it predicted the torques of the *other* joints
in the servo 3 and 4 hanging runs with slopes of 0.5-1.0 and correlations 0.84-0.99. Joint friction is not modelled.

Axis 7 is assumed z-type (the CAD tail joint is rigid).
"""

from __future__ import annotations

import numpy as np
CM=0.01
# CAD export (cm, world frame at the straight hanging pose). Joint k = servo k; motor k sits ON axis k.
AX={1:((0,0,1),(0,0,1.05)),2:((0,1,0),(0,-3.48,-3.68)),3:((0.083,0,0.997),(-0.29,-8.2,-7.14)),
    4:((-0.002,1,0),(-0.67,-11.67,-11.85)),5:((0.062,0,0.998),(-0.88,-16.4,-15.32)),
    6:((0.036,0.999,-0.002),(-1.3,-19.87,-20.02)),7:((0,0,1),(-1.77,-24.59,-23.48))}   # J7 axis assumed z-type (CAD tail joint is rigid)
MOT={1:(0.01,0.01,-0.42),2:(0,-4.95,-3.69),3:(-0.41,-8.21,-8.6),4:(-0.68,-13.15,-11.85),
     5:(-0.95,-16.4,-16.79),6:(-1.35,-21.34,-20.01),7:(-1.9,-24.58,-24.94)}
A={k:np.array(v[0],float)/np.linalg.norm(v[0]) for k,v in AX.items()}
Q={k:np.array(v[1],float)*CM for k,v in AX.items()}
P={k:np.array(v,float)*CM for k,v in MOT.items()}
G=9.81
ghat=(P[7]-P[1]); ghat/=np.linalg.norm(ghat)                 # hanging straight: gravity along the chain
def rot(a,phi):
    K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]]); return np.eye(3)+np.sin(phi)*K+(1-np.cos(phi))*K@K
def T(k,phi):      # rotation by phi about axis k through Q[k], as 4x4
    R=rot(A[k],phi); M=np.eye(4); M[:3,:3]=R; M[:3,3]=Q[k]-R@Q[k]; return M
def positions(phi):
    out={}; M=np.eye(4)
    for i in range(1,8):
        p=np.append(P[i],1.0); out[i]=(M@p)[:3]; M=M@T(i,phi[i-1])   # module i sits on axis i: unaffected by phi_i
    return out
def U(phi,m):
    r=positions(phi); return -G*sum(m[i-1]*(ghat@r[i]) for i in range(1,8))
def grad(phi,m,h=1e-6):
    g=np.zeros(7)
    for j in range(7):
        d=np.zeros(7); d[j]=h; g[j]=(U(phi+d,m)-U(phi-d,m))/(2*h)
    return g
def equilibrium(cmd,m,kp,phi0=None):
    phi=np.zeros(7) if phi0 is None else phi0.copy()
    for _ in range(60):
        f=kp*(cmd-phi)-grad(phi,m)               # motor torque - gravity torque = 0
        J=np.zeros((7,7)); h=1e-5
        for j in range(7):
            d=np.zeros(7); d[j]=h; J[:,j]=(( kp*(cmd-(phi+d))-grad(phi+d,m)) - f)/h
        step=np.linalg.solve(J,-f); phi=phi+step
        if np.max(abs(step))<1e-10: break
    return phi, kp*(cmd-phi)                      # joint angles, motor torques
def _demo() -> None:
    m=np.full(7,0.275); kp=0.8
    H=np.array([[ (grad(np.eye(7)[j]*1e-4,m)-grad(-np.eye(7)[j]*1e-4,m))[i]/2e-4 for j in range(7)] for i in range(7)])
    print("chain direction (gravity):",np.round(ghat,3))
    print("diagonal of the gravity Hessian (isolated-joint stiffness, N.m/rad):",np.round(np.diag(H),3))
    print("simple formula  g*0.5*m*0.059*n(n-1)/2     :",[round(G*0.5*0.275*0.059*(7-k)*(7-k+1)/2,3) for k in range(1,8)])
    print("\nMeasured-at-joint stiffness with the others held by kp=0.8 springs (command ±0.05 rad on joint k):")
    for k in range(1,8):
        c=np.zeros(7); c[k-1]=0.05; ph,t=equilibrium(c,m,kp); cm=-c; phm,tm=equilibrium(cm,m,kp)
        K=(t[k-1]-tm[k-1])/(ph[k-1]-phm[k-1])
        print(f"  servo {k}: effective K = {K:.3f} N.m/rad   (tested joint moves {ph[k-1]:+.4f} rad for a command of 0.05; torque {t[k-1]:+.4f})")


if __name__ == "__main__":
    _demo()
