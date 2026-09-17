import numpy as np

def atmosphere_from_reference():
    return dict(h=10000.0,V=1200.0,T=223.26,rho=0.413511,p=26499.0,
                a=299.532,M=4.0062,q=297720.0)

def geometry():
    L1,L2=0.12,0.18
    th1=np.deg2rad(15.0)
    th2=np.deg2rad(8.784531)
    R1=L1*np.tan(th1)
    Rmax=0.060
    return dict(L1=L1,L2=L2,theta1=th1,theta2=th2,R1=R1,Rmax=Rmax,
                L=L1+L2,Aref=np.pi*Rmax**2,c=0.30,b=0.12)

def inertia():
    I=np.array([[4.9138e-4,0,-5.1552e-4],
                [0,4.5830e-3,0],
                [-5.1552e-4,0,4.5126e-3]])
    return I

def rigid_body_angular_acceleration(omega, moment, I=None):
    if I is None: I=inertia()
    return np.linalg.solve(I, np.asarray(moment)-np.cross(omega,I@omega))

def quaternion_derivative(q, omega):
    # q=[qw,qx,qy,qz], body angular rate
    qw,qx,qy,qz=q
    p,rq,r=omega
    Om=np.array([[0,-p,-rq,-r],
                 [p,0,r,-rq],
                 [rq,-r,0,p],
                 [r,rq,-p,0]])
    return 0.5*Om@np.asarray(q)

def aero_coefficients(alpha, delta=0.0, q_rate=0.0, V=1200.0, c=0.30):
    CL0=0.0; CD0=0.208152; Cm0=0.0
    CLa=-0.228236; CLd=0.15; CDd=0.05
    Cmd=-0.50; Cmq=-2.0
    CL=CL0+CLa*alpha+CLd*delta
    CD=CD0+CDd*delta
    rate_term=(q_rate*c/(2*max(abs(V),1e-9)))
    Cm=Cm0+Cmq*rate_term+Cmd*delta
    return CL,CD,Cm

def forces_moments(alpha,delta,q_rate,V=1200.0,rho=0.413511):
    g=geometry(); qdyn=0.5*rho*V*V
    CL,CD,Cm=aero_coefficients(alpha,delta,q_rate,V,g["c"])
    # Body-axis diagnostic: X drag negative, Z lift convention
    Fx=-CD*qdyn*g["Aref"]
    Fz=-CL*qdyn*g["Aref"]
    My=Cm*qdyn*g["Aref"]*g["c"]
    return np.array([Fx,0.0,Fz]),np.array([0.0,My,0.0]),(CL,CD,Cm)

def actuator_second_order(delta,delta_dot,delta_cmd,dt,tau=0.005,zeta=1.0):
    dd=(delta_cmd-delta-2*zeta*tau*delta_dot)/(tau*tau)
    return delta_dot+dt*dd, dd

def clamp_actuator(delta,delta_dot):
    rate_max=2.617994; pos_max=0.261799
    delta_dot=np.clip(delta_dot,-rate_max,rate_max)
    delta=np.clip(delta,-pos_max,pos_max)
    return delta,delta_dot

def thermal_state(eta=1.0):
    # Bounded model from project archive; intentionally not a destructive-energy model.
    Tmax=223.26 + eta*(917.95-223.26)
    return Tmax

def orthotropic_surrogate(T=821.45):
    Tref=298.15; dT=T-Tref
    E1=145e9*(1-1e-4*dT)
    E2=65e9*(1-1.1e-4*dT)
    E3=12e9*(1-1.5e-4*dT)
    return dict(E1=E1,E2=E2,E3=E3,
                G12=8.5e9,G13=6.2e9,G23=4.1e9,
                alpha1=0.8e-6,alpha2=2.2e-6,alpha3=5.4e-6)

def run_case(duration=0.5,dt=0.00025,alpha0=np.deg2rad(2),
             p0=0.0,q0=np.deg2rad(1.0),r0=0.0,
             Kp=0.5,Kd=0.010,q_cmd=0.0):
    m=0.525; I=inertia()
    state=np.array([0.0,0.0,10000.0,1200.0,alpha0,q0])
    omega=np.array([p0,q0,r0],dtype=float)
    quat=np.array([1.,0.,0.,0.])
    delta=0.0; delta_dot=0.0
    rows=[]
    n=int(round(duration/dt))
    for k in range(n+1):
        t=k*dt
        V=max(state[3],1.0); alpha=state[4]
        # Reduced controller uses pitch rate and a bounded derivative signal.
        # Derivative is computed from the current model moment.
        _,M,_=forces_moments(alpha,delta,omega[1],V)
        qdot=rigid_body_angular_acceleration(omega,M,I)[1]
        delta_cmd=Kp*(omega[1]-q_cmd)+Kd*qdot
        delta_dot,dd=actuator_second_order(delta,delta_dot,delta_cmd,dt)
        delta,delta_dot=clamp_actuator(delta,delta_dot)
        F,M,aero=forces_moments(alpha,delta,omega[1],V)
        qdot=rigid_body_angular_acceleration(omega,M,I)[1]
        rows.append([t,state[2],V,alpha,omega[0],omega[1],delta,delta_dot,qdot,
                     F[0],F[2],M[1],aero[0],aero[1],aero[2],np.linalg.norm(quat)])
        # Lightweight explicit state update for repository smoke simulation.
        ax=F[0]/m
        V=max(1.0,V+ax*dt)
        alpha += omega[1]*dt
        omega[1] += qdot*dt
        quat += quaternion_derivative(quat,omega)*dt
        quat /= np.linalg.norm(quat)
        state[2] += V*np.sin(alpha)*dt
        state[3]=V
        state[4]=alpha
    return np.asarray(rows)

def save_csv(path,rows):
    header="t_s,h_m,V_mps,alpha_rad,p_rad_s,q_rad_s,delta_rad,delta_dot_rad_s,qdot_rad_s2,Fx_N,Fz_N,My_Nm,CL,CD,Cm,quat_norm"
    np.savetxt(path,rows,delimiter=",",header=header,comments="")
