import numpy as np
import control as control
import cvxpy
import matplotlib.pyplot as plt
import time

kelvin=273.15
class test_EMS_MPC_temperature():
    def __init__(self):


        self.ems_dt=10*60
        self.T = 60*60*1/self.ems_dt
        self.M=60*60*1/self.ems_dt #Number of simulation steps
        self.R = 0.0066
        self.cw = 1565600
        self.p_air = 1.292
        self.Vroom = 192
        self.cm_air = 1005
        self.cr = self.p_air * self.Vroom * self.cm_air

    def mpc_control(self, temp_cur, temp_wall_cur, temp_ext, elec_price):

        x = cvxpy.Variable(2, self.T + 1)
        u = cvxpy.Variable(1, self.T)
        epsi = cvxpy.Variable(1, self.T + 1)
        q = 100

        A, B, B1 = self.get_model()

        totalcost = 0.0
        constr = []
        for t in range(self.T):
            totalcost += elec_price[t]*u[t] #+ cvxpy.quad_form(epsi[t + 1], q)
            constr += [x[:, t + 1] == A * x[:, t] + B * u[t] + B1*temp_ext[t]]

            constr += [x[0, t + 1] < kelvin+30]#+epsi[t + 1]]
            constr += [x[0, t + 1] > kelvin+26]#-epsi[t + 1]]
            constr += [u[t] <= 5000]
            constr += [u[t] >= 0]
        constr += [x[:, 0] == np.array([[temp_cur],[temp_wall_cur]])]
        #constr += [epsi[0] == 30]

        prob = cvxpy.Problem(cvxpy.Minimize(totalcost), constr)
        t0 = time.time()
        prob.solve(verbose=True, solver=cvxpy.ECOS)
        t1 = time.time()
        print(t1 - t0)
        if prob.status == cvxpy.OPTIMAL:
            p_opt = np.array(u.value[0, :]).flatten()
            return p_opt[0]
        else:
            print("problem infeasible")

    def get_model(self):
        Acont=np.array([[-1/(self.cr*self.R), 1/(self.cr*self.R)],
                        [1/(self.cw * self.R), -2/(self.cw * self.R)]])
        Bcont=np.array([[1/self.cr, 0],
                        [0, 1/(self.cw*self.R)]])
        C=np.array([[1, 0]])
        D=np.array([[0,0]])
        sys=control.ss(Acont,Bcont,C,D)
        sysd = control.sample_system(sys,self.ems_dt)
        Adisc,Bdisc,Cdisc,Ddisc=control.ssdata(sysd)

        return Adisc, Bdisc[:,0],Bdisc[:,1]

    def update(self, current_t, wall_temperature, ext_temperature,  elec_price):
        p_opt = self.mpc_control(current_t, wall_temperature, ext_temperature, elec_price)
        return p_opt

def get_elec_price(N, M):
    elec_price = [0.02]
    for i in range(N + M - 1):
        if i <= round((N + M - 1) / 16):
            elec_price_t = 0.02
        if (i > round((N + M - 1) / 16)) and (i <= round((N + M - 1) / 8)):
            elec_price_t = 0.15
        if (i > round((N + M - 1) / 8)) and (i <= round(3 * (N + M - 1) / 16)):
            elec_price_t = 0.06
        if (i > round(3 * (N + M - 1) / 16)) and (i <= round((N + M - 1) / 4)):
            elec_price_t = 0.02
        if (i > round((N + M - 1) / 4)) and (i <= round(5 * (N + M - 1) / 16)):
            elec_price_t = 0.08
        if (i > round(5 * (N + M - 1) / 16)) and (i <= round(3 * (N + M - 1) / 8)):
            elec_price_t = 0.2
        if (i > round(3 * (N + M - 1) / 8)) and (i <= round(7 * (N + M - 1) / 16)):
            elec_price_t = 0.1
        if (i > round(7 * (N + M - 1) / 16)) and (i <= (N + M - 1) / 2):
            elec_price_t = 0.02
        if (i > round((N + M - 1) / 2)) and i <= round(9 * (N + M - 1) / 16):
            elec_price_t = 0.02
        if (i > round(9 * (N + M - 1) / 16)) and (i <= round(5 * (N + M - 1) / 8)):
            elec_price_t = 0.15
        if (i > round(5 * (N + M - 1) / 8)) and (i <= round(11 * (N + M - 1) / 16)):
            elec_price_t = 0.06
        if (i > round(11 * (N + M - 1) / 16)) and (i <= round(3 * (N + M - 1) / 4)):
            elec_price_t = 0.02
        if (i > round(3 * (N + M - 1) / 4)) and (i <= round(13 * (N + M - 1) / 16)):
            elec_price_t = 0.08
        if (i > round(13 * (N + M - 1) / 16)) and (i <= round(7 * (N + M - 1) / 8)):
            elec_price_t = 0.2
        if (i > round(7 * (N + M - 1) / 8)) and (i <= round(15 * (N + M - 1) / 16)):
            elec_price_t = 0.1
        if (i > round(15 * (N + M - 1) / 16)) and (i <= N + M - 1):
            elec_price_t = 0.02

        elec_price += [elec_price_t];
    return elec_price

def get_ext_temp(N,M):
    Tavg=kelvin+5
    Text=[Tavg]
    Magnetude=5
    for i in range(N+M-1):
        Text+=[Tavg+Magnetude*np.sin(2*np.pi*i/M)]
    return Text


ems=test_EMS_MPC_temperature()
ext_temperature = get_ext_temp(ems.T, ems.M)
elec_price = get_elec_price(ems.T, ems.M)

Tins=kelvin+27
Twall=(Tins+ext_temperature[0])/2
T=np.array([[Tins], [Twall]])

history_Tins=[]
history_P=[]
history_Tw=[]

for i in range(ems.M):
    optimal_P = ems.update(Tins, Twall, ext_temperature[i:i+ems.T], elec_price[i:i+ems.T])
    [A,B1,B2]=ems.get_model()
    T=np.dot(A,T)+np.dot(B1,optimal_P)+np.dot(B2,ext_temperature[i])
    Tins=T.item(0)
    Twall=T.item(1)
    history_Tins+=[Tins-kelvin]
    history_Tw+=[Twall-kelvin]
    history_P+=[optimal_P]

plt.figure()
plt.subplot(311)
plt.plot(range(ems.M),history_P)
plt.title("power")
plt.subplot(312)
plt.plot(range(ems.M),history_Tw, label="1")
plt.plot(range(ems.M),history_Tins, label="2")
plt.title("Temperature")
plt.subplot(313)
plt.plot(range(ems.M),elec_price[0:ems.M])
plt.title("Electricity price")
plt.show()

