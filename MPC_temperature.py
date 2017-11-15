__author__ = 'Olivier Van Cutsem'

from building_data_management.category_management.category_config import *
from ems_config import *
from ems_main import EnergyManagementSystem
from building_data_management.signal_and_constraint_management.constraint_data import Constraint
import numpy as np
import control as control
import cvxpy

import bmsinterface.bms_interface_config

from building_data_management.signal_and_constraint_management.signal_data import PiecewiseConstantSignal
class EMS_MPC_temperature(EnergyManagementSystem):

    def __init__(self, queues, unit=None, ems_time_step=None, simu_starting_day=None, data_path=None):
        """
        Initialize ClusteringManagementSystem object.
        Read messages from the BMS interface bubble messages through a Queue

        """

        super(EMS_MPC_temperature, self).__init__(queues, unit, ems_time_step, simu_starting_day, data_path)

    def initModelParameters(self):

        self.T = 96
        # self.N=96

        # self.surface=120
        # self.thickness=0.2
        # self.coeff_lambda=0.5
        # self.Rcond=self.thickness/(self.coeff_lambda*self.surface)

        # self.p_wall=1000
        # self.cm_wall=1000
        # self.cw=self.p_wall*self.thickness*self.surface*self.cm_wall

        # self.kc=5
        # self.Rconv=(self.kc*self.surface)

        # self.R=self.Rcond/2+self.Rconv

        for n_id,l_int in self.building_data.room(12).interfaces.items():
            for i in l_int:
                th_prop_obj = i.thermalProp

        l_interf = self.building_data.room(12).interfaces[13]
        int_interest = l_interf[0]

        self.R = int_interest.thermalProp.resistiveParameters[0]
        self.cw = int_interest.thermalProp.capacitiveParam[0]

        print(self.R)
        print(self.cw)

        self.p_air = 1.292
        self.Vroom = 192
        self.cm_air = 1005
        self.cr = self.p_air * self.Vroom * self.cm_air

        # self.upper_bond=np.array[[26],[np.inf]]
        # self.lower_bond = np.array[[20], [-np.inf]]

        self.t_wall = 15  # to define

    def mpc_control(self, temp_cur, temp_wall_cur, temp_ext, temp_constr, elec_price):

        x = cvxpy.Variable(2, self.T + 1)
        u = cvxpy.Variable(1, self.T)
        epsi = cvxpy.Variable(1, self.T + 1)
        q = 100

        A, B, B1 = self.get_model()

        totalcost = 0.0
        constr = []
        for t in range(self.T):

            min_bound = temp_constr.get_min(t)
            max_bound = temp_constr.get_max(t)

            totalcost += elec_price[t]*u[t]
            totalcost += cvxpy.quad_form(epsi[t + 1], q)
            constr += [x[:, t + 1] == A * x[:, t] + B * u[t] + B1*temp_ext[t]]
            constr += [x[0, t + 1] <= max_bound+epsi[t + 1]]
            constr += [x[0, t + 1] >= min_bound-epsi[t + 1]]
            constr += [u[t] <= 5000]
            constr += [u[t] >= 0]

        constr += [x[:, 0] == np.array([[temp_cur],[temp_wall_cur]])]
        constr += [epsi[0] == 30]
        prob = cvxpy.Problem(cvxpy.Minimize(totalcost), constr)

        prob.solve(verbose=False)

        if prob.status == cvxpy.OPTIMAL:

            p_opt = np.array(u.value[0, :]).flatten()
            print(p_opt)
            return p_opt[0]
        else:
            print("Problem infeasible")

    def get_model(self):
        Acont=np.array([[-1/(self.cr*self.R), 1/(self.cr*self.R)],
                        [1/(self.cw * self.R), -2/(self.cw * self.R)]])
        Bcont=np.array([[1/self.cr, 0],
                        [0, 1/(self.cw*self.R)]])
        C=np.array([[1, 0]])
        D=np.array([[0,0]])
        sys=control.ss(Acont,Bcont,C,D)
        sysd = control.sample_system(sys,SIMULATION_TIME_STEP)
        Adisc,Bdisc,Cdisc,Ddisc=control.ssdata(sysd)

        return Adisc, Bdisc[:,0],Bdisc[:,1]

    def update(self):
        """
        TODO
        """
        commands_set = []

        l_hvac = self.building_data.get_entity_list([EMS_CATEGORY_ENTITY_LOAD, EMS_CATEGORY_ENTITY_LOAD_THERM])

        for (hvac_id, hvac_obj) in l_hvac:

            r_id = self.get_thermal_load_room(hvac_id)
            obj_room = self.building_data.room(r_id)

            temp_constr = obj_room.get_comfort_constraint(EMS_COMFORT_temperature)
            temp_cur = obj_room.get_comfort_value(EMS_COMFORT_temperature)

            #for r_id in self.building_data.room_ids_list():
            #    if self.building_data.room(r_id).has_forcast((self.current_time, self.current_time+self.T, self.ems_dt),EMS_COMFORT_temperature):
             #       outside = self.building_data.room(r_id)
            outside = self.building_data.room(13)

            ext_temperature = outside.get_forecast(EMS_COMFORT_temperature,
                                                   (self.current_time, self.current_time+self.T, self.ems_dt),
                                                   self.environment_data)
            elec_price = self.get_elec_price((self.current_time, self.current_time+self.T, self.ems_dt))
            print("temp cur =" + str(temp_cur))

            if temp_cur != None:

                p_opt = self.mpc_control(temp_cur, self.t_wall, ext_temperature, temp_constr,elec_price)

                print(ext_temperature)
                print((self.starting_day * 24 * 60 * 60)%self.current_time)
                self.t_wall=(temp_cur+ext_temperature[(self.starting_day * 24 * 60 * 60)%self.current_time])/2 #TO CHANGE : design an observer
                command_hvac = self.hvac_set_point(hvac_id, p_opt)
                commands_set.append(command_hvac)

        return commands_set

    def get_thermal_load_room(self, th_l_id):
        """

        :param th_l_id: a numerical value that represents the id of the HVAC load
        :return: a numerical value that represents the id of the room that contains the thermal load of id th_l_id
        """
        for r_id in self.building_data.room_ids_list:
            if th_l_id in self.building_data.room(r_id).get_comfort_loads(EMS_COMFORT_temperature):
                return r_id

        return None

