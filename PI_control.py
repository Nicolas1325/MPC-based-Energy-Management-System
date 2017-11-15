__author__ = 'Olivier Van Cutsem'

from building_data_management.category_management.category_config import *
from ems_config import *
from ems_main import EnergyManagementSystem
from building_data_management.signal_and_constraint_management.constraint_data import Constraint

## Various strategies

class EMS_PI_control(EnergyManagementSystem):

    def __init__(self, queues, unit=None, ems_time_step=None, simu_starting_day=None, data_path=None):
        """
        Initialize ClusteringManagementSystem object.
        Read messages from the BMS interface bubble messages through a Queue

        """
        #Use 3 min time step for the controller
        self.previous_error=0
        self.error=0
        self.Kp = 0.15
        self.Ki = 0.02
        self.integral_error=0.0
        self.wind_up_value=0.0
        self.controller=0
        self.controller_output=0
        super(EMS_PI_control, self).__init__(queues, unit, ems_time_step, simu_starting_day, data_path)

    def update(self):
        """
        TODO
        """
        commands_set = []

        l_hvac = self.building_data.get_entity_list([EMS_CATEGORY_ENTITY_LOAD, EMS_CATEGORY_ENTITY_LOAD_THERM])

        for (hvac_id, hvac_obj) in l_hvac:

            r_id = self.get_thermal_load_room(hvac_id)
            obj_room = self.building_data.room(r_id)

            print("room volume = "+str(obj_room.volume))
            constr = obj_room.get_comfort_constraint(EMS_COMFORT_temperature)
            min_bound = constr.get_min(self.current_time)
            max_bound = constr.get_max(self.current_time)
            ref=28#float(max_bound)
            print("ref =" + str(ref))

            temp_cur = obj_room.get_comfort_value(EMS_COMFORT_temperature)

            print(type(temp_cur))
            print("temp cur =" + str(temp_cur))


            if temp_cur != None:
                self.error=ref-temp_cur
                self.integral_error=self.integral_error+(self.error+self.previous_error)/2.0*SIMULATION_TIME_STEP
                print("integral error=" + str(self.integral_error))
                self.previous_error=self.error

                self.controller = self.Kp*self.error+self.Ki*self.integral_error #add windup

                if self.controller > 100:
                    self.controller=100
                elif self.controller < 0:
                    self.controller=0
                print("controller =" + str(self.controller))

                 #self.wind_up_value = self.Ki / self.Kp * (self.controller - self.controller_output)
                print(self.wind_up_value)
                command_hvac = self.hvac_set_point(hvac_id, self.controller)
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
