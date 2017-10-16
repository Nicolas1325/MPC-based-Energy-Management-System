__author__ = 'Olivier Van Cutsem'

from building_data_management.category_management.category_config import *
from ems_config import *
from ems_main import EnergyManagementSystem
from building_data_management.signal_and_constraint_management.constraint_data import Constraint

## Various strategies

class EMS_Thermostat(EnergyManagementSystem):

    def __init__(self, queues, unit=None, ems_time_step=None, simu_starting_day=None, data_path=None):
        """
        Initialize ClusteringManagementSystem object.
        Read messages from the BMS interface bubble messages through a Queue

        """
        super(EMS_Thermostat, self).__init__(queues, unit, ems_time_step, simu_starting_day, data_path)

    def update(self):
        """
        TODO
        """
        commands_set = []

        l_hvac = self.building_data.get_entity_list([EMS_CATEGORY_ENTITY_LOAD, EMS_CATEGORY_ENTITY_LOAD_THERM])

        for (hvac_id, hvac_obj) in l_hvac:

            r_id = self.get_thermal_load_room(hvac_id)
            obj_room = self.building_data.room(r_id)

            constr = obj_room.get_comfort_constraint(EMS_COMFORT_temperature)
            min_bound = constr.get_min(self.current_time)
            max_bound = constr.get_max(self.current_time)
            print(min_bound)
            print(max_bound)
            temp_cur = obj_room.get_comfort_value(EMS_COMFORT_temperature)

            print(temp_cur)
            if temp_cur<min_bound:
                command_hvac = self.hvac_set_point(hvac_id, 100)
                commands_set.append(command_hvac)
                print('lower')
            elif temp_cur > max_bound:
                command_hvac = self.hvac_set_point(hvac_id, 0)
                commands_set.append(command_hvac)
                print('higher')

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
