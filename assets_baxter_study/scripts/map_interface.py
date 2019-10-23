import yaml
import numpy as np
import alloy
import random
import rospy
import os

class MapInterface():

    def __init__(self, dir_to_path_folder):
        self._dir_to_path_folder = dir_to_path_folder
        self._min_route_num = 1
        self._max_route_num = 2
    
    def get_random_route(self):
        #get the a random route in the range
        picked_route = random.randint(self._min_route_num, self._max_route_num)
        route_file_name = "route{}.yaml".format(picked_route)
        rospy.loginfo("picked route: {}".format(route_file_name))
        #process that route
        return self._process_route_file(route_file_name)
    
    def get_route(self, route_num):
        route_file_name = "route{}.yaml".format(route_num)
        return self._process_route_file(route_file_name)


    def _process_route_file(self, route_file_name):
        path_to_route = os.path.join(self._dir_to_path_folder, route_file_name)
        with open(path_to_route, 'r') as f:
            obj = yaml.load(f,Loader=yaml.FullLoader)
            
            init_dir = obj['init_dir']
            final_dir = obj['final_dir']
            cur_dir = init_dir
            
            route_instructions = obj['route']
            for i,n in enumerate(route_instructions):
                if i < len(route_instructions) - 1:
                    nxt_node = route_instructions[i+1]

                    cor1 = np.array([n['x'], n['y']])
                    cor2 = np.array([nxt_node['x'], nxt_node['y']])

                    #get the distance
                    dist = alloy.math.distance(cor1, cor2)
                    route_instructions[i]['dist'] = dist

                    dir_to_next_node = alloy.math.normalize(cor2 - cor1)
                    rot = alloy.math.find_rotation(cur_dir, dir_to_next_node)
                    route_instructions[i]['rot'] = rot
                    route_instructions[i]['clock'] = alloy.math.convert_2D_rot_to_clock(rot)

                    cur_dir = dir_to_next_node

                else:
                    #this is the last node
                    rot = alloy.math.find_rotation(cur_dir, final_dir)
                    route_instructions[i]['rot'] = rot
                    route_instructions[i]['clock'] = alloy.math.convert_2D_rot_to_clock(rot)

            obj['route'] = route_instructions
            return obj


    def get_example_route(self):

        inst = dict()
        inst['starting_description'] = "Let us start with you facing me here at the lobby."
        inst['route'] = [
            {
                "x":0,
                "y":0,
                "dist":10,
                "rot":-np.pi/2,
                "clock":3
            },
            {
                "x":10,
                "y":0,
                "dist":50,
                "rot":np.pi/2,
                "clock":9,
                "cue": " until the end of the hallway."
            },
            {
                "x":10,
                "y":50,
                "dist":20,
                "rot":-np.pi/2,
                "clock":3,
            },
            {
                "x":30,
                "y":50,
                "dist":10,
                "rot":np.pi/2,
                "clock":9,
                "cue": " and the hallway will now open up."
            },
            {
                "x":30,
                "y":40,
                "dist":0,
                "rot":np.pi/4,
                "clock":10
            }
        ]
        return inst


if __name__ == "__main__":
    f = FakeMap()
    print(f.get_route(1))