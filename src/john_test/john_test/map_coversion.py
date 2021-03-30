
import json
from datetime import datetime

from rmf_fleet_msgs.msg import Location
#from rna_task_msgs.msg import Location as rna_pos
import nudged



class map_transformer:
    def __init__(self, doman_pionts, range_pionts): # list of [x,y] points
        self.trans       = nudged.estimate(doman_pionts, range_pionts)
        self.mse         = nudged.estimate_error(self.trans, doman_pionts, range_pionts)
        self.matrix      = self.trans.get_matrix()
        self.scale       = self.trans.get_scale()
        self.rotation    = self.trans.get_rotation()
        self.translation = self.trans.get_translation()
        print('\t scale:{}'.format(self.scale))
        print('\t rotation:{}'.format(self.rotation))
        print('\t translation:{}'.format(self.translation))
        
    def transform(self, x, y):
        return self.trans.transform([x, y])

def robot_2_romih_coordinate(trans, x, y, heading):
    loc = Location()
    loc.x, loc.y = trans.transform(x, y)
    loc.yaw = heading + trans.rotation
    return loc
def romih_2_robot_coordinate(trans, location):
    x, y = trans.transform(location.x, location.y)
    heading = location.yaw + trans.rotation
    return (x, y, heading)

if __name__ == "__main__":
    ROBOT_MAP_PIONTS = [[-18.6, -10.9],
                    [-9.22, -11.1],
                    [-9.23, -21.4],
                    [-16.3, -21.4]]
    RMF_MAP_POINTS   = [[11.4, -13.8], 
                    [20.9, -13.7], 
                    [20.9, -24.8], 
                    [14.4, -24.8]] 
    print('robot2rmf_map_trans')
    robot2rmf_map_trans = map_transformer(ROBOT_MAP_PIONTS, RMF_MAP_POINTS)
    print('rmf2robot_map_trans')
    rmf2robot_map_trans = map_transformer( RMF_MAP_POINTS, ROBOT_MAP_PIONTS)

    
    
    
    
