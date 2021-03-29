#****************************************************************************
#
# Do NOT modify or remove this copyright and confidentiality notice!
#
# Copyright (c) 2019 - $Date: 2019/04/28 $ NCS PTE LTD.
#
# The code contained herein is CONFIDENTIAL to NCS.
# Portions are also trade secret. Any use, duplication, derivation, distribution
# or disclosure of this code, for any reason, not expressly authorized is
# prohibited. All other rights are expressly reserved by NCS PTE LTD.
#
# Designer: john.liao
# Description:RFM task format to json format converter.
#
#****************************************************************************
import json
from datetime import datetime

from rmf_fleet_msgs.msg import Location
#from rna_task_msgs.msg import Location as rna_pos
import nudged

# the robot 
ROBOT_UNIQUE_ID = os.environ.get('ROBOT_ID', 'RNA1')
# topic define: 
RMF_TASK                 = '/rna_task'
RMF_TASK_STATUS          = '/rna_task_status'
RMF_VSM_RECORD           = '/rna_vsm_record'
RMF_PARSE_REQUESTS       = '/parse_requests'
RMF_FLEET_STATES         = '/fleet_states'
RMF_MODE_REQUESTS        = '/robot_mode_requests'
RMF_PATH_REQUESTS        = '/robot_path_requests'
RMF_DESTINATION_REQUESTS = '/robot_destination_requests'

class map_transformer:
    def __init__(self, doman_pionts, range_pionts): # list of [x,y] points
        self.trans       = nudged.estimate(doman_pionts, range_pionts)
        self.mse         = nudged.estimate_error(self.trans, doman_pionts, range_pionts)
        self.matrix      = self.trans.get_matrix()
        self.scale       = self.trans.get_scale()
        self.rotation    = self.trans.get_rotation()
        self.translation = self.trans.get_translation()
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
def romih2_robot_position(trans, pos, bed_heading=None):
    x, y = trans.transform(pos.x, pos.y)
    heading = pos.heading + trans.rotation
    if bed_heading:
        bh = bed_heading + trans.rotation
        return(x, y, heading, bh)

    return (x, y , heading)
if __name__ == "__main__":
    #jdata = '{"name": "Brian", "city": "Seattle"}' #"""%s"""%str(patient_A)
    #jdata = '{"persons": [{"name": "Brian", "city": "Seattle"}, {"name": "David", "city": "Amsterdam"} ] }'
    #jdata ="""%s""" % str(patient_A)
    jdata = '{ "person":{"name":"Mr A","birthday":"1990 Jan 10","address":"30 cityhall road, s480029","contact":"98345678"}, \
            "ward_num"       : 25, \
            "bed_num"        : 10, \
            "facial_id"      : 123, \
            "barcode"        : 789, \
            "RFID"           : 456, \
            "navigation"     : {"destination":null, "position":[0.1, 0.2, 0.3]}, \
            "vsm_rec"        : {"2019-04-27 23:16:39":{"HR":80, "BP":[110, 70], "Temp":36.5, "RR":30, "SpO2":98}} \
            }'
    #print(jdata)
    
    #python_obj = json.loads(jdata)
    #print(python_obj)
    po = patient_obj(jdata)
    print(po)
    python_obj = po.profile
    
    ward_num = python_obj['ward_num']
    navig = python_obj['navigation']['position']
    print('ward_num={},type:{}'.format(ward_num, type(ward_num)))
    print(navig)
    #print (json.dumps(python_obj, ensure_ascii=False, indent=4))
    
    
    
    
