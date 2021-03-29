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
# Description: The task receiver and scheduler for Mission Control.
#
#****************************************************************************
import csv
import sys, time, os, json, argparse, schedule, traceback
from datetime import datetime
from threading import Thread
import rclpy
from rna_task_msgs.msg import RnaTask, RnaTaskstatus, Location, RnaEmergency, RnaPredefinepos
from rmf_fleet_msgs.msg import FleetState, RobotState, RobotMode
from rmf_fleet_msgs.msg import PathRequest, ModeRequest, DestinationRequest
from rna_utilities import RNA_Node, Topic, ResponseStatus, base_client, RNA_logger
from mission_control.rna_greeting import *
from std_msgs.msg import String
from rna_msgs.msg import NLPTTS
from .mc_constant import *
from .json_task import load_task, save_task, MCTask, rmf_to_mc_task, romih_2_robot_coordinate, robot_2_romih_coordinate, map_transformer
from rclpy.executors import MultiThreadedExecutor
from rna_utilities.constants import ROS_DISTRO_USE_ELOQUENT
if ROS_DISTRO_USE_ELOQUENT:
    from rclpy.utilities import remove_ros_args
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# four points coordinate which from rmf floor plan map to robot navigation map
# to be change below constants based on different place of follor plan accordingly
NURSE_STATION = [0.581, 19.511, -1.511]
HOME_POSITION = [-2.912, 18.559, 1.561]
Neutral_point = [-2.324, 21.091, 1.557, 1.557]
Bedside_right = [-1.485, 21.229, 2.143, 1.557]
ROBOT_MAP_PIONTS = [[NURSE_STATION[0], NURSE_STATION[1]],
                    [HOME_POSITION[0], HOME_POSITION[1]],
                    [Neutral_point[0], Neutral_point[1]],
                    [Bedside_right[0], Bedside_right[1]]]
RMF_MAP_POINTS   = [[75.787, -23.029], # map to NURSE_STATION
                    [73.013, -23.350], # map to HOME_POSITION
                    [73.537, -20.965], # map to Neutral_point 
                    [74.57, -20.78]]   # map to Bedside_right 

NOT_CHARGING =0
WAITING_FOR_CHARGING =1
CHARGING_ON_GOING=2
BATTERY_CHARGED=3
BatteryStatus_dict =dict(percentage=80.0, is_charging=False, is_charged=False)
FleetStatus_dict = dict(task_id='T001', mode=0, path=None)
ExecutorStatus2Mode={RobotStatus.IDLE_STATE: RobotMode.MODE_IDLE,
                     RobotStatus.AUTHEN_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.TEMP_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.SPO2_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.RR_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.BP_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.MD_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.VA_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.NLP_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.CHARGING_WIP: RobotMode.MODE_CHARGING,
                     RobotStatus.NAVI_WIP: RobotMode.MODE_MOVING,
                     RobotStatus.ITEM_DELIVERY_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.LOAD_ITEM_WIP: RobotMode.MODE_WAITING,
                     RobotStatus.GOING_HOME_WIP: RobotMode.MODE_GOING_HOME,
                     RobotStatus.GOING_NURSE_STATION: RobotMode.MODE_MOVING,
                     RobotStatus.EMERGENCY_ESCAPE: RobotMode.MODE_EMERGENCY,
                     RobotStatus.PATIENT_ORIENTATION: RobotMode.MODE_WAITING}

class RNA_robot_status(RNA_Node):
    def __init__(self, low_battery_percentage, power_ready_battery_percentage, charging_log=False, debug=True, logger=None):
        super().__init__('robotStatus', logger)
        self.debug = debug
        self.charging_log = charging_log
        self.low_battery_percentage = low_battery_percentage
        self.power_ready_battery_percentage = power_ready_battery_percentage
        self.robot2rmf_map_trans = map_transformer(ROBOT_MAP_PIONTS, RMF_MAP_POINTS)
        # initialize robot status
        self.robot_status = dict(battery=dict(percentage=80.0, is_charging=False), 
                                 location=dict(x=0.0, y=0.0, heading=0.0),
                                 map_ver='1.0',
                                 robot_state='Idle',
                                 robot_id=ROBOT_UNIQUE_ID,
                                 system=dict(temperature=30.0, fan_on=False))
        # pubilish the robot status to Backend
        self.pub_robot_status = self.rna_create_publisher(String, Topic.MC_ROBOT_STATUS, local_host_only=False)
        
        # pubilish the robot status to RMF
        self.pub_robot_status_2_rmf = self.rna_create_publisher(FleetState, Topic.RMF_FLEET_STATES, local_host_only=False)        
        
        # subscrip the base location 
        self.rna_create_subscription(
            String, Topic.BASE_LOCATION, self.base_location_callback)        
        # subscrip the battery status
        self.rna_create_subscription(
            String, Topic.BATTERY_STATUS, self.battery_status_callback)

        # subscrip robot task executing status 
        self.rna_create_subscription(
            String, Topic.MC_ROBOT_EXECUTING_STATUS, self.task_executing_status_callback)
        
        # subscrip robot battery top for charging/discharging debug
        if self.charging_log:
            self.rna_create_subscription(String, Topic.BATTERY_DEBUG, self.charging_debug_callback)

        # subscrip robot system temperature
        self.rna_create_subscription(
            String, Topic.SYS_TEMPERATURE, self.system_temperature_callback)

        # publish robot status in 1 second interval
        self.robot_status_tmr = self.create_timer(1.0, self.robot_status_timer_callbak)
    def system_temperature_callback(self, msg):
        recv_dict = json.loads(msg.data)
        self.robot_status['system']['temperature'] = recv_dict['temperature']
        self.robot_status['system']['fan_on'] = recv_dict['is_fan_on'] # True or False
    def charging_debug_callback(self, msg):
        info = json.loads(msg.data)
        log_file = os.path.expanduser('~/.ros/battery.csv')
        dtime = str(datetime.now())  # eg '2019-04-24 16:18:04.272857'
        dtime = dtime.split('.')[0].split(' ')
        dtime = dtime[0]+ '_' + dtime[1]
        is_file_exist=False
        if os.path.exists(log_file):
            is_file_exist = True

        with open(log_file, mode='a', newline='') as csv_file:
            writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            #way to write to csv file
            if not is_file_exist: # write header
                writer.writerow(['Date Time', 'Seconds', 'Current', 'Voltage', 'Percentage', 'Charging', 'Robot State'])
            row =list()
            row.append(dtime) # column Time
            row.append(int(time.monotonic())) # column Seconds
            row.append(round(info['current'], 3)) # column Current
            row.append(round(info['current_voltage'], 3)) # column Voltage
            row.append(round(info['percentage'], 1)) # column Percentage
            row.append(info['charging']) # column Charging
            row.append(self.robot_status['robot_state']) # column Robot State

            writer.writerow(row)

    def task_executing_status_callback(self, msg):
        recv_dict = json.loads(msg.data)
        self.robot_status['robot_state'] = recv_dict['executing']
        self.robot_status['robot_id'] = recv_dict['robot_id']
    def battery_status_callback(self, msg):
        battery = json.loads(msg.data)
        BatteryStatus_dict['percentage'] = round(battery['percentage'], 1)
        BatteryStatus_dict['is_charging'] = (battery['is_charging']== 'yes')
        self.robot_status['battery']['percentage']  = BatteryStatus_dict['percentage']
        is_charging = False # only battery > 50% report charging in progress, if chagring in progress, robot will not accept task
        if BatteryStatus_dict['is_charging'] and (BatteryStatus_dict['percentage'] < self.power_ready_battery_percentage):
            is_charging = True
        self.robot_status['battery']['is_charging'] = is_charging
    def base_location_callback(self, msg):
        base_status = json.loads(msg.data)
        self.robot_status['location']['x']       = round(base_status['x'], 3)
        self.robot_status['location']['y']       = round(base_status['y'], 3)
        self.robot_status['location']['heading'] = round(base_status['heading'], 3)
        self.robot_status['map_ver']             = base_status['map_version']

    def robot_status_timer_callbak(self):
        # status to dashboard
        msg = String()
        msg.data = json.dumps(self.robot_status)
        self.pub_robot_status.publish(msg)
        
        # status to RMF
        rmf_status = FleetState()
        rmf_status.name = 'rna'
        #rmf_status.robots = list()
        rna_status = RobotState()
        rna_status.name = ROBOT_UNIQUE_ID
        rna_status.model = 'NCS-RNA' 
        rna_status.task_id = FleetStatus_dict['task_id']
        rna_status.mode.mode = FleetStatus_dict['mode']
        #rna_status.mode.mode_request_id = # todo
        
        # battery info
        rna_status.battery_percent = self.robot_status['battery']['percentage']
        
        # current location
        loc = robot_2_romih_coordinate(self.robot2rmf_map_trans,
                                       self.robot_status['location']['x'],
                                       self.robot_status['location']['y'],
                                       self.robot_status['location']['heading'])
        loc.t = self.get_clock().now().to_msg()
        loc.level_name = 'B1' # todo: test code, need change it accordingly
        
        rna_status.location = loc
        if FleetStatus_dict['path']: # update the path field only robot is doing path_request
            rna_status.path.append(FleetStatus_dict['path'])

        rmf_status.robots.append(rna_status)
        self.pub_robot_status_2_rmf.publish(rmf_status)

class task_management(RNA_Node):
    def __init__(self, low_battery_percentage, power_ready_battery_percentage, debug=True, logger=None):        
        super().__init__('mcTaskMgnt', logger)
        self.callbackgroup = MutuallyExclusiveCallbackGroup()
        self.debug = debug
        self.low_battery_percentage = low_battery_percentage
        self.power_ready_battery_percentage = power_ready_battery_percentage        
        self.task_exector_status = RobotStatus.IDLE_STATE
        self.rmf_task_wip = False
        self.curr_executing_task_id = 'xxx'
        self.location_request = None
        self.nav_client = None
        self.mode_paused_request = False
        self.escape_on = False
        self.task_queue = list()
        self.scheduler = schedule.Scheduler()
        self.rmf2robot_map_trans = map_transformer(RMF_MAP_POINTS, ROBOT_MAP_PIONTS)
        # pubilish the task to task executor to execute    
        self.pub_2_task_executor = self.rna_create_publisher(String, Topic.MC_QUEUE_TASK)
        # publish tts msg
        self.pub_tts = self.rna_create_publisher(NLPTTS, Topic.NLP_TTS_MSG)
        # pubilish the task status to Backend
        self.pub_task_status = self.rna_create_publisher(String, Topic.MC_TASK_STATUS, local_host_only=False)
        # subscrip task from DDS(Backend/Adaptor)
        self.rna_create_subscription(
            String, Topic.MC_TASK_RECEIVER, self.task_receiver_callback, local_host_only=False)
        
        # pubilish the task status to RMF
        self.pub_rmf_task_status = self.rna_create_publisher(RnaTaskstatus, Topic.RMF_TASK_STATUS, local_host_only=False)        
        # subscrip task from RMF
        self.rna_create_subscription(
            RnaTask, Topic.RMF_TASK, self.rfm_task_receiver_callback, local_host_only=False)
        # subscrip ModeRequest from RMF
        self.rna_create_subscription(
            ModeRequest, Topic.RMF_MODE_REQUESTS, self.rfm_mode_request_callback, local_host_only=False)        
        # subscrip path request from RMF
        self.rna_create_subscription(
            PathRequest, Topic.RMF_PATH_REQUESTS, self.rfm_path_request_callback, local_host_only=False)                
        # subscrip destination request from RMF
        self.rna_create_subscription(
            DestinationRequest, Topic.RMF_DESTINATION_REQUESTS, self.rfm_destination_request_callback, local_host_only=False) 
        
        # subscrip robot task executing status 
        self.rna_create_subscription(
            String, Topic.MC_ROBOT_EXECUTING_STATUS, self.task_executing_status_callback)
        
        # create scheduler thread
        self.thread_scheduler = Thread(target=self.scheduler_run_pending)
        self.thread_scheduler.daemon = True
        self.thread_scheduler.start()  
       
        self.create_timer(1.0, self.timer_callbak)
        self.charging_status = NOT_CHARGING
        self.need_notification =False        
        self.load_config_data()
        self.is_robot_idle=False
        
        self.guard_condition = self.create_guard_condition(self.navigation_clinet, callback_group=self.callbackgroup)
    def voice_notification(self, tts, volume=100):
        msg = NLPTTS()
        msg.data = tts
        msg.volume = volume
        self.pub_tts.publish(msg)
    
    def get_robot_mode(self, mode=None, task_id=None):
        FleetStatus_dict['path'] = self.location_request            

        if mode == None or task_id == None:
            if not self.rmf_task_wip:
                FleetStatus_dict['mode'] = ExecutorStatus2Mode.get(self.task_exector_status, 0)
                FleetStatus_dict['task_id'] = self.curr_executing_task_id
        else:
            FleetStatus_dict['mode'] = mode
            FleetStatus_dict['task_id'] = task_id

    def task_executing_status_callback(self, msg):
        recv_dict = json.loads(msg.data)
        self.task_exector_status = recv_dict['executing']
        
        if self.task_exector_status == RobotStatus.IDLE_STATE or \
           self.task_exector_status == RobotStatus.GOING_NURSE_STATION or \
           self.task_exector_status == RobotStatus.GOING_HOME_WIP or (not self.rmf_task_wip):
            self.is_robot_idle = True
        else:
            self.is_robot_idle = False
    def load_scheduled_task(self):
        self.scheduled_task = dict()
        if os.path.exists(SCHEDULED_TASK_JSON):
            task = load_task(SCHEDULED_TASK_JSON)
            self.scheduled_task = task.task
            
            for hhmm in self.scheduled_task.keys():
                if self.debug:
                    self.get_logger().info('load task to schedule at :{}'.format(hhmm))
                # scheduling the job
                self.scheduling_job(hhmm)
    def save_scheduled_task(self):
        if self.debug:
            self.get_logger().info("scheduled_task:{}".format(self.scheduled_task))
        if len(self.scheduled_task):
            save_task(SCHEDULED_TASK_JSON, self.scheduled_task)
        else:
            os.remove(SCHEDULED_TASK_JSON)
    def load_config_data(self):
        #todo
        # load self.queue_scheduled_job and settings
        self.load_scheduled_task()
    def is_robot_low_battery(self, task):
        retval = False
        desc = 'dummy'

        if task.name != TaskName.CODE_RED_BLUE \
            and task.name != TaskName.GO_HOME \
            and task.name != TaskName.CANCEL_TASK:
            if self.charging_status == WAITING_FOR_CHARGING:
                retval = True
                desc = 'low battery, waiting for charging'
            elif self.charging_status == CHARGING_ON_GOING:
                retval = True
                desc = 'charging in progress'

        if retval:
            self.get_logger().info(desc)

        return (retval, desc)
    def low_batter_notification(self):
        if self.need_notification:
            if self.is_robot_idle or \
               self.task_exector_status == RobotStatus.EMERGENCY_ESCAPE or \
               self.task_exector_status == RobotStatus.NAVI_WIP:
                self.need_notification =False
                self.voice_notification(TTS_LOW_BATTERY)
    def timer_callbak(self):
        if BatteryStatus_dict['is_charging']:
            self.task_exector_status = RobotStatus.CHARGING_WIP
            self.charging_status = CHARGING_ON_GOING # charging
            if BatteryStatus_dict['percentage'] >= self.power_ready_battery_percentage:
                self.charging_status = BATTERY_CHARGED # charging, do nothing
        else:
            if BatteryStatus_dict['percentage'] > self.low_battery_percentage:
                self.charging_status = NOT_CHARGING
            elif self.charging_status != WAITING_FOR_CHARGING:
                self.need_notification =True
                self.charging_status = WAITING_FOR_CHARGING# low batter

        self.get_robot_mode()
        if self.charging_status == WAITING_FOR_CHARGING:
            self.low_batter_notification()

        if (not self.escape_on): # robot is not in Emergency status
            for index in range(len(self.task_queue)):
                json_task = self.task_queue[index]
                task = MCTask(json_task)
                is_batt_low, desc = self.is_robot_low_battery(task)
                if is_batt_low:
                    break
                else:
                    self.curr_executing_task_id = task.id
                    json_task = self.task_queue.pop(index)
                    task_msg = String()
                    task_msg.data = json_task
                    self.pub_2_task_executor.publish(task_msg)
                    if self.debug:
                        self.get_logger().info("send task to executor:{}".format(json_task))
                    break

    def put_task_into_queue(self, json_task):
        self.task_queue.append(json_task)
        if self.debug:
            self.get_logger().info("queue task:{}".format(json_task))
    def do_job(self, frequency):
        dtime = str(datetime.now())  # eg '2019-04-24 16:18:04.272857'
        dtime = dtime.split(' ')[1] # '16:18:04.272857'
        dtime = dtime.split(':')
        hhmm = dtime[0] + ':' + dtime[1]
        hhmm_key = frequency + '_'+ hhmm

        if self.debug:
            self.get_logger().info('queue_scheduled_job at: dtime:{} {}'.format(dtime, frequency))

        json_task = self.scheduled_task.get(hhmm_key)

        # put it into queue, sort the queue with priority
        if json_task != None:
            self.put_task_into_queue(json_task)
        else:
            self.get_logger().warn("no schedled job found at {}".format(hhmm_key))
    def scheduler_run_pending(self):
        while rclpy.ok():
            self.scheduler.run_pending()
            time.sleep(1.0)   
    def get_task_scheduled_time(self, task):
        scheduled_hhmm = None
        try:
            scheduled_time =  task.scheduled_time         # eg "2018-06-12 09:50:12"
            scheduled_time = scheduled_time.split(' ')[1]  # eg 09:50:12
            scheduled_hhmm = scheduled_time[:-3]           # eg 09:50
            scheduled_hhmm = task.schedule_type + '_' + scheduled_hhmm # append schedule type at prefix
        except:
            self.get_logger().error('Invalid schedule time format:{}'.format(task.scheduled_time))
            pass
        
        return scheduled_hhmm
    def scheduling_job(self, hhmm):
        hhmm_key = hhmm.split('_')[1] # remove the prefix from hhmm
        if SCHEDULE_TYPE_EVERY_HOUR in hhmm:
            mm = ':' + hhmm_key.split(':')[1]
            self.scheduler.every().hour.at(mm).do(self.every_hour_job)
        elif SCHEDULE_TYPE_EVERY_HALF_HOUR in hhmm:
            self.scheduler.every(30).minutes.do(self.every_half_hour_job)
        elif SCHEDULE_TYPE_EVERY_MON in hhmm:
            self.scheduler.every().monday.at(hhmm_key).do(self.every_monday_job)
        elif SCHEDULE_TYPE_EVERY_TUE in hhmm:
            self.scheduler.every().tuesday.at(hhmm_key).do(self.every_tuesday_job)
        elif SCHEDULE_TYPE_EVERY_WED in hhmm:
            self.scheduler.every().wednesday.at(hhmm_key).do(self.every_wednesday_job)
        elif SCHEDULE_TYPE_EVERY_THU in hhmm:
            self.scheduler.every().thursday.at(hhmm_key).do(self.every_thursday_job)
        elif SCHEDULE_TYPE_EVERY_FRI in hhmm:
            self.scheduler.every().friday.at(hhmm_key).do(self.every_friday_job)
        elif SCHEDULE_TYPE_EVERY_SAT in hhmm:
            self.scheduler.every().saturday.at(hhmm_key).do(self.every_saturday_job)
        elif SCHEDULE_TYPE_EVERY_SUN in hhmm:
            self.scheduler.every().sunday.at(hhmm_key).do(self.every_sunday_job)
        elif SCHEDULE_TYPE_EVERY_TWO_DAYS in hhmm:
            self.scheduler.every(2).days.at(hhmm_key).do(self.two_days_job)    
        else: #SCHEDULE_TYPE_DAILY in hhmm:
            self.scheduler.every().day.at(hhmm_key).do(self.daily_job)        
    def on_schedule(self, task):
        status = ASSIGNED_STATUS
        description = 'Task Received'
        if self.debug:
            self.get_logger().info('on_schedule')
        if task.schedule_type == NONE_SCHEDULED_TYPE: # put to executor queue immediately
            self.put_task_into_queue(task.to_json())
        else:
            #hhmm looks like: DAILY_10:30
            hhmm = self.get_task_scheduled_time(task)
            if hhmm not in self.scheduled_task:
                status = SCHEDULED_STATUS
                description = 'Task scheduled'
                if self.debug:
                    self.get_logger().info('schedled time:{}'.format(hhmm))
                # save the job into memeory and disk
                self.scheduled_task[hhmm] = task.to_json()
                self.save_scheduled_task()

                # scheduling the job
                self.scheduling_job(hhmm)
            else:
                description = 'Scheduled time conflict'
                status = SCHEDULE_TIME_CONFLICT_STATUS
                self.get_logger().warn("Another task already scheduled at {} , task:{} rejected to schedule ".format(hhmm, task.task_id))

        return (status, description)
    def every_hour_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_HOUR)
    def every_half_hour_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_HALF_HOUR)
    def every_monday_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_MON)
    def every_tuesday_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_TUE)
    def every_wednesday_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_WED)
    def every_thursday_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_THU)
    def every_friday_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_FRI)
    def every_saturday_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_SAT)
    def every_sunday_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_SUN)
    def two_days_job(self):
        self.do_job(SCHEDULE_TYPE_EVERY_TWO_DAYS)
    def daily_job(self):
        self.do_job(SCHEDULE_TYPE_DAILY)
    def is_valid_task(self, task):
        retval = SUCCESSFUL_STATUS
        description = 'Task received'
        error= task.validate_field_value(self.get_logger().info)
        if error:
            retval = FAILED_STATUS
            description = error
            self.get_logger().error(error)
        else:
            if task.name != TaskName.CANCEL_TASK:
                if task.schedule_type != NONE_SCHEDULED_TYPE:
                    hhmm = self.get_task_scheduled_time(task)
                    if hhmm in self.scheduled_task:
                        retval = SCHEDULE_TIME_CONFLICT_STATUS
                        description = 'schedule time conflict'
                        self.get_logger().info("Another task already scheduled at {} , task:{} rejected to schedule ".format(hhmm, task.task_id)) 
        return (retval, description)
    def cancel_task(self, tobe_cancelled_task):
        cancel_status = REJECTED_STATUS
        desc = ''
        tobe_cancelled_task_id = tobe_cancelled_task.id
        # delete from scheduled_task
        find_task =False
        json_tasks = self.scheduled_task.items()
        for (key, task) in json_tasks:
            if json.loads(task)['id'] == tobe_cancelled_task.id:
                find_task = True
                break
        if find_task: # remove the task from queue
            cancel_status = SUCCESSFUL_STATUS
            desc = 'Task is delected from scheduler '
            self.scheduled_task.pop(key)
            self.save_scheduled_task() # update the saved task
            # todo remove the scheduled job

        # delete from pending execution task
        for index in range(len(self.task_queue)):
            json_task = self.task_queue[index]
            task = MCTask(json_task)
            if task.id == tobe_cancelled_task.id:
                self.task_queue.pop(index)
                cancel_status = SUCCESSFUL_STATUS
                desc += 'Task is cancelled from task queue'
                break
        # delete from executing task
        if self.curr_executing_task_id == tobe_cancelled_task.id:
            if self.task_exector_status == RobotStatus.IDLE_STATE:
                desc = 'Cancel executing task fail, it is completed'            
            else: #if self.task_exector_status == RobotStatus.NAVI_WIP:
                pending_cancel= dict(task_id=tobe_cancelled_task.id, cancel_pending=True, escape_event=False)
                task_msg = String()
                task_msg.data = json.dumps(pending_cancel)
                self.pub_2_task_executor.publish(task_msg)
                desc = 'Executing task is pending to cancel'
                cancel_status = SUCCESSFUL_STATUS

        if len(desc) == 0:
            desc = 'No such task found'

        if self.debug:
            self.get_logger().info("task_id:{} {}".format(tobe_cancelled_task.id, desc))

        return (cancel_status, desc)
    def handle_mode_request(self, mode):
        if mode == RobotMode.MODE_WAITING:
            print('MODE_WAITING')
        elif mode == RobotMode.MODE_PAUSED:
            print('MODE_PAUSED')
            if self.nav_client:
                self.nav_client.cancel_goal()
                self.mode_paused_request = True
            else:
                self.get_logger().info('Robot is not moving')
        elif mode == RobotMode.MODE_MOVING:
            print('MODE_MOVING')
            if self.mode_paused_request: # resume pending navigation location
                self.mode_paused_request = False
                self.guard_condition.trigger()
            else:
                self.get_logger().info('No pending navigation location to resume')                
        elif mode == RobotMode.MODE_CHARGING:
            print('MODE_CHARGING')
        elif mode == RobotMode.MODE_IDLE:
            print('MODE_IDLE')
        elif mode == RobotMode.MODE_EMERGENCY:
            print('MODE_EMERGENCY')            
        elif mode == RobotMode.MODE_GOING_HOME:
            print('MODE_GOING_HOME')
        elif mode == RobotMode.MODE_DOCKING:
            print('MODE_DOCKING')
        elif mode == RobotMode.MODE_ADAPTER_ERROR:
            print('MODE_ADAPTER_ERROR')
        else:
            print('Undefined mode:{}'.format(mode))            
    def rfm_mode_request_callback(self, request):
        if self.debug: # temp for debugging
            print('mode_request', request.fleet_name, request.robot_name, request.task_id, request.mode.mode)
            for para in request.parameters:
                print(para.name, para.value)

        if request.robot_name != ROBOT_UNIQUE_ID:
            self.get_logger().info('Not interested robot_name:{}'.format(request.robot_name))
            return        

        self.curr_executing_task_id = request.task_id # update the task id        
        self.rmf_task_wip = True
        self.get_robot_mode(request.mode.mode, request.task_id)
        self.handle_mode_request(request.mode.mode)
        self.rmf_task_wip = False        
    def rfm_path_request_callback(self, request):
        if self.debug: # temp for debugging
            print('path_request', request.fleet_name, request.robot_name, request.task_id)
            for loc in request.path:
                print(loc.t, loc.x, loc.y, loc.yaw, loc.level_name)

        if request.robot_name != ROBOT_UNIQUE_ID:
            self.get_logger().info('Not interested robot_name:{}'.format(request.robot_name))
            return

        if self.mode_paused_request:
            self.get_logger().info('paused mode, abort the path request')
            return
        
        self.curr_executing_task_id = request.task_id # update the task id
        self.location_request = request.path[-1] # take the last destination for navigation
        self.guard_condition.trigger()
        self.rmf_task_wip = True
        self.get_robot_mode(RobotMode.MODE_MOVING, request.task_id)        
    def rfm_destination_request_callback(self, request):
        if self.debug: # temp for debugging
            print('destination_request', request.fleet_name, request.robot_name, request.task_id)
            loc = request.destination
            print(loc.t, loc.x, loc.y, loc.yaw, loc.level_name)

        if request.robot_name != ROBOT_UNIQUE_ID:
            self.get_logger().info('Not interested robot_name:{}'.format(request.robot_name))
            return

        if self.mode_paused_request:
            self.get_logger().info('paused mode, abort the destination request')
            return
        self.curr_executing_task_id = request.task_id # update the task id
        self.location_request = request.destination
        self.guard_condition.trigger()
        self.rmf_task_wip = True
        self.get_robot_mode(RobotMode.MODE_MOVING, request.task_id)        
    def navigation_clinet(self, timeout=6000):
        if self.debug:
            self.get_logger().info('start navigation_clinet...')

        if not self.location_request:
            self.get_logger().error('destination location is None...')
            return

        x, y, heading = romih_2_robot_coordinate(self.rmf2robot_map_trans, self.location_request)
        self.nav_client = base_client(x, y, heading, action_type='Location', timeout=timeout, cancel_interface=True)
        try:
            self.nav_client.wait_action_complete()
        finally:
            self.nav_client.destroy_action()
        action_result = self.nav_client.action_result
        self.get_logger().info('navigation status:{}: description:{}'.format(action_result.status, action_result.description))
        
        if not self.mode_paused_request:
            self.location_request = None # keep location_request for pause/resume request
            self.rmf_task_wip = False

        self.nav_client = None        
        return action_result
    def rfm_task_receiver_callback(self, task):
        msg = String()
        msg.data = rmf_to_mc_task(task, self.rmf2robot_map_trans)
        self.task_receiver_callback(msg)
    def task_receiver_callback(self, msg):
        try:
            task = MCTask(msg.data)
            if self.debug:
                self.get_logger().info('received task:{}'.format(msg.data))
            if task.name != TaskName.CODE_RED_BLUE: # code red blue does not check robot id
                if task.robot_id != ROBOT_UNIQUE_ID:
                    self.get_logger().info('Not interested robot_id:{}'.format(task.robot_id))
                    return

            (status, description)  = self.is_valid_task(task)
            if status == SUCCESSFUL_STATUS:
                if task.name == TaskName.CANCEL_TASK:
                    if task.escape_event: # and self.escape_on: #cancel code blue event
                        self.escape_on = False # turn off escape_on 
                        pending_cancel= dict(task_id=1, cancel_pending=True, escape_event=True)
                        task_msg = String()
                        task_msg.data = json.dumps(pending_cancel)
                        self.pub_2_task_executor.publish(task_msg)
                        description = 'Code blue event is pending to cancel'
                        status = SUCCESSFUL_STATUS                         
                    else:
                        (status, description) = self.cancel_task(task)
                        if description == 'Executing task is pending to cancel':
                            return # skip update as dashboard implementation limited requested
                elif task.name == TaskName.CODE_RED_BLUE:
                    if self.escape_on and task.escape_on:
                        description = 'Emergency event is already on, task rejected'
                        status = REJECTED_STATUS                        
                    else:
                        self.escape_on = task.escape_on
                        task_msg = String()
                        task_msg.data = msg.data
                        self.pub_2_task_executor.publish(task_msg)                    
                else:
                    if self.escape_on:
                        description = 'Emergency event is on, task rejected'
                        status = REJECTED_STATUS
                        self.voice_notification(TTS_EMERGENCY_ON_TASK_ABORTED)
                    else:
                        if task.schedule_type == NONE_SCHEDULED_TYPE: # low battery checking
                            is_batt_low, desc = self.is_robot_low_battery(task)
                            if is_batt_low:
                                status = REJECTED_STATUS
                                description = desc                                
                                self.voice_notification(TTS_LOW_BATTERY_TASK_ABORTED)
                        
                        if status == SUCCESSFUL_STATUS:
                            (status, description) = self.on_schedule(task)
                        else:
                            self.get_logger().error('task aborted:{}'.format(description))

            task.status = status
            task.status_description = description
            self.publish_task_status(task)
        except:
            self.get_logger().info('task_receiver_callback except:{}'.format(traceback.format_exc()))
            pass
    def publish_task_status(self, task):
        # update to dashboard
        status_msg = String()
        status_msg.data = task.status_json()
        self.pub_task_status.publish(status_msg) 
        
        # update to RMF:
        rmf_status = RnaTaskstatus()
        rmf_status.task_id = task.id
        rmf_status.status = task.status
        rmf_status.started_time = task.started_time
        rmf_status.ended_time = task.ended_time
        rmf_status.description = task.status_description
        self.pub_rmf_task_status.publish(rmf_status)
        
def parse_argv(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--debug', dest='debug', action='store_true', default=False,
        help='enable debug info printing')
    parser.add_argument("-low", "--low_battery_percentage", type=float, help="low battery percentage", default=30.0)
    parser.add_argument("-ready", "--power_ready_battery_percentage", type=float, help="power_ready_battery_percentage", default=50.0)
    parser.add_argument('--log', dest='log', action='store_true', default=False, help='charging,discharging log enable')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    if ROS_DISTRO_USE_ELOQUENT:
        argv = remove_ros_args(args=argv)
    args = parser.parse_args(argv)

    return args

def main(argv=sys.argv[1:]):
    ## disable localhost loopback
    #set_localhost_loopback(False)

    args = parse_argv()
    rclpy.init(args=argv)
    
    logger = RNA_logger('mc_task_mgnt')
    tm_node = task_management(args.low_battery_percentage, args.power_ready_battery_percentage, args.debug, logger)
    rs_node = RNA_robot_status(args.low_battery_percentage, args.power_ready_battery_percentage, args.log, args.debug, logger)
    
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(tm_node)
    executor.add_node(rs_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        tm_node.destroy_node()
        rs_node.destroy_node()
        rclpy.shutdown()         
if __name__ == '__main__':
    main()
