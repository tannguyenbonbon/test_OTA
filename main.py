# Copyright (c) Quectel Wireless Solution, Co., Ltd.All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
@file      :tracker_tb.py
@author    :Jack Sun (jack.sun@quectel.com)
@brief     :Tracker by ThingsBoard.
@version   :2.2.0
@date      :2023-04-14 14:30:13
@copyright :Copyright (c) 2022
"""

import sys
import utime
import _thread
import osTimer

from misc       import Power
from queue      import Queue
from machine    import RTC
from machine    import Timer
import ujson    as json

from usr.working_mode    import SettingWorkingMode
from usr.settings        import Settings
from usr.settings_user   import UserConfig
from usr.modules.battery import Battery
from usr.modules.history import History
from usr.modules.logging import getLogger
from usr.sw_watchdog     import WatchDog
from usr.watchdog        import WatchDogTimer
from usr.modules.net_manage   import NetManager
from usr.modules.thingsboard  import TBDeviceMQTTClient
from usr.modules.power_manage import PowerManage, PMLock
from usr.app_fota             import AppFota

from usr.modules.location import (
    GNSS,
    GNSSBase,
    CellLocator,
    WiFiLocator,
    CoordinateSystemConvert,
)

log = getLogger(__name__)

class Tracker:
    def __init__(self):
        self.__server       = None
        self.__server_ota   = None
        self.__battery      = None
        self.__history      = None
        self.__gnss         = None
        self.__cell         = None
        self.__wifi         = None
        self.__csc          = None
        self.__net_manager  = None
        self.__pm           = None
        self.__settings     = None

        # self.__business_lock = PMLock("block")
        self.__business_tid         = None
        self.__business_rtc         = RTC()
        self.__business_queue       = Queue()
        self.__sw_wdg               = WatchDog(30) # 30 * 10 = 300s
        self.__business_timer       = osTimer()
        self.__business_wdg_timer   = osTimer()
        self.__server_reconn_timer  = osTimer()
        self.__gps_checkin_timer    = osTimer()
        self.__server_conn_tag      = 0
        self.__server_disconn_tag   = 0
        self.__server_reconn_count  = 0
        self.__reset_tag            = 0
        self.__running_tag          = 0
        self.__business_tag         = 0
        self.__loc_report_count     = 0

        # self.__server_ota_flag      = 0

        self.__is_parking               = False  # Flag to track if vehicle is parked
        self.__last_reported_parking    = False  # Flag to prevent multiple reports
        self.__parking_speed_threshold  = (5.0)  # Speed threshold (in km/h) to consider as parking

    def __business_start(self):
        if not self.__business_tid or (self.__business_tid and not _thread.threadIsRunning(self.__business_tid)):
            _thread.stack_size(0x2000)
            self.__business_tid = _thread.start_new_thread(self.__business_running, ())

    def __business_stop(self):
        if self.__business_tid and _thread.threadIsRunning(self.__business_tid):
            try:
                _thread.stop_thread(self.__business_tid)
            except Exception as e:
                sys.print_exception(e)
                log.error(str(e))
        self.__business_tid = None

    def __business_running(self):
        log.debug("Thread __business_start created!\n")
        while self.__business_tid is not None or self.__business_queue.size() > 0:
            data = self.__business_queue.get()
            # with self.__business_lock:
            self.__business_tag = 1
            if data[0] == 0:
                if data[1] == "loc_report":
                    self.__loc_report()
                if data[1] == "server_connect":
                    self.__server_connect()
                if data[1] == "power_reset_reason_report":
                    self.__power_reset_reason()
                if data[1] == "batt_report":
                    self.__battery_info()
                if data[1] == "working_mode_report":
                    self.__working_mode_report()

            if data[0] == 1:
                self.__server_option(data[1])
            self.__business_tag = 0

    def __loc_report(self):
        # Report current location.
        # log.debug("__loc_report")
        self.__sw_wdg.feed()
        loc_state, properties = self.__get_loc_data()
        if loc_state == 1:
            # Check if the vehicle is parking (speed is below threshold)
            speed = properties.get("speed", 0)  # Assuming speed is in km/h
            ret = False

            if speed < self.__parking_speed_threshold:
                if not self.__is_parking:
                    # Vehicle just parked, report to server
                    log.debug("Vehicle is parking, sending report to server.")
                    ret = self.__send_telemetry_or_save(properties)
                    self.batt_report()
                    self.__is_parking = True
                    self.__loc_report_count = self.__loc_report_count + 1
                else:
                    if self.__last_reported_parking == False:
                        self.__last_reported_parking = True
                        # log.debug("Vehicle is still parked, no report needed.")
            else:
                if self.__is_parking:
                    # Vehicle started moving again
                    # log.debug("Vehicle is moving again, resetting parking status.")
                    self.__is_parking = False
                    self.__last_reported_parking = False

                self.batt_report()
                ret = self.__send_telemetry_or_save(properties)
                self.__loc_report_count = self.__loc_report_count + 1

            #BUG: Proactive call sleep func will be wakeup after 5mins, need to fix
            # if ret == True and (self.__loc_report_count) > 1:
            #     log.debug("Push GPS Location Success, Going To Sleep | Location Report Count: {}".format(self.__loc_report_count))
            #     self.__into_sleep(sleep_type=2)
            #END BUG

        # Report history location.
        if self.__server.status:
            self.__history_report()

        # Start report again timer.
        # user_cfg = self.__settings.read("user")
        # self.__set_rtc(user_cfg["work_cycle_period"], self.loc_report)

        #NOTE: GPS Will push at least 2 GPS point before timeout and go to sleep
        user_cfg = self.__working_mode.get_mode_config()
        checkin_timer = int((user_cfg["loc_gps_read_timeout"] / 2) - 10)
        self.__gps_checkin_timer.start(checkin_timer * 1000, 0, self.loc_report)

    def __send_telemetry_or_save(self, properties):
        """Send telemetry to the server or save to history if the send fails."""
        try:
            res = False
            if self.__server.status:
                res = self.__server.send_telemetry(properties)

            if not res:
                log.debug("Failed to send telemetry, saving to history.")
                self.__history.write([properties])
        except Exception as e:
            log.error("Error in sending telemetry or saving to history: %s" % str(e))
            self.__history.write([properties])  # Save to history in case of error

        return res

    def __history_report(self):
        failed_datas = []
        his_datas = self.__history.read()
        if his_datas["data"]:
            for item in his_datas["data"]:
                res = self.__server.send_telemetry(item)
                if not res:
                    failed_datas.append(item)
        if failed_datas:
            self.__history.write(failed_datas)

    def __get_loc_data(self):
        loc_state = 0
        loc_data = {
            "longitude": 0.0,
            "latitude": 0.0,
            "altitude": 0.0,
            "speed": 0.0,
            "fix_mode": 0,
            "timestamp": 0,
            "course": 0,
            "date": 0,
        }
        loc_cfg = self.__settings.read("loc")
        user_cfg = self.__settings.read("user")
        if user_cfg["loc_method"] & UserConfig._loc_method.gps:
            res = self.__gnss.read()
            # log.debug("gnss read %s" % str(res))
            if res["state"] == "A":
                if res["fix_mode"] != 1:
                    loc_data["latitude"] = float(res["lat"]) * (
                        1 if res["lat_dir"] == "N" else -1
                    )
                    loc_data["longitude"] = float(res["lng"]) * (
                        1 if res["lng_dir"] == "E" else -1
                    )
                    loc_data["altitude"] = res["altitude"]
                    loc_data["speed"] = res["speed"]
                    loc_data["timestamp"] = res["timestamp"]
                    loc_data["date"] = res["date"]
                    loc_data["fix_mode"] = res["fix_mode"]
                    loc_data["course"] = res["course"]

                    loc_state = 1
        if loc_state == 0 and user_cfg["loc_method"] & UserConfig._loc_method.cell:
            res = self.__cell.read()
            if isinstance(res, tuple):
                loc_data["longitude"] = res[0]
                loc_data["latitude"] = res[1]
                loc_state = 1
        if loc_state == 0 and user_cfg["loc_method"] & UserConfig._loc_method.wifi:
            res = self.__wifi.read()
            if isinstance(res, tuple):
                loc_data["longitude"] = res[0]
                loc_data["latitude"] = res[1]
                loc_state = 1
        if loc_state == 1 and loc_cfg["map_coordinate_system"] == "GCJ02":
            lng, lat = self.__csc.wgs84_to_gcj02(
                loc_data["longitude"], loc_data["latitude"]
            )
            loc_data["longitude"] = lng
            loc_data["latitude"] = lat
        return (loc_state, loc_data)

    def __set_rtc(self, period, callback):
        # Set the RTC to wake up the device
        # Disable any active alarms first
        self.__business_rtc.enable_alarm(0)

        # Register the callback if it is provided and callable
        if callback and callable(callback):
            self.__business_rtc.register_callback(callback)

        # Get the RTC time
        rtc_time = self.__business_rtc.datetime()

        # Print the RTC time
        log.debug("Current RTC time: %s." % (str(rtc_time)))

        # Get current time, add period (in seconds), and compute alarm time
        atime = utime.localtime(utime.mktime(utime.localtime()) + period)

        # Adjust the weekday to match the RTC format
        # utime weekday [0-6] (Mon-Sun) -> RTC weekday [0-6] (Sun-Sat)
        rtc_weekday = (atime[6] + 1) % 7

        # Create alarm time in the required format for the RTC
        alarm_time = (
            atime[0],  # year
            atime[1],  # month
            atime[2],  # day
            rtc_weekday,  # weekday
            atime[3],  # hour
            atime[4],  # minute
            atime[5],  # second
            0,
        )  # RTC alarm is at 0 milliseconds (or a similar value depending on RTC specs)

        # Set the alarm
        _res = self.__business_rtc.set_alarm(alarm_time)

        # Log alarm time and result of setting the alarm
        log.debug("alarm_time: %s, set_alarm res %s." % (str(alarm_time), _res))

        # Enable the alarm if setting it was successful
        return self.__business_rtc.enable_alarm(1) if _res == 0 else -1

    def __power_reset_reason(self):
        power_on_reason = Power.powerOnReason()
        power_dw_reason = Power.powerDownReason()
        power_batt = Power.getVbatt()

        power_cycle_infomation = {
            "power_on_reason": power_on_reason,
            "power_dw_reason": power_dw_reason,
            "power_batt": power_batt,
        }

        if self.__server.status:
            res = self.__server.send_telemetry(power_cycle_infomation)

        log.debug(
            "power_on_reason %d|power_dw_reason %d|power_batt %d"
            % (power_on_reason, power_dw_reason, power_batt)
        )

    def __battery_info(self):
        battery_voltage = self.__battery.voltage
        battery_energy = self.__battery.energy
        battery_charge_status = self.__battery.charge_status

        battery_information = {
            "battery_voltage"       : battery_voltage,
            "battery_energy"        : battery_energy,
            "battery_charge_status" : battery_charge_status,
        }

        if self.__server.status:
            res = self.__server.send_telemetry(battery_information)

        log.debug("battery_voltage %d|battery_energy %d|battery_charge_status %d" %(battery_voltage, battery_energy, battery_charge_status))

    def __server_connect(self):
        if not self.__server.status:
            if self.__net_manager.net_status():
                log.debug("Start server connect\n")
                self.__server.disconnect()
                self.__server.connect()
            if not self.__server.status:
                log.debug("Start __server_reconn_timer")
                self.__server_reconn_timer.stop()
                self.__server_reconn_timer.start(30 * 1000, 0, self.server_connect)
                self.__server_reconn_count += 1
            else:
                self.__server_reconn_count = 0

                # Start __business_timer.
                log.debug("Start __business_timer")
                user_cfg = self.__working_mode.get_mode_config()
                self.__business_timer.stop()
                self.__business_timer.start(user_cfg["loc_gps_read_timeout"] * 1000, 0, self.into_sleep)

                #Request Shared Attribute to update Working mode
                res = self.__server.send_shared_attributes_request(keys = "working_mode_attrb")
                if res == True:
                    log.debug("Send shared attribute request success!")
                else:
                    log.debug("Request Failed!")

                #Public start up report
                self.__public_startup_report()
        else:
            self.__server_reconn_count = 0

        # When server not connect success after 5 miuntes, to reset device.
        if self.__server_reconn_count >= 5:
            _thread.stack_size(0x1000)
            _thread.start_new_thread(self.__power_restart, ())
        self.__server_conn_tag = 0

    def __public_startup_report(self):
        # Report Working Mode
        self.working_mode_report()
        # Report Power reset reason
        self.power_reset_reason_report()
        # Report Battery status
        self.batt_report()

    def __server_disconnect(self):
        if self.__server_disconn_tag == 0:
            self.__server_disconn_tag = 1
            ret_disconn = self.__server.disconnect()
            log.debug("__server_disconnect %d" % (ret_disconn))

    def __server_option(self, args):
        topic, data = args

        response_topic = (topic.decode("utf-8")).replace('request', 'response')
        response_data = {}

        # NOTE: Get Method, Params
        try:
            _msg = json.loads(data)

            method = _msg.get("method", "")
            params = _msg.get("params", "")
            shared_attrb = _msg.get('shared', None)

            # print("Topic: {}".format(topic))
            # print("Data: {}".format(data))
            # print("Method: {}".format(method))
            # print("Params: {}".format(params))

        except Exception as e:
            sys.print_exception(e)
            log.error(str(e))
        
        if(shared_attrb != None):
            # log.debug("Shared Attribute Found : {}".format(shared_attrb))
            self.__server.process_shared_attributes(data)

        if method == "set_working_mode":
            response_data = self.__set_working_mode(params)
        elif method == "get_working_mode":
            # TODO : Process get working mode here
            pass
        elif method == "control":
            self.__process_control_method(params)

        #NOTE: Process RPC Response
        if self.__server.status and response_data:
            res = self.__server.__mqtt.publish(response_topic, json.dumps(response_data))

    def __working_mode_report(self, args=None):
        mode = self.__working_mode.read("Working_Mode")

        cfg = self.__working_mode.get_mode_config()
        gps_timeout_mins  = (cfg["loc_gps_read_timeout"]) / 60
        sw_wdg_timer_mins = (cfg["work_cycle_watchdog"])  / 60
        wakeup_time_mins  = (cfg["work_cycle_period"])    / 60

        # print("mode: {}".format(mode))
        # print("gps_timeout_mins: {}".format(int(gps_timeout_mins)))
        # print("sw_wdg_timer_mins: {}".format(int(sw_wdg_timer_mins)))
        # print("wakeup_time_mins: {}".format(int(wakeup_time_mins)))

        working_mode_params = {
            "working_mode": mode,
            "gps_timeout" : int(gps_timeout_mins),
            "watchdog"    : int(sw_wdg_timer_mins),
            "wakeup_timer": int(wakeup_time_mins),
        }

        if self.__server.status:
            res = self.__server.send_telemetry(working_mode_params)

    def __process_control_method(self, args):
        data = args
        if isinstance(data, dict):
            cmd = data.get("cmd")
        else:
            cmd = None

        if cmd == "reset":
            self.__power_restart_now()


    def __power_restart(self, args=None):
        if self.__reset_tag == 1:
            return
        self.__reset_tag = 1
        count = 0
        while (
            self.__business_queue.size() > 0 or self.__business_tag == 1
        ) and count < 30:
            count += 1
            utime.sleep(1)
        log.debug("__power_restart")
        Power.powerRestart()

    def __power_restart_now(self, arg=None):
        log.debug("__power_restart")
        Power.powerRestart()

    def __reset_timer(self, args=None):
        usr_cfg = self.__working_mode.get_mode_config()

        #Reset Business Timer
        usr_cfg_business_timer = usr_cfg["loc_gps_read_timeout"]
        self.__business_timer.stop()
        self.__business_timer.start(usr_cfg_business_timer * 1000, 0, self.into_sleep)

        #Reset Business WatchDog Timer
        usr_cfg_wdg_timer = usr_cfg["work_cycle_watchdog"]
        self.__business_wdg_timer.stop()
        self.__business_wdg_timer.start(usr_cfg_wdg_timer * 1000, 0, self.__power_restart)

        #Reset Hardware WatchDog Timer
        usr_cfg_hw_wdg_timer = (usr_cfg["work_cycle_period"]) + 60
        hw_watchdog.reset(usr_cfg_hw_wdg_timer)

    def __into_sleep(self, sleep_type=1):
        log.debug("__into_sleep\n")

        user_cfg = self.__working_mode.get_mode_config()
        wakeup_time = user_cfg["work_cycle_period"]
        wakeup_time_mins = wakeup_time / 60

        #Push Sleep Debug Status
        log.debug("Sleep Type : {} | Wakeup Time: {} mins".format(sleep_type, wakeup_time_mins))
        sleep_status = {
            "into_sleep_type"  : sleep_type,
            "wakeup_time_mins" : wakeup_time_mins,
        }

        if self.__server.status:
            res = self.__server.send_telemetry(sleep_status)
        #End Push

        self.__business_timer.stop()
        self.__business_wdg_timer.stop()
        self.__gps_checkin_timer.stop()
        self.__server_reconn_timer.stop()

        self.__business_wdg_timer.delete_timer()
        log.debug("delete_timer __business_wdg_timer")
        self.__business_timer.delete_timer()
        log.debug("delete_timer __business_timer")
        self.__gps_checkin_timer.delete_timer()
        log.debug("delete_timer __gps_checkin_timer")
        self.__server_reconn_timer.delete_timer()
        log.debug("delete_timer __server_reconn_timer")

        # Waiting for business logic done before enter sleep mode, timeout 30s
        count = 0
        while (
            self.__business_queue.size() > 0 or self.__business_tag == 1
        ) and count < 30:
            count += 1
            utime.sleep(1)
        self.__business_stop()

        self.__gnss.stop()
        self.__server_disconnect()

        utime.sleep(3)

        ret = self.__net_manager.net_disconnect(True)  # Explixit diconnect Network by the user

        MAX_RETRIES = 3  # Maximum number of retry attempts
        RETRY_DELAY = 2  # Time to wait (in seconds) between retries

        if not ret:
            retries = 0
            while retries < MAX_RETRIES:
                if not self.net_status():
                    log.error(
                        "Network problem, retrying in %d seconds (Attempt %d of %d)"
                        % (RETRY_DELAY, retries + 1, MAX_RETRIES)
                    )
                    retries += 1
                    utime.sleep(RETRY_DELAY)  # Wait for 5 seconds before retrying
                else:
                    log.info("Network is up.")
                    break  # If the network is up, exit the loop

            if retries == MAX_RETRIES:
                log.error(
                    "Network still down after multiple retries, performing __power_restart"
                )
                self.__power_restart()

        # NOTE : Config Wakeup
        if user_cfg["work_cycle_period"] < user_cfg["work_mode_timeline"]:
            ret = self.__pm.autosleep(1)
            log.debug("Autosleep Status: {}".format(ret))
        else:
            self.__pm.set_psm(mode=1, tau=user_cfg["work_cycle_period"], act=5)

        # # Enable RTC to wake up the device at a specified time
        self.__set_rtc(wakeup_time, self.__power_restart)
        self.__sw_wdg.stop()

    def add_module(self, module):
        if isinstance(module, TBDeviceMQTTClient):
            self.__server = module
        elif isinstance(module, PowerManage):
            self.__pm = module
        elif isinstance(module, Battery):
            self.__battery = module
        elif isinstance(module, History):
            self.__history = module
        elif isinstance(module, GNSSBase):
            self.__gnss = module
        elif isinstance(module, CellLocator):
            self.__cell = module
        elif isinstance(module, WiFiLocator):
            self.__wifi = module
        elif isinstance(module, CoordinateSystemConvert):
            self.__csc = module
        elif isinstance(module, NetManager):
            self.__net_manager = module
        elif isinstance(module, Settings):
            self.__settings = module
        elif isinstance(module, SettingWorkingMode):
            self.__working_mode = module
        else:
            return False
        return True

    def running(self, arg=None):
        if self.__running_tag == 1:
            log.error("running already")
            return
        self.__running_tag = 1
        log.debug("main running\n")

        # Start the sub-thread of listening for business event message queues
        self.__business_start()

        self.server_connect(None)

        # Send location data reporting event (including network connection, device data acquisition and device data reporting)
        self.loc_report(None)

        # log.debug("Start __business_wdg_timer")
        user_cfg = self.__working_mode.get_mode_config()

        self.__business_wdg_timer.stop()
        self.__business_wdg_timer.start(user_cfg["work_cycle_watchdog"] * 1000, 0, self.__power_restart)
        self.__sw_wdg.start()

        self.__running_tag = 0

    def server_callback(self, topic, data):
        self.__business_queue.put((1, (topic, data)))

    def server_error_callback(err):
        log.error("thread err:")
        log.error(err)

    def net_callback(self, args):
        """
        Registers a callback function. When the network status changes, such as when the network is disconnected or the reconnection is successful, this callback function will be triggered to inform the user of the network status.
        args[0]	Integer	PDP context ID, indicating which PDP network state has changed
        args[1]	Integer	Network status. 0 means the network is disconnected and 1 means the network is connected
        """
        log.debug("net_callback args: %s" % str(args))
        if args[1] == 0:
            if self.__server_disconn_tag != 0:
                # is it necessary? due to umqtt have reconnect mechanism already
                # self.__server.disconnect()
                # self.__server_reconn_timer.stop()
                # log.debug("Due to NW unexpected disconnect, we need re-establish connection to the NW? to the server")
                # log.debug("Start __server_reconn_timer")
                # self.__server_reconn_timer.start(30 * 1000, 0, self.server_connect)
                # else:
                self.__server.close()
                # self.__server_disconn_tag = 0
                log.debug("Explicitly disconnect server by the user")
        else:
            self.__server_reconn_timer.stop()
            self.server_connect(None)

    def into_sleep(self, args):
        #Type 1: Timeout (Default)
        #Type 2: Poactively Call 
        self.__into_sleep()

    def loc_report(self, args=None):
        self.__business_queue.put((0, "loc_report"))

    def power_reset_reason_report(self, args=None):
        self.__business_queue.put((0, "power_reset_reason_report"))

    def batt_report(self, args=None):
        self.__business_queue.put((0, "batt_report"))

    def server_connect(self, args):
        if self.__server_conn_tag == 0:
            self.__server_conn_tag = 1
            self.__business_queue.put((0, "server_connect"))
    def working_mode_report(self, args=None):
        self.__business_queue.put((0, "working_mode_report"))

    def __set_working_mode(self, args):
        params = args
        if isinstance(params, dict):
            mode = params.get("Mode")   # mode = 1
            enable = params.get("Enable")  # enable = False
        else:
            mode = None
            enable = None

        key = "Working_Mode"

        response_data = {mode: enable}

        if enable == True:
            pass
        else:
            log.debug("Turn off Working Mode %d" %(mode))
            return response_data

        #Save New Working Mode
        current_mode = self.__working_mode.read(key)
        log.debug("Change Working Mode: %d ⭢ %d" % (current_mode, mode))
        res = self.__working_mode.save({key : mode})
        if res == True:
            log.debug("Updated Successfully! New Working Mode: MODE %d" %(mode))
        else:
            log.debug("Error: Update working mode failed !")

        #Update Working Mode Params
        self.__working_mode.set_mode_config(mode)

        #Reset Timer to update new mode params
        self.__reset_timer()

        #NOTE: Update Working Mode Telemetry
        self.working_mode_report()

        #NOTE: Return RPC Response
        if self.__server.status:
            return response_data


def main_application():
    """
    When running this routine manually, you can remove this delay. If you change the file name of the routine to main.py, you need to add this delay when you want to start the routine automatically. Otherwise, you cannot see the information printed in poweron_print_once() below from the CDC interface.
    """
    utime.sleep(10)
    
    # Init working mode parameters
    working_mode = SettingWorkingMode()

    #Init Hardware WatchDog Timer 
    wdg_cfg = working_mode.get_mode_config()

    #NOTE : Hardware Watchdog Timer should be triggered after 60s if device doesn't wakeup(reset func callback) after sleep time.
    global hw_watchdog 
    hw_watchdog = WatchDogTimer()
    hw_watchdog.start((wdg_cfg["work_mode_timeline"]))

    # Init settings.
    settings = Settings()

    # Init battery.
    battery = Battery()

    # Init history
    history = History()

    # Init power manage and set device low energy.
    power_manage = PowerManage()

    # Init net modules and start net connect.
    user_apn_cfg = settings.read("user")
    net_manager = NetManager(user_apn_cfg["apn"])
    _thread.stack_size(0x1000)
    _thread.start_new_thread(net_manager.net_connect, ())

    # Init GNSS modules and start reading and parsing gnss data.
    loc_cfg = settings.read("loc")
    gnss = GNSS(**loc_cfg["gps_cfg"])
    gnss.set_trans(0)
    gnss.start()

    # Init cell and wifi location modules.
    # cell = CellLocator(**loc_cfg["cell_cfg"])
    # wifi = WiFiLocator(**loc_cfg["wifi_cfg"])
    # Init coordinate system convert modules.
    cyc = CoordinateSystemConvert()

    # Init server modules.
    server_cfg = settings.read("server")
    server = TBDeviceMQTTClient(**server_cfg)

    # Init tracker business modules.
    tracker = Tracker()

    # Register the basic objects to the Tracker class for control
    tracker.add_module(settings)
    tracker.add_module(battery)
    tracker.add_module(history)
    tracker.add_module(net_manager)
    tracker.add_module(server)
    tracker.add_module(power_manage)
    tracker.add_module(gnss)
    tracker.add_module(cyc)
    tracker.add_module(working_mode)
    # tracker.add_module(cell)
    # tracker.add_module(wifi)

    # Set net modules callback.
    net_manager.set_callback(tracker.net_callback)

    # Set server modules callback.
    server.set_callback(tracker.server_callback)
    server.set_error_callback(tracker.server_error_callback)

    # Start tracker business.
    tracker.running()

if __name__ == "__main__":
    main_application()