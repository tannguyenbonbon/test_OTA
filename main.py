import sys
import utime
import _thread
import osTimer

from misc                       import Power
from queue                      import Queue
from machine                    import RTC
from machine                    import Timer

import ujson                    as json

from usr.working_mode           import SettingWorkingMode
from usr.settings               import Settings
from usr.settings_user          import UserConfig
from usr.modules.battery        import Battery
from usr.modules.history        import History
from usr.modules.logging        import getLogger
from usr.watchdog_sw            import WatchDog
from usr.watchdog_hw               import WatchDogTimer
from usr.modules.net_manage     import NetManager
from usr.modules.net_manage     import ApnConfig
from usr.modules.power_manage   import PowerManage, PMLock
from usr.system_monitor         import SystemMonitor

from usr.app_fota               import AppFOTA
from usr.sys_fota               import SysFOTA

from usr.modules.thingsboard    import THINGSBOARD_SERVER as server


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
        self.__gps_fixed            = 0
        self.__server_conn_tag      = 0
        self.__server_disconn_tag   = 0
        self.__server_reconn_count  = 0
        self.__reset_tag            = 0
        self.__running_tag          = 0
        self.__business_tag         = 0

        self.__old_satellites_data  = {}
        self.__loc_state_failed     = 0
        self.__history_report_failed = 0

        self.__is_parking               = False  # Flag to track if vehicle is parked
        self.__last_reported_parking    = False  # Flag to prevent multiple reports
        self.__parking_speed_threshold  = (5.0)  # Speed threshold (in km/h) to consider as parking

        self.__apn_config           = ApnConfig()

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
                if data[1] == "device_info_report":
                    self.__device_info_report()
                if data[1] == "sim_info_report":
                    self.__sim_info_report()

            if data[0] == 1:
                self.__server_option(data[1])
            self.__business_tag = 0

    def __loc_report(self):
        # Report current location.
        # log.debug("__loc_report")
        loc_state, properties, satellites_data = self.__get_loc_data()

        if loc_state == 1:
            self.__gps_fixed = 1
            # Check if the vehicle is parking (speed is below threshold)
            speed = properties.get("speed", 0)  # Assuming speed is in km/h
            # res = False

            if speed < self.__parking_speed_threshold:
                if not self.__is_parking:
                    # Vehicle just parked, report to server
                    log.debug("Vehicle is parking, sending report to server")  
                    res = self.__send_telemetry_or_save(properties)
                    self.__is_parking = True
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

                res = self.__send_telemetry_or_save(properties)
        else:
            if(self.__old_satellites_data != satellites_data):
                res = self.__send_telemetry_or_save(satellites_data)      
                self.__old_satellites_data = satellites_data

            self.__loc_state_failed +=1

        # Report history location.
        if server.status:
            self.__history_report()
            self.batt_report()
        else:
            self.__history_report_failed += 1
            log.error("Location Report Failed: {}".format(self.__history_report_failed))
        
        #History push failed to server > 5 mins -> reset
        #GPS not fix in 30 mins -> reset
        if(self.__history_report_failed >= 10) or (self.__loc_state_failed >= 60):
                log.error("Location Report Failed Timeout, Device will be reset")
                _thread.stack_size(0x1000)
                _thread.start_new_thread(self.__power_restart, ())
        else:
            if(current_wkm == 7):
                self.__reset_timer()
            else:
                self.__sw_wdg.feed()

        #*Temporarily set checkin timer here, but it should be placed in config 
        checkin_timer = 30 # 30 Seconds
        self.__gps_checkin_timer.start(checkin_timer * 1000, 0, self.loc_report)

    def __send_telemetry_or_save(self, properties):
        """Send telemetry to the server or save to history if the send fails."""
        try:
            res = False
            if server.status:
                res = server.send_telemetry(properties)

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
                res = server.send_telemetry(item)
                if not res:
                    failed_datas.append(item)
        if failed_datas:
            self.__history.write(failed_datas)

    def __get_loc_data(self):
        satellites_data = {}
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
            "satellites": 0,
        }
        loc_cfg = self.__settings.read("loc")
        user_cfg = self.__settings.read("user")
        if user_cfg["loc_method"] & UserConfig._loc_method.gps:
            res = self.__gnss.read()
            # log.debug("gnss read %s" % str(res))
            satellites_data = {
                "satellites": res["satellites"],
                "state": res["state"]
            }
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
                    loc_data["satellites"] = res["satellites"]
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
        return (loc_state, loc_data, satellites_data)

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

        if server.status:
            res = server.send_telemetry(power_cycle_infomation)

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

        if server.status:
            res = server.send_telemetry(battery_information)

        log.info("battery_voltage %d|battery_energy %d|battery_charge_status %d" %(battery_voltage, battery_energy, battery_charge_status))

    def __device_info_report(self):
        user_cfg = self.__settings.read("user")
        device_info = {
            "app_current_version" : user_cfg["ota_status"]["app_current_version"],
            "sys_current_version" : user_cfg["ota_status"]["sys_current_version"],
        }

        if server.status:
            res = server.send_telemetry(device_info)

        log.info("Device Info: %s" % (str(device_info)))

    def __sim_info_report(self):
        sim_info = {
            "modem_imei"        : self.__net_manager.modem_imei,
            "sim_iccid"         : self.__net_manager.sim_iccid,
            "sim_phoneNumber"   : self.__net_manager.sim_phoneNumber,
            "sim_imsi"          : self.__net_manager.sim_imsi,
            "sim_operator"      : self.__net_manager.sim_operator,
            "sim_signal_csq"    : self.__net_manager.sim_signal_csq,
            "sim_status"        : self.__net_manager.sim_status(),
            "current_apn"       : self.__net_manager.current_apn,
        }
        # for key, value in sim_info.items():
        #     log.debug("SIM Info: %s = %s" % (key, value))
        # log.debug("SIM Info: %s" % (str(sim_info)))

        if server.status:
            res = server.send_telemetry(sim_info)
            if not res:
                log.error("Failed to push SIM infomation to server")

    def __server_connect(self):
        if not server.status:
            if self.__net_manager.net_status():
                log.info("Start server connect\n")
                server.disconnect()
                server.connect(clean_session=True)
            if not server.status:
                log.debug("Start __server_reconn_timer")
                self.__server_reconn_timer.stop()
                self.__server_reconn_timer.start(30 * 1000, 0, self.server_connect)
                self.__server_reconn_count += 1
            else:
                self.__server_reconn_count = 0

                # Start __business_timer.
                log.debug("Start __business_timer")

                wkm_cfg = self.__working_mode.get_config()
                self.__business_timer.stop()
                self.__business_timer.start(wkm_cfg["loc_gps_read_timeout"] * 1000, 0, self.into_sleep)

                res = server.send_shared_attributes_request("working_mode_attrb")
                if res != True:
                    log.error("Request working_mode_attrb Failed!")

                res = server.send_shared_attributes_request("targetFwVer","targetFwUrl")
                if res != True:
                    log.error("Request targetFWVer Failed!")

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
        # Report Device Info
        self.device_info_report()
        # Report Working Mode
        self.working_mode_report()
        # Report Power reset reason
        self.power_reset_reason_report()
        # Report Battery status
        self.batt_report()
        # Report SIM Information
        self.sim_info_report()

    def __server_disconnect(self):
        if self.__server_disconn_tag == 0:
            self.__server_disconn_tag = 1
            ret_disconn = server.disconnect()
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

            #* Check response after request
            shared_attrb_rsp = _msg.get('shared', None)

            #* Check attribute key
            ota_key = _msg.get('targetFwVer', None)
            working_mode_key = _msg.get('working_mode_attrb', None)
            apn_key =_msg.get('apn_attrb', None)

            # print("Topic: {}".format(topic))
            # print("Data: {}".format(data))

            #RPC
            # print("Method: {}".format(method))
            # print("Params: {}".format(params))

        except Exception as e:
            sys.print_exception(e)
            log.error(str(e))

        #*=====================Process Attribute=================
        if (shared_attrb_rsp != None):
            server.process_shared_attributes_rsp(data)
        elif (working_mode_key != None):
            self.__working_mode.update_new_working_mode(working_mode_key)
        elif (apn_key != None):
            self.__apn_config.update_new_apn(apn_key)
        elif (ota_key != None):
            targetFWVer = _msg.get("targetFwVer", None)
            targetFWUrl = _msg.get("targetFwUrl", None)
            log.info("Found OTA Firmware Version : {}".format(targetFWVer))
            log.info("Found OTA Firmware URL: {}".format(targetFWUrl))

            #* Check OTA Type (Sys or App)
            if targetFWVer.find("EC800M") != -1:
                ota_type = "sys"
            else:
                ota_type = "app"

            log.info("OTA Type: {}".format(ota_type))

            if(ota_type == "sys"):
                #* Start System OTA
                # server.start_sys_fota(targetFWVer, targetFWUrl)
                pass
            elif(ota_type == "app"):
                #* Start Application OTA
                self.__app_fota.start_app_fota(targetFWVer, targetFWUrl)
            else:
                pass
    
        #*=================Process RPC=================
        if method == "control":
            self.__process_control_method(params)

        # if method == "set_working_mode":
        #     response_data = self.__set_working_mode(params)
        # elif method == "get_working_mode":
        #     # TODO : Process get working mode here
        #     pass

        # #NOTE: Process RPC Response
        # if server.status and response_data:
        #     res = server.__mqtt.publish(response_topic, json.dumps(response_data))

    def __working_mode_report(self, args=None):
        cfg = self.__working_mode.get_config()
        gps_timeout_mins  = (cfg["loc_gps_read_timeout"]) / 60
        sw_wdg_timer_mins = (cfg["work_cycle_watchdog"])  / 60
        wakeup_time_mins  = (cfg["work_cycle_period"])    / 60

        working_mode_params = {
            "working_mode": current_wkm,
            "gps_timeout" : int(gps_timeout_mins),
            "watchdog"    : int(sw_wdg_timer_mins),
            "wakeup_timer": int(wakeup_time_mins),
        }

        if server.status:
            res = server.send_telemetry(working_mode_params)

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
        log.debug("__power_restart_now")
        Power.powerRestart()

    def __reset_timer(self, args=None):
        wkm_cfg = self.__working_mode.get_config()

        #Reset Business Timer
        usr_cfg_business_timer = wkm_cfg["loc_gps_read_timeout"]
        self.__business_timer.stop()
        # self.__business_timer.start(usr_cfg_business_timer * 1000, 0, self.into_sleep)

        #Reset Business WatchDog Timer
        usr_cfg_wdg_timer = wkm_cfg["work_cycle_watchdog"]
        self.__business_wdg_timer.stop()
        self.__business_wdg_timer.start(usr_cfg_wdg_timer * 1000, 0, self.__power_restart)

        #Feed Software WatchDog Timer
        self.__sw_wdg.feed()

        #Reset Hardware WatchDog Timer
        usr_cfg_hw_wdg_timer = (wkm_cfg["work_cycle_period"]) + 60
        hw_watchdog.reset(usr_cfg_hw_wdg_timer)

    def __into_sleep(self):
        log.debug("__into_sleep\n")

        user_cfg = self.__working_mode.get_config()
        wakeup_time = user_cfg["work_cycle_period"]
        wakeup_time_mins = wakeup_time / 60

        #Push Sleep Debug Status
        log.debug("Wakeup Time: {} mins | GPS Fix: {}".format(wakeup_time_mins, self.__gps_fixed))
        sleep_status = {
            "wakeup_time_mins"      : wakeup_time_mins,
            "gps_fix_before_sleep"  : self.__gps_fixed,
        }

        if server.status:
            res = server.send_telemetry(sleep_status)
        #End Push

        self.__business_timer.stop()
        self.__business_wdg_timer.stop()
        self.__gps_checkin_timer.stop()
        self.__server_reconn_timer.stop()
        self.__sw_wdg.stop()

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

    def  add_module(self, module):
        if isinstance(module, PowerManage):
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
        elif isinstance(module, AppFOTA):
            self.__app_fota = module
        elif isinstance(module, SysFOTA):
            self.__sys_fota = module
        else:
            return False
        return True

    def running(self, arg=None):
        if self.__running_tag == 1:
            log.error("running already")
            return
        self.__running_tag = 1

        # Start the sub-thread of listening for business event message queues
        self.__business_start()

        self.server_connect(None)

        # Send location data reporting event (including network connection, device data acquisition and device data reporting)
        self.loc_report(None)

        # log.debug("Start __business_wdg_timer")
        user_cfg = self.__working_mode.get_config()

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
        # log.debug("net_callback args: %s" % str(args))
        if args[1] == 0:
            if self.__server_disconn_tag != 0:
                # is it necessary? due to umqtt have reconnect mechanism already
                # server.disconnect()
                # self.__server_reconn_timer.stop()
                # log.debug("Due to NW unexpected disconnect, we need re-establish connection to the NW? to the server")
                # log.debug("Start __server_reconn_timer")
                # self.__server_reconn_timer.start(30 * 1000, 0, self.server_connect)
                # else:
                server.close()
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

    def device_info_report(self, args=None):
        self.__business_queue.put((0, "device_info_report"))

    def sim_info_report(self, args=None):
        self.__business_queue.put((0, "sim_info_report"))

    def server_connect(self, args):
        if self.__server_conn_tag == 0:
            self.__server_conn_tag = 1
            self.__business_queue.put((0, "server_connect"))
    def working_mode_report(self, args=None):
        self.__business_queue.put((0, "working_mode_report"))

    # def __set_working_mode(self, args):
    #     params = args
    #     if isinstance(params, dict):
    #         mode = params.get("Mode")   # mode = 1
    #         enable = params.get("Enable")  # enable = False
    #     else:
    #         mode = None
    #         enable = None

    #     key = "Working_Mode"

    #     response_data = {mode: enable}

    #     if enable == True:
    #         pass
    #     else:
    #         log.debug("Turn off Working Mode %d" %(mode))
    #         return response_data

    #     #Save New Working Mode
    #     current_mode = self.__working_mode.read(key)
    #     log.debug("Change Working Mode: %d ⭢ %d" % (current_mode, mode))
    #     res = self.__working_mode.save({key : mode})
    #     if res == True:
    #         log.debug("Updated Successfully! New Working Mode: MODE %d" %(mode))
    #     else:
    #         log.debug("Error: Update working mode failed !")

    #     #Update Working Mode Params
    #     self.__working_mode.set_config(mode)

    #     #Reset Timer to update new mode params
    #     self.__reset_timer()

    #     #Update Working Mode Telemetry
    #     self.working_mode_report()

    #     #NOTE: Return RPC Response
    #     if server.status:
    #         return response_data


def main_application():
    """
    When running this routine manually, you can remove this delay. If you change the file name of the routine to main.py, you need to add this delay when you want to start the routine automatically. Otherwise, you cannot see the information printed in poweron_print_once() below from the CDC interface.
    """
    utime.sleep(10)

    #Print Device Infomation
    # sys_inf = SystemMonitor()
    # sys_inf.start_monitor(RAM=False, ROM=True)
    
    # Init working mode parameters
    working_mode = SettingWorkingMode()
    global current_wkm
    current_wkm = working_mode.get_current_mode

    #Init Hardware WatchDog Timer 
    wdg_cfg = working_mode.get_config()

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
    net_manager = NetManager()
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

    # Init Application FOTA
    app_fota = AppFOTA()
    # Init System FOTA
    sys_fota = SysFOTA()

    # Init tracker business modules.
    tracker = Tracker()

    # Register the basic objects to the Tracker class for control
    tracker.add_module(settings)
    tracker.add_module(battery)
    tracker.add_module(history)
    tracker.add_module(net_manager)
    tracker.add_module(power_manage)
    tracker.add_module(gnss)
    tracker.add_module(cyc)
    tracker.add_module(working_mode)
    tracker.add_module(app_fota)
    tracker.add_module(sys_fota)
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
