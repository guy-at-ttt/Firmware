import pyulog
import csv
import math

dir_path = "C:/Users/VivekReddyAlla/Documents/PX4 logs/"

indoor_flight = dir_path + '2019-02-28/log_66_2019-2-27-15-19-00'
crash = dir_path + '2019-03-01/crash/log_72_2019-3-1-15-23-06'
armed_NO_thrust = dir_path + '2019-03-01/armed_NO_thrust/log_73_2019-3-1-15-57-22'
armed_spinning_props = dir_path + '2019-03-01/armed_spinning_props/log_74_2019-3-1-16-00-06'

log = {
    'ac'    : '_actuator_controls_0_0.csv',
    'ao0'   : '_actuator_outputs_0.csv',
    'ao1'   : '_actuator_outputs_1.csv',
    'bs'    : '_battery_status_0.csv',
    'c'     : '_cpuload_0.csv',
    'ei'    : '_ekf2_innovations_0.csv',
    'et'    : '_ekf2_timestamps_0.csv',
    'es'    : '_estimatior_status_0.csv',
    'ir'    : '_input_rc_0.csv',
    'mcs'   : '_manual_control_setpoint_0.csv',
    'rcs'   : '_rate_ctrl_status_0.csv',
    'sc'    : '_sensor_combined_0.csv',
    'spf'   : '_sensor_preflight_0.csv',
    'ss'    : '_sensor_selection_0.csv',
    'sp'    : '_system_power_0.csv',
    'ts0'   : '_telemetry_status_0.csv',
    'ts1'   : '_telemetry_status_1.csv',
    'ts2'   : '_telemetry_status_2.csv',
    'vad'   : '_vehicle_air_data_0.csv',
    'va'    : '_vehicle_attitude_0.csv',
    'vld'   : '_vehicle_land_detected_0.csv',
    'vlp'   : '_vehicle_local_position_0.csv',
    'vm'    : '_vehicle_magnetometer_0.csv',
    'vrs'   : '_vehicle_rates_setpoint_0.csv',
    'vs'    : '_vehicle_status_0.csv',
    'vsf'   : '_vehicle_status_flags_0.csv'
}


# Quaternions are in (... + log['sc'])
def quat_to_euler(q0, q1, q2, q3):
    xroll = 1.0 - (2.0 * (math.pow(q1, 2) + math.pow(q2, 2)))
    yroll = 2.0 * ((q0 * q1) + (q2 * q3))
    roll = math.atan2(yroll, xroll)

    a = 2.0 * ((q0 * q2) - (q3 * q1))
    if abs(a) >= 1:
        pitch = math.copysign(math.pi / 2, a)
    else:
        pitch = math.asin(a)

    xyaw = 1.0 - (2.0 * (math.pow(q2, 2) + math.pow(q3, 2)))
    yyaw = 2.0 * ((q0 * q3) + (q1 * q2))
    yaw = math.atan2(yyaw, xyaw)

    return roll, pitch, yaw


ts = []
roll = []
pitch = []
yaw = []
with open(crash + log['va'], newline='') as csvfile:
    data = csv.reader(csvfile, delimiter=',',)
    count = 0
    for idx, row in enumerate(data):
        if idx is 0:
            continue
        ts.append(int(row[0]))
        r, p, y = quat_to_euler(float(row[4]), float(row[5]), float(row[6]), float(row[7]))
        roll.append(r)
        pitch.append(p)
        yaw.append(y)
        count += 1

