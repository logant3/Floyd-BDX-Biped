# hold_zero.py — Floyd
# Adapted from Kayden's stand.py
# Enables all 8 motors and holds them at 0.0 rad (zero pose).
# Use this to verify zeroing worked and that Floyd can stand on his own.
# Ctrl+C to stop and disable.

import can
import time
import struct
import numpy as np
import traceback

# --- CONFIGURATION ---
HOST_ID = 0xFD

CAN_CONFIGS = [
    ('can0', [1, 2, 3, 4, 5, 6, 7, 8])
    # 1=right_ankle  2=left_ankle
    # 3=right_knee   4=left_knee
    # 5=right_hip_pitch  6=left_hip_pitch
    # 7=right_hip_roll   8=left_hip_roll
]

# Gains from floyd_deployment_config_template.yaml
# RS02 ankles (1,2): kp=16.581, kd=1.056
# RS03 hips+knees (3-8): kp=78.957, kd=5.027
MOTOR_GAINS = {
    1: {'kp': 16.581, 'kd': 1.056},   # right_ankle  (RS02)
    2: {'kp': 16.581, 'kd': 1.056},   # left_ankle   (RS02)
    3: {'kp': 78.957, 'kd': 5.027},   # right_knee   (RS03)
    4: {'kp': 78.957, 'kd': 5.027},   # left_knee    (RS03)
    5: {'kp': 78.957, 'kd': 5.027},   # right_hip_pitch (RS03)
    6: {'kp': 78.957, 'kd': 5.027},   # left_hip_pitch  (RS03)
    7: {'kp': 78.957, 'kd': 5.027},   # right_hip_roll  (RS03)
    8: {'kp': 78.957, 'kd': 5.027},   # left_hip_roll   (RS03)
}

# --- PROTOCOL CONSTANTS ---
MUX_ENABLE  = 0x03
MUX_CONTROL = 0x01
MUX_DISABLE = 0x04

# --- MOTOR PARAMETERS ---
MOTOR_TYPE_PARAMS = {
    'O2': {
        'P_MIN': -12.57, 'P_MAX': 12.57,
        'V_MIN': -44.0,  'V_MAX': 44.0,
        'T_MIN': -17.0,  'T_MAX': 17.0,
        'KP_MIN': 0.0,   'KP_MAX': 500.0,
        'KD_MIN': 0.0,   'KD_MAX': 5.0,
    },
    'O3': {
        'P_MIN': -12.57, 'P_MAX': 12.57,
        'V_MIN': -20.0,  'V_MAX': 20.0,
        'T_MIN': -60.0,  'T_MAX': 60.0,
        'KP_MIN': 0.0,   'KP_MAX': 5000.0,
        'KD_MIN': 0.0,   'KD_MAX': 100.0,
    },
}

MOTOR_ID_TO_TYPE = {
    1: 'O2',   # right_ankle
    2: 'O2',   # left_ankle
    3: 'O3',   # right_knee
    4: 'O3',   # left_knee
    5: 'O3',   # right_hip_pitch
    6: 'O3',   # left_hip_pitch
    7: 'O3',   # right_hip_roll
    8: 'O3',   # left_hip_roll
}

def scale_to_u16(value, v_min, v_max):
    return int(65535.0 * (np.clip(value, v_min, v_max) - v_min) / (v_max - v_min))

def send_control_command(bus, motor_id, pos, vel, kp, kd, torque, params):
    angle_u16  = scale_to_u16(pos,    params['P_MIN'],  params['P_MAX'])
    vel_u16    = scale_to_u16(vel,    params['V_MIN'],  params['V_MAX'])
    kp_u16     = scale_to_u16(kp,     params['KP_MIN'], params['KP_MAX'])
    kd_u16     = scale_to_u16(kd,     params['KD_MIN'], params['KD_MAX'])
    torque_u16 = scale_to_u16(torque, params['T_MIN'],  params['T_MAX'])

    arb_id = (MUX_CONTROL << 24) | (torque_u16 << 8) | motor_id
    data   = struct.pack('>HHHH', angle_u16, vel_u16, kp_u16, kd_u16)

    try:
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True, dlc=8))
    except can.CanOperationError:
        pass

# --- MAIN ---
if __name__ == "__main__":
    buses = {}

    print("="*50)
    print("Floyd — Hold Zero Pose")
    print("Holds all 8 motors at 0.0 rad until Ctrl+C.")
    print("="*50)

    input("\nEnsure Floyd is powered and on his stand. Press Enter to START...")

    try:
        for interface, motor_ids in CAN_CONFIGS:
            buses[interface] = can.interface.Bus(channel=interface, bustype='socketcan')
            print(f"Connected to {interface}.")

        # Enable all motors
        print("\n[1] Enabling motors...")
        for interface, motor_ids in CAN_CONFIGS:
            bus = buses[interface]
            for mid in motor_ids:
                enable_id = (MUX_ENABLE << 24) | (HOST_ID << 8) | mid
                bus.send(can.Message(arbitration_id=enable_id, is_extended_id=True, dlc=8))

        time.sleep(1.0)

        # Hold at zero
        print("\n[2] Holding at 0.0 rad. Press Ctrl+C to stop.")
        while True:
            for interface, motor_ids in CAN_CONFIGS:
                bus = buses[interface]
                while bus.recv(timeout=0) is not None:
                    pass
                for mid in motor_ids:
                    params = MOTOR_TYPE_PARAMS[MOTOR_ID_TO_TYPE[mid]]
                    gains  = MOTOR_GAINS[mid]
                    send_control_command(bus, mid, 0.0, 0.0, gains['kp'], gains['kd'], 0.0, params)
            time.sleep(0.0025)  # 400 Hz

    except KeyboardInterrupt:
        print("\nCtrl+C — stopping.")
    except Exception:
        traceback.print_exc()
    finally:
        print("\n[Final] Disabling all motors...")
        for interface, motor_ids in CAN_CONFIGS:
            if interface in buses:
                bus = buses[interface]
                for mid in motor_ids:
                    disable_id = (MUX_DISABLE << 24) | (HOST_ID << 8) | mid
                    try:
                        bus.send(can.Message(arbitration_id=disable_id, is_extended_id=True, dlc=8))
                    except Exception:
                        pass
                bus.shutdown()
        print("Done.")
