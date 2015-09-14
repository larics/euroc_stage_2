/*
* Copyright (c) 2015, Ascending Technologies
*/

#ifndef TRINITY_DEFINES_H_
#define TRINITY_DEFINES_H_

/* User ACI defines
* Variables
* Used in old SDK (deprecated)
*=0x01-0x05
* 0x100-0x112
* 0x200-0x208
* 0x300-0x308
* 0x600-0x607
* 0x700-0x703
*
* Commands
* Used in old SDK (deprecated)
* 0x500-0x50E
* 0x600-0x602
* 0x700-0x70x
*
* Parameters
* Used in old SDK (deprecated)
* 0x01-0x05
* 0x400-0x401
*/

const static uint32_t ACI_USER_VAR_IMU_ATTITUDE_QUAT = 0x010;

//VARTYPE_QUAT ; Attitude as Quaternion in radians
const static uint32_t ACI_USER_VAR_IMU_ATTITUDE_EULER = 0x011; //VARTYPE_VECTOR_3F ; Attitude in euler angles in radians
const static uint32_t ACI_USER_VAR_IMU_W = 0x012; //VARTYPE_VECTOR_3F; rotational speeds in radians/s
const static uint32_t ACI_USER_VAR_IMU_ACC = 0x013; //VARTYPE_VECTOR_3F; accelerations im m/s^2 in vehicle frame
const static uint32_t ACI_USER_VAR_IMU_BAROMETRICHEIGHT = 0x014; //VARTYPE_FLOAT; baro height in m
const static uint32_t ACI_USER_VAR_IMU_M = 0x015; //VARTYPE_VECTOR_3F; magnetic field. 1.0=48uT
const static uint32_t ACI_USER_VAR_IMU_W_NAV1 = 0x02C; //VARTYPE_VECTOR_3F; rotational speeds in radians/s
const static uint32_t ACI_USER_VAR_IMU_ACC_NAV1 = 0x02D; //VARTYPE_VECTOR_3F; accelerations im m/s^2 in vehicle frame
const static uint32_t ACI_USER_VAR_IMU_BAROMETRICHEIGHT_NAV1 = 0x02E; //VARTYPE_FLOAT; baro height in m
const static uint32_t ACI_USER_VAR_IMU_M_NAV1 = 0x02F; //VARTYPE_VECTOR_3F; magnetic field. 1.0=48uT
const static uint32_t ACI_USER_VAR_IMU_RAW_GYRO = 0x016; //VARTYPE_VECTOR_3I; raw gyro values
const static uint32_t ACI_USER_VAR_IMU_RAW_ACC = 0x017; //VARTYPE_VECTOR_3I; raw acc values
const static uint32_t ACI_USER_VAR_IMU_RAW_ACC_ALT = 0x018; //VARTYPE_VECTOR_3I; raw alternate acc sensor
const static uint32_t ACI_USER_VAR_IMU_RAW_MAG = 0x019; //VARTYPE_VECTOR_3I; raw mag values
const static uint32_t ACI_USER_VAR_IMU_RAW_PRESSURE = 0x02A; //VARTYPE_INT32; raw pressure value
const static uint32_t ACI_USER_VAR_IMU_RAW_TEMPERATURE = 0x02B; //VARTYPE_SINGLE: raw temperature value

const static uint32_t ACI_USER_VAR_MOTOR0_CMD = 0x210; //VARTYPE_UINT16; current command in rpm
const static uint32_t ACI_USER_VAR_MOTOR1_CMD = 0x211; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR2_CMD = 0x212; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR3_CMD = 0x213; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR4_CMD = 0x214; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR5_CMD = 0x215; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR6_CMD = 0x216; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR7_CMD = 0x217; //VARTYPE_UINT16; in rpm

const static uint32_t ACI_USER_VAR_MOTOR0_RPM = 0x220; //VARTYPE_UINT16; current measured rpm in rpm
const static uint32_t ACI_USER_VAR_MOTOR1_RPM = 0x221; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR2_RPM = 0x222; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR3_RPM = 0x223; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR4_RPM = 0x224; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR5_RPM = 0x225; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR6_RPM = 0x226; //VARTYPE_UINT16; in rpm
const static uint32_t ACI_USER_VAR_MOTOR7_RPM = 0x227; //VARTYPE_UINT16; in rpm

const static uint32_t ACI_USER_VAR_MOTOR0_PWM = 0x230; //VARTYPE_UINT8; current PWM in 0..200
const static uint32_t ACI_USER_VAR_MOTOR1_PWM = 0x231; //VARTYPE_UINT8; 0..200
const static uint32_t ACI_USER_VAR_MOTOR2_PWM = 0x232; //VARTYPE_UINT8; 0..200
const static uint32_t ACI_USER_VAR_MOTOR3_PWM = 0x233; //VARTYPE_UINT8; 0..200
const static uint32_t ACI_USER_VAR_MOTOR4_PWM = 0x234; //VARTYPE_UINT8; 0..200
const static uint32_t ACI_USER_VAR_MOTOR5_PWM = 0x235; //VARTYPE_UINT8; 0..200
const static uint32_t ACI_USER_VAR_MOTOR6_PWM = 0x236; //VARTYPE_UINT8; 0..200
const static uint32_t ACI_USER_VAR_MOTOR7_PWM = 0x237; //VARTYPE_UINT8; 0..200

const static uint32_t ACI_USER_VAR_FLIGHTMODE = 0x310;  //VARTYPE_UINT32;
const static uint32_t ACI_FLIGHTMODE_ACC = 0x01; ///< Absolute angle control active (always active with standard firmware). Actually, the sticks command acceleration.
const static uint32_t ACI_FLIGHTMODE_POS = 0x02; ///< XY-position controller (usually GPS based) active.
const static uint32_t ACI_FLIGHTMODE_FLYING = 0x04; ///< Motors are running.
const static uint32_t ACI_FLIGHTMODE_EMERGENCY = 0x08; ///< RC signal lost.
const static uint32_t ACI_FLIGHTMODE_TRAJECTORY = 0x10; ///< Automatic navigation active (waypoints, trajectories, etc.).
const static uint32_t ACI_FLIGHTMODE_HEIGHT = 0x20; ///< Altitude control active.
const static uint32_t ACI_FLIGHTMODE_MOTOR_CURRENT_CALIB = 0x40; ///< Calibration running.
const static uint32_t ACI_FLIGHTMODE_AUTO_COMPASS_CALIB = 0x80; ///< Calibration running.
const static uint32_t ACI_FLIGHTMODE_HOVER_CALIB = 0x100; ///< Calibration running.

const static uint32_t ACI_USER_VAR_FLIGHTTIME = 0x311; //VARTYPE_UINT16; in 0.1s
const static uint32_t ACI_USER_VAR_USCOUNTER = 0x312; //VARTYPE_UINT64; in us
const static uint32_t ACI_USER_VAR_CMD_STATUS = 0x313; //VARTYPE_UINT8; 0=all external commands deactive.  =0x01=ACI control activated by safety pilot. 0x02=CMD_STICK active. 0x04=CMD_MOTORS active 0x08=CMD_ATTITUDE active 0x10=CMD_EXT_POSITION_ active
const static uint32_t ACI_USER_VAR_VEHICLE_TYPE = 0x710; //VARTYPE_UINT8;
const static uint32_t ACI_VEHICLE_TYPE_FALCON = 0x01;
const static uint32_t ACI_VEHICLE_TYPE_FIREFLY = 0x02;
const static uint32_t ACI_VEHICLE_TYPE_HUMMINGBIRD = 0x03;
const static uint32_t ACI_VEHICLE_TYPE_NEO_6_9 = 0x04;
const static uint32_t ACI_VEHICLE_TYPE_NEO_6_11 = 0x05;
const static uint32_t ACI_USER_VAR_SAFETY_PILOT_STATE = 0x714; // VARTYPE_UINT8;
const static uint32_t ACI_SAFETY_PILOT_ARMED = 0x01; ///< Safety pilot is prepared to take over.
const static uint32_t ACI_SAFETY_PILOT_ACTIVE = 0x02;  ///< Safety pilot is in control.
const static uint32_t ACI_SAFETY_PILOT_STICKS_ACTIVE = 0x04; ///< RC stick inputs are active, such that the MAV can be landed.

const static uint32_t ACI_USER_VAR_VOLTAGE = 0x711; //VARTYPE_UINT16; in mV
const static uint32_t ACI_USER_VAR_CURRENT = 0x712; //VARTYPE_UINT16; in mA
const static uint32_t ACI_USER_VAR_CPU_TIME = 0x713; //VARTYPE_UINT16; in us
const static uint32_t ACI_USER_VAR_TEST = 0x800; //VARTYPE_UINT32; is also a "command", can be used to debug roundtrip times.

const static uint32_t ACI_USER_VAR_RCDATA_LOCK = 0x410; //VARTYPE_UINT8 1=lock
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_PITCH = 0x411; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_ROLL = 0x412; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_YAW = 0x413; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_THRUST = 0x414; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_MODE = 0x415; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_CAM_PITCH = 0x416; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_POWERONOFF = 0x417; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_TRIGGER = 0x418; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_SAFETYPILOTSWITCH = 0x419; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_EXTCMDSWITCH = 0x420; //VARTYPE_INT16; 0..32768
const static uint32_t ACI_USER_VAR_RCDATA_CHANNEL_CAM_YAW = 0x421; //VARTYPE_INT16; 0..32768

const static uint32_t ACI_USER_VAR_GPS_LATITUDE = 0x106; //VARTYPE_INT32; latitude raw from GPS in deg * 10^7
const static uint32_t ACI_USER_VAR_GPS_LONGITUDE = 0x107; //VARTYPE_INT32; latitude raw from GPS in deg * 10^7
const static uint32_t ACI_USER_VAR_GPS_HEIGHT = 0x108; //VARTYPE_INT32; height in mm
const static uint32_t ACI_USER_VAR_GPS_SPEED_X = 0x109; //VARTYPE_UNT32; speed east/west in mm/s
const static uint32_t ACI_USER_VAR_GPS_SPEED_Y = 0x10A; //VARTYPE_UNT32; speed north/south in mm/s
const static uint32_t ACI_USER_VAR_GPS_HEADING = 0x10B; //VARTYPE_INT32; heading in deg*1000
const static uint32_t ACI_USER_VAR_GPS_POS_ACCURACY = 0x10C; //VARTYPE_UNT32; position accuracy in mm
const static uint32_t ACI_USER_VAR_GPS_HEIGHT_ACCURACY = 0x10D; //VARTYPE_UNT32; heigh accuracy in mm
const static uint32_t ACI_USER_VAR_GPS_SPEED_ACCURACY = 0x10E; //VARTYPE_UNT32; speed accuracy in mm/s
const static uint32_t ACI_USER_VAR_GPS_SAT_NUM = 0x10F; //VARTYPE_UNT32; number of sats
const static uint32_t ACI_USER_VAR_GPS_STATUS = 0x110; //VARTYPE_INT32; status
const static uint32_t ACI_USER_VAR_GPS_TIMEOFWEEK = 0x111; //VARTYPE_INT32; time of week in ms. 1 week= 604800s
const static uint32_t ACI_USER_VAR_GPS_WEEK = 0x112; //VARTYPE_INT32; week since 1980
const static uint32_t ACI_USER_VAR_GPS_PDOP = 0x113; //VARTYPE_UINT16; position DOP
const static uint32_t ACI_USER_VAR_GPS_HDOP = 0x114; //VARTYPE_UINT16; horizontal DOP
const static uint32_t ACI_USER_VAR_GPS_VDOP = 0x115; //VARTYPE_UINT16; vertical DOP
const static uint32_t ACI_USER_VAR_GPS_TDOP = 0x116; //VARTYPE_UINT16; timing DOP
const static uint32_t ACI_USER_VAR_GPS_EDOP = 0x117; //VARTYPE_UINT16; easting DOP
const static uint32_t ACI_USER_VAR_GPS_NDOP = 0x118; //VARTYPE_UINT16; northing DOP
const static uint32_t ACI_USER_VAR_GPS_SPEED_Z = 0x119; //VARTYPE_UNT32; speed up/down in mm/s

const static uint32_t ACI_USER_VAR_POSITION_ORIGIN_LAT = 0x510; //VARTYPE_INT32; latitude of origin in deg * 10^7
const static uint32_t ACI_USER_VAR_POSITION_ORIGIN_LON = 0x511; //VARTYPE_INT32; longitude of origin in deg * 10^7
const static uint32_t ACI_USER_VAR_POSITION_ORIGIN_HEIGHT = 0x512; //VARTYPE_FLOAT; height of origin in m
const static uint32_t ACI_USER_VAR_POSITION = 0x513; //VARTYPE_VECTOR_3F; position in m relative to origin in ENU world frame
const static uint32_t ACI_USER_VAR_SPEED = 0x514; //VARTYPE_VECTOR_3F; speeds in m/s relative to origin in ENU world frame
const static uint32_t ACI_USER_VAR_ACCELERATION = 0x515; //VARTYPE_VECTOR_3F; accelerations in m/s^2 relative to origin in ENU world frame

const static uint32_t ACI_USER_VAR_NAV_STATUS_STATUS = 0x610; //VARTYPE_UINT32; Navigation status; see NAV_STATUS_* defines
const static uint32_t ACI_USER_VAR_NAV_STATUS_WAYPTID = 0x611; //VARTYPE_UINT16; Waypoint ID
const static uint32_t ACI_USER_VAR_NAV_STATUS_DISTTOWP = 0x612; //VARTYPE_FLOAT; Distance To Waypoint
const static uint32_t ACI_USER_VAR_NAV_STATUS_DISTTOGOAL = 0x613; //VARTYPE_FLOAT; Distance To Goal
const static uint32_t ACI_USER_VAR_NAV_STATUS_UPDATECNT = 0x614; //VARTYPE_UINT8; is increased by Trinity when VAR_NAV_STATUS_ variables are updated

const static uint32_t ACI_USER_VAR_NAV_LIST_STATUS_UPDATECNT = 0x620; //VARTYPE_UINT8; is increased by Trinity when VAR_NAV_LIST_STATUS_ variables are updated
const static uint32_t ACI_USER_VAR_NAV_LIST_STATUS = 0x621; //VARTYPE_UINT8;
const static uint32_t ACI_USER_VAR_NAV_LIST_PARAM1 = 0x622; //VARTYPE_UINT16;
const static uint32_t ACI_USER_VAR_NAV_LIST_PARAM2 = 0x623; //VARTYPE_UINT16;

//COMMANDS
const static uint32_t ACI_USER_CMD_STICK_UPDATE = 0x010; //VARTYPE_UINT8; increase with every update of CMD_STICK_ commands. Timeout is 0.5s.
const static uint32_t ACI_USER_CMD_STICK_FLAGS = 0x011; //VARTYPE_UINT32; see EXT_CMD_FLAG_
const static uint32_t ACI_USER_CMD_STICK_PITCH = 0x012; //VARTYPE_INT16; 0..4095. Deadzone is 205 around middle 2048 if EXT_CMD_FLAG_DISABLE_DEADZONE is not set
const static uint32_t ACI_USER_CMD_STICK_ROLL = 0x013; //VARTYPE_INT16; 0..4095. Deadzone is 205 around middle 2048 if EXT_CMD_FLAG_DISABLE_DEADZONE is not set
const static uint32_t ACI_USER_CMD_STICK_YAW = 0x014; //VARTYPE_INT16; 0..4095. Deadzone is 205 around middle 2048 if EXT_CMD_FLAG_DISABLE_DEADZONE is not set
const static uint32_t ACI_USER_CMD_STICK_THRUST = 0x015; //VARTYPE_INT16; 0..4095. Deadzone is 205 around middle 2048 if EXT_CMD_FLAG_DISABLE_DEADZONE is not set and height controller is enabled
const static uint32_t ACI_USER_CMD_STICK_MODE = 0x016; //VARTYPE_INT16; 0..4095. Mode: 0=manual 2048=height 4095=height+GPS
const static uint32_t ACI_USER_CMD_STICK_CAM_PITCH = 0x017; //VARTYPE_INT16; -2047..2047=-110..+110
const static uint32_t ACI_USER_CMD_STICK_POWERONOFF = 0x018; //VARTYPE_INT16; 0..4095. Thrust to 0 and poweronoff to 4095 activate/deactivate motors
const static uint32_t ACI_USER_CMD_STICK_CAM_YAW = 0x019; //VARTYPE_INT16; -2047..2047
const static uint32_t ACI_USER_CMD_STICK_TRIGGER = 0x01A; //VARTYPE_INT16; 0..4095. 4095=trigger camera once
const static uint32_t ACI_USER_CMD_STICK_AUX1 = 0x01B; //VARTYPE_INT16; 0..4095.
const static uint32_t ACI_USER_CMD_STICK_AUX2 = 0x01C; //VARTYPE_INT16; 0..4095.

const static uint32_t ACI_USER_CMD_MOTORS_UPDATE = 0x020; //VARTYPE_UINT8; increase with every update of CMD_MOTORS_ commands. After 100ms commmands motor commands are active. Then motor commands are switched back to controller after 200ms timeout.
const static uint32_t ACI_USER_CMD_MOTORS_0 = 0x021; //VARTYPE_UINT16; rpm
const static uint32_t ACI_USER_CMD_MOTORS_1 = 0x022; //VARTYPE_UINT16; rpm
const static uint32_t ACI_USER_CMD_MOTORS_2 = 0x023; //VARTYPE_UINT16; rpm
const static uint32_t ACI_USER_CMD_MOTORS_3 = 0x024; //VARTYPE_UINT16; rpm
const static uint32_t ACI_USER_CMD_MOTORS_4 = 0x025; //VARTYPE_UINT16; rpm
const static uint32_t ACI_USER_CMD_MOTORS_5 = 0x026; //VARTYPE_UINT16; rpm
const static uint32_t ACI_USER_CMD_MOTORS_6 = 0x027; //VARTYPE_UINT16; rpm
const static uint32_t ACI_USER_CMD_MOTORS_7 = 0x028; //VARTYPE_UINT16; rpm

const static uint32_t ACI_USER_CMD_ATTITUDE_UPDATE = 0x030; //VARTYPE_UINT8; increase with every update of CMD_ATTITUDE_ commands. Timeout 500ms
const static uint32_t ACI_USER_CMD_ATTITUDE_FLAGS = 0x031; //VARTYPE_UINT32; flags
const static uint32_t ACI_USER_CMD_ATTITUDE_QUAT = 0x032; //VARTYPE_QUAT: desired quaternion in world coordinates
const static uint32_t ACI_USER_CMD_ATTITUDE_THRUST = 0x033; //VARTYPE_FLOAT: thrust in m/s^2
const static uint32_t ACI_USER_CMD_ATTITUDE_YAW_RATE = 0x034; //VARTYPE_FLOAT: yaw desired speed in radians/s
const static uint32_t EXT_ATTITUDE_CTRL_ACTIVE = 0x01;
const static uint32_t EXT_ATTITUDE_YAW_RATE_ENABLED = 0x02;
const static uint32_t EXT_ATTITUDE_THRUST_ENABLED = 0x04;
const static uint32_t EXT_ATTITUDE_QUATERNION_ENABLED = 0x08;

const static uint32_t ACI_USER_CMD_EXT_POSITION_UPDATE = 0x040; //VARTYPE_UINT8; increase with every update of EXT_POSITION_ commands. Timeout 500ms
const static uint32_t ACI_USER_CMD_EXT_POSITION_FLAGS = 0x041; //VARTYPE_UINT32; flags
const static uint32_t ACI_USER_CMD_EXT_POSITION_INPUT = 0x042; //VARTYPE_VECTOR_3F: external position x/y/z in m
const static uint32_t ACI_USER_CMD_EXT_POSITION_INPUT_QUALITY = 0x043; //VARTYPE_FLOAT: external position quality in x/y/z from 0 (bad) to 1.0(best)
const static uint32_t ACI_USER_CMD_EXT_SPEED_INPUT = 0x044; //VARTYPE_VECTOR_3F: external speeds x/y/z in m/s
const static uint32_t ACI_USER_CMD_EXT_SPEED_INPUT_QUALITY = 0x045; //VARTYPE_FLOAT: external speed quality in x/y/z from 0 (bad) to 1.0(best)
const static uint32_t ACI_USER_CMD_EXT_POSITION_ATTITUDE = 0x046; //VARTYPE_QUAT: external orientation. Currently only yaw is used
const static uint32_t ACI_USER_CMD_EXT_POSITION_DELAY = 0x047; //VARTYPE_UINT16: position delay in ms. Max 400ms
const static uint32_t ACI_USER_CMD_EXT_SPEED_DELAY = 0x048; //VARTYPE_UINT16: speed delay in ms. Max 400ms

//nav commands are not depending on external control switch of the safety pilot. Can also be combined with CMD_STICK and work only in GPS/POSITION+HEIGHT mode.
const static uint32_t ACI_USER_CMD_NAV_LIST_UPDATE = 0x050; //VARTYPE_UINT8; increase with every update of CMD_NAV_LIST_* commands
const static uint32_t ACI_USER_CMD_NAV_LIST_CMD = 0x051; //VARTYPE_UINT8;
const static uint32_t ACI_USER_CMD_NAV_LIST_PARAM1 = 0x052; //VARTYPE_UINT16;
const static uint32_t ACI_USER_CMD_NAV_LIST_PARAM2 = 0x053; //VARTYPE_UINT16;
const static uint32_t ACI_USER_CMD_WP_UPLOAD = 0x054; //VARTYPE_STRUCT; Waypoint struct ATOS_MSG_SINGLE_WAYPOINT



#endif /* TRINITY_DEFINES_H_ */
