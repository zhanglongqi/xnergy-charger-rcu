int16 RCU_NOT_CONNECTED = 250

int16 RCU_IDLE = 0

int16 RCU_HANDSHAKE_0 = 1

int16 RCU_HANDSHAKE_1 = 2
int16 RCU_HANDSHAKE_2 = 3
int16 RCU_HANDSHAKE_3 = 4

int16 RCU_CHARGING = 5
int16 RCU_STOP = 6

int16 RCU_SEC_RESTART_DELAY			 = 7
int16 RCU_SEC_WAIT_TIE_TO_BOOTLOADER = 8
int16 RCU_SEC_PRE_READY				 = 9
int16 RCU_SEC_WAIT_RUN_CMD			 = 10
int16 RCU_SEC_RAMP_DOWN				 = 11
int16 RCU_SEC_MANUAL_CHARGE			 = 12

int16 RCU_ERROR = 238

int16 RCU_SEC_DEBUG_READY			= 240
int16 RCU_SEC_DEBUG_UNPAIR			= 241
int16 RCU_SEC_DEBUG_PAIRED			= 242
int16 RCU_SEC_DEBUG_SEC_ENABLE_PRI	= 243
int16 RCU_SEC_DEBUG_ERROR			= 255


Header header
int16 state
float32 runtime_voltage_setting
float32 runtime_current_setting
float32 battery_voltage
float32 output_current
uint32 error_code
uint32 shadow_error_code
string message

string firmware_version
