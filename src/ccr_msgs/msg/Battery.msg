uint8 DOC_NOT_CONNECTED=0
uint8 INITIAL_CHECKING_TEMPRATURE=1
uint8 INITIAL_CHECKING_CURRENT=2
uint8 NOW_CHARGING=3
uint8 COMPLETE_CHARGING=4
uint8 WAITING_COLD=5
uint8 CHARGING_ERROR=6

Header header
uint8 status			
float32 voltage			
int32 config_option			
int32 config_current		
int32 config_charging_voltage			
int32 config_charging_current			

