# Load Board Commands

**Commands** (case matters)

*Start*
- ( A ) - automatic testing configuration
- ( M ) - manual testing configuration

*Manual*
- ( m ) - toggle automatic PCC_Relay 
  - *On - provides power to turbine board when load voltage < 3.3V*
  - *Off - user can toggle PCC_Relay manually*
- ( p ) - toggle PCC_Relay
  - *On - provides power to turbine board*
  - *Off - turbine board runs on rectified power*
- ( r___ ) - Sets load value (0 - 255)
- ( t___ ) - sets theta (100 - 3200)
- ( a___ ) - sets alpha (0 - 90)
- ( w___ ) - sets windspeed (0.0 - 16.0)
- ( 1___ ) - sets k1 param
- ( 2___ ) - sets k2 param
- ( 3___ ) - sets k3 param
- ( h___ ) - sets rpm threshold param
- ( s ) - toggle data logging to SD card
  
*Automatic*
- ( Q ) - quit test
- ( P ) - pause test

