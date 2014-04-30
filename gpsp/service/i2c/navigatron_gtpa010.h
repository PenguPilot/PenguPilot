
#ifndef NAVIGATRON_GTPA010_H
#define NAVIGATRON_GTPA010_H 




#define I2C_GPS_ADDRESS                      0x20       
/*************** I2C GSP register definitions *********************************/

#define I2C_GPS_STATUS                            0x00 //(Read only)
        #define I2C_GPS_STATUS_NEW_DATA       0x01
        #define I2C_GPS_STATUS_2DFIX          0x02
        #define I2C_GPS_STATUS_3DFIX          0x04
        #define I2C_GPS_STATUS_WP_REACHED     0x08      //Active waypoint has been reached (not cleared until new waypoint is set)
        #define I2C_GPS_STATUS_NUMSATS        0xF0

#define I2C_GPS_COMMAND                           0x01 //(write only)
        #define I2C_GPS_COMMAND_POSHOLD       0x01      //copy current position to internal target location register
        #define I2C_GPS_COMMAND_RESUME        0x02      //copy last active WP to internal target location register
        #define I2C_GPS_COMMAND_SET_WP        0x04      //copy current position to given WP
        #define I2C_GPS_COMMAND_ACTIVATE_WP   0x08      //copy given WP position to internal target location register
        #define I2C_GPS_COMMAND_WP            0xF0      //Waypoint number

#define I2C_GPS_WP_REG                            0x06   //Waypoint register (Read only)
        #define I2C_GPS_WP_REG_ACTIVE             0x0F      //Active Waypoint
        #define I2C_GPS_WP_REG_PERVIOUS        	  0xF0      //pervious Waypoint
        
#define I2C_GPS_GROUND_SPEED                      0x1f   //GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE                          0x21   //GPS altitude in meters (uint16_t)           (Read Only)
//vermutlich falscher Wert
#define I2C_GPS_TIME                              0x27   //UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_DISTANCE                          0x0f   //Distance between current pos and internal target location register in meters (uint16_t) (Read Only)
#define I2C_GPS_DIRECTION                         0x11   //direction towards interal target location reg from current position (+/- 180 degree)    (read Only)
//vermutlich falscher Wert --> 0x7 liefert Daten
#define I2C_GPS_LOCATION                          0x07   //current position (8 bytes, lat and lon, 1 degree = 10 000 000                           (read only)
#define I2C_GPS_WP0                               0x1B   //Waypoint 0 used for RTH location      (R/W)
#define I2C_GPS_WP1                               0x23
#define I2C_GPS_WP2                               0x2B
#define I2C_GPS_WP3                               0x33
#define I2C_GPS_WP4                               0x3B
#define I2C_GPS_WP5                               0x43
#define I2C_GPS_WP6                               0x4B
#define I2C_GPS_WP7                               0x53
#define I2C_GPS_WP8                               0x5B
#define I2C_GPS_WP9                               0x63
#define I2C_GPS_WP10                              0x6B
#define I2C_GPS_WP11                              0x73
#define I2C_GPS_WP12                              0x7B
#define I2C_GPS_WP13                              0x83
#define I2C_GPS_WP14                              0x8B
#define I2C_GPS_WP15                              0x93
#define I2C_GPS_WP_NAV_PAR1			  0x9B	//Waypoint navigation parameter 1
#define I2C_GPS_WP_NAV_PAR1_REACH_LIMIT		  0x0F    //lover 4 bit, waypoint reached distance
#define I2C_GPS_GROUND_COURSE			  0x9C  //GPS ground course (uint16_t)






#endif /* NAVIGATRON_GTPA010_H */
