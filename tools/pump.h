/*  Here are the masks used to identify the key */

// This is for Byte1 of the player short
#define BTN_COIN      0x04
#define BTN_TEST      0x02
#define BTN_SERVICE   0x40
#define BTN_CLEAR     0x80

// This is for Byte0 of the player short
#define SENSOR_LU     0x01
#define SENSOR_RU     0x02
#define SENSOR_CN     0x04
#define SENSOR_LD     0x08
#define SENSOR_RD     0x10

//  This is for Byte1 of player output short
#define LIGHT_HALO_L1      	0x04
#define LIGHT_HALO_L2      	0x02
#define LIGHT_HALO_R1      	0x01

//  This is for Byte0 of player output short
#define LIGHT_HALO_R2      	0x0080
#define LIGHT_CCFL_LU		0x04
#define LIGHT_CCFL_RU		0x08
#define LIGHT_CCFL_CN		0x10
#define LIGHT_CCFL_LD		0x20
#define LIGHT_CCFL_RD		0x40

/*
          COIN                  COIN
 _____________________  _____________________
|                     ||                     |
|  ####        ####   ||  ####        ####   |
| #    #      #    #  || #    #      #    #  |
| #    #      #    #  || #    #      #    #  |
| #    #      #    #  || #    #      #    #  |
|  ####  ####  ####   ||  ####  ####  ####   |
|       #    #        ||       #    #        |
|       #    #        ||       #    #        |
|       #    #        ||       #    #        |
|  ####  ####  ####   ||  ####  ####  ####   |
| #    #      #    #  || #    #      #    #  |
| #    #      #    #  || #    #      #    #  |
| #    #      #    #  || #    #      #    #  |
|  ####        ####   ||  ####        ####   |
|_____________________||_____________________|
*/
