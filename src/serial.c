#include "board.h"
#include "mw.h"

#include "cli.h"
#include "telemetry_common.h"

// Multiwii Serial Protocol 0
#define MSP_VERSION              0
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FLAPS                   ((uint32_t)1 << 3)

#define MSP_IDENT                100    //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan and more
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         altitude, variometer
#define MSP_ANALOG               110    //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113    //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114    //out message         powermeter trig
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119    //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120    //out message         Servo settings
#define MSP_NAV_STATUS           121    //out message         Returns navigation status
#define MSP_NAV_CONFIG           122    //out message         Returns navigation parameters

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203    //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_SET_WP               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210    //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211    //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212    //in message          Servo settings
#define MSP_SET_MOTOR            214    //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215    //in message          Sets nav config parameters - write to the eeprom

// #define MSP_BIND                 240    //in message          no param

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUGMSG             253    //out message         debug string buffer
#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

// Additional commands that are not compatible with MultiWii
#define MSP_UID                  160    //out message         Unique device ID
#define MSP_ACC_TRIM             240    //out message         get acc angle trim values
#define MSP_SET_ACC_TRIM         239    //in message          set acc angle trim values
#define MSP_GPSSVINFO            164    //out message         get Signal Strength (only U-Blox)

#define INBUF_SIZE 64

struct box_t {
    const uint8_t boxIndex;         // this is from boxnames enum
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      //
} boxes[] = {
    { BOXARM, "ARM;", 0 },
    { BOXANGLE, "ANGLE;", 1 },
    { BOXHORIZON, "HORIZON;", 2 },
    { BOXBARO, "BARO;", 3 },
    { BOXVARIO, "VARIO;", 4 },
    { BOXMAG, "MAG;", 5 },
    { BOXHEADFREE, "HEADFREE;", 6 },
    { BOXHEADADJ, "HEADADJ;", 7 },
    { BOXCAMSTAB, "CAMSTAB;", 8 },
    { BOXCAMTRIG, "CAMTRIG;", 9 },
    { BOXGPSHOME, "GPS HOME;", 10 },
    { BOXGPSHOLD, "GPS HOLD;", 11 },
    { BOXPASSTHRU, "PASSTHRU;", 12 },
    { BOXBEEPERON, "BEEPER;", 13 },
    { BOXLEDMAX, "LEDMAX;", 14 },
    { BOXLEDLOW, "LEDLOW;", 15 },
    { BOXLLIGHTS, "LLIGHTS;", 16 },
    { BOXCALIB, "CALIB;", 17 },
    { BOXGOV, "GOVERNOR;", 18 },
    { BOXOSD, "OSD SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { CHECKBOXITEMS, NULL, 0xFF }
};

// this is calculated at startup based on enabled features.
static uint8_t availableBoxes[CHECKBOXITEMS];
// this is the number of filled indexes in above array
static uint8_t numberBoxItems = 0;
// from mixer.c
extern int16_t motor_disarmed[MAX_MOTORS];

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef struct mwproto {
	serialPort_t *port;
	uint8_t checksum, indRX, inBuf[INBUF_SIZE];
	uint8_t cmdMSP;
	uint8_t offset;
	uint8_t dataSize;
	enum _serial_state {
		IDLE,
		HEADER_START,
		HEADER_M,
		HEADER_ARROW,
		HEADER_SIZE,
		HEADER_CMD,
	} c_state;
} mwproto_t;

//3 serial instances, first is main port, second and third possible softserial.
#define NUMSERIALINSTANCES	3
mwproto_t serialinstances[NUMSERIALINSTANCES];

void serialize32(mwproto_t *instance, uint32_t a)
{
    static uint8_t t;
    t = a;
    serialWrite(instance->port, t);
    instance->checksum ^= t;
    t = a >> 8;
    serialWrite(instance->port, t);
    instance->checksum ^= t;
    t = a >> 16;
    serialWrite(instance->port, t);
    instance->checksum ^= t;
    t = a >> 24;
    serialWrite(instance->port, t);
    instance->checksum ^= t;
}

void serialize16(mwproto_t *instance, int16_t a)
{
    static uint8_t t;
    t = a;
    serialWrite(instance->port, t);
    instance->checksum ^= t;
    t = a >> 8 & 0xff;
    serialWrite(instance->port, t);
    instance->checksum ^= t;
}

void serialize8(mwproto_t *instance, uint8_t a)
{
    serialWrite(instance->port, a);
    instance->checksum ^= a;
}

uint8_t read8(mwproto_t *instance)
{
    return instance->inBuf[instance->indRX++] & 0xff;
}

uint16_t read16(mwproto_t *instance)
{
    uint16_t t = read8(instance);
    t += (uint16_t) read8(instance) << 8;
    return t;
}

uint32_t read32(mwproto_t *instance)
{
    uint32_t t = read16(instance);
    t += (uint32_t) read16(instance) << 16;
    return t;
}

void headSerialResponse(mwproto_t *instance, uint8_t err, uint8_t s)
{
    serialize8(instance, '$');
    serialize8(instance, 'M');
    serialize8(instance, err ? '!' : '>');
    instance->checksum = 0;               // start calculating a new checksum
    serialize8(instance, s);
    serialize8(instance, instance->cmdMSP);
}

void headSerialReply(mwproto_t *instance, uint8_t s)
{
    headSerialResponse(instance, 0, s);
}

void headSerialError(mwproto_t *instance, uint8_t s)
{
    headSerialResponse(instance, 1, s);
}

void tailSerialReply(mwproto_t *instance)
{
    serialize8(instance, instance->checksum);
}

void s_struct(mwproto_t *instance, uint8_t *cb, uint8_t siz)
{
    headSerialReply(instance, siz);
    while (siz--)
        serialize8(instance, *cb++);
}

void serializeNames(mwproto_t *instance, const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(instance, *c);
}

void serializeBoxNamesReply(mwproto_t *instance)
{
    int i, idx, j, flag = 1, count = 0, len;

reset:
    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (i = 0; i < numberBoxItems; i++) {
        idx = availableBoxes[i];
        len = strlen(boxes[idx].boxName);
        if (flag) {
            count += len;
        } else {
            for (j = 0; j < len; j++)
                serialize8(instance, boxes[idx].boxName[j]);
        }
    }

    if (flag) {
        headSerialReply(instance, count);
        flag = 0;
        goto reset;
    }
}

void serialInit(uint32_t baudrate)
{
    int idx;

    for (idx = 0; idx < NUMSERIALINSTANCES; idx++) {
    	serialinstances[idx].c_state = IDLE;
    	serialinstances[idx].port = 0;
    	serialinstances[idx].offset = 0;
    	serialinstances[idx].dataSize = 0;
    }
    serialinstances[0].port = core.mainport = uartOpen(USART1, NULL, baudrate, MODE_RXTX);
    // Only open the softserial ports if loopback is not defined, and FEATURE_SOFTSERIAL is enabled
#ifndef SOFTSERIAL_LOOPBACK
    if (feature(FEATURE_SOFTSERIAL)) {
    	// Don't bind to a softserial port that is being used by telemetry.
    	if (mcfg.telemetry_port != TELEMETRY_PORT_SOFTSERIAL_1)
    		serialinstances[1].port = (serialPort_t*)&(softSerialPorts[0]);
    	if (mcfg.telemetry_port != TELEMETRY_PORT_SOFTSERIAL_2)
    		serialinstances[2].port = (serialPort_t*)&(softSerialPorts[1]);
    }
#endif

    // calculate used boxes based on features and fill availableBoxes[] array
    memset(availableBoxes, 0xFF, sizeof(availableBoxes));

    idx = 0;
    availableBoxes[idx++] = BOXARM;
    if (sensors(SENSOR_ACC)) {
        availableBoxes[idx++] = BOXANGLE;
        availableBoxes[idx++] = BOXHORIZON;
    }
    if (sensors(SENSOR_BARO)) {
        availableBoxes[idx++] = BOXBARO;
        if (feature(FEATURE_VARIO))
            availableBoxes[idx++] = BOXVARIO;
    }
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        availableBoxes[idx++] = BOXMAG;
        availableBoxes[idx++] = BOXHEADFREE;
        availableBoxes[idx++] = BOXHEADADJ;
    }
    if (feature(FEATURE_SERVO_TILT))
        availableBoxes[idx++] = BOXCAMSTAB;
    if (feature(FEATURE_GPS)) {
        availableBoxes[idx++] = BOXGPSHOME;
        availableBoxes[idx++] = BOXGPSHOLD;
    }
    if (mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_AIRPLANE)
        availableBoxes[idx++] = BOXPASSTHRU;
    availableBoxes[idx++] = BOXBEEPERON;
    if (feature(FEATURE_INFLIGHT_ACC_CAL))
        availableBoxes[idx++] = BOXCALIB;
    availableBoxes[idx++] = BOXOSD;
    if (feature(FEATURE_TELEMETRY && mcfg.telemetry_switch))
        availableBoxes[idx++] = BOXTELEMETRY;
    numberBoxItems = idx;
}

static void evaluateCommand(mwproto_t *instance)
{
    uint32_t i, tmp, junk;
    uint8_t wp_no;
    int32_t lat = 0, lon = 0, alt = 0;

    switch (instance->cmdMSP) {
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++)
            rcData[i] = read16(instance);
        headSerialReply(instance, 0);
        break;
    case MSP_SET_ACC_TRIM:
        cfg.angleTrim[PITCH] = read16(instance);
        cfg.angleTrim[ROLL]  = read16(instance);
        headSerialReply(instance, 0);
        break;
    case MSP_SET_RAW_GPS:
        f.GPS_FIX = read8(instance);
        GPS_numSat = read8(instance);
        GPS_coord[LAT] = read32(instance);
        GPS_coord[LON] = read32(instance);
        GPS_altitude = read16(instance);
        GPS_speed = read16(instance);
        GPS_update |= 2;        // New data signalisation to GPS functions
        headSerialReply(instance, 0);
        break;
    case MSP_SET_PID:
        for (i = 0; i < PIDITEMS; i++) {
            cfg.P8[i] = read8(instance);
            cfg.I8[i] = read8(instance);
            cfg.D8[i] = read8(instance);
        }
        headSerialReply(instance, 0);
        break;
    case MSP_SET_BOX:
        for (i = 0; i < numberBoxItems; i++)
            cfg.activate[availableBoxes[i]] = read16(instance);
        headSerialReply(instance, 0);
        break;
    case MSP_SET_RC_TUNING:
        cfg.rcRate8 = read8(instance);
        cfg.rcExpo8 = read8(instance);
        cfg.rollPitchRate = read8(instance);
        cfg.yawRate = read8(instance);
        cfg.dynThrPID = read8(instance);
        cfg.thrMid8 = read8(instance);
        cfg.thrExpo8 = read8(instance);
        headSerialReply(instance, 0);
        break;
    case MSP_SET_MISC:
        read16(instance); // powerfailmeter
        mcfg.minthrottle = read16(instance);
        mcfg.maxthrottle = read16(instance);
        mcfg.mincommand = read16(instance);
        cfg.failsafe_throttle = read16(instance);
        read16(instance);
        read32(instance);
        cfg.mag_declination = read16(instance) * 10;
        mcfg.vbatscale = read8(instance);           // actual vbatscale as intended
        mcfg.vbatmincellvoltage = read8(instance);  // vbatlevel_warn1 in MWC2.3 GUI
        mcfg.vbatmaxcellvoltage = read8(instance);  // vbatlevel_warn2 in MWC2.3 GUI
        read8(instance);                            // vbatlevel_crit (unused)
        headSerialReply(instance, 0);
        break;
    case MSP_SET_MOTOR:
        for (i = 0; i < 8; i++)
            motor_disarmed[i] = read16(instance);
        headSerialReply(instance, 0);
        break;
    case MSP_SELECT_SETTING:
        if (!f.ARMED) {
            mcfg.current_profile = read8(instance);
            if (mcfg.current_profile > 2)
                mcfg.current_profile = 0;
            // this writes new profile index and re-reads it
            writeEEPROM(0, false);
        }
        headSerialReply(instance, 0);
        break;
    case MSP_SET_HEAD:
        magHold = read16(instance);
        headSerialReply(instance, 0);
        break;
    case MSP_IDENT:
        headSerialReply(instance, 7);
        serialize8(instance, VERSION);                // multiwii version
        serialize8(instance, mcfg.mixerConfiguration); // type of multicopter
        serialize8(instance, MSP_VERSION);            // MultiWii Serial Protocol Version
        serialize32(instance, CAP_PLATFORM_32BIT | CAP_DYNBALANCE | (mcfg.flaps_speed ? CAP_FLAPS : 0));        // "capability"
        break;
    case MSP_STATUS:
        headSerialReply(instance, 11);
        serialize16(instance, cycleTime);
        serialize16(instance, i2cGetErrorCounter());
        serialize16(instance, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        // OK, so you waste all the fucking time to have BOXNAMES and BOXINDEXES etc, and then you go ahead and serialize enabled shit simply by stuffing all
        // the bits in order, instead of setting the enabled bits based on BOXINDEX. WHERE IS THE FUCKING LOGIC IN THIS, FUCKWADS.
        // Serialize the boxes in the order we delivered them, until multiwii retards fix their shit
        junk = 0;
        tmp = f.ANGLE_MODE << BOXANGLE | f.HORIZON_MODE << BOXHORIZON |
                    f.BARO_MODE << BOXBARO | f.MAG_MODE << BOXMAG | f.HEADFREE_MODE << BOXHEADFREE | rcOptions[BOXHEADADJ] << BOXHEADADJ |
                    rcOptions[BOXCAMSTAB] << BOXCAMSTAB | rcOptions[BOXCAMTRIG] << BOXCAMTRIG |
                    f.GPS_HOME_MODE << BOXGPSHOME | f.GPS_HOLD_MODE << BOXGPSHOLD |
                    f.PASSTHRU_MODE << BOXPASSTHRU |
                    rcOptions[BOXBEEPERON] << BOXBEEPERON |
                    rcOptions[BOXLEDMAX] << BOXLEDMAX |
                    rcOptions[BOXLLIGHTS] << BOXLLIGHTS |
                    rcOptions[BOXVARIO] << BOXVARIO |
                    rcOptions[BOXCALIB] << BOXCALIB |
                    rcOptions[BOXGOV] << BOXGOV |
                    rcOptions[BOXOSD] << BOXOSD |
                    rcOptions[BOXTELEMETRY] << BOXTELEMETRY |
                    f.ARMED << BOXARM;
        for (i = 0; i < numberBoxItems; i++) {
            int flag = (tmp & (1 << availableBoxes[i]));
            if (flag)
                junk |= 1 << i;
        }
        serialize32(instance, junk);
        serialize8(instance, mcfg.current_profile);
        break;
    case MSP_RAW_IMU:
        headSerialReply(instance, 18);
        // Retarded hack until multiwiidorks start using real units for sensor data
        if (acc_1G > 1024) {
            for (i = 0; i < 3; i++)
                serialize16(instance, accSmooth[i] / 8);
        } else {
            for (i = 0; i < 3; i++)
                serialize16(instance, accSmooth[i]);
        }
        for (i = 0; i < 3; i++)
            serialize16(instance, gyroData[i]);
        for (i = 0; i < 3; i++)
            serialize16(instance, magADC[i]);
        break;
    case MSP_SERVO:
        s_struct(instance, (uint8_t *)&servo, 16);
        break;
    case MSP_SERVO_CONF:
        headSerialReply(instance, 56);
        for (i = 0; i < MAX_SERVOS; i++) {
            serialize16(instance, cfg.servoConf[i].min);
            serialize16(instance, cfg.servoConf[i].max);
            serialize16(instance, cfg.servoConf[i].middle);
            serialize8(instance, cfg.servoConf[i].rate);
        }
        break;
    case MSP_SET_SERVO_CONF:
        headSerialReply(instance, 0);
        for (i = 0; i < MAX_SERVOS; i++) {
            cfg.servoConf[i].min = read16(instance);
            cfg.servoConf[i].max = read16(instance);
            cfg.servoConf[i].middle = read16(instance);
            cfg.servoConf[i].rate = read8(instance);
        }
        break;
    case MSP_MOTOR:
        s_struct(instance, (uint8_t *)motor, 16);
        break;
    case MSP_RC:
        headSerialReply(instance, 16);
        for (i = 0; i < 8; i++)
            serialize16(instance, rcData[i]);
        break;
    case MSP_RAW_GPS:
        headSerialReply(instance, 16);
        serialize8(instance, f.GPS_FIX);
        serialize8(instance, GPS_numSat);
        serialize32(instance, GPS_coord[LAT]);
        serialize32(instance, GPS_coord[LON]);
        serialize16(instance, GPS_altitude);
        serialize16(instance, GPS_speed);
        serialize16(instance, GPS_ground_course);
        break;
    case MSP_COMP_GPS:
        headSerialReply(instance, 5);
        serialize16(instance, GPS_distanceToHome);
        serialize16(instance, GPS_directionToHome);
        serialize8(instance, GPS_update & 1);
        break;
    case MSP_ATTITUDE:
        headSerialReply(instance, 6);
        for (i = 0; i < 2; i++)
            serialize16(instance, angle[i]);
        serialize16(instance, heading);
        break;
    case MSP_ALTITUDE:
        headSerialReply(instance, 6);
        serialize32(instance, EstAlt);
        serialize16(instance, vario);
        break;
    case MSP_ANALOG:
        headSerialReply(instance, 7);
        serialize8(instance, vbat);
        serialize16(instance, 0); // power meter trash
        serialize16(instance, rssi);
        serialize16(instance, 0); // amperage
        break;
    case MSP_RC_TUNING:
        headSerialReply(instance, 7);
        serialize8(instance, cfg.rcRate8);
        serialize8(instance, cfg.rcExpo8);
        serialize8(instance, cfg.rollPitchRate);
        serialize8(instance, cfg.yawRate);
        serialize8(instance, cfg.dynThrPID);
        serialize8(instance, cfg.thrMid8);
        serialize8(instance, cfg.thrExpo8);
        break;
    case MSP_PID:
        headSerialReply(instance, 3 * PIDITEMS);
        for (i = 0; i < PIDITEMS; i++) {
            serialize8(instance, cfg.P8[i]);
            serialize8(instance, cfg.I8[i]);
            serialize8(instance, cfg.D8[i]);
        }
        break;
    case MSP_PIDNAMES:
        headSerialReply(instance, sizeof(pidnames) - 1);
        serializeNames(instance, pidnames);
        break;
    case MSP_BOX:
        headSerialReply(instance, 2 * numberBoxItems);
        for (i = 0; i < numberBoxItems; i++)
            serialize16(instance, cfg.activate[availableBoxes[i]]);
        break;
    case MSP_BOXNAMES:
        // headSerialReply(sizeof(boxnames) - 1);
        serializeBoxNamesReply(instance);
        break;
    case MSP_BOXIDS:
        headSerialReply(instance, numberBoxItems);
        for (i = 0; i < numberBoxItems; i++)
            serialize8(instance, availableBoxes[i]);
        break;
    case MSP_MISC:
        headSerialReply(instance, 2 * 6 + 4 + 2 + 4);
        serialize16(instance, 0); // intPowerTrigger1 (aka useless trash)
        serialize16(instance, mcfg.minthrottle);
        serialize16(instance, mcfg.maxthrottle);
        serialize16(instance, mcfg.mincommand);
        serialize16(instance, cfg.failsafe_throttle);
        serialize16(instance, 0); // plog useless shit
        serialize32(instance, 0); // plog useless shit
        serialize16(instance, cfg.mag_declination / 10); // TODO check this shit
        serialize8(instance, mcfg.vbatscale);
        serialize8(instance, mcfg.vbatmincellvoltage);
        serialize8(instance, mcfg.vbatmaxcellvoltage);
        serialize8(instance, 0);
        break;
    case MSP_MOTOR_PINS:
        headSerialReply(instance, 8);
        for (i = 0; i < 8; i++)
            serialize8(instance, i + 1);
        break;
    case MSP_WP:
        wp_no = read8(instance);    // get the wp number
        headSerialReply(instance, 18);
        if (wp_no == 0) {
            lat = GPS_home[LAT];
            lon = GPS_home[LON];
        } else if (wp_no == 16) {
            lat = GPS_hold[LAT];
            lon = GPS_hold[LON];
        }
        serialize8(instance, wp_no);
        serialize32(instance, lat);
        serialize32(instance, lon);
        serialize32(instance, AltHold);           // altitude (cm) will come here -- temporary implementation to test feature with apps
        serialize16(instance, 0);                 // heading  will come here (deg)
        serialize16(instance, 0);                 // time to stay (ms) will come here
        serialize8(instance, 0);                  // nav flag will come here
        break;
    case MSP_SET_WP:
        wp_no = read8(instance);    //get the wp number
        lat = read32(instance);
        lon = read32(instance);
        alt = read32(instance);     // to set altitude (cm)
        read16(instance);           // future: to set heading (deg)
        read16(instance);           // future: to set time to stay (ms)
        read8(instance);            // future: to set nav flag
        if (wp_no == 0) {
            GPS_home[LAT] = lat;
            GPS_home[LON] = lon;
            f.GPS_HOME_MODE = 0;        // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
            f.GPS_FIX_HOME = 1;
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
        } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
            GPS_hold[LAT] = lat;
            GPS_hold[LON] = lon;
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
            nav_mode = NAV_MODE_WP;
            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
        }
        headSerialReply(instance, 0);
        break;
    case MSP_RESET_CONF:
        if (!f.ARMED)
            checkFirstTime(true);
        headSerialReply(instance, 0);
        break;
    case MSP_ACC_CALIBRATION:
        if (!f.ARMED)
            calibratingA = CALIBRATING_ACC_CYCLES;
        headSerialReply(instance, 0);
        break;
    case MSP_MAG_CALIBRATION:
        if (!f.ARMED)
            f.CALIBRATE_MAG = 1;
        headSerialReply(instance, 0);
        break;
    case MSP_EEPROM_WRITE:
        if (f.ARMED) {
            headSerialError(instance, 0);
        } else {
            writeEEPROM(0, true);
            headSerialReply(instance, 0);
        }
        break;
    case MSP_DEBUG:
        headSerialReply(instance, 8);
        // make use of this crap, output some useful QA statistics
        debug[3] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]
        for (i = 0; i < 4; i++)
            serialize16(instance, debug[i]);      // 4 variables are here for general monitoring purpose
        break;

    // Additional commands that are not compatible with MultiWii
    case MSP_ACC_TRIM:
        headSerialReply(instance, 4);
        serialize16(instance, cfg.angleTrim[PITCH]);
        serialize16(instance, cfg.angleTrim[ROLL]);
        break;
    case MSP_UID:
        headSerialReply(instance, 12);
        serialize32(instance, U_ID_0);
        serialize32(instance, U_ID_1);
        serialize32(instance, U_ID_2);
        break;
    case MSP_GPSSVINFO:
        headSerialReply(instance, 1 + (GPS_numCh * 4));
        serialize8(instance, GPS_numCh);
           for (i = 0; i < GPS_numCh; i++){
               serialize8(instance, GPS_svinfo_chn[i]);
               serialize8(instance, GPS_svinfo_svid[i]);
               serialize8(instance, GPS_svinfo_quality[i]);
               serialize8(instance, GPS_svinfo_cno[i]);
            }
        break;
    default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
        headSerialError(instance, 0);
        break;
    }
    tailSerialReply(instance);
}

// evaluate all other incoming serial data
static void evaluateOtherData(uint8_t sr)
{
    if (sr == '#')
        cliProcess();
    else if (sr == mcfg.reboot_character)
        systemReset(true);      // reboot to bootloader
}

void serialInstanceCom(mwproto_t *instance, bool isMainPort)
{
    uint8_t c;

    while (serialTotalBytesWaiting(instance->port)) {
        c = serialRead(instance->port);

        if (instance->c_state == IDLE) {
        	instance->c_state = (c == '$') ? HEADER_START : IDLE;
            if (instance->c_state == IDLE && !f.ARMED && isMainPort)
                evaluateOtherData(c); // if not armed evaluate all other incoming serial data
        } else if (instance->c_state == HEADER_START) {
        	instance->c_state = (c == 'M') ? HEADER_M : IDLE;
        } else if (instance->c_state == HEADER_M) {
        	instance->c_state = (c == '<') ? HEADER_ARROW : IDLE;
        } else if (instance->c_state == HEADER_ARROW) {
            if (c > INBUF_SIZE) {       // now we are expecting the payload size
            	instance->c_state = IDLE;
                continue;
            }
            instance->dataSize = c;
            instance->offset = 0;
            instance->checksum = 0;
            instance->indRX = 0;
            instance->checksum ^= c;
            instance->c_state = HEADER_SIZE;      // the command is to follow
        } else if (instance->c_state == HEADER_SIZE) {
        	instance->cmdMSP = c;
        	instance->checksum ^= c;
        	instance->c_state = HEADER_CMD;
        } else if (instance->c_state == HEADER_CMD && instance->offset < instance->dataSize) {
        	instance->checksum ^= c;
        	instance->inBuf[instance->offset++] = c;
        } else if (instance->c_state == HEADER_CMD && instance->offset >= instance->dataSize) {
            if (instance->checksum == c) {        // compare calculated and transferred checksum
                evaluateCommand(instance);      // we got a valid packet, evaluate it
            }
            instance->c_state = IDLE;
        }
    }
}

void serialCom(void)
{
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }

	int idx;
	for (idx = 0; idx < 3; idx++) {
		if (serialinstances[idx].port)
			serialInstanceCom(&serialinstances[idx], idx == 0);
	}
}
