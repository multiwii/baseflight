#include "board.h"
#include "mw.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliCMix(char *cmdline);
static void cliDefaults(char *cmdline);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliSave(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);

// from sensors.c
extern uint8_t batteryCellCount;
extern uint8_t accHardware;

// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
const char * const featureNames[] = {
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "SPEKTRUM", "MOTOR_STOP",
    "SERVO_TILT", "GYRO_SMOOTHING", "LED_RING", "GPS",
    "FAILSAFE", "SONAR", "TELEMETRY",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] = {
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

// 
const char * const accNames[] = {
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct {
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    { "cmix", "design custom mixer", cliCMix },
    { "defaults", "reset to defaults and reboot", cliDefaults },
    { "exit", "", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "help", "", cliHelp },
    { "map", "mapping of rc channel order", cliMap },
    { "mixer", "mixer name or list", cliMixer },
    { "save", "save and reboot", cliSave },
    { "set", "name=value or blank or * for list", cliSet },
    { "status", "show system status", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

typedef enum {
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct {
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "deadband", VAR_UINT8, &cfg.deadband, 0, 32 },
    { "yawdeadband", VAR_UINT8, &cfg.yawdeadband, 0, 100 },
    { "alt_hold_throttle_neutral", VAR_UINT8, &cfg.alt_hold_throttle_neutral, 1, 250 },
    { "midrc", VAR_UINT16, &cfg.midrc, 1200, 1700 },
    { "minthrottle", VAR_UINT16, &cfg.minthrottle, 0, 2000 },
    { "maxthrottle", VAR_UINT16, &cfg.maxthrottle, 0, 2000 },
    { "mincommand", VAR_UINT16, &cfg.mincommand, 0, 2000 },
    { "mincheck", VAR_UINT16, &cfg.mincheck, 0, 2000 },
    { "maxcheck", VAR_UINT16, &cfg.maxcheck, 0, 2000 },
    { "retarded_arm", VAR_UINT8, &cfg.retarded_arm, 0, 1 },
    { "failsafe_delay", VAR_UINT8, &cfg.failsafe_delay, 0, 200 },
    { "failsafe_off_delay", VAR_UINT8, &cfg.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle", VAR_UINT16, &cfg.failsafe_throttle, 1000, 2000 },
    { "motor_pwm_rate", VAR_UINT16, &cfg.motor_pwm_rate, 50, 498 },
    { "servo_pwm_rate", VAR_UINT16, &cfg.servo_pwm_rate, 50, 498 },
    { "serial_baudrate", VAR_UINT32, &cfg.serial_baudrate, 1200, 115200 },
    { "gps_baudrate", VAR_UINT32, &cfg.gps_baudrate, 1200, 115200 },
    { "spektrum_hires", VAR_UINT8, &cfg.spektrum_hires, 0, 1 },
    { "vbatscale", VAR_UINT8, &cfg.vbatscale, 10, 200 },
    { "vbatmaxcellvoltage", VAR_UINT8, &cfg.vbatmaxcellvoltage, 10, 50 },
    { "vbatmincellvoltage", VAR_UINT8, &cfg.vbatmincellvoltage, 10, 50 },
    { "power_adc_channel", VAR_UINT8, &cfg.power_adc_channel, 0, 9 },
    { "yaw_direction", VAR_INT8, &cfg.yaw_direction, -1, 1 },
    { "tri_yaw_middle", VAR_UINT16, &cfg.tri_yaw_middle, 0, 2000 },
    { "tri_yaw_min", VAR_UINT16, &cfg.tri_yaw_min, 0, 2000 },
    { "tri_yaw_max", VAR_UINT16, &cfg.tri_yaw_max, 0, 2000 },
    { "wing_left_min", VAR_UINT16, &cfg.wing_left_min, 0, 2000 },
    { "wing_left_mid", VAR_UINT16, &cfg.wing_left_mid, 0, 2000 },
    { "wing_left_max", VAR_UINT16, &cfg.wing_left_max, 0, 2000 },
    { "wing_right_min", VAR_UINT16, &cfg.wing_right_min, 0, 2000 },
    { "wing_right_mid", VAR_UINT16, &cfg.wing_right_mid, 0, 2000 },
    { "wing_right_max", VAR_UINT16, &cfg.wing_right_max, 0, 2000 },
    { "pitch_direction_l", VAR_INT8, &cfg.pitch_direction_l, -1, 1 },
    { "pitch_direction_r", VAR_INT8, &cfg.pitch_direction_r, -1, 1 },
    { "roll_direction_l", VAR_INT8, &cfg.roll_direction_l, -1, 1 },
    { "roll_direction_r", VAR_INT8, &cfg.roll_direction_r, -1, 1 },
    { "gimbal_flags", VAR_UINT8, &cfg.gimbal_flags, 0, 255},
    { "gimbal_pitch_gain", VAR_INT8, &cfg.gimbal_pitch_gain, -100, 100 },
    { "gimbal_roll_gain", VAR_INT8, &cfg.gimbal_roll_gain, -100, 100 },
    { "gimbal_pitch_min", VAR_UINT16, &cfg.gimbal_pitch_min, 100, 3000 },
    { "gimbal_pitch_max", VAR_UINT16, &cfg.gimbal_pitch_max, 100, 3000 },
    { "gimbal_pitch_mid", VAR_UINT16, &cfg.gimbal_pitch_mid, 100, 3000 },
    { "gimbal_roll_min", VAR_UINT16, &cfg.gimbal_roll_min, 100, 3000 },
    { "gimbal_roll_max", VAR_UINT16, &cfg.gimbal_roll_max, 100, 3000 },
    { "gimbal_roll_mid", VAR_UINT16, &cfg.gimbal_roll_mid, 100, 3000 },
    { "align_gyro_x", VAR_INT8, &cfg.align[ALIGN_GYRO][0], -3, 3 },
    { "align_gyro_y", VAR_INT8, &cfg.align[ALIGN_GYRO][1], -3, 3 },
    { "align_gyro_z", VAR_INT8, &cfg.align[ALIGN_GYRO][2], -3, 3 },
    { "align_acc_x", VAR_INT8, &cfg.align[ALIGN_ACCEL][0], -3, 3 },
    { "align_acc_y", VAR_INT8, &cfg.align[ALIGN_ACCEL][1], -3, 3 },
    { "align_acc_z", VAR_INT8, &cfg.align[ALIGN_ACCEL][2], -3, 3 },
    { "align_mag_x", VAR_INT8, &cfg.align[ALIGN_MAG][0], -3, 3 },
    { "align_mag_y", VAR_INT8, &cfg.align[ALIGN_MAG][1], -3, 3 },
    { "align_mag_z", VAR_INT8, &cfg.align[ALIGN_MAG][2], -3, 3 },
    { "acc_hardware", VAR_UINT8, &cfg.acc_hardware, 0, 3 },
    { "acc_lpf_factor", VAR_UINT8, &cfg.acc_lpf_factor, 0, 250 },
    { "acc_lpf_for_velocity", VAR_UINT8, &cfg.acc_lpf_for_velocity, 1, 250 },
    { "acc_trim_pitch", VAR_INT16, &cfg.angleTrim[PITCH], -300, 300 },
    { "acc_trim_roll", VAR_INT16, &cfg.angleTrim[ROLL], -300, 300 },
    { "gyro_lpf", VAR_UINT16, &cfg.gyro_lpf, 0, 256 },
    { "gyro_cmpf_factor", VAR_UINT16, &cfg.gyro_cmpf_factor, 100, 1000 },
    { "mpu6050_scale", VAR_UINT8, &cfg.mpu6050_scale, 0, 1 },
    { "baro_tab_size", VAR_UINT8, &cfg.baro_tab_size, 0, BARO_TAB_SIZE_MAX },
    { "baro_noise_lpf", VAR_FLOAT, &cfg.baro_noise_lpf, 0, 1 },
    { "baro_cf", VAR_FLOAT, &cfg.baro_cf, 0, 1 },
    { "moron_threshold", VAR_UINT8, &cfg.moron_threshold, 0, 128 },
    { "mag_declination", VAR_INT16, &cfg.mag_declination, -18000, 18000 },
    { "gps_type", VAR_UINT8, &cfg.gps_type, 0, 3 },
    { "gps_pos_p", VAR_UINT8, &cfg.P8[PIDPOS], 0, 200 },
    { "gps_pos_i", VAR_UINT8, &cfg.I8[PIDPOS], 0, 200 },
    { "gps_pos_d", VAR_UINT8, &cfg.D8[PIDPOS], 0, 200 },
    { "gps_posr_p", VAR_UINT8, &cfg.P8[PIDPOSR], 0, 200 },
    { "gps_posr_i", VAR_UINT8, &cfg.I8[PIDPOSR], 0, 200 },
    { "gps_posr_d", VAR_UINT8, &cfg.D8[PIDPOSR], 0, 200 },
    { "gps_nav_p", VAR_UINT8, &cfg.P8[PIDNAVR], 0, 200 },
    { "gps_nav_i", VAR_UINT8, &cfg.I8[PIDNAVR], 0, 200 },
    { "gps_nav_d", VAR_UINT8, &cfg.D8[PIDNAVR], 0, 200 },
    { "gps_wp_radius", VAR_UINT16, &cfg.gps_wp_radius, 0, 2000 },
    { "nav_controls_heading", VAR_UINT8, &cfg.nav_controls_heading, 0, 1 },
    { "nav_speed_min", VAR_UINT16, &cfg.nav_speed_min, 10, 2000 },
    { "nav_speed_max", VAR_UINT16, &cfg.nav_speed_max, 10, 2000 },
    { "nav_slew_rate", VAR_UINT8, &cfg.nav_slew_rate, 0, 100 },
    { "looptime", VAR_UINT16, &cfg.looptime, 0, 9000 },
		{ "pt1f_cut", VAR_UINT16, &cfg.f_cut, 0, 100 },
		{ "p_pitch", VAR_UINT8, &cfg.P8[PITCH], 0, 200 },
    { "i_pitch", VAR_UINT8, &cfg.I8[PITCH], 0, 200 },
    { "d_pitch", VAR_UINT8, &cfg.D8[PITCH], 0, 200 },
    { "p_roll", VAR_UINT8, &cfg.P8[ROLL], 0, 200 },
    { "i_roll", VAR_UINT8, &cfg.I8[ROLL], 0, 200 },
    { "d_roll", VAR_UINT8, &cfg.D8[ROLL], 0, 200 },
    { "p_yaw", VAR_UINT8, &cfg.P8[YAW], 0, 200 },
    { "i_yaw", VAR_UINT8, &cfg.I8[YAW], 0, 200 },
    { "d_yaw", VAR_UINT8, &cfg.D8[YAW], 0, 200 },
    { "p_alt", VAR_UINT8, &cfg.P8[PIDALT], 0, 200 },
    { "i_alt", VAR_UINT8, &cfg.I8[PIDALT], 0, 200 },
    { "d_alt", VAR_UINT8, &cfg.D8[PIDALT], 0, 200 },
    { "p_level", VAR_UINT8, &cfg.P8[PIDLEVEL], 0, 200 },
    { "i_level", VAR_UINT8, &cfg.I8[PIDLEVEL], 0, 200 },
    { "d_level", VAR_UINT8, &cfg.D8[PIDLEVEL], 0, 200 },
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

static void cliSetVar(const clivalue_t *var, const int32_t value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0) 
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0) {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    } else 
        *i2a(i, a, r) = 0;
    return a;
} 

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    int frac = 0;
    double sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p) ) {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0;
    if (*p == '-') {
        sign = -1.0;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0;
    while (valid_digit(*p)) {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.') {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;

        // Calculate scaling factor.
        while (expon >= 50) { scale *= 1E50; expon -= 50; }
        while (expon >=  8) { scale *= 1E8;  expon -=  8; }
        while (expon >   0) { scale *= 10.0; expon -=  1; }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t) (x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void cliCMix(char *cmdline)
{
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    float mixsum[3];
    char *ptr;

    len = strlen(cmdline);

    if (len == 0) {
        uartPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++) {
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
            mixsum[i] = 0.0f;
            num_motors++;
            printf("#%d:\t", i + 1);
            printf("%s\t", ftoa(cfg.customMixer[i].throttle, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].roll, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].pitch, buf));
            printf("%s\r\n", ftoa(cfg.customMixer[i].yaw, buf));
        }
        for (i = 0; i < num_motors; i++) {
            mixsum[0] += cfg.customMixer[i].roll;
            mixsum[1] += cfg.customMixer[i].pitch;
            mixsum[2] += cfg.customMixer[i].yaw;
        }
        uartPrint("Sanity check:\t");
        for (i = 0; i < 3; i++)
            uartPrint(fabs(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        uartPrint("\r\n");
        return;
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    uartPrint("Invalid mixer type...\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i);
                    printf("Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_MOTORS) {
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].throttle = _atof(++ptr); 
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].roll = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].pitch = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].yaw = _atof(++ptr);
                check++;
            }
            if (check != 4) {
                uartPrint("Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            } else {
                cliCMix("");
            }
        } else {
            printf("Motor number must be between 1 and %d\r\n", MAX_MOTORS);
        }
    }
}

static void cliDefaults(char *cmdline)
{
    uartPrint("Resetting to defaults...\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting...");
    delay(10);
    systemReset(false);
}

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode...\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
    cliMode = 0;
    // save and reboot... I think this makes the most sense
    cliSave(cmdline);
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (len == 0) {
        uartPrint("Enabled features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        uartPrint("Available features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                uartPrint("Invalid feature name...\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0) {
                if (remove) {
                    featureClear(1 << i);
                    uartPrint("Disabled ");
                } else {
                    featureSet(1 << i);
                    uartPrint("Enabled ");
                }
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    uartPrint("Available commands:\r\n");    
    for (i = 0; i < CMD_COUNT; i++)
        printf("%s\t%s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];

    len = strlen(cmdline);

    if (len == 8) {
        // uppercase it
        for (i = 0; i < 8; i++)
            cmdline[i] = toupper(cmdline[i]);
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            uartPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    uartPrint("Current assignment: ");
    for (i = 0; i < 8; i++)
        out[cfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    printf("%s\r\n", out);
}

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len;
    
    len = strlen(cmdline);

    if (len == 0) {
        printf("Current mixer: %s\r\n", mixerNames[cfg.mixerConfiguration - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        uartPrint("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            printf("%s ", mixerNames[i]);
        }
        uartPrint("\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            uartPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            cfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeParams(0);
    uartPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type) {
        case VAR_UINT8:
            value = *(uint8_t *)var->ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)var->ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)var->ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)var->ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)var->ptr;
            break;

        case VAR_FLOAT:
            printf("%s", ftoa(*(float *)var->ptr, buf));
            if (full) {
                printf(" %s", ftoa((float)var->min, buf));
                printf(" %s", ftoa((float)var->max, buf));
            }
            return; // return from case for float only
    }
    printf("%d", value);
    if (full)
        printf(" %d %d", var->min, var->max);
}

static void cliSetVar(const clivalue_t *var, const int32_t value)
{
    switch (var->type) {
        case VAR_UINT8:
        case VAR_INT8:
            *(char *)var->ptr = (char)value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(short *)var->ptr = (short)value;
            break;

        case VAR_UINT32:
            *(int *)var->ptr = (int)value;
            break;

        case VAR_FLOAT:
            *(float *)var->ptr = *(float *)&value;
            break;
    }
}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    float valuef = 0;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            printf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
        }
    } else if ((eqptr = strstr(cmdline, "="))) {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max) { // here we compare the float value since... it should work, RIGHT?
                    cliSetVar(val, valueTable[i].type == VAR_FLOAT ? *(uint32_t *)&valuef : value); // this is a silly dirty hack. please fix me later.
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    uartPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        uartPrint("ERR: Unknown variable name\r\n");
    }
}

static void cliStatus(char *cmdline)
{
    uint8_t i;
    uint32_t mask;

    printf("System Uptime: %d seconds, Voltage: %d * 0.1V (%dS battery)\r\n",
        millis() / 1000, vbat, batteryCellCount);
    mask = sensorsMask();

    printf("CPU %dMHz, detected sensors: ", (SystemCoreClock / 1000000));
    for (i = 0; ; i++) {
        if (sensorNames[i] == NULL)
            break;
        if (mask & (1 << i))
            printf("%s ", sensorNames[i]);
    }
    if (sensors(SENSOR_ACC))
        printf("ACCHW: %s", accNames[accHardware]);
    uartPrint("\r\n");

    printf("Cycle Time: %d, I2C Errors: %d\r\n", cycleTime, i2cGetErrorCounter());
}

static void cliVersion(char *cmdline)
{
    uartPrint("Afro32 CLI version 2.1 " __DATE__ " / " __TIME__);
}

void cliProcess(void)
{
    if (!cliMode) {
        cliMode = 1;
        uartPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
        cliPrompt();
    }

    while (uartAvailable()) {
        uint8_t c = uartRead();
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            int i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex]) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                uartPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    uartPrint(cmd->name);
                    uartWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                uartWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {
            // clear screen
            uartPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
            uartPrint("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate
            
            target.name = cliBuffer;
            target.param = NULL;
            
            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else
                uartPrint("ERR: Unknown command, try 'help'");

            memset(cliBuffer, 0, sizeof(cliBuffer));
            bufferIndex = 0;

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;
            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                uartPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == 32)
                continue;
            cliBuffer[bufferIndex++] = c;
            uartWrite(c);
        }
    }
}
