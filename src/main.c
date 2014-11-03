/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"

#include "telemetry_common.h"

core_t core;
int hw_revision = 0;

extern rcReadRawDataPtr rcReadRawFunc;

// receiver read function
extern uint16_t pwmReadRawRC(uint8_t chan);

// from system_stm32f10x.c
void SetSysClock(bool overclock);

#ifdef USE_LAME_PRINTF
// gcc/GNU version
static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(core.mainport, c);
}
#else
// keil/armcc version
int fputc(int c, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    while (!isSerialTransmitBufferEmpty(core.mainport));
    serialWrite(core.mainport, c);
    return c;
}
#endif

int main(void)
{
    uint8_t i;
    int id;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;
    bool sensorsOK = false;
#ifdef SOFTSERIAL_LOOPBACK
    serialPort_t* loopbackPort1 = NULL;
    serialPort_t* loopbackPort2 = NULL;
#endif

    initEEPROM();
    checkFirstTime(false);
    readEEPROM();

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(mcfg.emf_avoidance);

    // determine hardware revision
    if (hse_value == 8000000)
        hw_revision = NAZE32;
    else if (hse_value == 12000000)
        hw_revision = NAZE32_REV5;

    systemInit();
#ifdef USE_LAME_PRINTF
    init_printf(NULL, _putc);
#endif

    activateConfig();

#ifndef CJMCU
    id = spiInit();
    if (id == SPI_DEVICE_MPU && hw_revision == NAZE32_REV5)
        hw_revision = NAZE32_SP;
#endif

    if (hw_revision != NAZE32_SP)
        i2cInit(I2C_DEVICE);

    // configure power ADC
    if (mcfg.power_adc_channel > 0 && (mcfg.power_adc_channel == 1 || mcfg.power_adc_channel == 9))
        adc_params.powerAdcChannel = mcfg.power_adc_channel;
    else {
        adc_params.powerAdcChannel = 0;
        mcfg.power_adc_channel = 0;
    }

    if (mcfg.rssi_adc_channel > 0 && (mcfg.rssi_adc_channel == 1 || mcfg.rssi_adc_channel == 9) && mcfg.rssi_adc_channel != mcfg.power_adc_channel)
        adc_params.rssiAdcChannel = mcfg.rssi_adc_channel;
    else {
        adc_params.rssiAdcChannel = 0;
        mcfg.rssi_adc_channel = 0;
    }

    adcInit(&adc_params);
    // Check battery type/voltage
    if (feature(FEATURE_VBAT))
        batteryInit();
    initBoardAlignment();

    // We have these sensors; SENSORS_SET defined in board.h depending on hardware platform
    sensorsSet(SENSORS_SET);
    // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
    sensorsOK = sensorsAutodetect();

    // production debug output
#ifdef PROD_DEBUG
    productionDebug();
#endif

    // if gyro was not detected due to whatever reason, we give up now.
    if (!sensorsOK)
        failureMode(3);

    LED1_ON;
    LED0_OFF;
    for (i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        BEEP_ON;
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;

    imuInit(); // Mag is initialized inside imuInit
    mixerInit(); // this will set core.useServo var depending on mixer type

    serialInit(mcfg.serial_baudrate);

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SERIALRX); // spektrum/sbus support uses UART too
    pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SERIALRX); // disable inputs if using spektrum
    pwm_params.useServos = core.useServo;
    pwm_params.extraServos = cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = mcfg.motor_pwm_rate;
    pwm_params.servoPwmRate = mcfg.servo_pwm_rate;
    pwm_params.pwmFilter = mcfg.pwm_filter;
    pwm_params.idlePulse = PULSE_1MS; // standard PWM for brushless ESC (default, overridden below)
    if (feature(FEATURE_3D))
        pwm_params.idlePulse = mcfg.neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors
    pwm_params.servoCenterPulse = mcfg.midrc;
    pwm_params.failsafeThreshold = cfg.failsafe_detect_threshold;
    switch (mcfg.power_adc_channel) {
        case 1:
            pwm_params.adcChannel = PWM2;
            break;
        case 9:
            pwm_params.adcChannel = PWM8;
            break;
        default:
            pwm_params.adcChannel = 0;
            break;
    }

    pwmInit(&pwm_params);
    core.numServos = pwm_params.numServos;

    // configure PWM/CPPM read function and max number of channels. spektrum or sbus below will override both of these, if enabled
    for (i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;
    rcReadRawFunc = pwmReadRawRC;
    core.numRCChannels = MAX_INPUTS;

    if (feature(FEATURE_SERIALRX)) {
        switch (mcfg.serialrx_type) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                spektrumInit(&rcReadRawFunc);
                break;
            case SERIALRX_SBUS:
                sbusInit(&rcReadRawFunc);
                break;
            case SERIALRX_SUMD:
                sumdInit(&rcReadRawFunc);
                break;
            case SERIALRX_MSP:
                mspInit(&rcReadRawFunc);
                break;
        }
    }
#ifndef CJMCU
    // Optional GPS - available in both PPM, PWM and serialRX input mode, in PWM input, reduces number of available channels by 2.
    // gpsInit will return if FEATURE_GPS is not enabled.
    gpsInit(mcfg.gps_baudrate);
#endif
#ifdef SONAR
    // sonar stuff only works with PPM
    if (feature(FEATURE_PPM)) {
        if (feature(FEATURE_SONAR))
            Sonar_init();
    }
#endif

#ifndef CJMCU
    if (feature(FEATURE_SOFTSERIAL)) {
        //mcfg.softserial_baudrate = 19200; // Uncomment to override config value

        setupSoftSerialPrimary(mcfg.softserial_baudrate, mcfg.softserial_1_inverted);
        setupSoftSerialSecondary(mcfg.softserial_2_inverted);

#ifdef SOFTSERIAL_LOOPBACK
        loopbackPort1 = (serialPort_t*)&(softSerialPorts[0]);
        serialPrint(loopbackPort1, "SOFTSERIAL 1 - LOOPBACK ENABLED\r\n");

        loopbackPort2 = (serialPort_t*)&(softSerialPorts[1]);
        serialPrint(loopbackPort2, "SOFTSERIAL 2 - LOOPBACK ENABLED\r\n");
#endif
        //core.mainport = (serialPort_t*)&(softSerialPorts[0]); // Uncomment to switch the main port to use softserial.
    }

    if (feature(FEATURE_TELEMETRY))
        initTelemetry();
#endif

    previousTime = micros();
    if (mcfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = CALIBRATING_ACC_CYCLES;
    calibratingG = CALIBRATING_GYRO_CYCLES;
    calibratingB = CALIBRATING_BARO_CYCLES;             // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
    f.SMALL_ANGLE = 1;

    // loopy
    while (1) {
        loop();
#ifdef SOFTSERIAL_LOOPBACK
        if (loopbackPort1) {
            while (serialTotalBytesWaiting(loopbackPort1)) {
                uint8_t b = serialRead(loopbackPort1);
                serialWrite(loopbackPort1, b);
                //serialWrite(core.mainport, 0x01);
                //serialWrite(core.mainport, b);
            };
        }

        if (loopbackPort2) {
            while (serialTotalBytesWaiting(loopbackPort2)) {
#ifndef OLIMEXINO // PB0/D27 and PB1/D28 internally connected so this would result in a continuous stream of data
                serialRead(loopbackPort2);
#else
                uint8_t b = serialRead(loopbackPort2);
                serialWrite(loopbackPort2, b);
                //serialWrite(core.mainport, 0x02);
                //serialWrite(core.mainport, b);
#endif // OLIMEXINO
            };
    }
#endif
    }
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(mcfg.mincommand);
    while (1);
}
