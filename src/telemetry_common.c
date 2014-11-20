#include "board.h"
#include "mw.h"

#include "telemetry_frsky.h"
#include "telemetry_hott.h"

static bool isTelemetryConfigurationValid = false; // flag used to avoid repeated configuration checks

bool isTelemetryProviderFrSky(void)
{
    return mcfg.telemetry_provider == TELEMETRY_PROVIDER_FRSKY;
}

bool isTelemetryProviderHoTT(void)
{
    return mcfg.telemetry_provider == TELEMETRY_PROVIDER_HOTT;
}

bool canUseTelemetryWithCurrentConfiguration(void)
{

    if (!feature(FEATURE_TELEMETRY)) {
        return false;
    }

    if (!feature(FEATURE_SOFTSERIAL)) {
        if (mcfg.telemetry_port == TELEMETRY_PORT_SOFTSERIAL_1 || mcfg.telemetry_port == TELEMETRY_PORT_SOFTSERIAL_2) {
            // softserial feature must be enabled to use telemetry on softserial ports
            return false;
        }
    }

    if (isTelemetryProviderHoTT()) {
        if (mcfg.telemetry_port == TELEMETRY_PORT_UART) {
            // HoTT requires a serial port that supports RX/TX mode swapping
            return false;
        }
    }

    return true;
}

void initTelemetry(void)
{
    // Force telemetry to uart when softserial disabled
    if (!feature(FEATURE_SOFTSERIAL))
        mcfg.telemetry_port = TELEMETRY_PORT_UART;

    isTelemetryConfigurationValid = canUseTelemetryWithCurrentConfiguration();

    if (mcfg.telemetry_port == TELEMETRY_PORT_SOFTSERIAL_1)
        core.telemport = &(softSerialPorts[0].port);
    else if (mcfg.telemetry_port == TELEMETRY_PORT_SOFTSERIAL_2)
        core.telemport = &(softSerialPorts[1].port);
    else
        core.telemport = core.mainport;

    checkTelemetryState();
}

static bool telemetryEnabled = false;

bool determineNewTelemetryEnabledState(void)
{
    bool enabled = true;

    if (mcfg.telemetry_port == TELEMETRY_PORT_UART) {
        if (!mcfg.telemetry_switch)
            enabled = f.ARMED;
        else
            enabled = rcOptions[BOXTELEMETRY];
    }

    return enabled;
}

bool shouldChangeTelemetryStateNow(bool newState)
{
    return newState != telemetryEnabled;
}

static void configureTelemetryPort(void)
{
    if (isTelemetryProviderFrSky()) {
        configureFrSkyTelemetryPort();
    }

    if (isTelemetryProviderHoTT()) {
        configureHoTTTelemetryPort();
    }
}

void freeTelemetryPort(void)
{
    if (isTelemetryProviderFrSky()) {
        freeFrSkyTelemetryPort();
    }

    if (isTelemetryProviderHoTT()) {
        freeHoTTTelemetryPort();
    }
}

void checkTelemetryState(void)
{
    if (!isTelemetryConfigurationValid) {
        return;
    }

    bool newEnabledState = determineNewTelemetryEnabledState();

    if (!shouldChangeTelemetryStateNow(newEnabledState)) {
        return;
    }

    if (newEnabledState)
        configureTelemetryPort();
    else
        freeTelemetryPort();

    telemetryEnabled = newEnabledState;
}

void handleTelemetry(void)
{
    if (!isTelemetryConfigurationValid || !determineNewTelemetryEnabledState())
        return;

    if (isTelemetryProviderFrSky()) {
        handleFrSkyTelemetry();
    }

    if (isTelemetryProviderHoTT()) {
        handleHoTTTelemetry();
    }
}
