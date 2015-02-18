/**
 * Copyright (C) 2012-2015 baseflight
 *
 * License: http://www.gnu.org/licenses/gpl.html GPL version 3 or higher
 */

#ifndef TELEMETRY_FRSKY_H_
#define TELEMETRY_FRSKY_H_

void handleFrSkyTelemetry(void);
void checkFrSkyTelemetryState(void);

void configureFrSkyTelemetryPort(void);
void freeFrSkyTelemetryPort(void);

#endif
