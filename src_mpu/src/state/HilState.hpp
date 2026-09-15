#pragma once

#include "../models/FlightModel.hpp"

// State used only by the WOKWI_SIM serial adapter. Real hardware receives
// these values from sensors and radio input instead.
#ifdef WOKWI_SIM
volatile double simGpsLat = 36.123456;
volatile double simGpsLon = -80.123456;
volatile bool   simGpsFix = false;

volatile bool hilGatePending = false;
volatile bool hilGateApproved = false;
volatile FlightPhase hilGateNext = PHASE_PARKED;
volatile TransitionReason hilGateReason = REASON_NONE;
#endif