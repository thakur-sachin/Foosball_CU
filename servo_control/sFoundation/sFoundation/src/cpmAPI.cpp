//*****************************************************************************
// $Workfile: cpmAPI.cpp $
//
// DESCRIPTION:
/**
    \file
    \brief ClearPath-SC Programming Interface Implementation.

    \defgroup ClearPathgrp ClearPath Programming Interface
    \brief Functions related to ClearPath functionality.

    Provides:
        - checking on the operation state of the node
        - initiating and canceling motion
        - accessing and changing I/O
        - accessing and modifying the parameter table

    \remark C and C++ Applications should include the master header file
    \c meridianHdrs.h in the source code to insure the various function
    prototypes and data types are available for your application.

    @{
**/
//
// CREATION DATE:
//      8/2/2011 DS refactored from Meridian iscAPI.cpp
//
// COPYRIGHT NOTICE:
//      (C)Copyright 1998-2018  Teknic, Inc.  All rights reserved.
//
//      This copyright notice must be reproduced in any copy, modification,
//      or portion thereof merged into another program. A copy of the
//      copyright notice must be included in the object library of a user
//      program.
//                                                                            *
//*****************************************************************************

//*****************************************************************************
// NAME                                                                       *
//   cpscAPI.cpp headers
//
#include "mnDiags.h"
#include "pubNetAPI.h"
#include "netCmdAPI.h"
#include "pubCpmRegs.h"
#include "pubCpmAPI.h"
#include "pubIscAPI.h"
#include "sFoundResource.h"
#include "netCmdPrivate.h"
#include "converterLib.h"
#include "iscRegs.h"
#include "cpmRegs.h"
#include "pubCpmAdvAPI.h"
/// \cond   INTERNAL_DOC
#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include "lnkAccessCommon.h"

#if defined(_MSC_VER)
    #pragma warning(disable:4996)
    #if _MSC_VER<=1800
        #define snprintf sprintf_s
    #endif
#endif
#if (defined(_WIN32)||defined(_WIN64))
    #include <crtdbg.h>
#else
    #include <string.h>
#endif
/// \endcond                                                                  *
//*****************************************************************************


//*****************************************************************************
// NAME                                                                       *
//   cpscAPI.cpp function prototypes
//
/// \cond INTERNAL_DOC
#define Sgn(x) (((x)>0) ? 1 : (((x)==0) ? 0 : -1))
#define Q15_MAX (32767./32768.)
#define Q11_MAX 2048
// Module private functions
// Add back enum that we hide
#define CPM_P_FACT_DRV_ENC_EFF_DENS (cpmParams)322

// Buffer management items
// "Class" constructor
cnErrCode cpmClassSetup(multiaddr theMultiAddr);
// "Class" destructors
void cpmClassDelete(multiaddr theMultiAddr);
/// \endcond                                                                  *
//*****************************************************************************



//*****************************************************************************
// NAME                                                                       *
//  cpscAPI.cpp imports
//
/// \cond INTERNAL_DOC
extern mnNetInvRecords SysInventory[NET_CONTROLLER_MAX];
/// \endcond                                                                  *
//                                                                            *
//*****************************************************************************



//*****************************************************************************
// NAME                                                                       *
//  cpscAPI.cpp constants
//
/// \cond INTERNAL_DOC
const double CP_MON_SCALE = 1 << 16;        // Format of scale/ampl control
const double CP_MON_MAX_VEL = 8192;         // Vel. full-scale (quads/sample-time)
const double CP_MON_MIN_VEL = 4;            // Minimum velocity to display
const double CP_MON_MAX_POS_LEGACY = 8192;  // Position error full-scale
const double CP_MON_MAX_POS = 32768;        // Position error full-scale
const double CP_MON_MAX_POS_MEAS = (1 << 20);
const double CP_MON_MAX_JRK = 0.000001;
const double CP_MON_MAX_INTG = 1UL << 31;
#define CP_ADC_SCALE (0.8*(double)(1<<13))  // ADC measure to torque values

// RAS selector table
typedef struct _rasTarg {
    double target;                      // Optimal value
    unsigned code;                      // RAS code for this
} rasTarg;

#define ISC_MV_NEG_LIMIT  0x800000  // Largest negative move
#define ISC_MV_POS_LIMIT  0x7FFFFF  // Largest positive move

const double ISC_MON_SCALE = 1 << 16;   // Format of scale/ampl control
const double ISC_MON_MAX_VEL = 8192;    // Vel. full-scale (quads/sample-time)
const double ISC_MON_MIN_VEL = 4;       // Minimum velocity to display
const double ISC_MON_MAX_POS_LEGACY = 8192; // Position error full-scale
const double ISC_MON_MAX_POS = 32768;       // Position error full-scale
const double ISC_MON_MAX_POS_MEAS = (1 << 20);
const double ISC_MON_MAX_JRK = 0.000001;
const double ISC_MON_MAX_INTG = 1UL << 31;
// Gated velocity error (V2 feature)
const double MON_MAX_VEL_SAMPLES = 512 - 64 - 4;
const double MON_MAX_VEL_ERR_GATED = 2048;
const double MON_MAX_VEL_GATED_Q = 16;

// Hidden types
const monTestPoints MON_VEL_RAS = iscMonVars(59);
const monTestPoints MON_POSN_CMD = iscMonVars(76);


// Parameter locations that we don't want exposed publicly
const int CPM_P_DRV_ENC_DENS = 310;
const int CPM_P_DRV_MTR_POLES = 311;

// Status register bit/field names
const char *CPSCStatusBitStrs[] = {
    "Warning", "UserAlert", "NotReady", "MoveBufAvail",
    "Ready", "PowerEvent", "Alert Present", "#7",
    "InPosLimit", "InNegLimit", "MotionBlocked", "WasHomed",
    "Homing", "GoingDisabled", "StatusEvent", "Enabled",
    "MoveCanceled", "MoveDone", "OutOfRange", "BFromEnd",
    "AbovePosn", "AtTargetVel", "InA", "InB",
    "InvInA", "InvInB", "#26", "#27",
    "AFromStart", "MoveCmdNeg", "Disabled", "TimerExpired",
    "InMotionPos", "InMotionNeg", "InDisableStop", "InCtrlStop",
    "FanOn", "VectorSearch", "MoveCmdComplete", "InHardStop",
    "ShutdownState", "ShutdownState1", "HwFailure", "TriggerArmed",
    "StepsActive", "IndexMtr", "SoftwareInputs"
};
// Shutdown states (note integer offset for enum)
const char *CPSCshutdowns[] = {
    "OK", "Shutdown Imminent", "Shutdown Ramping", "Shutdown"
};
// In motion states (note integer offset for enum)
const char *CPSCinMotions[] = {
    "Stopped", "+", "-", "+/-"
};

// Alert/Warning Register field names
const char *CPSCalertsBitStrs[] = {
    "FwSelfTest", "FwNetBufferOverrun", "FwError0", "FwError1",
    "FwStackOverrun", "FwWatchdogRestarted", "FwInvalidConfiguration",
    "#7", "#8", "#9", "#10",
    "HwPowerProblem", "HwClockProblem", "HwEEPROMdead", "HwFlashCorrupt",
    "HwFlashChanged", "HwRAM", "HwADC", "HwADCsat",
    "#19", "#20", "#21", "#22", "#23",
    "NetVoltageLow", "NetWatchdog",
    "#26", "#27",
    "EStopped", "ConfigOutOfDate", "RunTimeErr",
    "#31", "#32", "#33", "#34",
    "MoveGenRange", "JrkLimRequestBad", "MoveBufUnderrun",
    "JrkLimVelRequestBad", "MoveSpecAltered", "PhaseSensorFailed",
    "LimSwitchActivated", "SoftLimitExceeded",
    "#43", "#44", "#45", "#46", "#47", "#48", "#49", "#50", "#51",
    "AClost",
    "ACphaseLost", "#54", "LowTemp", "#56", "#57",
    "MtrVectorBad",
    "#59",
    "MtrEncGlitch", "MtrEncOverspeed",
    "#62", "#63",
    "MtrPhaseOverload",
    "#65", "MtrBadSetupParams", "HardStopBrokeFree", "#68",
    "TrackingShutdown", "RMSOverload", "RMSOverloadShutdown",
    "#72", "#73",
    "BusVoltSat", "TrqSat", "NoCommSweepFailed", "NoCommSweepReversed",
    "NoCommFailed", "IndexCountZeroWarn", "TempAmbientHigh",
    "StatorHot",
    "BusOverCurrent", "BusOverVoltage", "BusVoltageLow", "BusRMSOverload",
    "#86",
    "MtrEncIndexMissing",
    "BusVoltageUnderOperatingV",
    "#89", "#90", "#91", "#92",
    "MtrEncIndexMisplaced", "StepsDuringPosnRecovery", "#95"
};

/// \endcond                                                                  *
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//                   Value Converters for ClearPath-EC Motors
//
//          Group these together for Doxygen with INTERNAL_DOC tag
//*****************************************************************************
//*****************************************************************************

/// \cond INTERNAL_DOC

//*****************************************************************************
//  NAME                                                                      *
//      convertADCmax
//
//  DESCRIPTION:
/**
    Conversion to and from ADC max parameter

    \param xxx description
    \return description

    Detailed description.
**/
//  SYNOPSIS:
static double MN_DECL convertADCmax(
    nodebool valIsBits,                 // TRUE=>drive bits, FALSE=> app bits
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // To/From value
    byNodeDB *pNodeDB) {            // Parameter information
    double iMax;
    cnErrCode theErr;
    theErr = cpmGetParameter(theMultiAddr, CPM_P_DRV_I_MAX, &iMax);
    if (theErr == MN_OK) {
        return (iMax / convVal);
    }
    return (0);
}
//                                                                            *
//*****************************************************************************


/******************************************************************************
 *  !NAME!
 *      convertAmperes
 *
 *  DESCRIPTION: */
/**
 *      Convert a 1.15 type number into a fraction of I_MAX.
 *
 *      \return double
 **/
//  SYNOPSIS:
static double MN_DECL convertAmperes(
    nodebool valIsBits,                 // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue iMax;                    // Full scale
    double newInt;                      // Temp variables
    cnErrCode theErr;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_I_MAX, NULL, &iMax);
    // The basis value is OK?
    if (theErr != MN_OK || iMax.value <= 0) {
        return (0);
    }

    if (valIsBits)   {
        // Convert to Amperes from the fraction of full scale
        return (convVal * iMax.value);
    }
    else {
        newInt = (convVal / iMax.value);
        if (newInt > Q15_MAX) {
            newInt = Q15_MAX;
        }
        return (newInt);
    }
}
/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertAmpsRMS
 *
 *  DESCRIPTION:
 *      Convert RMS level from base units. This RMS level is returned
 *      referenced to ADC max.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertAmpsRMS(
    nodebool valIsBits,                 // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue adcMax;          // Basis information
    double finalVal;
    cnErrCode theErr;
//                             %     Trq->A                 Q Factor
//  const double CONST_FACT = 100 / (2^28) ^ 0.5

    // Maximum current
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ADC_MAX,
                                 NULL, &adcMax);
    if (theErr != MN_OK) {
        return (0);
    }

    if (valIsBits) {
        // Prevent errors
        if (convVal < 0) {
            convVal = 0;
        }
        finalVal = (sqrt(convVal) * adcMax.value);
    }
    else {
        finalVal = convVal / adcMax.value;
        finalVal *= finalVal;
    }
    return (finalVal);
}

/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
*   !NAME!
*       calcDHeatFact
*
*   DESCRIPTION:
*       Calculate the appropriate d-current for heating based on the user
*       defined "heat factor" parameter
*
*   \return
*       double
*
*   SYNOPSIS:                                                               */
static double MN_DECL calcDHeatFact(
    nodebool valIsBits,                 // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue iMax;            // Basis information
    paramValue rWinding;
    double finalVal;
    cnErrCode theErr;

    // Maximum current
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_I_MAX, NULL, &iMax);
    if (theErr != MN_OK) {
        return (0);
    }

    // Winding resistance
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_MTR_OHMS,
                                 NULL, &rWinding);
    if (theErr != MN_OK) {
        return (0);
    }

    if (valIsBits) {
        finalVal = convVal * iMax.value;
        finalVal = finalVal * finalVal;
        finalVal = (finalVal * (rWinding.value * .75));
    }
    else {
        // Prevent errors
        if (convVal < 0) {
            convVal = 0;
        }
        finalVal = sqrt(convVal / (rWinding.value * .75));
        finalVal = finalVal / iMax.value;
    }

    return (finalVal);
}

/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertMeasAmperes
 *
 *  DESCRIPTION:
 *      Convert a 1.15 type number into a fraction of ADC_MAX.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertMeasAmperes(
    nodebool valIsBits,                 // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue adcMax;                  // Full scale
    double newInt;                      // Temp variables
    cnErrCode theErr;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ADC_MAX,
                                 NULL, &adcMax);
    // The basis value is OK?
    if (theErr != MN_OK || adcMax.value <= 0) {
        return (0);
    }

    if (valIsBits)   {
        // Convert to Amperes from the fraction of full scale
        return (convVal * adcMax.value);
    }
    else {
        newInt = (convVal / adcMax.value);
        if (newInt > Q15_MAX) {
            newInt = Q15_MAX;
        }
        return (newInt);
    }
}
/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertMeasVolts
 *
 *  DESCRIPTION:
 *      Convert a 1.15 type number into a fraction of full scale range.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertMeasVolts(
    nodebool valIsBits,                 // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue fsBusV;                  // Full scale
    cnErrCode theErr;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_FACT_FS_BUSV,
                                 NULL, &fsBusV);
    // The basis value is OK?
    if (theErr != MN_OK || fsBusV.value <= 0) {
        return (0);
    }

    if (valIsBits) {
        // Convert to Volts from the fraction of full scale
        return (convVal * fsBusV.value);
    }
    else {
        return (convVal / fsBusV.value);
    }
}
/*                                                               !end!      */
/****************************************************************************/


//****************************************************************************
//  NAME
//      convertMonGain
//
//  DESCRIPTION:
//      Convert the monitor port gain setting to/from a full-scale range
//      value.
//
//  \return
//      double
//
//  SYNOPSIS:
static double MN_DECL convertMonGain(
    nodebool valIsBits,                 // TRUE=>scaled, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    iscMonVars theVar,                  // The variable to adjust
    double convVal) {
    paramValue sampleTime, iMax, adcMax;
    cnErrCode theErr;
    double baseVal, fsVal;
    paramValue cmdRes, encRes;
    double resScale = 1;
#define FG_RATE_MAX 100000

    netaddr cNum = NET_NUM(theMultiAddr);
    nodeaddr addr = NODE_ADDR(theMultiAddr);
    iscState *pState;
    if (!(pState = (iscState *)SysInventory[cNum].NodeInfo[addr].pNodeSpecific)) {
        return (0);
    }
    monState &monNow = pState->MonState;
    monNow.TestPoint = theVar;
    // Assume failure until we get to the end
    monNow.Set = false;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_I_MAX, NULL, &iMax);
    if (theErr != MN_OK) {
        return (0);
    }
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ADC_MAX,
                                 NULL, &adcMax);
    if (theErr != MN_OK) {
        return (0);
    }
    if (iMax.value <= 0 || adcMax.value <= 0) {
        return (0);
    }

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_CMD_CNTS_PER_REV,
                                 NULL, &cmdRes);
    if (theErr != MN_OK) {
        return (0);
    }

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ENC_DENS,
                                 NULL, &encRes);
    if (theErr != MN_OK) {
        return (0);
    }
    // Scale the display by the encoder to cmd ratio
    if (cmdRes.value > 0 && encRes.value > 0) {
        resScale = encRes.value / cmdRes.value;
    }

    // Get the sample period
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTime);
    if (fabs(sampleTime.value) < 0.001)
        // Protect from divide-by-zero
    {
        return (0);
    }

    if (valIsBits)  {
        // Convert from fixed point
        convVal /= ISC_MON_SCALE;
    }
    // Protect from \ 0
    if (fabs(convVal) < 0.0001)  {
        // Show as "off"
        return (0);
    }

    // return the full scale value
    switch (theVar & ~(MON_OPTION_MASKS)) {
        case MON_VEL_MEAS:
        case MON_VEL_CMD:
        case MON_VEL_TRK:
        case MON_VEL_TRK_SERVO:
        case MON_VEL_RAS:
        case MON_VEL_ERR_GATED:
            // Return the monitor value in Hertz
            baseVal = ((1e3 * ISC_MON_MAX_VEL)
                       / (sampleTime.value * convVal * resScale));
            break;

        case MON_VEL_STEP:
            baseVal = (4e3 * ISC_MON_MAX_VEL)
                      / (sampleTime.value * convVal * resScale);
            break;

        case MON_JRK_CMD:
            baseVal = (1e12 * ISC_MON_MAX_VEL)
                      / (pow(sampleTime.value, 3) * convVal * resScale);
            break;

        case MON_ACC_CMD:
            baseVal = (1e6 * ISC_MON_MAX_VEL)
                      / (sampleTime.value * sampleTime.value * convVal
                         * resScale);
            break;

        case MON_POSN_TRK:
        case MON_POSN_DIR_TRK:
        case MON_TRK_LD:
        case MON_POSN_DIR_TRK_MTR:
        case MON_POSN_TRK_MTR:
        case MON_COUPLING:
            baseVal = (ISC_MON_MAX_POS / (convVal * resScale));
            break;

        case MON_SINE_R:        // (2.14 format)
        case MON_COS_R:
            baseVal = (200 / convVal);
            break;

        case MON_TRQ_MEAS:      // (1.15 format) scaled to trq amps
        case MON_TRQ_MEAS_PEAK:
        case MON_TRQ_D_MEAS:
        case MON_TRQ_TRK:
        case MON_TRQ_TRK_PEAK:
            baseVal = (100 * adcMax.value / iMax.value) / convVal;
            break;

        case MON_POSN_MEAS:
        case MON_POSN_CMD:                // Position Cmd
            if (!valIsBits) {
                double maxRange;
                // Pin at maximum displayable value
                maxRange = ISC_MON_MAX_POS_MEAS / resScale * 2;
                // Manage range limitation
                if (convVal > maxRange) {
                    convVal = maxRange;
                }
            }
            baseVal = (ISC_MON_MAX_POS_MEAS / (convVal * resScale));
            break;

        case (monTestPoints)77: // FG rate
            baseVal = FG_RATE_MAX / convVal;
            break;

        case MON_INTEGRATOR:
            baseVal = (ISC_MON_MAX_INTG / convVal);
            break;

        case MON_SGN_CMD_VEL:
        case MON_SGN_CMD_STEP:
            baseVal = (100. / convVal);
            break;

        case MON_BUS_VOLTS:
            theErr = netGetParameterDbl(theMultiAddr,
                                        mnParams(CPM_P_DRV_FACT_FS_BUSV),
                                        &fsVal);
            if (theErr != MN_OK) {
                return theErr;
            }
            baseVal = fsVal / convVal;
            break;


        case MON_TRQ_CMD:       // (1.15 format)
        case MON_CALIBRATE:
        default:
            baseVal = (100 / convVal);
            break;
    }

    if (!valIsBits)  {
        baseVal *= 0x10000;             // Scale to (16.16)
        // Put at maximum number
        if (baseVal > 0x7fffffff) {
            baseVal = 0x7fffffff;
        }
        if (baseVal < 1) {
            baseVal = 1;
        }
        // Get rescaled value
        monNow.FullScale = convertMonGain(true, theMultiAddr, theVar, baseVal);
        monNow.Set = true;
    }
    else {
        monNow.FullScale = baseVal;
        monNow.Set = true;
    }
    return (baseVal);
}
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertRMSlevel
 *
 *  DESCRIPTION:
 *      Convert RMS level from base units. This RMS level is a
 *      0 TO 100 value where 100 corresponds to the RMS shutdown point.  This
 *      converter does not need to convert to base units as it is a display
 *      value only.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertRMSlevel(
    nodebool scaled,                    // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue rmsLimit, adcMax;            // Basis information
    double finalVal;
    cnErrCode theErr;
//                             %     Trq->A                 Q Factor
//  const double CONST_FACT = 100 / (2^28) ^ 0.5
//  const double CONST_FACT = 0.006103515625;

    // Maximum current
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ADC_MAX,
                                 NULL, &adcMax);
    if (theErr != MN_OK || adcMax.value == 0) {
        return (0);
    }

    // RMS maximum amperes
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_RMS_LIM,
                                 NULL, &rmsLimit);
    if (theErr != MN_OK || rmsLimit.value == 0) {
        return (0);
    }

    if (convVal < 0 || convVal > 0x7FFFFFFF) {
        return (0);
    }

    // Convert to integer with rounding
    //                                            ADCmax
    finalVal = (nodelong)(((100 * sqrt(convVal / (1 << 28)) * adcMax.value)
                           / rmsLimit.value) + 0.5);

    // Insure the numbers don't exceed the range
    if (finalVal > 100) {
        finalVal = 100;
    }
    if (finalVal < 0) {
        finalVal = 0;
    }

    return (finalVal);
}
//                                                                          */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertRMSlevelSlow
 *
 *  DESCRIPTION:
 *      Convert RMS level from base units. This RMS level is a
 *      0 TO 100 value where 100 corresponds to the RMS shutdown point.  This
 *      converter does not need to convert to base units as it is a display
 *      value only.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertRMSlevelSlow(
    nodebool scaled,                    // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue rmsLimit, adcMax;            // Basis information
    double finalVal;
    cnErrCode theErr;
//                             %     Trq->A                 Q Factor
//  const double CONST_FACT = 100 / (2^28) ^ 0.5
//  const double CONST_FACT = 0.006103515625;

    // Maximum current
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ADC_MAX,
                                 NULL, &adcMax);
    if (theErr != MN_OK || adcMax.value == 0) {
        return (0);
    }

    // RMS maximum amperes
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_RMS_SLOW_LIM,
                                 NULL, &rmsLimit);
    if (theErr != MN_OK || rmsLimit.value == 0) {
        return (0);
    }

    if (convVal < 0 || convVal > 0x7FFFFFFF) {
        return (0);
    }

    // Convert to integer with rounding
    //                                            ADCmax
    finalVal = (nodelong)(((100 * sqrt(convVal / (1 << 28)) * adcMax.value)
                           / rmsLimit.value) + 0.5);

    // Insure the numbers don't exceed the range
    if (finalVal > 100) {
        finalVal = 100;
    }
    if (finalVal < 0) {
        finalVal = 0;
    }

    return (finalVal);
}
//                                                                          */
/****************************************************************************/


/*****************************************************************************
*   !NAME!
*       checkPosnLimit
*
*   DESCRIPTION:
*       This converter is used to limit tracking and in-position limits
*       for new and older firmwares.
*
*   \return
*       double
*
*   SYNOPSIS:                                                               */
static double MN_DECL checkPosnLimit(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information
    double fwVers;
    cnErrCode theErr;

    theErr = netGetParameterDbl(theMultiAddr, mnParams(CPM_P_FW_VERS),
                                &fwVers);
    if (theErr == MN_OK) {
        // Get the maximum current
        if (!valIsBits) {
            if (FW_MILESTONE_DUAL_RMS > fwVers && convVal > INT16_MAX) {
                convVal = INT16_MAX;
            }
        }
    }
    return (convVal);
}
/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertRMSlimit
 *
 *  DESCRIPTION:
 *      Unit conversion to convert the RMS limit settings to and from the
 *      unscaled values.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertRMSlimit(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information
    paramValue adcMax;
    cnErrCode theErr;
    int rVal;

    const double CONST_FACT = 1 << 12; //(4.12 basis value)
    const double TRQ_MAX_PCT = 0.9999;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ADC_MAX,
                                 NULL, &adcMax);
    if (theErr == MN_OK) {
        // Make sure the adcMax is reasonable
        if (adcMax.value <= 0) {
            return (0);
        }
        // Make sure the value in is reasonable
        if (convVal <= 0) {
            return (0);
        }

        // Get the maximum current
        if (valIsBits)
            // Convert to RMS from unscaled
        {
            return (sqrt(convVal * adcMax.value * adcMax.value / CONST_FACT));
        }
        else {
            // Convert RMS to unscaled value
            if (convVal > adcMax.value)
                rVal = ((int)(CONST_FACT *
                              ((TRQ_MAX_PCT) * (TRQ_MAX_PCT))));
            else
                rVal = ((int)(CONST_FACT *
                              ((convVal / adcMax.value)
                               * (convVal / adcMax.value)) + 0.5));
            return (rVal);
        }
    }
    return (0);
}

/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertRMSlimit32
 *
 *  DESCRIPTION:
 *      Unit conversion to convert the RMS limit settings to and from the
 *      unscaled values.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertRMSlimit32(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information
    paramValue adcMax;
    cnErrCode theErr;
    int rVal;

    const double CONST_FACT = 1 << 28; //(4.28 basis value)
    const double TRQ_MAX_PCT = 0.9999;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ADC_MAX,
                                 NULL, &adcMax);
    if (theErr == MN_OK) {
        // Make sure the adcMax is reasonable
        if (adcMax.value <= 0) {
            return (0);
        }
        // Make sure the value in is reasonable
        if (convVal <= 0) {
            return (0);
        }

        // Get the maximum current
        if (valIsBits)
            // Convert to RMS from unscaled
        {
            return (sqrt(convVal * adcMax.value * adcMax.value / CONST_FACT));
        }
        else {
            // Convert RMS to unscaled value
            if (convVal > adcMax.value)
                rVal = ((int)(CONST_FACT *
                              ((TRQ_MAX_PCT) * (TRQ_MAX_PCT))));
            else
                rVal = ((int)(CONST_FACT *
                              ((convVal / adcMax.value)
                               * (convVal / adcMax.value)) + 0.5));
            return (rVal);
        }
    }
    return (0);
}

/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertRMSFactor
 *
 *  DESCRIPTION:
 *      Amount to scale RMS values by if the motor velocity is below the RMS
 *  stopped speed
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertRMSFactor(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information

    // Make sure the value in is reasonable
    if (convVal <= 0) {
        return (0);
    }

    // Get the maximum current
    if (valIsBits) {
        // Convert to scale factor from drive value
        return (sqrt(convVal));
    }
    else {
        // Convert scale factor to drive value value
        return (convVal * convVal);
    }
    return (0);
}

/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertRMStc
 *
 *  DESCRIPTION:
 *      Convert RMS time constants to and from seconds and base units.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertRMStc(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information
    paramValue sampleTimeV;
    cnErrCode theErr;
    double b;
    double retVal;

    // Winding Amperes to torque Amperes factor
    const int SCALE_FACT_Q = 23;              // Parameter window scale

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTimeV);
    if (theErr != MN_OK) {
        return (0);
    }
    sampleTimeV.value *=  1e-6;     // Convert to seconds

    // Make sure the sampleTime is reasonable
    if (sampleTimeV.value == 0) {
        return (0);
    }

    // Insure problems return zero
    retVal = 0;

    // Convert TC to and from base units
    if (valIsBits)   {
        // Convert base to seconds
        b = (1. - (convVal / (1 << SCALE_FACT_Q)));
        if (b > 0 && b < 1)  {
            b = log(b) / sampleTimeV.value;
            if (b != 0) {
                retVal = log(8. / 9.) / b;
            }
        }
    }
    else {
        // Convert seconds to base units

        // Enforce minimal values
        if (convVal < 0.01) {
            convVal = 0.01;
        }
        b = 1 - pow(8. / 9., (sampleTimeV.value / convVal));
        // Convert to integer / rounding
        retVal = (nodelong)((b * (1 << SCALE_FACT_Q)) + 0.5);
        // Prevent the value from turning "negative"
        if (retVal > 32767) {
            retVal = 32767;
        }
        // Prevent the value from going to zero
        if (retVal < 1) {
            retVal = 1;
        }
    }
    return (retVal);
}
//                                                                          */
/****************************************************************************/

/*****************************************************************************
 *  !NAME!
 *      convertRMSSlowtc
 *
 *  DESCRIPTION:
 *      Convert RMS time constants to and from minutes and base units.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertRMSSlowtc(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information
    paramValue sampleTimeV;
    cnErrCode theErr;
    double b;
    double retVal;

    // Winding Amperes to torque Amperes factor
    const int SCALE_FACT_Q = 23;              // Parameter window scale
    const int SLOW_RMS_Q = 8;              // Parameter window scale
    const int SECS_PER_MINUTE = 60;        // Scale to minutes

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTimeV);
    if (theErr != MN_OK) {
        return (0);
    }
    sampleTimeV.value *=  1e-6;     // Convert to seconds

    // Make sure the sampleTime is reasonable
    if (sampleTimeV.value == 0) {
        return (0);
    }

    // Insure problems return zero
    retVal = 0;

    // Convert TC to and from base units
    if (valIsBits)   {
        // Convert base to minutes
        b = (1. - (convVal / (1 << SCALE_FACT_Q)));
        if (b > 0 && b < 1)  {
            b = log(b) / sampleTimeV.value;
            if (b != 0) {
                retVal = log(8. / 9.) / b;
            }
        }
        retVal = retVal * (1 << SLOW_RMS_Q) / SECS_PER_MINUTE;
    }
    else {
        // Convert seconds to base units

        // Enforce minimal values
        convVal = convVal * SECS_PER_MINUTE / (1 << SLOW_RMS_Q);
        if (convVal < 0.01) {
            convVal = 0.01;
        }
        b = 1 - pow(8. / 9., (sampleTimeV.value / convVal));
        // Convert to integer / rounding
        retVal = (nodelong)((b * (1 << SCALE_FACT_Q)) + 0.5);
        // Prevent the value from turning "negative"
        if (retVal > 32767) {
            retVal = 32767;
        }
        // Prevent the value from going to zero
        if (retVal < 1) {
            retVal = 1;
        }
    }
    return (retVal);
}
//                                                                          */
/****************************************************************************/


//****************************************************************************
//  NAME
//      convertAngle
//
//  DESCRIPTION:
//      Convert to and from the commutation angle in encoder ticks to an
//      angle in electrical degrees.
//
//  \return
//      double
//
//  SYNOPSIS:
static double MN_DECL convertAngle(
    nodebool valIsBits,                 // TRUE=>drive bits, FALSE=>mech degrees
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {
    paramValue poles, encDens;
    cpmHwConfigReg hwConfig;
    optionReg opts;

    double ticksPerDegree, degreesPerTurn, rVal;
    cnErrCode theErr;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_MTR_POLES,
                                 NULL, &poles);
    if (theErr != MN_OK) {
        return (0);
    }

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ENC_DENS,
                                 NULL, &encDens);
    if (theErr != MN_OK) {
        return (0);
    }

    theErr = cpmGetHwConfigReg(theMultiAddr, &hwConfig);
    if (theErr != MN_OK) {
        return (0);
    }

    double pVal;
    theErr = cpmGetParameter(theMultiAddr, CPM_P_OPTION_REG, &pVal);
    if (theErr != MN_OK) {
        return (0);
    }
    opts.bits = CAST_UINT32(pVal);

    degreesPerTurn = poles.value * 180.;
    if (poles.value != 0) {
        ticksPerDegree = encDens.value / degreesPerTurn;
    }
    else {
        return (0);
    }

    if (valIsBits)  {
        // Convert convVal to electrical degrees
        rVal = convVal / ticksPerDegree;
        if (opts.cpm.VectorLock) {
            rVal += 90;
        }
        // Keep result between 0 and degreesPerTurn
        if (rVal >= degreesPerTurn) {
            rVal -= degreesPerTurn;
        }
        if (rVal < 0) {
            rVal += degreesPerTurn;
        }
        // Return the value
        return (rVal);
    }
    else {
        // Convert convVal to drive bits
        rVal = convVal;
        if (opts.cpm.VectorLock) {
            rVal -= 90;
        }
        // Scale back to ticks
        rVal *= ticksPerDegree;
        // Bounds test the results between 0 and ticksPerTurn-1
        if (rVal < 0) {
            rVal += ticksPerDegree * degreesPerTurn;
        }
        if (rVal >= ticksPerDegree * degreesPerTurn) {
            rVal -= ticksPerDegree * degreesPerTurn;
        }
        // Return the final value
        return (((nodelong)(rVal + 0.5)));
    }
}
/****************************************************************************/


//****************************************************************************
//  NAME
//      convertSpdLim
//
//  DESCRIPTION:
//      This function converts to and from ticks/second and ticks/sample-time.
//
//  RETURNS:
//      double
//
//  SYNOPSIS:
double MN_DECL convertSpdLim(   // encoder ticks/sample-time <=> user ticks/sec
    nodebool valIsBits,                 // TRUE=>ticks/sample-time, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *nodeData) {
    paramValue sampleTime;
    cnErrCode theErr;
    paramValue userDens, encDens;

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTime);
    // Avoid divide-by-0 problems and other errors
    if (theErr != MN_OK || sampleTime.value == 0) {
        return (0);
    }

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_DRV_ENC_DENS,
                                 NULL, &encDens);
    if (theErr != MN_OK) {
        return (0);
    }

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_CMD_CNTS_PER_REV,
                                 NULL, &userDens);
    if (theErr != MN_OK) {
        return (0);
    }

    if (valIsBits)  {
        return (1e6 * convVal * userDens.value
                / (encDens.value * sampleTime.value));
    }
    else {
        // convVal in ms => us / # us/sample-time
        return (1e-6 * convVal * sampleTime.value * encDens.value
                / userDens.value);
    }
}
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertFilt99pctMilliseconds
 *
 *  DESCRIPTION:
 *      Convert IIR time constants from milliseconds to and from the 99%
 *      trip points.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertFilt99pctMilliseconds(
    nodebool scaled,                    // TRUE=>milliseconds, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue sampleTime;
    cnErrCode theErr;
    nodelong intVal;
    double x;

    const double TripPoint = 1 - 0.99;  // Trip point

    // Get the sample period
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
        NULL, &sampleTime);
    // Was it OK?
    if (theErr != MN_OK && sampleTime.value <= 0) {
        return (0);
    }

    if (scaled) {
        // Convert to milliseconds from base units
        if (convVal > 32767 || convVal <= 0) {
            return (0);
        }
        return (.001 * sampleTime.value * log(TripPoint)
            / log(convVal / 32768.));
    }
    else {
        // Convert to base units from milliseconds
        if (convVal <= 0) {
            return (0);
        }
        x = pow(TripPoint, .001 * sampleTime.value / convVal);
        intVal = (nodelong)((32768. * x) + 0.5);
        if (intVal > 32767) {
            intVal = 32767;
        }
        return (intVal);
    }
}
/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertVectorFilt99pctMilliseconds
 *
 *  DESCRIPTION:
 *      Convert IIR time constants from milliseconds to and from the 99%
 *      trip points for filters updated by the firmware at vector rate.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertVectorFilt99pctMilliseconds(
    nodebool scaled,                    // TRUE=>milliseconds, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information

    if (scaled) {
        // Convert to milliseconds from base units
        return convertFilt99pctMilliseconds(scaled, theMultiAddr, parameter, convVal, pNodeDB) / 4;
    }
    else {
        // Convert to base units from milliseconds
        return convertFilt99pctMilliseconds(scaled, theMultiAddr, parameter, convVal * 4, pNodeDB);
    }
}
/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertFilt1TCMilliseconds
 *
 *  DESCRIPTION:
 *      Convert IIR time constants from milliseconds to and from the 99%
 *      trip points.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertFilt1TCMilliseconds(
    nodebool scaled,                    // TRUE=>milliseconds, FALSE=>unscaled
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // From value
    byNodeDB *pNodeDB) {                // Parameter information
    paramValue sampleTime;
    cnErrCode theErr;
    nodelong intVal;
    double x;

    const double TripPoint = 0.367879;  // Trip point 1/e

    // Get the sample period
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTime);
    // Was it OK?
    if (theErr != MN_OK && sampleTime.value <= 0) {
        return (0);
    }

    if (scaled)  {
        // Convert to milliseconds from base units
        if (convVal > 32767 || convVal <= 0) {
            return (0);
        }
        return (.001 * sampleTime.value * log(TripPoint)
                / log(convVal / 32768.));
    }
    else {
        // Convert to base units from milliseconds
        if (convVal <= 0) {
            return (0);
        }
        x = pow(TripPoint, .001 * sampleTime.value / convVal);
        intVal = (nodelong)((32768.*x) + 0.5);
        if (intVal > 32767) {
            intVal = 32767;
        }
        return (intVal);
    }
}
/*                                                               !end!      */
/****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      convertIBRMStc
 *
 *  DESCRIPTION:
 *      Convert IB RMS time constants to and from seconds and base units.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertIBRMStc(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information
    paramValue sampleTimeV;
    cnErrCode theErr;
    double b;
    double retVal;

    // Winding Amperes to torque Amperes factor
    const int SCALE_FACT_Q = 23;              // Parameter window scale

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTimeV);
    if (theErr != MN_OK) {
        return (0);
    }
    sampleTimeV.value *=  1e-6 / 4;     // Convert to seconds

    // Make sure the sampleTime is reasonable
    if (sampleTimeV.value == 0) {
        return (0);
    }

    // Insure problems return zero
    retVal = 0;

    // Convert TC to and from base units
    if (valIsBits)   {
        // Convert base to seconds
        b = (1. - (convVal / (1 << SCALE_FACT_Q)));
        if (b > 0 && b < 1)  {
            b = log(b) / sampleTimeV.value;
            if (b != 0) {
                retVal = log(8. / 9.) / b;
            }
        }
        if (retVal < 0.01) {
            retVal = 0.01;
        }
    }
    else {
        // Convert seconds to base units

        // Enforce minimal values
        if (convVal < 0.01) {
            convVal = 0.01;
        }
        b = 1 - pow(8. / 9., (sampleTimeV.value / convVal));
        retVal = (nodelong)(b * (1 << SCALE_FACT_Q));
        // Prevent the value from turning "negative"
        if (retVal > 32767) {
            retVal = 32767;
        }
    }
    return (retVal);
}


/*****************************************************************************
 *  !NAME!
 *      convertIBRMSSlowtc
 *
 *  DESCRIPTION:
 *      Convert IB RMS time constants to and from seconds and base units.
 *
 *  \return
 *      double
 *
 *  SYNOPSIS:                                                               */
static double MN_DECL convertIBRMSSlowtc(
    nodebool valIsBits,             // TRUE=>RMS, FALSE=>unscaled
    multiaddr theMultiAddr,         // Node address
    appNodeParam parameter,         // Target parameter
    double convVal,                 // From value
    byNodeDB *pNodeDB) {            // Parameter information
    paramValue sampleTimeV;
    cnErrCode theErr;
    double b;
    double retVal;

    // Winding Amperes to torque Amperes factor
    const int SCALE_FACT_Q = 23;              // Parameter window scale
    const int SLOW_RMS_Q = 7;                 // Parameter window scale

    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTimeV);
    if (theErr != MN_OK) {
        return (0);
    }
    sampleTimeV.value *=  1e-6 / 4;     // Convert to seconds

    // Make sure the sampleTime is reasonable
    if (sampleTimeV.value == 0) {
        return (0);
    }

    // Insure problems return zero
    retVal = 0;

    // Convert TC to and from base units
    if (valIsBits)   {
        // Convert base to seconds
        b = (1. - (convVal / (1 << SCALE_FACT_Q)));
        if (b > 0 && b < 1)  {
            b = log(b) / sampleTimeV.value;
            if (b != 0) {
                retVal = log(8. / 9.) / b * (1 << SLOW_RMS_Q);
            }
        }
        if (retVal < 0.01) {
            retVal = 0.01;
        }
    }
    else {
        // Convert seconds to base units

        // Enforce minimal values
        convVal = convVal / (1 << SLOW_RMS_Q);
        if (convVal < 0.01) {
            convVal = 0.01;
        }
        b = 1 - pow(8. / 9., (sampleTimeV.value / convVal));
        retVal = (nodelong)(b * (1 << SCALE_FACT_Q));
        // Prevent the value from turning "negative"
        if (retVal > 32767) {
            retVal = 32767;
        }
    }
    return (retVal);
}


//****************************************************************************
//  NAME
//      convertJerk
//
//  DESCRIPTION:
//      Convert jerk specified in units of milliseconds to the internal
//      codes.
//
//      TO_KILL: When we stop support of v5.0-v5.1 Meridian Firmare
//
//  \return
//      double
//
//  SYNOPSIS:
static double MN_DECL convertJerk(
    nodebool valIsBits,                 // TRUE=>drive bits, FALSE=>msecs
    multiaddr theMultiAddr,             // Node address
    appNodeParam parameter,             // Target parameter
    double convVal,                     // To/From value
    byNodeDB *pNodeDB) {                // Parameter information
    optionReg optReg;
    packetbuf resp;
    cnErrCode theErr;
    bool isKdrive = false, isEnhanced;
    double KdrvMaxRas, fwVer, sampleTimeMicroSec;
//  // Code boundaries
//  static const int classicStart = 0;
//  static const int shallowCodeStart = 16;
//  static const int deepCodeStart = 48;
//  static const int enhancedRASlen = 30;
//  static const int rasFracLSD = 100;              // 0.01 is least significant digit

#define SHALLOW_CODE 10
#define DEEP_CODE    20

    // Get base information
    theErr = cpmGetParameter(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                             &sampleTimeMicroSec);
    if (theErr != MN_OK) {
        return (ISC_RAS_OFF);
    }
    theErr = cpmGetParameter(theMultiAddr, CPM_P_FW_VERS, &fwVer);
    if (theErr != MN_OK) {
        return (ISC_RAS_OFF);
    }
    isEnhanced = TRUE;

    if (!isEnhanced) {
        //K-drive check only matters in legacy, pre-IEX FW
        theErr = netGetParameter(theMultiAddr, CPM_P_OPTION_REG, &resp);
        if (theErr != MN_OK) {
            return (ISC_RAS_OFF);
        }
        optReg = *((optionReg *)(&resp.Byte.Buffer[0]));
        if (optReg.DrvFld.Product == 1) {
            isKdrive = true;
        }
    }

    if (valIsBits) {
        // conVal = drive bits convert to a UI code
        // rasSelector codeParser;
        // codeParser.Bits = (unsigned)convVal;
        if (isEnhanced) {
            return (convVal);           // Pass bits through
        }
        else {
            KdrvMaxRas = ISC_RAS_9MS;
            if (isKdrive && convVal > KdrvMaxRas) {
                convVal = KdrvMaxRas;
            }
            switch ((int)convVal)  {
                case ISC_RAS_OFF:
                    return (0);
                case ISC_RAS_3MS:
                    return (3);
                case ISC_RAS_5MS:
                    return (5);
                case ISC_RAS_9MS:
                    return (9);
                case ISC_RAS_15MS:
                    return (15);
                case ISC_RAS_24MS:
                    return (24);
                case ISC_RAS_44MS:
                    return (44);
            }
        }
        return (ISC_RAS_OFF);
    }
    else {
        // Enhanced SST sub-system
        if (isEnhanced) {
            return (convVal);               // Return untouched
        }
        // Classic SST codes
        else {
            // conVal = time in milliseconds, create node selection bit pattern.
            KdrvMaxRas = 9;
            if (isKdrive && convVal > KdrvMaxRas) {
                convVal = KdrvMaxRas;
            }
            if (convVal < 1.5) {
                return (ISC_RAS_OFF);
            }

            if (convVal >= 1.5 && convVal < 4) {
                return (ISC_RAS_3MS);
            }
            else if (convVal >= 4 && convVal < 7) {
                return (ISC_RAS_5MS);
            }
            else if (convVal >= 7 && convVal < 12) {
                return (ISC_RAS_9MS);
            }
            else if (convVal >= 12 && convVal < 19.5) {
                return (ISC_RAS_15MS);
            }
            else if (convVal >= 19.5 && convVal < 34) {
                return (ISC_RAS_24MS);
            }
            else {
                return (ISC_RAS_44MS);
            }
        }
    }
    return (ISC_RAS_OFF);
}
/******************************************************************************/
// END OF CONVERTERS
//  cond INTERNAL_DOC
/// \endcond
/******************************************************************************/


//*****************************************************************************
// NAME                                                                       *
//  cpscAPI.cpp static variables
//
/// \cond INTERNAL_DOC
// Note: Setting length to negative means valid up to labs(size)
//
// The parameter handler table
static const paramInfoLcl cpmInfoDB[] = {
//        1/x,   signed,      type,      unit,         size,             scale,        config key id,                 param group,           [converter],   [FW Milestone], [HW factory override]
//=======Core Node Parameters===============
/*  0*/ { FALSE, ST_UNSIGNED, PT_RO,     DEV_ID,       2,                1<<8,         PARAM_NULL,                    PG_NULL,               STR_PARAM_DEVID },
/*  1*/ { FALSE, ST_SIGNED,   PT_RO,     FW_VERS,      2,                1,            PARAM_FIRMWARE_VERSION,        PG_DRIVE_INFO,         STR_PARAM_FWVERS },
/*  2*/ { FALSE, ST_UNSIGNED, PT_FAC,    HW_VERS,      2,                1,            PARAM_HW_REV,                  PG_DRIVE_INFO,         STR_PARAM_HWVERS },
/*  3*/ { FALSE, ST_UNSIGNED, PT_RO,     NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_RESELLERID },
/*  4*/ { FALSE, ST_SIGNED,   PT_FAC,    NO_UNIT,      4,                1,            PARAM_SERIAL_NUMBER,           PG_DRIVE_INFO,         STR_PARAM_SERIAL_NUM },
/*  5*/ { FALSE, ST_SIGNED,   PT_FCFG,   BIT_FIELD,    4,                1,            PARAM_OPTION_REG,              PG_FACTORY_SETTINGS,   STR_PARAM_OPT_REG },
/*  6*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      2,                1,            PARAM_ROM_UPD_ACK,             PG_NON_DISPLAY,        STR_PARAM_ROMSUM_ACK },
/*  7*/ { FALSE, ST_UNSIGNED, PT_RO,     NO_UNIT,      2,                1,            PARAM_ROM_SUM,                 PG_NON_DISPLAY,        STR_PARAM_FWSUM },
/*  8*/ { FALSE, ST_UNSIGNED, PT_RO,     UNIT_HZ,      4,                1000,         PARAM_SAMP_RATE,               PG_NON_DISPLAY,        STR_PARAM_SAMP_PER },
/*  9*/ { FALSE, ST_SIGNED,   PT_RO_RT,  BIT_FIELD,    12,               1,            PARAM_ALERT_REG,               PG_STATUS_INFO,        STR_PARAM_ALERT_REG },
/* 10*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,                1,            PARAM_STOP_TYPE,               PG_SAFETY_INFO,        STR_PARAM_STOPTYPE },
/* 11*/ { FALSE, ST_UNSIGNED, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_WATCHDOG_TIME,           PG_SAFETY_INFO,        STR_PARAM_WDTC,            convertTimeMS },
/* 12*/ { FALSE, ST_UNSIGNED, PT_RO,     NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_NETSTAT },
/* 13*/ { FALSE, ST_SIGNED,   PT_ROC_RT, BIT_FIELD,    6,                1,            PARAM_STATUS1_ACCUM,           PG_STATUS_INFO,        STR_PARAM_STATUS },
/* 14*/ { FALSE, ST_SIGNED,   PT_ROC_RT, BIT_FIELD,    6,                1,            PARAM_STATUS1_ATTN_RISE,       PG_STATUS_INFO,        STR_PARAM_STATUS_ATTN_RISE },
/* 15*/ { FALSE, ST_SIGNED,   PT_FCFG,   BIT_FIELD,    4,                1,            PARAM_DRV_MODES,               PG_FACTORY_SETTINGS,   STR_UNKNOWN },
/* 16*/ { FALSE, ST_SIGNED,   PT_RO_RT,  BIT_FIELD,    6,                1,            PARAM_STATUS1_RT,              PG_STATUS_INFO,        STR_PARAM_STATUS_RT },
/* 18*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  TIME_MSEC,    2,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_TIMESTAMP16,     convertTimeMS },
/* 17*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  TIME_MSEC,    1,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_TIMESTAMP,       convertTimeMS },
/* 19*/ { FALSE, ST_UNSIGNED, PT_FAC,    STRING,       MN_PART_NUM_SIZE, 1,            PARAM_PART_NUM,                PG_FACTORY_SETTINGS,   STR_PARAM_PART_NUM },
/* 20*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      2,                1,            PARAM_EE_UPD_ACK,              PG_NON_DISPLAY,        STR_PARAM_EE_ACK },
/* 21*/ { FALSE, ST_UNSIGNED, PT_RO,     NO_UNIT,      2,                1,            PARAM_EE_VER,                  PG_NON_DISPLAY,        STR_PARAM_EE_VER },
/* 22*/ { FALSE, ST_SIGNED,   PT_ROC_RT, BIT_FIELD,    6,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_STATUS_FALL },
/* 23*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    4,                1,            PARAM_CFG_HW,                  PG_CONFIGURATION_INFO, STR_PARAM_CFG_HW },
/* 24*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    4,                1,            PARAM_CFG_APP,                 PG_CONFIGURATION_INFO, STR_PARAM_CFG_FEAT },
/* 25*/ { FALSE, ST_SIGNED,   PT_ROC_RT, BIT_FIELD,    12,               1,            PARAM_WARN_ACCUM_REG,          PG_NON_DISPLAY,        STR_WARN_ACC },
/* 26*/ { FALSE, ST_UNSIGNED, PT_ROC_RT, NO_UNIT,      12,               1,            PARAM_NULL,                    PG_NULL,               STR_WARN_RT },
/* 27*/ { FALSE, ST_UNSIGNED, PT_VOL,    NO_UNIT,      12,               1,            PARAM_NULL,                    PG_NULL,               STR_USER_WARN_MASK_REG },
/* 28*/ { FALSE, ST_UNSIGNED, PT_VOL,    NO_UNIT,      12,               1,            PARAM_NULL,                    PG_NULL,               STR_USER_ALERT_MASK_REG },
/* 29*/ { FALSE, ST_UNSIGNED, PT_FAC_RT, HOURS,        4,                10.*60*60,    PARAM_ON_TIME,                 PG_DRIVE_INFO,         STR_ON_TIME },   // .1 Sec -> Hours
/* 30*/ { FALSE, ST_SIGNED,   PT_RO_RTA, DX_TICK,      4,                1,            PARAM_POSN_CAP_INA_HI_SPD,     PG_MISCELLANEOUS_INFO, STR_PARAM_POSN_CAP_INA_HI_SPD },
/* 31*/ { FALSE, ST_UNSIGNED, PT_VOL,    NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_USER_RAM0 },
/* 32*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      MN_USER_NV_SIZE,  1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_USER_EE0 },
/* 33*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      MN_USER_NV_SIZE,  1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_USER_EE1 },
/* 34*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      MN_USER_NV_SIZE,  1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_USER_EE2 },
/* 35*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      MN_USER_NV_SIZE,  1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_USER_EE3 },
/* 36*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_APP_CHKSUM },
/* 37*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_APP_FRAG },
/* 38*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_APP_STRAY },
/* 39*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_APP_OVERRUN },
/* 40*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_DIAG_CHKSUM },
/* 41*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_DIAG_FRAG },
/* 42*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_DIAG_STRAY },
/* 43*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      2,                1,            PARAM_NULL,                    PG_NULL,               STR_NETERR_DIAG_OVERRUN },
/* 44*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,                1,            PARAM_NULL,                    PG_NULL,               STR_UNKNOWN },
/* 45*/ { FALSE, ST_SIGNED,   PT_VOLA,   BIT_FIELD,    ATTN_MASK_OCTETS, 1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_ATTN_MASK,       spscWatchAttnMask },
/* 46*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  TIME_MSEC,    2,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_GP_TIMER,        convertTimeMS },
/* 47*/ { FALSE, ST_SIGNED,   PT_NV_RWA, DX_TICK,      4,                1,            PARAM_AT_POSN_LOC,             PG_MISCELLANEOUS_INFO, STR_PARAM_POSN_TRIG_PT },
/* 48*/ { FALSE, ST_POS_ONLY, PT_NV_RWA, DX_TICK,      4,                1,            PARAM_A_AFTER_DIST,            PG_MISCELLANEOUS_INFO, STR_PARAM_A_START },
/* 49*/ { FALSE, ST_POS_ONLY, PT_NV_RWA, DX_TICK,      4,                1,            PARAM_B_BEFORE_DIST,           PG_MISCELLANEOUS_INFO, STR_PARAM_B_END },
/* 50*/ { FALSE, ST_SIGNED,   PT_VOL,    BIT_FIELD,    2,                1,            PARAM_XPS_USER_OUT_REG,        PG_IO_INFO,            STR_PARAM_USER_OUT_REG },
/* 51*/ { FALSE, ST_SIGNED,   PT_FAC_RT, BIT_FIELD,    12,               1,            PARAM_ALERT_ACCUM_REG,         PG_STATUS_INFO,        STR_UNKNOWN,               NULL,          FW_MILESTONE_SC_HAS_AC_REG },
/* 52*/ { FALSE, ST_SIGNED,   PT_RO_RT,  BIT_FIELD,    2,                1,            PARAM_OUTPUT_REG,              PG_IO_INFO,            STR_PARAM_OUT_REG },
/* 53*/ { FALSE, ST_POS_ONLY, PT_MTR,    VEL_TICKS_S,  2,                1<<2,         PARAM_SPEED_LIM,               PG_MOTOR_INFO,         STR_DRV_SPEED_LIM,         convertSpdLim, FW_MILESTONE_ALL_VERS, HW2_NON_OVERRIDE },
/* 54*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  BIT_FIELD,    4,                1,            PARAM_TP_IOP1,                 PG_NON_DISPLAY,        STR_DRV_TP_IOP1 },
/* 55*/ { FALSE, ST_UNSIGNED, PT_VOL,    NO_UNIT,      ATTN_MASK_OCTETS, 1,            PARAM_ATTN_MASK_DRVR,          PG_NON_DISPLAY,        STR_UNKNOWN,               spscWatchAttnMask },
/* 56*/ { FALSE, ST_SIGNED,   PT_VOL,    BIT_FIELD,    6,                1,            PARAM_GRP_SHUTDOWN_MASK,       PG_NON_DISPLAY,        STR_UNKNOWN },
//=======Motion Constraint group
/* 57*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  TIME_MSEC,    2,                1,            PARAM_CMD_TUNE_DELAY,          PG_NON_DISPLAY,        STR_RAS_DELAY,             convertTimeMS },
/* 58*/ { FALSE, ST_POS_ONLY, PT_NV_RW,  VEL_TICKS_S,  4,                1<<17,        PARAM_JOG_VEL_LIM,             PG_MOTION_CONSTRAINTS, STR_PARAM_VEL_LIMIT,       convertVel },
/* 59*/ { FALSE, ST_POS_ONLY, PT_NV_RW,  VEL_TICKS_S2, 4,                1<<17,        PARAM_JOG_ACC_LIM,             PG_MOTION_CONSTRAINTS, STR_PARAM_ACC_LIMIT,       convertAcc },
/* 60*/ { FALSE, ST_SIGNED,   PT_RO_RT,  VEL_TICKS_S,  4,                1<<18,        PARAM_RAS_MAX_VEL,             PG_NON_DISPLAY,        STR_UNKNOWN,               convertVel },
/* 61*/ { FALSE, ST_POS_ONLY, PT_NV_RW,  VEL_TICKS_S,  4,                1<<17,        PARAM_VEL_LIM,                 PG_MOTION_CONSTRAINTS, STR_PARAM_VEL_LIMIT,       convertVel },
/* 62*/ { FALSE, ST_POS_ONLY, PT_NV_RW,  VEL_TICKS_S2, 4,                1<<17,        PARAM_ACC_LIM,                 PG_MOTION_CONSTRAINTS, STR_PARAM_ACC_LIMIT,       convertAcc },
/* 63*/ { FALSE, ST_SIGNED,   PT_CFG_T,  BIT_FIELD,    4,                1,            PARAM_CMD_TUNE_REG,            PG_TUNING_INFO,        STR_PARAM_RAS_CON_REG,     convertJerk },
/* 64*/ { FALSE, ST_POS_ONLY, PT_NV_RWA, VEL_TICKS_S2, 4,                1<<17,        PARAM_DEC_LIM,                 PG_MOTION_CONSTRAINTS, STR_PARAM_DEC_LIMIT,       convertAcc },
/* 65*/ { FALSE, ST_POS_ONLY, PT_CFG,    VEL_TICKS_S2, 4,                1<<17,        PARAM_ESTOP_DECEL,             PG_MOTION_CONSTRAINTS, STR_PARAM_STOPACC_LIM,     convertAcc },
/* 66*/ { FALSE, ST_POS_ONLY, PT_NV_RWA, DX_TICK,      4,                1,            PARAM_HEAD_DX,                 PG_MOTION_CONSTRAINTS, STR_PARAM_HEAD_DX },
/* 67*/ { FALSE, ST_POS_ONLY, PT_NV_RWA, DX_TICK,      4,                1,            PARAM_TAIL_DX,                 PG_MOTION_CONSTRAINTS, STR_PARAM_TAIL_DX },
/* 68*/ { FALSE, ST_POS_ONLY, PT_NV_RWA, VEL_TICKS_S,  4,                1<<17,        PARAM_HT_VEL_LIM,              PG_MOTION_CONSTRAINTS, STR_PARAM_HEADTAIL_VEL,    convertVel },
/* 69*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  TIME_MSEC,    2,                1,            PARAM_MOVE_DWELL,              PG_NON_DISPLAY,        STR_PARAM_DWELL,           convertTimeMS },
/* 70*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      4,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_SOFT_LIM_POS },
/* 71*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      4,                1,            PARAM_NULL,                    PG_NULL,               STR_PARAM_SOFT_LIM_NEG },
/* 72*/ { FALSE, ST_POS_ONLY, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_STOP_QUAL_TC,            PG_SAFETY_INFO,        STR_UNKNOWN,               convertTimeMS },
/* 73*/ { FALSE, ST_SIGNED,   PT_RAM,    PERCENT_MAX,  2,                (1<<15)/100., PARAM_VOLTSQ_CMD,              PG_NON_DISPLAY,        STR_UNKNOWN },
/* 74*/ { FALSE, ST_POS_ONLY, PT_CFG,    VEL_TICKS_S,  4,                1<<18,        PARAM_STOP_QUAL_VEL,           PG_SAFETY_INFO,        STR_UNKNOWN,               convertVel },
/* 75*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  VEL_TICKS_S,  4,                1<<17,        PARAM_AT_MAX_VEL,              PG_TUNING_INFO,        STR_PARAM_VEL_LIMIT_MAX,   convertVel }, /*CPM_P_DRV_VEL_LIM_MAX*/
/* 76*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  VEL_TICKS_S2, 4,                1<<17,        PARAM_AT_MAX_ACC,              PG_NON_DISPLAY,        STR_PARAM_ACC_LIMIT_MAX,   convertAcc }, /*CPM_P_DRV_ACC_LIM_MAX*/
/* 77*/ { FALSE, ST_POS_ONLY, PT_NV_RW,  VEL_TICKS_S2, 4,                1<<17,        PARAM_NULL/*AT_MAX_DECEL*/,    PG_NULL,               STR_PARAM_STOPACC_LIM_MAX, convertAcc }, /*CPM_P_DRV_DEC_LIM_MAX*/
/* 78*/ { FALSE, ST_POS_ONLY, PT_FAC,    NO_UNIT,      4,                1<<15,        PARAM_RMS_STOPPED_FACTOR,      PG_FACTORY_SETTINGS,   STR_UNKNOWN,               convertRMSFactor,    FW_MILESTONE_INDEX_IB },
//=======Motion Status Group
/* 79*/ { FALSE, ST_POS_ONLY, PT_FMTR,   VEL_TICKS_S,  2,                1<<2,         PARAM_RMS_STOPPED_SPEED,       PG_FACTORY_SETTINGS,   STR_UNKNOWN,               convertSpdLim,       FW_MILESTONE_INDEX_IB },
/* 80*/ { FALSE, ST_SIGNED,   PT_RO_RT,  TORQUE_LIMIT, 2,                1<<15,        PARAM_TRQ_CMD,                 PG_STATUS_INFO,        STR_DRV_TRQ_CMD,           convertAmperes },
/* 81*/ { FALSE, ST_SIGNED,   PT_RO_RT,  CURRENT,      2,                1<<15,        PARAM_NULL,                    PG_NULL,               STR_DRV_TRQ_MEAS,          convertMeasAmperes },
/* 82*/ { FALSE, ST_POS_ONLY, PT_RO_RT,  PCNT_SHUTDOWN,4,                1,            PARAM_RMS_LVL,                 PG_STATUS_INFO,        STR_DRV_RMS_LEVEL,         convertRMSlevel },
/* 83*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      3,                1,            PARAM_POSN_MEAS,               PG_STATUS_INFO,        STR_PARAM_I_MEAS_POSN },
/* 84*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      3,                1,            PARAM_POSN_CMD,                PG_STATUS_INFO,        STR_PARAM_I_CMD_POSN },
/* 85*/ { FALSE, ST_SIGNED,   PT_RO_RT,  VEL_TICKS_S,  4,                1<<18,        PARAM_VEL_MEAS,                PG_STATUS_INFO,        STR_PARAM_MEAS_VEL,        convertVel },
/* 86*/ { FALSE, ST_SIGNED,   PT_RO_RT,  VEL_TICKS_S,  4,                1<<18,        PARAM_VEL_CMD,                 PG_STATUS_INFO,        STR_PARAM_CMD_VEL,         convertVel },
/* 87*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      3,                1,            PARAM_POSN_MTR,                PG_NON_DISPLAY,        STR_PARAM_I_MEAS_POSN_MTR },
/* 88*/ { FALSE, ST_POS_ONLY, PT_RO_RT,  PCNT_SHUTDOWN,4,                1,            PARAM_RMS_SLOW_LVL,            PG_STATUS_INFO,        STR_DRV_RMS_LEVEL,         convertRMSlevelSlow, FW_MILESTONE_DUAL_RMS },
/* 89*/ { FALSE, ST_POS_ONLY, PT_RO_RT,  VEL_TICKS_S2, 4,                1<<18,        PARAM_NULL,                    PG_NULL,               STR_PARAM_ACC_MAX,         convertAcc },
/* 90*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      4,                1,            PARAM_TRK_ERR,                 PG_NON_DISPLAY,        STR_PARAM_POSN_TRK },
/* 91*/ { FALSE, ST_SIGNED,   PT_CFG,    DX_TICK,      4,                1,            PARAM_IN_RANGE_WIN,            PG_CONFIGURATION_INFO, STR_PARAM_POS_TRK_RNG,     checkPosnLimit },
/* 92*/ { FALSE, ST_POS_ONLY, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_MV_DN_TC,                PG_CONFIGURATION_INFO, STR_DRV_MV_DN_TC,          convertTimeMS },
/* 93*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      4,                1,            PARAM_POSN_MTR_INDX,           PG_NON_DISPLAY,        STR_PARAM_INDEX_POSN_CAP },
/* 94*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      4,                1,            PARAM_NULL,                    PG_NULL,               STR_UNKNOWN }, /*Hi-Res cmd posn*/
/* 95*/ { FALSE, ST_SIGNED,   PT_RO_RT,  DX_TICK,      4,                1,            PARAM_NULL,                    PG_NULL,               STR_UNKNOWN }, /*Hi-Res meas posn*/
/* 96*/ { FALSE, ST_UNSIGNED, PT_RO_RT,  NO_UNIT,      4,                1,            PARAM_NULL,                    PG_NULL,               STR_UNKNOWN }, /*Steps accumulator*/
/* 97*/ { FALSE, ST_POS_ONLY, PT_CFG,    PERCENT_MAX,  2,                (1<<15)/100., PARAM_IN_RANGE_VEL,            PG_CONFIGURATION_INFO, STR_UNKNOWN },
/* 98*/ { FALSE, ST_SIGNED,   PT_NV_RWA, BIT_FIELD,    2,                1,            PARAM_MOVE_OPTIONS,            PG_MOTION_CONSTRAINTS, STR_UNKNOWN,             NULL,           FW_MILESTONE_SC_MOVE_OPTIONS },
/* 99*/ { FALSE, ST_UNSIGNED, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_IN1_TC,                  PG_IO_INFO,            STR_INPUT1_TC,             convertTimeMS },
/*100*/ { FALSE, ST_SIGNED,   PT_FAC,    SHIPOUT_VERS, 4,                1,            PARAM_SHIPOUT_VERSION,         PG_FACTORY_SETTINGS,   STR_UNKNOWN,               NULL,               FW_MILESTONE_2R0 },
/*101*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,                1,            PARAM_NULL,                    PG_NULL,               STR_UNKNOWN },
/*102*/ { FALSE, ST_UNSIGNED, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_IN1_IN2_TC,              PG_IO_INFO,            STR_INPUT2_TC,             convertTimeMS },
/*103*/ { FALSE, ST_POS_ONLY, PT_NONE,   DEGREES,      4,                1,            PARAM_VECTOR_DRIFT_LIM,        PG_FACTORY_SETTINGS,   STR_UNKNOWN },
//=======Sensorless Start Group
/*104*/ { FALSE, ST_POS_ONLY, PT_CFG,    DEGREES,      4,                (1<<17)/360., PARAM_COMM_ANGLE_LIM,          PG_COMMUTATION_INFO,   STR_DRV_COMM_CHK_ANGLE_LIM },
/*105*/ { FALSE, ST_UNSIGNED, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_HLESS_RAMPUP_TIME,       PG_COMMUTATION_INFO,   STR_DRV_HLESS_RAMPUP_TIME, convertTimeMS },
/*106*/ { FALSE, ST_UNSIGNED, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_HLESS_SWEEP_TIME,        PG_COMMUTATION_INFO,   STR_DRV_HLESS_SWEEP_TIME,  convertTimeMS },
/*107*/ { FALSE, ST_UNSIGNED, PT_CFG,    TIME_MSEC,    2,                1,            PARAM_HLESS_SETTLE_TIME,       PG_COMMUTATION_INFO,   STR_DRV_HLESS_SETTLE_TIME, convertTimeMS },
/*108*/ { FALSE, ST_UNSIGNED, PT_FMTR,   VOLT,         2,                1<<5,         PARAM_HLESS_VOLTS,             PG_COMMUTATION_INFO,   STR_DRV_HLESS_VOLTS },
/*109*/ { FALSE, ST_UNSIGNED, PT_FMTR,   TIME_SAMPLE,  2,                1,            PARAM_HLESS_RDG,               PG_COMMUTATION_INFO,   STR_DRV_HLESS_RDG_SAMPLES },
/*110*/ { FALSE, ST_UNSIGNED, PT_FMTR,   TIME_SAMPLE,  2,                1,            PARAM_HLESS_SETUP,             PG_COMMUTATION_INFO,   STR_DRV_HLESS_SETUP_SAMPLES },
/*111*/ { FALSE, ST_SIGNED,   PT_CFG,    TIME_MSEC,    2,                1,            PARAM_HLESS_VERIFY_TRQ_TIME,   PG_COMMUTATION_INFO,   STR_UNKNOWN,               convertTimeMS },
/*112*/ { FALSE, ST_UNSIGNED, PT_RAM_RT, NO_UNIT,      24,               1,            PARAM_NULL,                    PG_NULL,               STR_ENC_QUAL },
/*113*/ { FALSE, ST_UNSIGNED, PT_CFG,    DEGREES,      2,                (1<<15)/360., PARAM_HLESS_VERIFY_MAX_MOTION, PG_COMMUTATION_INFO,   STR_UNKNOWN },
/*114*/ { FALSE, ST_UNSIGNED, PT_CFG,    DEGREES,      2,                (1<<15)/360., PARAM_HLESS_VERIFY_MIN_MOTION, PG_COMMUTATION_INFO,   STR_UNKNOWN },
/*115*/ { FALSE, ST_SIGNED,   PT_CFG,    TIME_MSEC,    2,                1,            PARAM_HLESS_VERIFY_RAMP_TIME,  PG_COMMUTATION_INFO,   STR_UNKNOWN,               convertTimeMS },
//        1/x,   signed,      type,      unit,         size,             scale,        config key id,                 param group,           description,               [converter],   [FW Milestone], [HW factory override]
};
#define PARAM_BASE_COUNT (sizeof(cpmInfoDB)/sizeof(paramInfoLcl))
// Node class database
static byNodeClassDB cpmClassDB = {
    NULL                    // Parameter change function
};


static const paramInfoLcl cpmDrvInfoDB[] = {
//        1/x,   signed,      type,       unit,           size, scale,         config key id,              param group,           description,             [converter],                  [FW Milestone], [HW factory override]
//=======Factory Settings===================
/*256*/ { FALSE, ST_SIGNED,   PT_FCFG,    CURRENT,        2,    1<<14,         PARAM_ADC_MAX,              PG_FACTORY_SETTINGS,   STR_DRV_ADC_MAX,         convertADCmax },
/*257*/ { FALSE, ST_SIGNED,   PT_FCFG,    CURRENT,        2,    1<<9,          PARAM_I_MAX,                PG_DRIVE_INFO,         STR_DRV_DRV_I_MAX },
/*258*/ { FALSE, ST_POS_ONLY, PT_FCFG,    CURRENT,        2,    1,             PARAM_RMS_MAX,              PG_FACTORY_SETTINGS,   STR_DRV_RMS_MAX,         convertRMSlimit },
/*259*/ { FALSE, ST_UNSIGNED, PT_FAC,     NO_UNIT,        2,    1<<14,         PARAM_IR_CAL_FACTOR,        PG_FACTORY_SETTINGS,   STR_DRV_IR_CAL },
/*260*/ { FALSE, ST_UNSIGNED, PT_FAC,     NO_UNIT,        2,    1<<14,         PARAM_IS_CAL_FACTOR,        PG_FACTORY_SETTINGS,   STR_DRV_IS_CAL },
/*261*/ { FALSE, ST_UNSIGNED, PT_FAC,     NO_UNIT,        2,    1,             PARAM_KM_FACT,              PG_FACTORY_SETTINGS,   STR_P_DRV_PWR_LIM_EXP },
/*262*/ { FALSE, ST_POS_ONLY, PT_FAC,     OHM,            2,    1<<9,          PARAM_MTR_OHMS_MIN,         PG_FACTORY_SETTINGS,   STR_P_DRV_RES_MIN },
/*263*/ { FALSE, ST_POS_ONLY, PT_FAC,     TIME_MSEC,      2,    1,             PARAM_IB_TC,                PG_FACTORY_SETTINGS,   STR_P_IB_FILT_TC,        convertIBRMStc },
/*264*/ { FALSE, ST_SIGNED,   PT_FAC,     CURRENT_2,      2,    1<<15,         PARAM_IB_TRIP,              PG_FACTORY_SETTINGS,   STR_P_IB_TRIP,           convertAmpsRMS },
/*265*/ { FALSE, ST_POS_ONLY, PT_FAC,     CURRENT,        2,    1<<15,         PARAM_I_TRIP,               PG_FACTORY_SETTINGS,   STR_P_PHASE_TRIP,        convertMeasAmperes },
/*266*/ { FALSE, ST_POS_ONLY, PT_FAC,     VOLT,           2,    1<<5,          PARAM_BUS_V_MAX,            PG_FACTORY_SETTINGS,   STR_DRV_BUS_VOLTS },
/*267*/ { FALSE, ST_POS_ONLY, PT_CFG,     CURRENT_2,      2,    1<<15,         PARAM_NO_COMM_START_TRQ,    PG_COMMUTATION_INFO,   STR_DRV_HLESS_TRQ,       convertAmperes },
/*268*/ { FALSE, ST_POS_ONLY, PT_FAC,     VOLT,           2,    1<<5,          PARAM_OVER_V_TRIP,          PG_FACTORY_SETTINGS,   STR_OVER_VOLTS_TRIP },
/*269*/ { FALSE, ST_UNSIGNED, PT_FAC,     VOLT,           2,    1<<14,         PARAM_BUS_V_CAL,            PG_FACTORY_SETTINGS,   STR_DRV_VBUS_CAL },
//=======Safety Related  Parameters=========
/*270*/ { FALSE, ST_POS_ONLY, PT_MTR_R,   CURRENT_2,      4,    1,             PARAM_RMS_LIM,              PG_MOTOR_INFO,         STR_DRV_RMS_LIM,         convertRMSlimit32 },
/*271*/ { FALSE, ST_POS_ONLY, PT_MTR,     TIME_S,         2,    1,             PARAM_RMS_TC,               PG_MOTOR_INFO,         STR_DRV_RMS_TC,          convertRMStc },
/*272*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   DX_TICK,        4,    1,             PARAM_TRK_ERR_LIM,          PG_SAFETY_INFO,        STR_DRV_TRK_ERR_LIM,     checkPosnLimit },
/*273*/ { FALSE, ST_SIGNED,   PT_CFG,     PCNT_SHUTDOWN,  2,    (1<<15)/100.,  PARAM_RMS_INIT,             PG_MOTOR_INFO,         STR_DRV_RMS_INIT },
/*274*/ { FALSE, ST_POS_ONLY, PT_RO_RT,   CURRENT,        4,    1U<<31,        PARAM_IB_PEAK,              PG_STATUS_INFO,        STR_P_IB_RT,             convertAmpsRMS,      FW_MILESTONE_SC_HAS_IBPEAK },
/*275*/ { FALSE, ST_POS_ONLY, PT_MTR_R,   CURRENT_2,      4,    1,             PARAM_RMS_SLOW_LIM,         PG_MOTOR_INFO,         STR_DRV_RMS_LIM,         convertRMSlimit32,   FW_MILESTONE_DUAL_RMS },
/*276*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   VOLT,           2,    32768/3.3,     PARAM_NULL,                 PG_NULL,               STR_UNKNOWN },
/*277*/ { FALSE, ST_SIGNED,   PT_RO_RTA,  DX_TICK,        4,    1,             PARAM_POSN_CAP_INB,         PG_MISCELLANEOUS_INFO, STR_PARAM_POSN_CAP_INB },
//=======Status Related Parameters==========
/*278*/ { FALSE, ST_POS_ONLY, PT_RO_RT,   VOLT,           2,    1<<15,         PARAM_BUS_AT_ENBL,          PG_NON_DISPLAY,        STR_BUS_AT_ENBL,         convertMeasVolts },
/*279*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   VOLT,           2,    32768/6.6,     PARAM_TP5V,                 PG_STATUS_INFO,        STR_PARAM_TP_5V },
/*280*/ { FALSE, ST_POS_ONLY, PT_RO_RT,   VOLT,           2,    1<<15,         PARAM_BUS_V_MEAS,           PG_DRIVE_INFO,         STR_DRV_BUS_VOLTS,       convertMeasVolts },
/*281*/ { FALSE, ST_UNSIGNED, PT_RW_RT_T, NO_UNIT,        2,    1,             PARAM_NV_MODIFIED,          PG_NON_DISPLAY,        STR_DRV_DIRTY },
/*282*/ { FALSE, ST_UNSIGNED, PT_RO,      NO_UNIT,        2,    1,             PARAM_NULL,                 PG_NULL,               STR_DRV_VECTOR_RATE },
/*283*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   VOLT,           2,    32768/3.3,     PARAM_TP1_65V,              PG_STATUS_INFO,        STR_PARAM_TP_REF },
/*284*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   VOLT,           2,    32768/36.3,    PARAM_TP12V,                PG_STATUS_INFO,        STR_PARAM_TP_12V },
/*285*/ { FALSE, ST_SIGNED,   PT_RO_RT,   BIT_FIELD,      4,    1,             PARAM_TP_IOP,               PG_NON_DISPLAY,        STR_DRV_TP_IOP },
/*286*/ { FALSE, ST_SIGNED,   PT_RO_RT,   CURRENT,        2,    1<<15,         PARAM_NULL,                 PG_NULL,               STR_DRV_TP_IR,           convertMeasAmperes },
/*287*/ { FALSE, ST_SIGNED,   PT_RO_RT,   CURRENT,        2,    1<<15,         PARAM_NULL,                 PG_NULL,               STR_DRV_TP_IS,           convertMeasAmperes },
/*288*/ { FALSE, ST_UNSIGNED, PT_FAC,     TIME_MSEC,      2,    1,             PARAM_I_TP_FILT,            PG_FACTORY_SETTINGS,   STR_P_IR_IS_TP_FILT_TC,  convertFilt1TCMilliseconds },
/*289*/ { FALSE, ST_SIGNED,   PT_RO_RT,   CURRENT,        2,    1<<15,         PARAM_TP_IR,                PG_NON_DISPLAY,        STR_DRV_TP_IR_FILT,      convertMeasAmperes },
/*290*/ { FALSE, ST_SIGNED,   PT_RO_RT,   CURRENT,        2,    1<<15,         PARAM_TP_IS,                PG_NON_DISPLAY,        STR_DRV_TP_IS_FILT,      convertMeasAmperes },
/*291*/ { FALSE, ST_SIGNED,   PT_RO_RT,   DEG_C,          2,    1,             PARAM_DRV_TEMP,             PG_STATUS_INFO,        STR_P_PWBA_TEMP },
/*292*/ { FALSE, ST_SIGNED,   PT_VOL,     NO_UNIT,        2,    1,             PARAM_NULL,                 PG_NULL,               STR_P_TEMP_SIM },
/*293*/ { FALSE, ST_UNSIGNED, PT_RO,      NO_UNIT,        2,    1,             PARAM_NULL,                 PG_NULL,               STR_P_DSP_INFO },
/*294*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   CURRENT,        4,    1U<<31,        PARAM_IB,                   PG_STATUS_INFO,        STR_P_IB_RMS,            convertAmpsRMS },
/*295*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   TIME_USEC,      4,    1000,          PARAM_TSPD,                 PG_NON_DISPLAY,        STR_P_TSPD },
/*296*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_BAD_TSPD_CNT },
/*297*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_BAD_SLOT_CNT },
/*298*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_LOAD_SLOT0 },
/*299*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_LOAD_SLOT1 },
/*300*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_LOAD_SLOT2 },
/*301*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_LOAD_SLOT3 },
/*302*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_AVG_LOAD },
/*303*/ { FALSE, ST_SIGNED,   PT_NONE,    NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_UNKNOWN },
/*304*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        2,    1,             PARAM_NULL,                 PG_NULL,               STR_UNKNOWN },
/*305*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_UNKNOWN },
/*306*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_IR_OFFSET },
/*307*/ { FALSE, ST_UNSIGNED, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_P_IS_OFFSET },
/*308*/ { FALSE, ST_POS_ONLY, PT_FAC,     DEG_C,          2,    1,             PARAM_TEMP_LIM,             PG_FACTORY_SETTINGS,   STR_P_OVER_TEMP },
//=======Motor Related Params===============
/*309*/ { FALSE, ST_SIGNED,   PT_CFG_T,   NO_UNIT,        2,    1,             PARAM_KIP_ADJ,              PG_TUNING_INFO,        STR_DRV_KIP },
/*310*/ { FALSE, ST_POS_ONLY, PT_MTR,     CNTS_PER_REV,   4,    1,             PARAM_ENC_DENS,             PG_MOTOR_INFO,         STR_DRV_ENC_DENS,        NULL,                         FW_MILESTONE_ALL_VERS,           HW2_NON_OVERRIDE },
/*311*/ { FALSE, ST_POS_ONLY, PT_MTR,     NO_UNIT,        2,    1,             PARAM_POLES,                PG_MOTOR_INFO,         STR_DRV_POLES,           NULL,                         FW_MILESTONE_ALL_VERS,           HW2_NON_OVERRIDE },
/*312*/ { FALSE, ST_POS_ONLY, PT_MTR,     VPEAK_PER_KRPM, 4,    1<<7,          PARAM_MTR_KE,               PG_MOTOR_INFO,         STR_DRV_MTR_KE,          NULL,                         FW_MILESTONE_ALL_VERS,           HW2_NON_OVERRIDE },
/*313*/ { FALSE, ST_POS_ONLY, PT_MTR,     OHM,            2,    1<<9,          PARAM_MTR_OHMS,             PG_MOTOR_INFO,         STR_DRV_MTR_OHMS,        NULL,                         FW_MILESTONE_ALL_VERS,           HW2_NON_OVERRIDE },
/*314*/ { FALSE, ST_POS_ONLY, PT_MTR,     TIME_MSEC,      2,    1<<9,          PARAM_MTR_ELECT_TC,         PG_MOTOR_INFO,         STR_DRV_MTR_ELEC_TC,     NULL,                         FW_MILESTONE_ALL_VERS,           HW2_NON_OVERRIDE },
/*315*/ { FALSE, ST_SIGNED,   PT_FMTR,    DEGREES,        2,    32768/360.,    PARAM_RO,                   PG_MOTOR_INFO,         STR_DRV_RO },
/*316*/ { FALSE, ST_UNSIGNED, PT_NONE,    NO_UNIT,        0,    1,             PARAM_NULL,                 PG_NULL,               STR_NULL },
/*317*/ { FALSE, ST_POS_ONLY, PT_FAC_RT,  DEGREES,        4,    1,             PARAM_MECH_ANGLE,           PG_NON_DISPLAY,        STR_DRV_ANGLE,           convertAngle },
/*318*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        2,    1,             PARAM_KIP,                  PG_TUNING_INFO,        STR_DRV_KIP },
/*319*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        2,    1,             PARAM_KII,                  PG_TUNING_INFO,        STR_DRV_KII },
/*320*/ { FALSE, ST_POS_ONLY, PT_CFG,     CNTS_PER_REV,   4,    1,             PARAM_STEP_RES,             PG_MODE_INFO,          STR_DRV_CMD_DENS }, // request step/rev
/*321*/ { FALSE, ST_SIGNED,   PT_FMTR,     VEL_TICKS_S,    2,    1<<2,          PARAM_INDEX_DETECT_SPEED,   PG_FACTORY_SETTINGS,   STR_DRV_SPEED_LIM,       convertSpdLim,                FW_MILESTONE_INDEX_IB,           HW2_NON_OVERRIDE },
/*322*/ { FALSE, ST_POS_ONLY, PT_FMTR,    CNTS_PER_REV,   4,    1,             PARAM_STEP_RES_ACTIVE,      PG_DRIVE_INFO,         STR_UNKNOWN,             NULL,                         FW_MILESTONE_ALL_VERS,           HW2_NON_OVERRIDE },
/*323*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        2,    1<<9,          PARAM_KR,                   PG_TUNING_INFO,        STR_DRV_KR },
/*324*/ { FALSE, ST_SIGNED,   PT_CFG_T,   NO_UNIT,        2,    1,             PARAM_KII_ADJ,              PG_TUNING_INFO,        STR_DRV_KII },
//=======Tuning Related Params==============
/*325*/ { FALSE, ST_SIGNED,   PT_CFG_T,   BIT_FIELD,      4,    1,             PARAM_CFG_TUNE,             PG_TUNING_INFO,        STR_PARAM_CFG_TUNE },
/*326*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KV,                   PG_TUNING_INFO,        STR_DRV_KV },
/*327*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KP_OUT,               PG_TUNING_INFO,        STR_DRV_KP },
/*328*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KI_OUT,               PG_TUNING_INFO,        STR_DRV_KI },
/*329*/ { FALSE, ST_SIGNED,   PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KFV,                  PG_TUNING_INFO,        STR_DRV_KFV },
/*330*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KFA,                  PG_TUNING_INFO,        STR_DRV_KFA,             limit2To27 },
/*331*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KFJ,                  PG_TUNING_INFO,        STR_DRV_KFJ,             limit2To27 },
/*332*/ { FALSE, ST_SIGNED,   PT_RO_RT,   BIT_FIELD,      4,    1,             PARAM_NULL,                 PG_NULL,               STR_UNKNOWN },
/*333*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KNV,                  PG_TUNING_INFO,        STR_DRV_KNV },
/*334*/ { FALSE, ST_UNSIGNED, PT_CFG_T,   TIME_MSEC,      2,    1,             PARAM_AH_VOLT_FILT_TC,      PG_TUNING_INFO,        STR_DRV_AH_FILT_TC,      convertVectorFilt99pctMilliseconds },
/*335*/ { FALSE, ST_SIGNED,   PT_CFG_T,   TORQUE_LIMIT,   2,    1<<15,         PARAM_TRQ_BIAS,             PG_TUNING_INFO,        STR_DRV_TRQ_BIAS,        convertAmperes },
/*336*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   DX_TICK,        2,    1,             PARAM_FUZZY_APERTURE,       PG_TUNING_INFO,        STR_DRV_FUZZ_AP },
/*337*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   DX_TICK,        2,    1,             PARAM_ANTI_HUNT_HYSTERESIS, PG_TUNING_INFO,        STR_DRV_FUZZ_HYST },
/*338*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   TIME_MSEC,      2,    1,             PARAM_AH_HOLDOFF,           PG_TUNING_INFO,        STR_DRV_AH_HOLDOFF,      convertTimeMS },
/*339*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KP_ZERO,              PG_TUNING_INFO,        STR_DRV_KZERO },
/*340*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1,             PARAM_KI_ZERO,              PG_TUNING_INFO,        STR_DRV_IZERO },
/*341*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   DX_TICK,        2,    1,             PARAM_TGT_WIN,              PG_TUNING_INFO,        STR_DRV_TARGET_WIN },
/*342*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   DX_TICK,        2,    1,             PARAM_STAB_WIN,             PG_TUNING_INFO,        STR_DRV_STAB_WIN },
/*343*/ { FALSE, ST_UNSIGNED, PT_NV_RW,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_MTR_INERTIA }, /*CPM_P_MTR_INERTIA*/
/*344*/ { FALSE, ST_UNSIGNED, PT_NV_RW,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_MTR_STAT_FRICTION }, /*CPM_P_MTR_STATIC_FRICTION*/
/*345*/ { FALSE, ST_UNSIGNED, PT_NV_RW,   NO_UNIT,        4,    1,             PARAM_NULL,                 PG_NULL,               STR_MTR_VISC_FRICTION }, /*CPM_P_MTR_VISCOUS_FRICTION*/
/*346*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1<<16,         PARAM_DRV_KP_FACTOR,        PG_TUNING_INFO,        STR_UNKNOWN },
/*347*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1<<16,         PARAM_DRV_KPZ_FACTOR,       PG_TUNING_INFO,        STR_UNKNOWN },
/*348*/ { FALSE, ST_POS_ONLY, PT_CFG_T,   NO_UNIT,        4,    1<<16,         PARAM_DRV_KI_FACTOR,        PG_TUNING_INFO,        STR_UNKNOWN },
/*349*/ { FALSE, ST_UNSIGNED, PT_CFG_T,   NO_UNIT,        2,    1,             PARAM_DRV_FINE_TUNE,        PG_TUNING_INFO,        STR_UNKNOWN },
//=======Foldback Related Parameters========
/*350*/ { FALSE, ST_POS_ONLY, PT_CFG,     TORQUE_LIMIT,   2,    1<<15,         PARAM_TRQ_LIM,              PG_SAFETY_INFO,        STR_DRV_TRQ_LIM,         convertAmperes },
/*351*/ { FALSE, ST_POS_ONLY, PT_CFGA,    TORQUE_LIMIT,   2,    1<<15,         PARAM_TRQ_FLDBACK_POS,      PG_FOLDBACK_INFO,      STR_DRV_TRQ_FLDBACK_POS, convertAmperes },
/*352*/ { FALSE, ST_POS_ONLY, PT_CFGA,    TIME_MSEC,      2,    1,             PARAM_TRQ_FLDBACK_POS_TC,   PG_FOLDBACK_INFO,      STR_DRV_TRQ_FB_POS_TC,   convertFilt99pctMilliseconds },
/*353*/ { FALSE, ST_POS_ONLY, PT_CFGA,    TORQUE_LIMIT,   2,    1<<15,         PARAM_TRQ_FLDBACK_NEG,      PG_FOLDBACK_INFO,      STR_DRV_TRQ_FLDBACK_NEG, convertAmperes },
/*354*/ { FALSE, ST_POS_ONLY, PT_CFGA,    TIME_MSEC,      2,    1,             PARAM_TRQ_FLDBACK_NEG_TC,   PG_FOLDBACK_INFO,      STR_DRV_TRQ_FB_NEG_TC,   convertFilt99pctMilliseconds },
/*355*/ { FALSE, ST_UNSIGNED, PT_NONE,    NO_UNIT,        0,    1,             PARAM_NULL,                 PG_NULL,               STR_UNKNOWN },
/*356*/ { FALSE, ST_UNSIGNED, PT_NONE,    NO_UNIT,        0,    1,             PARAM_NULL,                 PG_NULL,               STR_UNKNOWN },
/*357*/ { FALSE, ST_POS_ONLY, PT_CFG,     TORQUE_LIMIT,   2,    1<<15,         PARAM_TRQ_FLDBACK_HS,       PG_FOLDBACK_INFO,      STR_DRV_HS_FLDBACK,      convertAmperes },
/*358*/ { FALSE, ST_POS_ONLY, PT_CFG,     TIME_MSEC,      2,    1,             PARAM_TRQ_FLDBACK_HS_TC,    PG_FOLDBACK_INFO,      STR_DRV_HS_FB_TC,        convertFilt99pctMilliseconds },
/*359*/ { FALSE, ST_POS_ONLY, PT_CFG,     VEL_TICKS_S,    4,    1<<18,         PARAM_HS_STOP_VEL,          PG_HOMING,             STR_DRV_HS_QUAL_SPEED,   convertVel },
/*360*/ { FALSE, ST_POS_ONLY, PT_CFG,     PCNT_TRQ_LIM,   2,    (1U<<15)/100., PARAM_HS_THRESHOLD,         PG_HOMING,             STR_DRV_HS_TRQ_TRIP },
/*361*/ { FALSE, ST_POS_ONLY, PT_CFG,     TIME_MSEC,      2,    1,             PARAM_HS_TC,                PG_HOMING,             STR_DRV_HS_TC,           convertTimeMS },
/*362*/ { FALSE, ST_POS_ONLY, PT_CFG,     VOLT,           2,    1<<5,          PARAM_REGEN_OFF_V,          PG_SPECIAL_FUNCTIONS,  STR_UNKNOWN,             NULL,                        FW_MILESTONE_AC_GREEN },
/*363*/ { FALSE, ST_POS_ONLY, PT_CFG,     TORQUE_LIMIT,   2,    1<<15,         PARAM_HOMING_TRQ_LIM,       PG_HOMING,             STR_DRV_HS_TRQ_LIM,      convertAmperes },
/*364*/ { FALSE, ST_POS_ONLY, PT_MTR,     TIME_MIN,       2,    1,             PARAM_RMS_SLOW_TC,          PG_MOTOR_INFO,         STR_DRV_RMS_TC,          convertRMSSlowtc,            FW_MILESTONE_DUAL_RMS },
/*365*/ { FALSE, ST_SIGNED,   PT_ML_FAC,  DX_TICK,        4,    1,             PARAM_ACC_ENH_PHASE,        PG_FACTORY_SETTINGS,   STR_UNKNOWN },
/*366*/ { FALSE, ST_SIGNED,   PT_ML_FAC,  DX_TICK,        4,    1,             PARAM_ACC_ENH_AMPL,         PG_FACTORY_SETTINGS,   STR_UNKNOWN },
/*367*/ { FALSE, ST_POS_ONLY, PT_RO_RT,   NO_UNIT,        4,    1,             PARAM_MECH_POSN,            PG_NON_DISPLAY,        STR_UNKNOWN },
/*368*/ { FALSE, ST_UNSIGNED, PT_CFG,     TIME_MSEC,      2,    1,             PARAM_DLY_TO_DISABLE,       PG_SAFETY_INFO,        STR_UNKNOWN,             convertTimeMS },
/*369*/ { FALSE, ST_SIGNED,   PT_FAC,     UNIT_HZ,        4,    1,             PARAM_SOFT_START_FREQ,      PG_FACTORY_SETTINGS,   STR_UNKNOWN,             NULL,                        FW_MILESTONE_SC_VBUS_ADJ },
/*370*/ { FALSE, ST_POS_ONLY, PT_CFG,     TIME_MSEC,      2,    1,             PARAM_REGEN_PWR_TC,         PG_SPECIAL_FUNCTIONS,  STR_UNKNOWN,             convertFilt99pctMilliseconds,FW_MILESTONE_AC_GREEN },
/*371*/ { FALSE, ST_POS_ONLY, PT_CFG,     VOLT,           2,    1<<5,          PARAM_REGEN_ON_V,           PG_SPECIAL_FUNCTIONS,  STR_UNKNOWN,             NULL,                        FW_MILESTONE_AC_GREEN },
/*372*/ { FALSE, ST_POS_ONLY, PT_CFG,     TIME_MSEC,      2,    1,             PARAM_BUSV_ADJ_RATE,        PG_SPECIAL_FUNCTIONS,  STR_UNKNOWN,             NULL,                        FW_MILESTONE_SC_VBUS_ADJ },
/*373*/ { FALSE, ST_POS_ONLY, PT_RAM_RT,  NO_UNIT,        4,    1<<16,         PARAM_NULL,                 PG_NON_DISPLAY,        STR_UNKNOWN,             convertVel },
/*374*/ { FALSE, ST_POS_ONLY, PT_RAM_RT,  NO_UNIT,        2,    1,             PARAM_NULL,                 PG_NON_DISPLAY,        STR_UNKNOWN },
/*375*/ { FALSE, ST_SIGNED,   PT_RAM_RT,  BIT_FIELD,      4,    1,             PARAM_SET_FLAGS,            PG_NON_DISPLAY,        STR_UNKNOWN },
/*376*/ { FALSE, ST_UNSIGNED, PT_RAM,     VOLT,           2,    32768/3.3,     PARAM_NULL,                 PG_NULL,               STR_PARAM_TP_REF_SIM },
/*377*/ { FALSE, ST_UNSIGNED, PT_RAM,     VOLT,           2,    1<<15,         PARAM_NULL,                 PG_NULL,               STR_DRV_BUS_VOLTS_SIM,   convertMeasVolts },
/*378*/ { FALSE, ST_UNSIGNED, PT_RAM,     BIT_FIELD,      4,    1,             PARAM_RUNTIME_FLAGS,        PG_NON_DISPLAY,        STR_UNKNOWN },
/*379*/ { FALSE, ST_SIGNED,   PT_RO,      DEG_C,          2,    1<<6,          PARAM_UNDER_TEMP_TRIP,      PG_FACTORY_SETTINGS,   STR_UNKNOWN },
/*380*/ { FALSE, ST_SIGNED,   PT_RO_RT,   DEG_C,          2,    1<<6,          PARAM_ENCODER_TEMP,         PG_STATUS_INFO,        STR_UNKNOWN },
/*381*/ { FALSE, ST_SIGNED,   PT_RO_RT,   DEG_C,          2,    1<<6,          PARAM_STATOR_TEMP,          PG_STATUS_INFO,        STR_UNKNOWN },
/*382*/ { FALSE, ST_SIGNED,   PT_RO,      DEG_C,          2,    1<<6,          PARAM_STATOR_TEMP_TRIP,     PG_FACTORY_SETTINGS,   STR_UNKNOWN },
/*383*/ { FALSE, ST_SIGNED,   PT_RO,      DEG_C,          2,    1<<6,          PARAM_STATOR_TEMP_WARN,     PG_FACTORY_SETTINGS,   STR_UNKNOWN },
//        1/x,   signed,      type,       unit,           size, scale,         config key id,              param group,           description,             [converter],                  [FW Milestone], [HW factory override]
};

#define PARAM_DRV_COUNT (sizeof(cpmDrvInfoDB)/sizeof(paramInfoLcl))



// Application runtime
static const paramInfoLcl cpmAppInfoDB[] = {
//        1/x,   signed,      type,      unit,         size, scale, config key id,                param group,           description,      [converter], [FW Milestone]
//======Homing parameters====
/*512*/ { FALSE, ST_SIGNED,   PT_CFG,    VEL_TICKS_S,  4,    1<<17, PARAM_ABSPOSN_HOMING_VEL,     PG_HOMING,             STR_HOME_VEL,     convertVel },
/*513*/ { FALSE, ST_POS_ONLY, PT_CFG,    DX_TICK,      4,    1,     PARAM_HOMING_OFFSET,          PG_HOMING,             STR_HOME_OFFS }, //homing offset
/*514*/ { FALSE, ST_POS_ONLY, PT_CFG,    VEL_TICKS_S2, 4,    1<<17, PARAM_HOMING_ACCEL,           PG_HOMING,             STR_HOME_ACCEL,   convertAcc },
/*515*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*516*/ { FALSE, ST_POS_ONLY, PT_CFG,    TIME_MSEC,    2,    1,     PARAM_HS_DELAY_TIME,          PG_HOMING,             STR_HOME_DELAY,   convertTimeMS },
/*517*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    12,   1,     PARAM_EVENT_SHUTDOWN_MASK,    PG_CONFIGURATION_INFO, STR_UNKNOWN,      NULL,          FW_MILESTONE_CL_MIN_VOLT },
/*518*/ { FALSE, ST_POS_ONLY, PT_CFG,    TIME_MSEC,    2,    1,     PARAM_PWR_AC_LOSS_TC,         PG_SAFETY_INFO,        STR_UNKNOWN,      convertTimeMS, FW_MILESTONE_CL_MIN_VOLT },
/*519*/ { FALSE, ST_POS_ONLY, PT_CFG,    TIME_MSEC,    2,    1,     PARAM_PWR_AC_WIRING_ERROR_TC, PG_SAFETY_INFO,        STR_UNKNOWN,      convertTimeMS, FW_MILESTONE_CL_MIN_VOLT },
/*520*/ { FALSE, ST_POS_ONLY, PT_FAC,    VOLT,         2,    1<<5,  PARAM_BUS_V_LOW,              PG_DRIVE_INFO,         STR_SLESS_MIN_VOLTS },
/*521*/ { FALSE, ST_POS_ONLY, PT_CFG,    TIME_MSEC,    2,    1,     PARAM_POWERUP_HOLDOFF_TIME,   PG_MISCELLANEOUS_INFO, STR_UNKNOWN,      convertTimeMS },
/*522*/ { FALSE, ST_UNSIGNED, PT_FAC,    VOLT,         2,    1<<5,  PARAM_COMM_LOW_V,             PG_FACTORY_SETTINGS,   STR_COMM_LOW_V,   NULL,          FW_MILESTONE_SC_HAS_LPB },
/*523*/ { FALSE, ST_POS_ONLY, PT_FAC,    TIME_MSEC,    2,    1,     PARAM_IB_TC_SLOW,             PG_FACTORY_SETTINGS,   STR_P_IB_FILT_TC, convertIBRMSSlowtc, FW_MILESTONE_SC_MIN_VOLT },
/*524*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },

/*525*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*526*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*527*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*528*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*529*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*530*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*531*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*532*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*533*/ { FALSE, ST_SIGNED,   PT_RO_RT,  NO_UNIT,      4,    1,     PARAM_INDEX_COUNT,            PG_DRIVE_INFO,         STR_UNKNOWN },
/*534*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },

/*535*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*536*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*537*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*538*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*539*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
#define UDSZ SC_USER_DESCR_CHUNK
/*540*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      UDSZ, 1,     PARAM_NULL,                   PG_NULL,               STR_USR_DESC0 },
/*541*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      UDSZ, 1,     PARAM_NULL,                   PG_NULL,               STR_USR_DESC1 },
/*542*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      UDSZ, 1,     PARAM_NULL,                   PG_NULL,               STR_USR_DESC2 },
/*543*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      UDSZ, 1,     PARAM_NULL,                   PG_NULL,               STR_USR_DESC3 },
/*544*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      UDSZ, 1,     PARAM_NULL,                   PG_NULL,               STR_USR_DESC4 },
//======Cross-point switch====
/*545*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_OUT_SRC_REG,        PG_NON_DISPLAY,        STR_XPS_OUTPUT_SRC_REG },
/*546*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_IN_SRC_REG,         PG_IO_INFO,            STR_UNKNOWN },
/*547*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*548*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_USER_IN_REG,        PG_IO_INFO,            STR_UNKNOWN },
/*549*/ { FALSE, ST_SIGNED,   PT_RO_RT,  BIT_FIELD,    2,    1,     PARAM_XPS_ACTUAL_IN_REG,      PG_NON_DISPLAY,        STR_UNKNOWN },
/*550*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_INVERT_INPUT,       PG_NON_DISPLAY,        STR_XPS_INVERT_INPUT },
/*551*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_INVERT_OUTPUT,      PG_NON_DISPLAY,        STR_XPS_INVERT_OUTPUT },
/*552*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_XPS_FEAT_00 },
/*553*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_XPS_FEAT_01 },
/*554*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_ENABLE,        PG_NON_DISPLAY,        STR_XPS_FEAT_02 },
/*555*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_XPS_FEAT_03 },
/*556*/ { FALSE, ST_SIGNED,   PT_CFGA,   BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_TRIGGER,       PG_NON_DISPLAY,        STR_XPS_FEAT_04 },
/*557*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_NODE_STOP,     PG_NON_DISPLAY,        STR_XPS_FEAT_05 },
/*558*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_RESET_TIMER,   PG_NON_DISPLAY,        STR_XPS_FEAT_06 },
/*559*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_ATTN0,         PG_NON_DISPLAY,        STR_XPS_FEAT_07 },
/*560*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_ATTN1,         PG_NON_DISPLAY,        STR_XPS_FEAT_08 },
/*561*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_XPS_FEAT_09 },
/*562*/ { FALSE, ST_UNSIGNED, PT_NV_RW,  NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_XPS_FEAT_10 },
/*563*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_AT_HOME,       PG_NON_DISPLAY,        STR_XPS_FEAT_11 },
/*564*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_POS_TRQ_FLBK,  PG_NON_DISPLAY,        STR_XPS_FEAT_12 },
/*565*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_NEG_TRQ_FLBK,  PG_NON_DISPLAY,        STR_XPS_FEAT_13 },
/*566*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_IN_POS_LIM,    PG_NON_DISPLAY,        STR_XPS_FEAT_14 },
/*567*/ { FALSE, ST_SIGNED,   PT_CFG,    BIT_FIELD,    2,    1,     PARAM_XPS_FEAT_IN_NEG_LIM,    PG_NON_DISPLAY,        STR_XPS_FEAT_15 },
//======User Soft Limits====
/*568*/ { FALSE, ST_SIGNED,   PT_CFG,    DX_TICK,      4,    1,     PARAM_SOFT_LIM_POSN_1,        PG_MOTION_CONSTRAINTS, STR_UNKNOWN },
/*569*/ { FALSE, ST_SIGNED,   PT_CFG,    DX_TICK,      4,    1,     PARAM_SOFT_LIM_POSN_2,        PG_MOTION_CONSTRAINTS, STR_UNKNOWN },
/*570*/ { FALSE, ST_UNSIGNED, PT_RAM_RT, NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*571*/ { FALSE, ST_UNSIGNED, PT_RAM_RT, NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*572*/ { FALSE, ST_SIGNED,   PT_RO_RT,  BIT_FIELD,    4,    1,     PARAM_PWR_STATUS,             PG_STATUS_INFO,        STR_UNKNOWN,      NULL,            FW_MILESTONE_SC_HAS_AC_REG },
/*573*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*574*/ { FALSE, ST_SIGNED,   PT_FAC,    NO_UNIT,      4,    1,     PARAM_DRV_PB_SER_NUM,         PG_DRIVE_INFO,         STR_UNKNOWN },
/*575*/ { FALSE, ST_UNSIGNED, PT_FAC,    HW_VERS,      2,    1,     PARAM_DRV_PB_REV,             PG_DRIVE_INFO,         STR_UNKNOWN },
/*576*/ { FALSE, ST_SIGNED,   PT_VOL,    NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_P_TEMP_SIM },
/*577*/ { FALSE, ST_UNSIGNED, PT_NONE,   NO_UNIT,      0,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*578*/ { FALSE, ST_POS_ONLY, PT_CFG,    VOLT,         2,    1<<5,  PARAM_DRV_MIN_OPER_VOLTS,     PG_DRIVE_INFO,         STR_UNKNOWN,      NULL,            FW_MILESTONE_SC_MIN_VOLT },
/*579*/ { FALSE, ST_POS_ONLY, PT_CFG,    DEG_C,        2,    1,     PARAM_TEMP_LIM_USER,          PG_DRIVE_INFO,         STR_UNKNOWN,      NULL,            FW_MILESTONE_SC_USER_TEMP },
/*580*/ { FALSE, ST_SIGNED,   PT_CFG,    CURRENT_2,    2,    1<<15, PARAM_IB_TRIP_USER,           PG_DRIVE_INFO,         STR_P_IB_TRIP,    convertAmpsRMS,  FW_MILESTONE_INDEX_IB },
/*581*/ { FALSE, ST_UNSIGNED, PT_RAM_RT, NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*582*/ { FALSE, ST_UNSIGNED, PT_RAM_RT, NO_UNIT,      2,    1,     PARAM_NULL,                   PG_NULL,               STR_UNKNOWN },
/*583*/ { FALSE, ST_POS_ONLY, PT_CFG,    DX_TICK,      4,    1,     PARAM_NULL,                   PG_HOMING,             STR_UNKNOWN,      NULL,            FW_MILESTONE_2R0 },    // Physical home clearance
//        1/x,   signed,      type,      unit,         size, scale, config key id,                param group,           description,      [converter], [FW Milestone]
};
#undef UDSZ
#define PARAM_APP_COUNT (sizeof(cpmAppInfoDB)/sizeof(paramInfoLcl))

// ClearPath 2.0 features
static const paramInfoLcl cpmApp20InfoDB[] = {
//        1/x,   signed,      type,        unit,         size, scale, config key id,             param group,      description,             [converter],    [FW Milestone]
/*768*/ { FALSE, ST_SIGNED,   PT_NV_RW,    DX_TICK,      4,    1,     PARAM_SHAFT_HOME_TARGET,   PG_HOMING,        STR_UNKNOWN,             NULL,           FW_MILESTONE_2R0 },
/*769*/ { FALSE, ST_SIGNED,   PT_NV_RW_RT, DX_TICK,      4,    1,     PARAM_PRECISION_HOME_POSN, PG_HOMING,        STR_UNKNOWN,             NULL,           FW_MILESTONE_2R0 },
/*770*/ { FALSE, ST_SIGNED,   PT_RO_RT,    DX_TICK,      4,    1,     PARAM_PRECISION_HOME_LAST, PG_HOMING,        STR_UNKNOWN,             NULL,           FW_MILESTONE_2R0 },
/*771*/ { FALSE, ST_UNSIGNED, PT_NONE,     NO_UNIT,      0,    1,     PARAM_NULL,                PG_NULL,          STR_UNKNOWN },
/*772*/ { FALSE, ST_UNSIGNED, PT_NONE,     NO_UNIT,      0,    1,     PARAM_NULL,                PG_NULL,          STR_UNKNOWN },
/*773*/ { FALSE, ST_UNSIGNED, PT_NONE,     NO_UNIT,      0,    1,     PARAM_NULL,                PG_NULL,          STR_UNKNOWN },
/*774*/ { FALSE, ST_UNSIGNED, PT_NONE,     NO_UNIT,      0,    1,     PARAM_NULL,                PG_NULL,          STR_UNKNOWN },
/*775*/ { FALSE, ST_UNSIGNED, PT_NONE,     NO_UNIT,      0,    1,     PARAM_NULL,                PG_NULL,          STR_UNKNOWN },
/*776*/ { FALSE, ST_UNSIGNED, PT_NONE,     NO_UNIT,      0,    1,     PARAM_NULL,                PG_NULL,          STR_UNKNOWN },
/*777*/ { FALSE, ST_UNSIGNED, PT_NONE,     NO_UNIT,      0,    1,     PARAM_NULL,                PG_NULL,          STR_UNKNOWN },
/*778*/ { FALSE, ST_POS_ONLY, PT_NV_RW,    NO_UNIT,      4,    1<<15, PARAM_DHEAT_I,             PG_DRIVE_INFO,    STR_UNKNOWN,             calcDHeatFact,  FW_MILESTONE_2R0 },
/*779*/ { FALSE, ST_POS_ONLY, PT_NV_RW,    NO_UNIT,      4,    1<<15, PARAM_DHEAT_K,             PG_DRIVE_INFO,    STR_UNKNOWN,             NULL,           FW_MILESTONE_2R0 },
/*780*/ { FALSE, ST_SIGNED,   PT_RO_RT,    BIT_FIELD,    4,    1,     PARAM_ML_MTR_STATUS_REG,   PG_STATUS_INFO,   STR_UNKNOWN,             NULL,           FW_MILESTONE_2R0 },
/*781*/ { FALSE, ST_POS_ONLY, PT_RO_RT,    OHM,          2,    1<<9,  PARAM_ML_PHS_RESISTANCE,   PG_MOTOR_INFO,    STR_UNKNOWN,             NULL,           FW_MILESTONE_2R0 },
//        1/x,   signed,      type,        unit,         size, scale, config key id,             param group,      description,             [converter],    [FW Milestone]
};

#define PARAM_APP20_COUNT (sizeof(cpmApp20InfoDB)/sizeof(paramInfoLcl))
//                                                                            *
//*****************************************************************************



//*****************************************************************************
//  NAME                                                                      *
//      cpmClassDelete
//
//  DESCRIPTION:
//      Delete any memory this node allocated.
//
//  \return MN_OK if successful
//
//  SYNOPSIS:
void cpmClassDelete(multiaddr theMultiAddr) {
    register byNodeDB *pNodeDB;
    netaddr cNum = NET_NUM(theMultiAddr);
//#define DEL_PRN
#ifdef DEL_PRN
    _RPT2(_CRT_WARN, "%.1f cpmClassDelete: node %d...\n",
          infcCoreTime(), theMultiAddr);
#endif
    pNodeDB = &SysInventory[cNum].NodeInfo[NODE_ADDR(theMultiAddr)];
    // Delete if no one else has
    if (pNodeDB->paramBankList) {
        for (unsigned iBank = 0; iBank < pNodeDB->bankCount; iBank++) {
            free((void *)(pNodeDB->paramBankList[iBank]).valueDB);
            (pNodeDB->paramBankList[iBank]).valueDB = NULL;
        }
        pNodeDB->bankCount = 0;
        pNodeDB->theID.devCode = NODEID_UNK;
        free((void *)pNodeDB->paramBankList);
        pNodeDB->paramBankList = NULL;
        if (pNodeDB->pNodeSpecific) {
            delete (iscState *)pNodeDB->pNodeSpecific;
            pNodeDB->pNodeSpecific = NULL;
        }
#ifdef DEL_PRN
        _RPT2(_CRT_WARN, "%.1f cpmClassDelete: %d...done\n",
              infcCoreTime(), theMultiAddr);
#endif
    }
#ifdef DEL_PRN
#ifdef _DEBUG
    else {
        _RPT2(_CRT_WARN, "%.1f cpmClassDelete: %d already gone\n",
              infcCoreTime(), theMultiAddr);
    }
#endif
#endif
}
//                                                                            *
//*****************************************************************************


//*****************************************************************************
//  NAME                                                                      *
//      cpmClassSetup
//
//  DESCRIPTION:
//      Setup the parameter manager and node database for this node.
//
//  \return  MN_OK if successful
//
//  SYNOPSIS:
cnErrCode cpmClassSetup(
    multiaddr theMultiAddr) {
    cnErrCode errRet;
    packetbuf resp, fwVers, dummy;
    register byNodeDB *pNodeDB;
    netaddr cNum = NET_NUM(theMultiAddr);

    // Stop using old memory
    cpmClassDelete(theMultiAddr);
    // Make sure this is a Integrated Servo Controller
    errRet = netGetParameter(theMultiAddr, MN_P_NODEID, &resp);
    if (errRet == MN_OK)  {
        errRet = netGetParameter(theMultiAddr, CPM_P_FW_VERS, &fwVers);
        if (errRet == MN_OK)  {
            // Shorthand to access the device ID field
#define DEVID ((devID_t *)(&resp.Byte.Buffer[0]))
#define FWVER ((versID_t *)(&fwVers.Byte.Buffer[0]))
            // Fill in the database from the node if this is a ClearPath-SC
            if (DEVID->fld.devType == NODEID_CS || DEVID->fld.devType == NODEID_GS || DEVID->fld.devType == NODEID_EP)     {
                // Initialize the register/shortcut
                pNodeDB = &SysInventory[cNum].NodeInfo[NODE_ADDR(theMultiAddr)];
                // Wire in our destructor
                pNodeDB->delFunc = cpmClassDelete;
                // Initialize the per node database
                pNodeDB->bankCount = 4;
                // Update the ID area
                pNodeDB->theID.fld.devType = DEVID->fld.devType;
                pNodeDB->theID.fld.devModel = DEVID->fld.devModel;
                // Create and initialize the parameter banks
                pNodeDB->paramBankList = (paramBank *)calloc(pNodeDB->bankCount,
                                         sizeof(paramBank));
                // Bank 0 information
                (pNodeDB->paramBankList[0]).nParams = PARAM_BASE_COUNT;
                (pNodeDB->paramBankList[0]).fixedInfoDB = &cpmInfoDB[0];
                // Create the value storage area together
                (pNodeDB->paramBankList[0]).valueDB
                    = (paramValue *)calloc(PARAM_BASE_COUNT,
                                           sizeof(paramValue));
                // Bank 1 information
                (pNodeDB->paramBankList[1]).nParams = PARAM_DRV_COUNT;
                (pNodeDB->paramBankList[1]).fixedInfoDB = &cpmDrvInfoDB[0];
                // Create the value storage area together
                (pNodeDB->paramBankList[1]).valueDB
                    = (paramValue *)calloc(PARAM_DRV_COUNT,
                                           sizeof(paramValue));
                // Bank 2 information
                (pNodeDB->paramBankList[2]).nParams = PARAM_APP_COUNT;
                (pNodeDB->paramBankList[2]).fixedInfoDB = &cpmAppInfoDB[0];
                // Create the value storage area together
                (pNodeDB->paramBankList[2]).valueDB
                    = (paramValue *)calloc(PARAM_APP_COUNT,
                                           sizeof(paramValue));
                (pNodeDB->paramBankList[3]).nParams = PARAM_APP20_COUNT;
                (pNodeDB->paramBankList[3]).fixedInfoDB = &cpmApp20InfoDB[0];
                // Create the value storage area together
                (pNodeDB->paramBankList[3]).valueDB
                    = (paramValue *)calloc(PARAM_APP20_COUNT,
                                           sizeof(paramValue));

                // Create by node information
                pNodeDB->pNodeSpecific = new iscState;

                // Initialize the class specific items
                pNodeDB->pClassInfo = &cpmClassDB;

                // Reset all node diagnostics by reading them (clear on read)
                netGetParameter(theMultiAddr, CPM_P_NETERR_APP_CHKSUM, &dummy);
                netGetParameter(theMultiAddr, CPM_P_NETERR_APP_FRAG, &dummy);
                netGetParameter(theMultiAddr, CPM_P_NETERR_APP_STRAY, &dummy);
                netGetParameter(theMultiAddr, CPM_P_NETERR_APP_OVERRUN, &dummy);

                errRet = coreUpdateParamInfo(theMultiAddr);
            }
            else {
                errRet = MN_ERR_WRONG_NODE_TYPE;
            }
        }
    }
    return (errRet);
}
//                                                                            *
//*****************************************************************************


//******************************************************************************
//  !NAME!
//      cpmInitializeEx
//
//  DESCRIPTION:
//      This function will configure the network and download the parameters
//      into a local copy of the parameters for the addressed node.
//
//  \return
//      MN_OK if successful
//
//  SYNOPSIS:
cnErrCode cpmInitializeEx(
    multiaddr theMultiAddr,                     // Ptr to return array
    nodebool warmInitialize) {
    return (coreInitializeEx(theMultiAddr, warmInitialize, cpmClassSetup));
}

/*                                                               !end!      */
/****************************************************************************/



/*****************************************************************************
 *  !NAME!
 *      cpmInitialize
 *
 *  DESCRIPTION:
 *      This function will configure the network and download the parameters
 *      into a local copy of the parameters for the addressed node.
 *
 *  \return
 *      MN_OK if successful
 *
 *  SYNOPSIS:                                                                */
cnErrCode cpmInitialize(
    multiaddr theMultiAddr) {                   // Ptr to return array
    return (cpmInitializeEx(theMultiAddr, FALSE));
}
//                                                                            *
/*****************************************************************************/


/*****************************************************************************
 *  !NAME!
 *      cpmParamChangeFunc
 *
 *  DESCRIPTION:
 *      Set the parameter change callback function.
 *
 *  \return
 *      Old handler
 *
 *  SYNOPSIS:                                                               */
MN_EXPORT paramChangeFunc MN_DECL cpmParamChangeFunc(
    paramChangeFunc newFunc) {      // Node number of changed
    paramChangeFunc oldFunc = cpmClassDB.paramChngFunc;
    cpmClassDB.paramChngFunc = newFunc;
    return (oldFunc);
}
//                                                                            *
//==============================================================================
/// \endcond
// END OF PARAMETER ORIENTED INTERNAL_DOC
//==============================================================================


//==============================================================================
// UNDOCUMENTED API FUNCTIONS GO BELOW THIS LINE
/// \cond INTERNAL_DOC
//==============================================================================
//******************************************************************************
//  NAME                                                                       *
//      cpmGetMotorFileName
//
//  DESCRIPTION:
/**
    Get the configuration file name for this node. This information is normally
    maintained by the configuration file loading API.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pMotorFileNameStr Ptr to result area.
    \param[in] maxBufSize Maximum size of the \a pMotorFileNameStr buffer.

    \return  MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetMotorFileName(
    multiaddr theMultiAddr,
    char *pMotorFileNameStr,
    Uint16 maxBufSize) {
    packetbuf nameBuf, cmdBuf;
    cnErrCode theErr;
    unsigned i;

    cmdBuf.Byte.Buffer[CMD_LOC] = ISC_CMD_MOTOR_FILE;
    for (i = CMD_LOC + 1; i < (MN_FILENAME_SIZE + CMD_LOC + 1); i++) {
        cmdBuf.Byte.Buffer[i] = 0;
    }
    cmdBuf.Fld.Addr = theMultiAddr;
    cmdBuf.Fld.PktLen = 1;

    theErr = netRunCommand(NET_NUM(theMultiAddr), &cmdBuf, &nameBuf);

    if (theErr == MN_OK) {
        // Limit return size to buffer size
        i = (nameBuf.Fld.PktLen < maxBufSize)
            ? nameBuf.Fld.PktLen : maxBufSize;
        strncpy((char *)&pMotorFileNameStr[0],
                (const char *)&nameBuf.Byte.Buffer[CMD_LOC], i);
        pMotorFileNameStr[i] = 0;               // Insure term'd
    }
    else {
        pMotorFileNameStr[0] = 0;
    }
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetMotorFileName
//
//  DESCRIPTION:
/**
    Set the configuration file name for this node.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pNewName Ptr null terminated ANSI string up to 25 characters in
                length.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetMotorFileName(
    multiaddr theMultiAddr,
    const char *pNewName) {
    packetbuf nameBuf, respBuf;
    size_t i;
    size_t nSize = strlen(pNewName);
    cnErrCode theErr = MN_OK;

    if (pNewName == NULL) {
        return (MN_ERR_BADARG);
    }

    if (nSize > MN_FILENAME_SIZE) {
        nSize = MN_FILENAME_SIZE;
    }

    // Clear out buffer with 0 padding
    for (i = nSize; i < MN_FILENAME_SIZE; i++) {
        nameBuf.Byte.Buffer[i + CMD_LOC + 1] = 0;
    }

    nameBuf.Byte.Buffer[CMD_LOC] = ISC_CMD_MOTOR_FILE;
    // Cmd Byte + Motor File Octets
    nameBuf.Fld.PktLen = 1 + MN_FILENAME_SIZE;
    nameBuf.Fld.Addr = NODE_ADDR(theMultiAddr);
    strncpy((char *)&nameBuf.Byte.Buffer[CMD_LOC + 1],
            (char *)&pNewName[0], nSize);
    // Ensure that the buffer is null-terminated
    nameBuf.Byte.Buffer[MN_NET_PACKET_MAX - 1] = 0;

    theErr = netRunCommand(NET_NUM(theMultiAddr), &nameBuf, &respBuf);

    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetUserID
//
//  DESCRIPTION:
/**
    Get the User ID for this node. This is an identifier the application
    may use for "plug-and-play" or diagnostic purpose.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pUserIdStr Return buffer area
    \param[in] maxBufSize Size of \p pUserIdStr area

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetUserID(
    multiaddr theMultiAddr,
    char *pUserIdStr,
    Uint16 maxBufSize) {
    return netGetUserID(theMultiAddr, pUserIdStr, maxBufSize);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetUserID
//
//  DESCRIPTION:
/**
    Set the User ID for this node. Any ANSI character string maybe used
    up to 13 characters.

    This maybe used as an identifier for application "plug-and-play" or
    diagnostic purposes. If set to NULL, the APS application will apply
    a generic name "Axis \<addr\>" on its display.  A typical name could
    be "X Axis".

    \param[in] theMultiAddr The address code for this node.
    \param[in] pNewName Ptr to NULL terminated ANSI string.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetUserID(
    multiaddr theMultiAddr,
    const char *pNewName) {
    return netSetUserID(theMultiAddr, pNewName);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetStimulus
//
//  DESCRIPTION:
/**
    Get the current stimulus generator settings.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pState Ptr to the result area updated with current state
            information.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetStimulus(
    multiaddr theMultiAddr, // Target node addr
    iscStimState *pState) {
    packetbuf theCmd, theResp;
    paramValue sampleTime;
    cnErrCode theErr = MN_OK;
    netaddr cNum;
    iscStimCmdPkt *pStimData =
        ((iscStimCmdPkt *)&theResp.Byte.Buffer[RESP_LOC]);

    cNum = coreController(theMultiAddr);

    // Get sample rate conversion factor
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTime);
    if (theErr != MN_OK || sampleTime.value == 0) {
        return (MN_ERR_RESP_FMT);
    }
    sampleTime.value *= 0.001;          // Convert microseconds to milliseconds

    // Fill in the command head parts
    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = 1;
    theCmd.Byte.Buffer[CMD_LOC] = SC_CMD_GET_SET_STIMULUS;
    theErr = netRunCommand(cNum, &theCmd, &theResp);
    // DEBUG PRINT
    //ISC_DUMP_PKT(theMultiAddr, "Get Stim", &theResp);
    if (theErr != MN_OK) {
        return (theErr);
    }

    // Properly formatted response?
    if (theResp.Fld.PktLen != CP_STIM_RESP_STATUS_OCTETS) {
        return (MN_ERR_RESP_FMT);
    }
    // Crack the return buffer into standard units
    pState->mode = (stimModes)pStimData->Mode;
    pState->period = (nodelong)(0.5 + (pStimData->Period * sampleTime.value));
    pState->bits = pStimData->Status;
    switch (pState->mode) {
        case STIM_OFF:
            pState->amplitude = 0;
            break;
        case STIM_VEL:              // Velocity (ticks/ms)
            pState->amplitude = (CP_MON_MAX_VEL * pStimData->Amplitude
                                 / sampleTime.value)
                                / 32767;
            break;
        case STIM_CAL:              // Calibrate/test (100% FS)
        case STIM_TRQ:              // Torque
            pState->amplitude = pStimData->Amplitude / 327.67;
            break;
        case STIM_POSN:             // Position (ticks)
            pState->amplitude = pStimData->Amplitude;
            break;
        case STIM_MOVE_ONCE:
        case STIM_MOVE_RECP:
        case STIM_MOVE_REPEAT:
            pState->amplitude = pStimData->Amplitude;
            pState->slew = pStimData->Slew;
            // Convert dwell back to milliseconds
            if (sampleTime.value != 0)
                pState->dwell = static_cast<nodelong>(pStimData->Dwell
                                                      / sampleTime.value);
            else {
                pState->dwell = 0;
            }
            break;
        default:
            return (MN_ERR_BADARG);
    }
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetStimulus
//
//  DESCRIPTION:
/**
    Change the stimulus generator setting.

    \param[in] theMultiAddr The address code for this node.
    \param[in] pNewState Ptr to the new state desired.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetStimulus(
    multiaddr theMultiAddr, // Target node addr
    iscStimState *pNewState) {
    packetbuf theCmd, theResp;
    paramValue sampleTime;
    nodelong lVal, tVal;
    cnErrCode theErr = MN_OK;
    netaddr cNum;

    cNum = coreController(theMultiAddr);

    // Get sample rate conversion factor
    theErr = netGetParameterInfo(theMultiAddr, CPM_P_SAMPLE_PERIOD,
                                 NULL, &sampleTime);
    if (theErr != MN_OK || sampleTime.value == 0) {
        return (MN_ERR_BADARG);
    }
    sampleTime.value *= 0.001;          // Convert microseconds to milliseconds

    // Fill in the command head parts
    theCmd.Fld.Addr = theMultiAddr;
    switch (pNewState->mode) {
        case STIM_MOVE_ONCE:
        case STIM_MOVE_RECP:
        case STIM_MOVE_REPEAT:
            theCmd.Fld.PktLen = CP_STIM_CMD_PKT_PROF_SIZE;
            break;
        default:
            theCmd.Fld.PktLen = CP_STIM_CMD_PKT_SIZE;
            break;
    }
    theCmd.Byte.Buffer[CMD_LOC] = SC_CMD_GET_SET_STIMULUS;
    // Shortcut to the command buffer
    iscStimCmdPkt *pSimCmd =
        ((iscStimCmdPkt *)&theCmd.Byte.Buffer[CMD_LOC + 1]);

    // Set the mode
    pSimCmd->Mode = (nodeshort)pNewState->mode;
    // Saturate @ largest value
    // Set the 0.5*period(ms) in sample-time(us) counts
    lVal = (nodelong)(0.5 + (pNewState->period / sampleTime.value));
    // Saturate @ largest value
    if (lVal > 32767) {
        lVal = 32767;
    }
    pSimCmd->Period = (nodeshort)(lVal);
    // Zero out potentially unused fields
    pSimCmd->Dwell = 0;
    pSimCmd->Status = 0;
    pSimCmd->Slew = 0;

    // Figure out the scaling for the amplitude based on the mode
    switch (pNewState->mode)  {
        case STIM_OFF:
            lVal = 0;
            break;
        case STIM_VEL:              // Velocity (ticks/ms)
            lVal = (nodelong)(32767. * pNewState->amplitude * sampleTime.value
                              / CP_MON_MAX_VEL 
                              + ((pNewState->amplitude < 0) ? (-.5) : (.5)));
            if (lVal > 32767) {
                lVal = 32767;
            }
            break;
        case STIM_CAL:              // Calibrate/test (100% FS)
        case STIM_TRQ:              // Torque
            lVal = (nodelong)(327.67 * pNewState->amplitude);
            if (lVal > 32767) {
                lVal = 32767;
            }
            break;
        case STIM_POSN:             // Position (ticks)
            if (pNewState->amplitude < 32767) {
                lVal = (nodelong)pNewState->amplitude;
            }
            else {
                lVal = 32767;
            }
            break;
        case STIM_MOVE_ONCE:
        case STIM_MOVE_RECP:
        case STIM_MOVE_REPEAT:
            pSimCmd->Period = static_cast<nodeshort>(pNewState->period);
            lVal = (nodelong)pNewState->amplitude;
            // Saturate @ largest value
            if (pNewState->amplitude > LONG_MAX) {
                lVal = (nodelong)LONG_MAX;
            }
            pSimCmd->Slew = (nodeshort)pNewState->slew;
            // Convert request milliseconds to sample-counts
            tVal = (nodelong)(0.5 + (pNewState->dwell / sampleTime.value));
            if (tVal > INT_MAX) {
                tVal = INT_MAX;
            }
            pSimCmd->Dwell = (nodeshort)tVal;
            break;
        default:
            return (MN_ERR_BADARG);
    }

    pSimCmd->Amplitude = lVal;

    // Make it so, wait for return
    //ISC_DUMP_PKT(theMultiAddr, "Set Stim", &theCmd);

    theErr = netRunCommand(cNum, &theCmd, &theResp);
    if (theErr == MN_OK && theResp.Fld.PktLen != 0) {
        return (MN_ERR_RESP_FMT);
    }
    // Test we get back our settings
    //{
    //   iscStimState x;
    //   theErr = cpmGetStimulus(theMultiAddr, &x);
    //}
    return (theErr);
}
//                                                                             *
//******************************************************************************

//*****************************************************************************
//  NAME                                                                      *
//      cpmForkMoveVelEx
//
//  DESCRIPTION:
/**
    Sends a move that ramps up or down in speed to Velocity (using the
    previously set limits for acceleration and jerk [RAS]).

    Program execution does not suspend.  Once Velocity is achieved, the
    move stays at Velocity until explicitly changed. Velocity is specified
    as steps/second.

    \return cnErrCode; MN_OK if successful
**/
//
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmForkMoveVelEx(
    multiaddr theMultiAddr,         // The node
    double velTargetStepPerSec,     // Target velocity (steps/sec)
    nodelong positionTarget,        // Position Target [optional]
    mgVelStyle moveType) {          // Enhanced style selection
    nodelong bufsLeft;
    return cpmForkVelMove(theMultiAddr, velTargetStepPerSec, false, &bufsLeft);
}
//                                                                            *
//*****************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetMonitor
//
//  DESCRIPTION:
/**
    Get the monitor state for the selected node.

    \param[in] theMultiAddr The address code for this node.
    \param[in] channel Targets which monitor port channel. Set this to zero.
    \param[out] pState Ptr to result area.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetMonitor(
    multiaddr theMultiAddr, // Target node addr
    nodeushort channel,     // Target monitor port if supported
    iscMonState *pState) {
    packetbuf theCmd, theResp;
    cnErrCode theErr = MN_OK;
    appNodeParam param;
    netaddr cNum;

    // Pointer to data overlay in packet
    iscMonNodeState *MON_AREA_GET
        = ((iscMonNodeState *)&theResp.Byte.Buffer[RESP_LOC]);

    cNum = coreController(theMultiAddr);
    // Check for channel # suppprt
    double fwVers;
    theErr = netGetParameterDbl(theMultiAddr, MN_P_FW_VERSION, &fwVers);
    if (theErr != MN_OK) {
        return theErr;
    }

    // We only support one channel
    nodeushort chMax = fwVers >= FW_MILESTONE_2R0 ? 1 : 0;
    if (channel > chMax) {
        return (MN_ERR_BADARG);
    }

    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = 1;
    theCmd.Byte.Buffer[CMD_LOC] = (channel == 0) ? SC_CMD_GET_SET_MONITOR
                                  : SC_CMD_GET_SET_MONITOR1;
    // Get the raw data
    theErr = netRunCommand(cNum, &theCmd, &theResp);
    //ISC_DUMP_PKT(theMultiAddr, "Get Mon", &theResp);
    if (theErr != MN_OK) {
        return (theErr);
    }
    // Properly formatted response?
    if (theResp.Fld.PktLen < 9 || theResp.Fld.PktLen > 10) {
        return (MN_ERR_RESP_FMT);
    }
    // Fill in the return buffer
    if (pState)  {
        // Convert buffer to VB friendly structure
        pState->var = (iscMonVars)MON_AREA_GET->var;
        pState->tuneSync = (iscTuneSyncs)MON_AREA_GET->tuneSync;

        // Convert gain number to full scale equivalent
        pState->gain = convertMonGain(TRUE, theMultiAddr,
                                      pState->var,
                                      MON_AREA_GET->gain);

        // Convert filter TC bits to milliseconds
        param.bits = 0;                 // Not used by converter anyways
        pState->filterTC =
            convertFilt1TCMilliseconds(TRUE, theMultiAddr, param,
                MON_AREA_GET->filter,
                &SysInventory[cNum].NodeInfo[NODE_ADDR(theMultiAddr)]);

        // Copy tune syncs
    }
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetMonitor
//
//  DESCRIPTION:
/**
    Set the monitor state for the selected node. This function will
    also update the parameter values which mirror the individual states.

    \param[in] theMultiAddr The address code for this node.
    \param[in] channel Targets which monitor port channel. Set this to zero.
    \param[out] pNewState Ptr to desired new state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetMonitor(
    multiaddr theMultiAddr, // Target node addr
    nodeushort channel,     // Target monitor port if supported
    iscMonState *pNewState) {

    cnErrCode theErr = MN_OK;
    packetbuf theCmd, theResp;
    nodelong paramLng;
    appNodeParam param;
    netaddr cNum;

    // Shortcut to the command buffer
    iscMonNodeState *MON_AREA_SET
        = ((iscMonNodeState *)&theCmd.Byte.Buffer[CMD_LOC + 1]);

    // Check for channel # suppprt
    double fwVers;
    theErr = netGetParameterDbl(theMultiAddr, MN_P_FW_VERSION, &fwVers);
    if (theErr != MN_OK) {
        return theErr;
    }

    // We only support one channel
    nodeushort chMax = fwVers >= FW_MILESTONE_2R0 ? 1 : 0;
    if (channel > chMax) {
        return (MN_ERR_BADARG);
    }

    // Get controller context
    cNum = coreController(theMultiAddr);
    if (theErr != MN_OK) {
        return (theErr);
    }

    // Fill in the command head parts
    theCmd.Fld.Addr = NODE_ADDR(theMultiAddr);
    theCmd.Fld.PktLen = 10;
    theCmd.Byte.Buffer[CMD_LOC] = channel == 0 ? SC_CMD_GET_SET_MONITOR
                                  : SC_CMD_GET_SET_MONITOR1;

    // Set the variable member
    MON_AREA_SET->var = (nodeshort)pNewState->var;

    // Convert the full scale value to a gain number
    paramLng =
        (nodelong)convertMonGain(FALSE, theMultiAddr,
            (iscMonVars)(pNewState->var & ~(MON_SAVE_NV_MASK)),
            pNewState->gain);
    MON_AREA_SET->gain = paramLng;

    // Convert the filter constant to the internal representation
    param.bits = 0;                 // This is not used by the converter
    paramLng = (nodelong)convertFilt1TCMilliseconds(FALSE, theMultiAddr, param,
               pNewState->filterTC,
               &SysInventory[cNum].NodeInfo[NODE_ADDR(theMultiAddr)]);
    MON_AREA_SET->filter = (nodeshort)paramLng;

    // tuneSync
    MON_AREA_SET->tuneSync = (nodeshort)pNewState->tuneSync;
    theErr = netRunCommand(cNum, &theCmd, &theResp);
    //ISC_DUMP_PKT(theMultiAddr, "Set Mon", &theCmd);
    if (theErr == MN_OK && theResp.Fld.PktLen != 0) {
        return (MN_ERR_RESP_FMT);
    }
    return (theErr);
}
//                                                                             *
//******************************************************************************


//==============================================================================
// END OF UNDOCUMENTED API FUNCTIONS TO THIS LINE
/// \endcond
//==============================================================================

//==============================================================================
// DOCUMENTED API FUNCTIONS GO BELOW THIS LINE
//==============================================================================

//******************************************************************************
//  NAME                                                                       *
//      cpmGetParameter
//
//  DESCRIPTION:
/**
    Get parameter value with no extra information.

    \param[in] theMultiAddr The address code for this node.
    \param[in] theParam The ClearPath-SC parameter code
    \param[out] pParamVal Ptr to result

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetParameter(
    multiaddr theMultiAddr,     // Node address
    cpmParams theParam,         // Parameter index
    double *pParamVal) {        // Ptr to value area
    paramValue pVal;
    cnErrCode theErr;

    theErr = cpmGetParameterEx(theMultiAddr, theParam, &pVal, NULL);

    if (pParamVal != NULL) {
        *pParamVal = pVal.value;
    }
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetParameter
//
//  DESCRIPTION:
/**
    This function will update the local parameter value for the selected
    node and then change the value at the node if possible. If the
    parameter is inaccessible the appropriate error is returned.

    \param[in] theMultiAddr The address code for this node.
    \param[in] theParam The ClearPath-SC m parameter code
    \param[out] paramVal New value to set at node

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetParameter(
    multiaddr theMultiAddr,     // Node address
    cpmParams theParam,         // Parameter index
    double paramVal) {          // New value
    cnErrCode theErr = MN_OK;
    theErr = netSetParameterInfo(theMultiAddr,
                                 theParam,
                                 paramVal, FALSE);
#ifdef _DEBUG
    if (theErr != MN_OK)
        _RPT4(_CRT_WARN, "netSetParameterInfo(%d, %d, %f) failed, err=0x%x",
              theMultiAddr, theParam, paramVal, theErr);
#endif
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetParameterEx
//
//  DESCRIPTION:
/**
    This function will retreive the local parameter value for the
    selected node and return its value and information pertaining to it.

    \param[in] theMultiAddr The address code for this node.
    \param[in] theParam The cpsc parameter code
    \param[out] pTheValue Ptr to the parameter value / raw bits.
    \param[out] pTheInfo Ptr to optional information if non-NULL

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetParameterEx(
    multiaddr theMultiAddr,     // Node address
    cpmParams theParam,         // Parameter index
    paramValue *pTheValue,      // Ptr to returning value area
    paramInfo *pTheInfo) {      // Ptr to returning optional information

    cnErrCode theErr = MN_OK;
    theErr = netGetParameterInfo(theMultiAddr,
                                 theParam,
                                 pTheInfo,
                                 pTheValue);
    return (theErr);
}

//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetParameterEx
//
//  DESCRIPTION:
/**
    Set the parameter using the bytes described in the \a paramVal
    buffer.

    \param[in] theMultiAddr The address code for this node.
    \param[in] theParam The cpsc parameter code
    \param[out] pNewValue Ptr to bit-oriented packet buffer. The buffer size
                will determine the amount written to the node.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetParameterEx(
    multiaddr theMultiAddr,     // Node address
    cpmParams theParam,         // Parameter index
    packetbuf *pNewValue) {     // New value
    cnErrCode theErr = MN_OK;
    // Set the raw value at the node
    theErr = netSetParameterEx(theMultiAddr, theParam, pNewValue);
    return (theErr);
}
//                                                                             *
//******************************************************************************

//******************************************************************************
//  NAME                                                                       *
//      cpmGetHwConfigReg
//
//  DESCRIPTION:
/**
    Get the current state of the Hardware Configuration Register.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pHwConfigReg Ptr to the result.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetHwConfigReg(
    multiaddr theMultiAddr,         // Node address
    cpmHwConfigReg *pHwConfigReg) { // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_HW_CONFIG_REG,
                               &paramVal, NULL);
    *pHwConfigReg = *((cpmHwConfigReg *)&paramVal.raw.Byte.Buffer[0]);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetHwConfigReg
//      As of 01/11/10, this is a 32 bit register
//
//  DESCRIPTION:
/**
    Set the current state of the Hardware Configuration Register. This register
    controls many the hardware related options and features.  For example
    the IEX control is setup here.

    \param[in] theMultiAddr The address code for this node.
    \param[in] hwConfigReg New state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetHwConfigReg(
    multiaddr theMultiAddr,         // Node address
    cpmHwConfigReg hwConfigReg) {   // New setting
    // Update the double value
    return (cpmSetParameter(theMultiAddr, CPM_P_HW_CONFIG_REG,
                            hwConfigReg.bits));
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetAppConfigReg
//
//  DESCRIPTION:
/**
    Get the current state of the Hardware Configuration Register.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pAppConfigReg Ptr to the result.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetAppConfigReg(
    multiaddr theMultiAddr,         // Node address
    cpmAppConfigReg *pAppConfigReg) { // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_APP_CONFIG_REG,
                               &paramVal, NULL);
    *pAppConfigReg = *((cpmAppConfigReg *)&paramVal.raw.Byte.Buffer[0]);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetAppConfigReg
//
//  DESCRIPTION:
/**
    Set the current state of the Hardware Configuration Register. This register
    controls many the hardware related options and features.  For example
    the IEX control is setup here.

    \param[in] theMultiAddr The address code for this node.
    \param[in] appConfigReg New state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetAppConfigReg(
    multiaddr theMultiAddr,         // Node address
    cpmAppConfigReg appConfigReg) {     // New setting
    // Update the double value
    return (cpmSetParameter(theMultiAddr, CPM_P_APP_CONFIG_REG,
                            appConfigReg.bits));
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetStatusAccumReg
//
//  DESCRIPTION:
/**
    Get and clear the current Status Accumulation Register value.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pStatus Ptr to current state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetStatusAccumReg(
    multiaddr theMultiAddr,     // Node address
    cpmStatusReg *pStatus) {    // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_STATUS_ACCUM_REG,
                               &paramVal, NULL);
    // Use overlay master type
    mnStatusReg *pmnStat = (mnStatusReg *)pStatus;
    pmnStat->clear();
    *pStatus = *((cpmStatusReg *) & (paramVal.raw.Byte.Buffer[0]));
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetAttnStatusRiseReg
//
//  DESCRIPTION:
/**
    Get and clear the current accumulation of rising status register states.

    \note Any bits that are unmasked for Attention Generation always return
    zero.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pStatus Ptr to current state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetAttnStatusRiseReg(
    multiaddr theMultiAddr,     // Node address
    cpmStatusReg *pStatus) {    // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_STATUS_RISE_REG,
                               &paramVal, NULL);
    // Use overlay master type
    mnStatusReg *pmnStat = (mnStatusReg *)pStatus;
    pmnStat->clear();
    *pStatus = *((cpmStatusReg *)&paramVal.raw.Byte.Buffer[0]);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetStatusFallReg
//
//  DESCRIPTION:
/**
    Get and clear the current accumulation of the falling status register
    states.

    \note Any bits that are unmasked for Attention Generation always return
    zero.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pStatus Ptr to current state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetStatusFallReg(
    multiaddr theMultiAddr,     // Node address
    cpmStatusReg *pStatus) {    // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_STATUS_FALL_REG,
                               &paramVal, NULL);
    // Use overlay master type
    mnStatusReg *pmnStat = (mnStatusReg *)pStatus;
    pmnStat->clear();
    *pStatus = *((cpmStatusReg *)&paramVal.raw.Byte.Buffer[0]);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetStatusRTReg
//
//  DESCRIPTION:
/**
    Get a snapshot of the current status register states.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pStatus Ptr to current state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetStatusRTReg(
    multiaddr theMultiAddr,     // Node address
    cpmStatusReg *pStatus) {    // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_STATUS_RT_REG,
                               &paramVal, NULL);
    // Use overlay master type
    mnStatusReg *pmnStat = (mnStatusReg *)pStatus;
    pmnStat->clear();
    *pStatus = *((cpmStatusReg *)&paramVal.raw.Byte.Buffer[0]);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetAlertReg
//
//  DESCRIPTION:
/**
    Get the current state of the Alert Register.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pAlertReg Ptr to result area.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetAlertReg(
    multiaddr theMultiAddr,         // Node address
    cpmAlertReg *pAlertReg) {       // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_ALERT_REG, &paramVal, NULL);
    *pAlertReg = *((cpmAlertReg *)&paramVal.raw.Byte.Buffer[0]);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetWarningReg
//
//  DESCRIPTION:
/**
    Get the current state of the Warning Register.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pWarningReg Ptr to result area.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetWarningReg(
    multiaddr theMultiAddr,         // Node address
    cpmAlertReg *pWarningReg) {         // Current setting
    cnErrCode theErr;
    paramValue paramVal;

    theErr = cpmGetParameterEx(theMultiAddr, CPM_P_WARN_REG, &paramVal, NULL);
    *pWarningReg = *((cpmAlertReg *)&paramVal.raw.Byte.Buffer[0]);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmFactoryDefaults
//
//  DESCRIPTION:
/**
    Initialize the node to factory defaults. All tuning and features are
    restored to factory ship out state.

    \param[in] theMultiAddr The address code for this node.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmFactoryDefaults(
    multiaddr theMultiAddr) {       // Destination node
    return cpmFactoryDefaultsEx(theMultiAddr, false);
}

//******************************************************************************
//  NAME                                                                       *
//      cpmFactoryDefaultsEx
//
//  DESCRIPTION:
/**
Initialize the node to factory defaults. All tuning and features are
restored to factory ship out state.

\param[in] theMultiAddr The address code for this node.
\param[in] skipShutdownRisks Don't reset parameters which may cause false shutdowns

\return MN_OK if successful
**/
MN_EXPORT cnErrCode MN_DECL cpmFactoryDefaultsEx(
    multiaddr theMultiAddr,         // Destination node
    bool skipShutdownRisks) {
    return MN_ERR_DEPRECATED;
}
//                                                                             *
//******************************************************************************


//*****************************************************************************
//  NAME                                                                      *
//      cpmForkMoveVel
//
//  DESCRIPTION:
/**
    Sends a move that ramps up or down in speed to \a velTargetStepsPerSec
    (using the previously set limits for acceleration and jerk [RAS]).

    Program execution does not suspend.  Once target velocity is achieved, the
    move stays there until explicitly changed. The velocity is specified
    as steps/second.

    Setting the \a triggered parameter true will cause the move to await
    a trigger event before starting.

    \todo see ref ISCtriggerRefPage Starting Triggered Moves

    \param[in] theMultiAddr The address code for this node.
    \param[in] velTargetStepsPerSec Target velocity in encoder steps per second
        units.
    \param[in] triggered Set this parameter to download the move and await a
        trigger event.
    \param[out] pBuffersRemaining If MN_OK, this is count of buffers available
        more moves.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmForkVelMove(
    multiaddr theMultiAddr,         // Destination node
    double velTargetStepsPerSec,    // Target velocity (steps/sec)
    nodebool triggered,             // Start move with trigger
    nodelong *pBuffersRemaining) {  // Ptr to move buf remaining cnt
    mgVelStyle theStyle = triggered
                          ? MG_MOVE_VEL_STYLE_TRIG : MG_MOVE_VEL_STYLE;
    return iscForkMoveVelQueued(theMultiAddr, velTargetStepsPerSec,
                                0, theStyle, pBuffersRemaining);
}
//*
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmReVector
//
//  DESCRIPTION:
/**
    Reset the vector search flag. This function is mostly diagnostic in nature.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmReVector(
    multiaddr theMultiAddr) { // Target node addr
    packetbuf theCmd, theResp;
    cnErrCode theErr = MN_OK;
    netaddr cNum;

    cNum = coreController(theMultiAddr);
    // Fill in the command head parts
    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = 1;
    theCmd.Byte.Buffer[CMD_LOC] = SC_CMD_RE_VECTOR;
    theErr = netRunCommand(cNum, &theCmd, &theResp);
    return (theErr);
}
//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmAddToPosition
//
//  DESCRIPTION:
/**
    Change the node's measured and commanded positions by the \p theOffset
    amount. All position capture sources are adjusted to reflect the offset
    in number space. The change is atomically applied to all values.

    This function will not cause any motion to occur and provides a mechanism
    for aligning the application's number space with the node's.

    \remark Incoming capture sources properly reflect the offset in
    number space.
    \remark Since this command changes the position of the number space,
    Soft Limits are ignored after this command is performed until the next
    homing sequence if performed.

    \param[in] theMultiAddr The address code for this node.
    \param[in] theOffset Offset amount in encoder units.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmAddToPosition(
    multiaddr theMultiAddr,     // Node address
    double theOffset) {         // Offset amount
    packetbuf theCmd, theResp;  // Input and output buffers
    cnErrCode theErr;
    netaddr cNum;

    cNum = coreController(theMultiAddr);
    // Format the command
    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = SC_CMD_ADD_POSN_LEN;
    theCmd.Byte.Buffer[CMD_LOC] = SC_CMD_ADD_POSN;
    *((nodelong *)&theCmd.Byte.Buffer[CMD_LOC + 1])
        = (nodelong)(theOffset);
    // Run it and wait for response
    theErr = netRunCommand(cNum, &theCmd, &theResp);
    return theErr;
}
//                                                                             *
//******************************************************************************


//*****************************************************************************
//  NAME                                                                      *
//      cpmSetUserDesc
//
//  DESCRIPTION:
/**
    Set the user description to the null-terminated ANSI string. If the
    string is too long it will be truncated at the maximum point to
    allow for a null-terminated end.

    \see SC_USER_DESCR_SZ Use this for the maximum allowed string.

    \param[in] theMultiAddr Node Address
    \param[in] pNewDescr Pointer to null-terminated replacement string.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetUserDesc(
    multiaddr theMultiAddr,
    const char *pNewDescr) {
    return netSetUserDescription(theMultiAddr, pNewDescr);
}
//                                                                            *
//*****************************************************************************


//*****************************************************************************
//  NAME                                                                      *
//      cpmGetUserDesc
//
//  DESCRIPTION:
/**
    Get the user description string from the node. A null-terminated
    string of up to \a maxBufSize is returned.

    \see SC_USER_DESCR_SZ Use this for the maximum allowed string.

    \param[in] theMultiAddr Node Address
    \param[out] pDescription description
    \param[in] maxBufSize description

    \return  MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetUserDesc(
    multiaddr theMultiAddr,
    char *pDescription,
    Uint16 maxBufSize) {
    return netGetUserDescription(theMultiAddr, pDescription, maxBufSize);
}
//                                                                            *
//*****************************************************************************


//*****************************************************************************
//  NAME                                                                      *
//      cpmSendHome
//
//  DESCRIPTION:
/**
    Tell the node to initiate the homing sequencer

    \param[in] theMultiAddr Node Address

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSendHome(
    multiaddr theMultiAddr) {
    cnErrCode theErr =
        cpmSetParameter(theMultiAddr, CPM_P_DRV_SET_FLAGS, 4);
    return (theErr);
}
//                                                                            *
//*****************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmSetUserOutputReg
//
//  DESCRIPTION:
/**
    Set the state of the User Output Register. When the output register is
    controlled by bits in this register, changes here are reflected in the
    output register. This register typically is used to enable the drive and to
    set the General Purpose Outputs.

    \param[in] theMultiAddr The address code for this node.
    \param[in] pNewOutReg New state for User Output Register.

    \return  MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmSetUserOutputReg(
    multiaddr theMultiAddr,     // Node address
    cpmOutReg *pNewOutReg) {        // New setting
    // Update the double value
    plaOutReg *pOuts = (plaOutReg *)pNewOutReg;
    return (cpmSetParameter(theMultiAddr, CPM_P_USER_OUT_REG, pOuts->bits));
}

//                                                                             *
//******************************************************************************


//******************************************************************************
//  NAME                                                                       *
//      cpmGetUserOutputReg
//
//  DESCRIPTION:
/**
    Get the current setting of the User Output Register.

    \param[in] theMultiAddr The address code for this node.
    \param[out] pOutReg Ptr to current state.

    \return MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmGetUserOutputReg(
    multiaddr theMultiAddr,     // Node address
    cpmOutReg *pOutReg) { // Current setting
    double pVal;
    cnErrCode theErr;

    theErr = cpmGetParameter(theMultiAddr, CPM_P_USER_OUT_REG, &pVal);
    // Update the raw buffer
    plaOutReg *pOuts = (plaOutReg *)pOutReg;
    pOuts->bits = CAST_NODEUSHORT(pVal);
    return (theErr);
}


//*****************************************************************************
//  NAME                                                                      *
//      cpmForkPosnMove
//
//  DESCRIPTION:
/**
    This is the main function to initiate a positional moves and inquire
    on how many more pending moves the node will accept.

    The motion is constrained to the settings of the acceleration, velocity
    and jerk limits.  For the more complex head and tail moves the additional
    constraints of head and tail distances as well as head/tail velocity
    limits apply. Once this move has been accepted, any constraints values
    changed will apply only to the next move segment.

    The \a moveType field is most easily supplied a #mgPosnStyle enumeration
    type. This #mgPosnStyle provides a copy constructor to allow direct
    use of this enumeration.

    A C example for issuing a head/tail absolute position move.
    \code
    theErr = cpmForkPosnMove(m_nodeAddr, target, MG_MOVE_STYLE_HEADTAIL_ABS,
                             &nMovesLeft);
    \endcode

    \remark If a triggered move is accepted in a "stream" of other non-triggered
    moves, it will block upon its execution until a new trigger event
    occurs.

    \param[in] theMultiAddr The address code for this node.
    \param[in] posnTarget Target position. If this is a relative type move
        then this is the increment from the initial position. If this is
        and absolute move, then this is the final location.
    \param[in] moveType The move style. This is a bit oriented value that
        specifies the profile, triggering event handling and the handling
        of the target.
    \param[out] pBuffersRemaining Pointer to buffers remaining count.
        Upon return the node will accept up to \a pBuffersRemaining more
        calls to this function.

    \return  MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmForkPosnMove(
    multiaddr theMultiAddr,         // Destination node
    nodelong posnTarget,            // Target (steps)
    mgPosnStyle moveType,           // Style code
    nodelong *pBuffersRemaining) {  // Ptr to move buf remaining cnt
    mgMoveProfiledInfo spec;
    spec.value = posnTarget;
    spec.type = moveType.styleCode;
    return (iscMoveProfiledQueued(theMultiAddr, &spec, pBuffersRemaining));
}
//                                                                            *
//*****************************************************************************


//*****************************************************************************
//  NAME                                                                      *
//      cpmForkProfiledMove
//
//  DESCRIPTION:
/**
    This is the main function to initiate profiled moves and inquire
    on how many more pending moves the node will accept.

    \param[in] theMultiAddr         The address code for this node.
    \param[in] spec                 The move specification
    \param[out] pBuffersRemaining   Pointer to buffers remaining count.
        Upon return the node will accept up to \a pBuffersRemaining more
        calls to this function.

    \return  MN_OK if successful
**/
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL cpmForkProfiledMove(
    multiaddr theMultiAddr,         // Destination node
    mgMoveProfiledInfo *spec,       // Move specification
    nodelong *pBuffersRemaining) {  // Ptr to move buf remaining cnt
    return (iscMoveProfiledQueued(theMultiAddr, spec, pBuffersRemaining));
}
//                                                                            *
//*****************************************************************************

//*****************************************************************************
//  NAME                                                                      *
//      _cpmStatusRegFlds::StateStr
//
//  DESCRIPTION:
/**
Return a buffer of newline-delimited field names of all bits that
are set in the status register.

\param[in]      buffer      A buffer to store the created string
\param[in]      size        The size of the given buffer

\return     A pointer to the filled-in buffer
**/
//  SYNOPSIS:
char *MN_DECL _cpmStatusRegFlds::StateStr(char *buffer, size_t size) const {
    // Value to return
    std::string retVal, bitVals;
    retVal.reserve(size);
    // Convert to bits
    Uint16 mask;
    // Convert to our register view to get at bits
    cpmStatusReg &bits = *(cpmStatusReg *)this;
    size_t bitIndex = 47;

    // Token to append
    char token[50];
    retVal = "0x";
    int nInts = sizeof(bits) / sizeof(Uint16);
    for (int intNum = nInts - 1; intNum >= 0; intNum--) {
        snprintf(token, sizeof(token), "%04X ", bits.bits[intNum]);
        retVal += token;
        for (size_t bitNum = 0; bitNum < 16; bitNum++) {
            mask = 0x8000 >> bitNum;
            switch (bitIndex) {
                case 32:
                    snprintf(token, sizeof(token),
                             "[%02d-%02d] InMotion: %s\n",
                             int(bitIndex), int(bitIndex + 1),
                             CPSCinMotions[InMotion]);
                    bitVals += token;
                    break;
                case 33:
                    // MSB of InMotion covered already
                    break;
                case 40:
                    snprintf(token, sizeof(token),
                             "[%02d-%02d] ShutdownState: %s\n",
                             int(bitIndex), int(bitIndex + 1),
                             CPSCshutdowns[ShutdownState]);
                    bitVals += token;
                    break;
                case 41:
                    // MSB of ShutdownState covered already
                    break;
                default:
                    // Print only set bits
                    if (bits.bits[intNum] & mask) {
                        snprintf(token, sizeof(token),
                                 "[%02d] %s\n", int(bitIndex),
                                 CPSCStatusBitStrs[bitIndex]);
                        bitVals += token;
                    }
            }
            bitIndex--;
        }
    }
    retVal += "\n";
    retVal += bitVals;
    strncpy(buffer, retVal.c_str(), size);
    // Ensure that the buffer is null-terminated
    buffer[size - 1] = 0;
    return (buffer);
}
//                                                                            *
//*****************************************************************************


//*****************************************************************************
//  NAME                                                                      *
//      _cpmAlertFlds::StateStr
//
//  DESCRIPTION:
/**
Return a buffer of newline-delimited field names of all bits that
are set in the alert register.

\param[in]      buffer      A buffer to store the created string
\param[in]      size        The size of the given buffer

\return     A pointer to the filled-in buffer
**/
//  SYNOPSIS:
char *MN_DECL _cpmAlertFlds::StateStr(char *buffer, size_t size) const {
    // Value to return
    std::string retVal, bitVals;
    retVal.reserve(size);
    // Convert to bits
    Uint32 mask;
    // Convert to our register view to get at bits
    cpmAlertReg &bits = *(cpmAlertReg *)this;
    size_t bitIndex = 95;
    // Token to append
    char token[50];
    int nInts = sizeof(bits) / sizeof(Uint32);

    retVal = "0x";
    for (int intNum = nInts - 1; intNum >= 0; intNum--) {
        snprintf(token, sizeof(token), "%04X %04X ",
                 bits.bits[intNum] >> 16, bits.bits[intNum] & 0xFFFF);
        retVal += token;
        for (size_t bitNum = 0; bitNum < 32; bitNum++) {
            mask = 0x80000000 >> bitNum;
            // Print only set bits
            if (bits.bits[intNum] & mask) {
                snprintf(token, sizeof(token), "[%02d] %s\n", int(bitIndex),
                         CPSCalertsBitStrs[bitIndex]);
                bitVals += token;
            }
            bitIndex--;
        }
    }
    retVal += "\n";
    retVal += bitVals;
    strncpy(buffer, retVal.c_str(), size);
    // Ensure that the buffer is null-terminated
    buffer[size - 1] = 0;
    return (buffer);
}
//                                                                            *
//*****************************************************************************

/// @}
//==============================================================================
//  END OF FILE cpscAPI.cpp
//==============================================================================
