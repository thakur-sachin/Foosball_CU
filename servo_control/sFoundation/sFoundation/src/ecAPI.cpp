#ifdef IMPL_NODE_EC

#include "ecAPI.h"
#include "cpmRegs.h"
#include "netCmdPrivate.h"
/// \cond   INTERNAL_DOC

//******************************************************************************
//  NAME                                                                       *
//      infcGetP402ParamCount
//
//  DESCRIPTION:
//      This function will get the count of p402 parameters available.
//
//  RETURNS:
//      Standard return codes
//
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL infcGetP402ParamCount(multiaddr theMultiAddr, uint32_t *count)
{
    packetbuf theCmd, theResp;

    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = 1;
    theCmd.Byte.BufferSize = theCmd.Fld.PktLen + MN_API_PACKET_HDR_LEN;
    theCmd.Byte.Buffer[CMD_LOC] = _cpmCmds::SC_CMD_GET_ETHERPATH_PARAM_COUNT;

    cnErrCode theErr = infcRunCommand(NET_NUM(theMultiAddr), &theCmd, &theResp);
    if (theErr == MN_OK) {
        // Check that the response was legit
        theErr = coreGenErrCode(NET_NUM(theMultiAddr), &theResp, theCmd.Fld.Addr);
#ifdef RUN_CMD_ERR_DBG
        if (theErr != MN_OK) {
            _RPT3(_CRT_WARN, "%.1f netRunCommand(%d): parse error 0x%x\n",
                infcCoreTime(), NET_NUM(theMultiAddr), theErr);
        }
#endif
        *count = *(uint32_t *)&theResp.Byte.Buffer[RESP_LOC];
    }

    return theErr;
}

//******************************************************************************
//  NAME                                                                       *
//      infcGetP402Param
//
//  DESCRIPTION:
//      This function will get the p402 parameter at the specified index and 
//      subindex location.
//
//  RETURNS:
//      Standard return codes
//
//  SYNOPSIS:
MN_EXPORT cnErrCode MN_DECL infcGetP402ParamInfo(multiaddr theMultiAddr, uint16_t index, P402ParamInfo_t* result)
{
    packetbuf theCmd, theResp;

    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = 3;
    theCmd.Byte.BufferSize = theCmd.Fld.PktLen + MN_API_PACKET_HDR_LEN;
    theCmd.Byte.Buffer[CMD_LOC] = _cpmCmds::SC_CMD_GET_ETHERPATH_PARAM_INFO;
    uint16_t* cmdIndex = (uint16_t*)&(theCmd.Byte.Buffer[CMD_LOC + 1]);
    *cmdIndex = index;

    cnErrCode theErr = infcRunCommand(NET_NUM(theMultiAddr), &theCmd, &theResp);
    if (theErr == MN_OK) {
        // Check that the response was legit
        theErr = coreGenErrCode(NET_NUM(theMultiAddr), &theResp, theCmd.Fld.Addr);
#ifdef RUN_CMD_ERR_DBG
        if (theErr != MN_OK) {
            _RPT3(_CRT_WARN, "%.1f netRunCommand(%d): parse error 0x%x\n",
                infcCoreTime(), NET_NUM(theMultiAddr), theErr);
        }
#endif
        *result = {
            *(uint32_t*)&theResp.Byte.Buffer[RESP_LOC],
            *(uint32_t*)&theResp.Byte.Buffer[RESP_LOC+4],
        };
    }

    // TODO return the result from theResp
    return theErr;
}

MN_EXPORT cnErrCode MN_DECL infcGetP402ParamRaw(multiaddr theMultiAddr, uint16_t index, char* pParamBuf, uint16_t maxBufSize)
{
    packetbuf theCmd, theResp;

    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = 3;
    theCmd.Byte.BufferSize = theCmd.Fld.PktLen + MN_API_PACKET_HDR_LEN;
    theCmd.Byte.Buffer[CMD_LOC] = _cpmCmds::SC_CMD_GET_ETHERPATH_PARAM_VAL;
    uint16_t* cmdIndex = (uint16_t*)&(theCmd.Byte.Buffer[CMD_LOC + 1]);
    *cmdIndex = index;

    // Clear the output buffer
    memset(pParamBuf, 0, sizeof(char) * maxBufSize);

    cnErrCode theErr = infcRunCommand(NET_NUM(theMultiAddr), &theCmd, &theResp);
    if (theErr == MN_OK) {
        // Check that the response was legit
        theErr = coreGenErrCode(NET_NUM(theMultiAddr), &theResp, theCmd.Fld.Addr);
#ifdef RUN_CMD_ERR_DBG
        if (theErr != MN_OK) {
            _RPT3(_CRT_WARN, "%.1f netRunCommand(%d): parse error 0x%x\n",
                infcCoreTime(), NET_NUM(theMultiAddr), theErr);
        }
#endif
        if (theErr == MN_OK) {
            // Limit return size to buffer size
            uint16_t length = (theResp.Fld.PktLen < maxBufSize)
                ? theResp.Fld.PktLen : maxBufSize;
            memcpy(pParamBuf, (const char*)&theResp.Byte.Buffer[RESP_LOC], length);
        }
    }

    return theErr;
}
MN_EXPORT cnErrCode MN_DECL infcGetP402ParamValue(multiaddr theMultiAddr, uint16_t index, double* result)
{
    P402ParamInfo_t infoRes;
    cnErrCode infoErr = infcGetP402ParamInfo(theMultiAddr, index, &infoRes);
    if (infoErr != MN_OK) {
        return infoErr;
    }

    packetbuf theCmd, theResp;

    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = 3;
    theCmd.Byte.BufferSize = theCmd.Fld.PktLen + MN_API_PACKET_HDR_LEN;
    theCmd.Byte.Buffer[CMD_LOC] = _cpmCmds::SC_CMD_GET_ETHERPATH_PARAM_VAL;
    uint16_t* cmdIndex = (uint16_t*)&(theCmd.Byte.Buffer[CMD_LOC + 1]);
    *cmdIndex = index;

    cnErrCode theErr = infcRunCommand(NET_NUM(theMultiAddr), &theCmd, &theResp);
    if (theErr == MN_OK) {
        // Check that the response was legit
        theErr = coreGenErrCode(NET_NUM(theMultiAddr), &theResp, theCmd.Fld.Addr);
#ifdef RUN_CMD_ERR_DBG
        if (theErr != MN_OK) {
            _RPT3(_CRT_WARN, "%.1f netRunCommand(%d): parse error 0x%x\n",
                infcCoreTime(), NET_NUM(theMultiAddr), theErr);
        }
#endif
        switch (theResp.Fld.PktLen) {
        case 1:
            if (infoRes.fld.signedVal) {
                *result = *(int8_t*)&theResp.Byte.Buffer[RESP_LOC];
            }
            else {
                *result = *(uint8_t*)&theResp.Byte.Buffer[RESP_LOC];
            }
            break;
        case 2:
            if (infoRes.fld.signedVal) {
                *result = *(int16_t*)&theResp.Byte.Buffer[RESP_LOC];
            }
            else {
                *result = *(uint16_t*)&theResp.Byte.Buffer[RESP_LOC];
            }
            break;
        case 4:
            if (infoRes.fld.signedVal) {
                *result = *(int32_t*)&theResp.Byte.Buffer[RESP_LOC];
            }
            else {
                *result = *(uint32_t*)&theResp.Byte.Buffer[RESP_LOC];
            }
            break;
        default:
            break;
            //bad case do something
        }
    }

    // TODO return the result from theResp
    return theErr;
}

MN_EXPORT cnErrCode MN_DECL infcSetP402Param(multiaddr theMultiAddr, uint16_t index, uint8_t subindex, 
     void *pParam, uint16_t paramSize)
{
    packetbuf theCmd, theResp;

    theCmd.Fld.Addr = theMultiAddr;
    theCmd.Fld.PktLen = ((paramSize + 4) > MN_API_PAYLOAD_MAX) ? MN_API_PAYLOAD_MAX : paramSize + 4;
    theCmd.Byte.BufferSize = theCmd.Fld.PktLen + MN_API_PACKET_HDR_LEN;
    theCmd.Byte.Buffer[CMD_LOC] = _cpmCmds::SC_CMD_SET_ETHERPATH_PARAM_VAL;
    uint16_t *paramIndex = (uint16_t *)&(theCmd.Byte.Buffer[CMD_LOC + 1]);
    *paramIndex = index;
    uint8_t *paramSubindex = (uint8_t *)&(theCmd.Byte.Buffer[CMD_LOC + 3]);
    *paramSubindex = subindex;

    // copy the value into the command packet
    memcpy((char *)&theCmd.Byte.Buffer[CMD_LOC + 4], (const char *)pParam, theCmd.Fld.PktLen-4);

    cnErrCode theErr = infcRunCommand(NET_NUM(theMultiAddr), &theCmd, &theResp);
    if (theErr == MN_OK) {
        // Check that the response was legit
        theErr = coreGenErrCode(NET_NUM(theMultiAddr), &theResp, theCmd.Fld.Addr);
#ifdef RUN_CMD_ERR_DBG
        if (theErr != MN_OK) {
            _RPT3(_CRT_WARN, "%.1f netRunCommand(%d): parse error 0x%x\n",
                infcCoreTime(), NET_NUM(theMultiAddr), theErr);
        }
#endif
    }

    // TODO return the result from theResp
    return theErr;
}
#endif 
