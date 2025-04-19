/** @file PDD_Ensemble.c
 *  @brief Ensemble functions implementation
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */

#include "PDD_Ensemble.h"
#include "PDD_Types.h"

#include <stdlib.h> 

#ifdef __cplusplus
namespace tdym
{
#endif

#ifndef DONT_USE_MALLOC
 /** @brief Create and initialize PDD_WavesEnsemble structure
  *
  *   @returns Pointer to PDD_WavesEnsemble structure
  */
PDD_DECL PDD_WavesEnsemble* PDD_CreateWavesEnsemble()
{
    PDD_WavesEnsemble* pEns = malloc(sizeof(PDD_WavesEnsemble));
    if (pEns)
    {
        memset(pEns, 0, sizeof(PDD_WavesEnsemble));
    }
    return pEns;
}

/** @brief De-allocate memory for ensemble.  Remember to set pEns to NULL after calling this function.
*/
PDD_DECL void PDD_DestroyWavesEnsemble(PDD_WavesEnsemble* pEns)
{
    if (pEns)
    {
        free(pEns);
    }
}

/** @brief Create and initialize PD0Ensemble structure
 *
 *   @returns Pointer to PD0Ensemble structure
 */
PDD_DECL PDD_PD0Ensemble* PDD_CreatePD0Ensemble()
{
    PDD_PD0Ensemble* pEns = malloc(sizeof(PDD_PD0Ensemble));
    if (pEns)
    {
        memset(pEns, 0, sizeof(PDD_PD0Ensemble));
    }
    return pEns;
}

/** @brief De-allocate memory for ensemble.  Remember to set pEns to NULL after calling this function.
*/
PDD_DECL void PDD_DestroyPD0Ensemble(PDD_PD0Ensemble* pEns)
{
    if (pEns)
    {
        free(pEns);
    }
}
#endif

/** @brief Internal function to set first leader structure pointer and size
 */
static void SetFirstLeader(PDD_WavesEnsemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->firstLeader = (PDD_WavesFirstLeader*)ptr;
        pEns->firstLeaderSize = size;
    }
}

/** @brief Internal function to set waves data structure pointer and size
 */
static void SetWavesData(PDD_WavesEnsemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->wavesData = (PDD_WavesData*)ptr;
        pEns->wavesDataSize = size;
    }
}

/** @brief Internal function to set last leader structure pointer and size
 */
static void SetLastLeader(PDD_WavesEnsemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->lastLeader = (PDD_WavesLastLeader*)ptr;
        pEns->lastLeaderSize = size;
    }
}

/** @brief Internal function to set HPRs data structure pointer and size
 */
static void SetHPRData(PDD_WavesEnsemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->hprData = (PDD_HPRData*)ptr;
        pEns->hprDataSize = size;
    }
}

/** @brief Size of internal waves map to decode data
 */
#define SIZE_OF_WAVES_MAP (4)

 /** @brief Internal map to decode waves data
  */
static PDD_DataMap PDD_WAVES_MAP[SIZE_OF_WAVES_MAP] =
{
    { WavesFirstLeaderID, SetFirstLeader, 0},
    { WavesDataID, SetWavesData, 0},
    { WavesLastLeaderID, SetLastLeader, 0},
    { HPRDataID, SetHPRData, 0},
};

/** @brief Internal function to set pointers to structures
 */
static int SetWavesStructure(int key, PDD_WavesEnsemble* pEns, const unsigned char* ptr, int size)
{
    int i;
    for (i = 0; i < SIZE_OF_WAVES_MAP; i++)
    {
        if (PDD_WAVES_MAP[i].key == key)
        {
            PDD_WAVES_MAP[i].value(pEns, ptr, size, 0);
            return 1;
        }
    }
    return 0;
}

/** @brief Decode waves ensemble (checksum must be already verified)
*
*   @param data Buffer to ensemble data
*   @param count Ensemble size
*   @param pEns Pointer to ensemble
*
*   @returns 1 if successful, 0 otherwise
 */
PDD_DECL int PDD_DecodeWaves(const unsigned char* data, int count, PDD_WavesEnsemble* pEns)
{
    int i, dataCount;


    if (data == NULL || count < PDD_MIN_ENSEMBLE_SIZE || count > PDD_MAX_ENSEMBLE_SIZE)
    {
        return 0;
    }

    if (pEns->data != data)
    {
        memcpy(pEns->data, data, count);
    }

    pEns->ensSize = count;
    pEns->header = (PDD_Header*)pEns->data;

    dataCount = pEns->header->dataCount;
    pEns->headerSize = pEns->header->offset[0];
    for (i = 0; i < dataCount; i++)
    {
        int size, offset;
        const unsigned char* ptr;
        unsigned short id;
        offset = pEns->header->offset[i];
        size = (i == (dataCount - 1) ? (pEns->header->checksumOffset - 2) : pEns->header->offset[i + 1]);
        size -= offset;
        ptr = pEns->data + offset;
        id = *(unsigned short*)(ptr);
        SetWavesStructure(id, pEns, ptr, size);
    }
    return 1;
}

/** @brief Internal function to set fixed leader structure pointer and size
 */
static void SetFixedLeader(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->fixedLeader[index] = (PDD_FixedLeader*)(ptr);
        pEns->fixedLeaderSize[index] = size;
    }
}

/** @brief Internal function to set variable leader structure pointer and size
 */
static void SetVariableLeader(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->varLeader[index] = (PDD_VariableLeader*)(ptr);
        pEns->varLeaderSize[index] = size;
    }
}

/** @brief Internal function to set bottom track structure pointer and size
 */
static void SetBottomTrack(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->bottomTrack = (PDD_BottomTrack*)(ptr);
        pEns->bottomTrackSize = size;
    }
}

/** @brief Internal function to set velocity profile structure pointer and size
 */
static void SetVelocityProfile(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->velocity[index] = (PDD_ShortArray*)(ptr);
        pEns->velocitySize[index] = size;
    }
}

/** @brief Internal function to set correlation profile structure pointer and size
 */
static void SetCorrelationProfile(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->correlation[index] = (PDD_ByteArray*)(ptr);
        pEns->correlationSize[index] = size;
    }
}

/** @brief Internal function to set intensity profile structure pointer and size
 */
static void SetIntensityProfile(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->intensity[index] = (PDD_ByteArray*)(ptr);
        pEns->intensitySize[index] = size;
    }
}

/** @brief Internal function to set percent good profile structure pointer and size
 */
static void SetPercGoodProfile(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->pctGood[index] = (PDD_ByteArray*)(ptr);
        pEns->pctGoodSize[index] = size;
    }
}

/** @brief Internal function to set status profile structure pointer and size
 */
static void SetStatusProfile(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->status[index] = (PDD_ByteArray*)(ptr);
        pEns->statusSize[index] = size;
    }
}

/** @brief Internal function to set vertical beam leader structure pointer and size
 */
static void SetVertBeamLeader(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->vbLeader = (PDD_VertBeamLeader*)ptr;
        pEns->vbLeaderSize = size;
    }
}

/** @brief Internal function to set navigation parameters structure pointer and size
 */
static void SetNavigationParams(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->navParams = (PDD_NavigationParams*)ptr;
        pEns->navParamsSize = size;
    }
}

/** @brief Internal function to set transformation matrix structure pointer and size
 */
static void SetXformMatrix(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->xform = (PDD_TransformationMatrix*)ptr;
        pEns->xformSize = size;
    }
}

/** @brief Internal function to set vertical beam range structure pointer and size
 */
static void SetVertBeamRange(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->vbRange = (PDD_VertBeamRange*)ptr;
        pEns->vbRangeSize = size;
    }
}

/** @brief Internal function to set BT high resolution velocity structure pointer and size
 */
static void SetBTHiResVel(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->btHiResVel = (PDD_BTHighResVelocity*)ptr;
        pEns->btHiResVelSize = size;
    }
}

/** @brief Internal function to set bottom track range structure pointer and size
 */
static void SetBTRange(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->btRange = (PDD_BottomTrackRange*)ptr;
        pEns->btRangeSize = size;
    }
}

/** @brief Internal function to set ISM data structure pointer and size
 */
static void SetISMData(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->ismData = (PDD_ISMData*)ptr;
        pEns->ismDataSize = size;
    }
}

/** @brief Internal function to set ping attitude structure pointer and size
 */
static void SetPingAttitude(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->pingAttitude = (PDD_PingAttitude*)ptr;
        pEns->pingAttitudeSize = size;
    }
}

/** @brief Internal function to set ADC data structure pointer and size
 */
static void SetADCData(PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size, int index)
{
    if (pEns && ptr)
    {
        pEns->adcData = (PDD_ADCData*)ptr;
        pEns->adcDataSize = size;
    }
}

/** @brief Size of internal PD0 map to decode data
 */
#define PDD_SIZE_OF_MAP (24)

 /** @brief Internal map to decode PD0 data
  */
static PDD_DataMap PDD_PD0_MAP[PDD_SIZE_OF_MAP] =
{
    { FixedLeaderID, SetFixedLeader, 0},
    { FixedLeader2ID, SetFixedLeader, 1},
    { VariableLeaderID, SetVariableLeader, 0},
    { VariableLeader2ID, SetVariableLeader, 1},
    { BottomTrackID, SetBottomTrack, 0},
    { VelocityProfileID, SetVelocityProfile, 0},
    { VelocityProfile2ID, SetVelocityProfile, 1},
    { CorrelationProfileID, SetCorrelationProfile, 0},
    { CorrelationProfile2ID, SetCorrelationProfile, 1},
    { IntensityProfileID, SetIntensityProfile, 0},
    { IntensityProfile2ID, SetIntensityProfile, 1},
    { PercentGoodProfileID, SetPercGoodProfile, 0},
    { PercentGoodProfile2ID, SetPercGoodProfile, 1},
    { StatusProfileID, SetStatusProfile, 0},
    { StatusProfile2ID, SetStatusProfile, 1},
    { VertBeamLeaderID, SetVertBeamLeader, 0},
    { NavigationParamsID, SetNavigationParams, 0},
    { TransformationMatrixID, SetXformMatrix, 0},
    { VertBeamRangeID, SetVertBeamRange, 0},
    { BTHighResVelocityID, SetBTHiResVel, 0},
    { BottomTrackRangeID, SetBTRange, 0},
    { ISMDataID, SetISMData, 0},
    { PingAttitudeID, SetPingAttitude, 0},
    { ADCDataID, SetADCData, 0}
};

/** @brief Internal function to set pointers to structures
 */
static int SetStructure(int key, PDD_PD0Ensemble* pEns, const unsigned char* ptr, int size)
{
    int i;
    for (i = 0; i < PDD_SIZE_OF_MAP; i++)
    {
        if (PDD_PD0_MAP[i].key == key)
        {
            PDD_PD0_MAP[i].value(pEns, ptr, size, PDD_PD0_MAP[i].index);
            return 1;
        }
    }
    return 0;
}

/** @brief Decode PD0 ensemble (checksum must be already verified)
*
*   @param data Buffer to ensemble data
*   @param count Ensemble size
*   @param pEns Pointer to ensemble
*
*   @returns 1 if successful, 0 otherwise
 */
PDD_DECL int PDD_DecodePD0(const unsigned char* data, int count, PDD_PD0Ensemble* pEns)
{
    int i, dataCount;

    if (data == NULL || count < PDD_MIN_ENSEMBLE_SIZE || count > PDD_MAX_ENSEMBLE_SIZE)
    {
        return 0;
    }

    if (pEns->data != data)
    {
        memcpy(pEns->data, data, count);
    }

    pEns->ensSize = count;
    pEns->header = (PDD_Header*)pEns->data;

    dataCount = pEns->header->dataCount;
    pEns->headerSize = pEns->header->offset[0];
    for (i = 0; i < dataCount; i++)
    {
        const unsigned char* ptr;
        int size, offset;
        unsigned short id;
        offset = pEns->header->offset[i];
        size = (i == (dataCount - 1) ? (pEns->header->checksumOffset - 2) : pEns->header->offset[i + 1]);
        size -= offset;
        ptr = pEns->data + offset;
        id = *(unsigned short*)(ptr);
        SetStructure(id, pEns, ptr, size);
    }
    return 1;
}

#ifdef __cplusplus
}
#endif
