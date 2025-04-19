/** @file PDD_Decoder.c
 *  @brief PDD_Decoder functions implementation
 *
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */
#include "PDD_Decoder.h"
#include "PDD_Types.h"

#include <stdlib.h>

#ifdef __cplusplus
namespace tdym
{
#endif


/** @brief Calculate checksum on the buffer (exclude checksum bytes from count)
*
*   @param data Pointer to the buffer of bytes
*   @param count Number of bytes to use
*
*   @returns 2-byte calculated checksum
*/
PDD_DECL unsigned short PDD_CalculateChecksum(const unsigned char* data, int count)
{
    unsigned short sum = 0;
    if (data)
    {
        int i;
        for (i = 0; i < count; i++)
        {
            sum += data[i];
        }
    }

    return sum;
}

/** @brief Verify checksum on ensemble buffer that includes checksum (last 2 bytes)
*
*   @param data Pointer to the buffer of bytes
*   @param count Number of bytes to use
*
*   @returns 1 if successful, 0 otherwise
*/
PDD_DECL int PDD_VerifyChecksum(const unsigned char* data, int count)
{
    unsigned short checksum;
    unsigned short calcChecksum;
    PDD_Header* header = (PDD_Header*)data;
    calcChecksum = PDD_CalculateChecksum(data, count - 2);
    if (count < header->checksumOffset + 1)
    {
        return 0;
    }
    checksum = *(unsigned short*)(data + header->checksumOffset);
    return (checksum == calcChecksum) ? 1 : 0;
}


/** @brief Fix checksum on ensemble buffer that includes checksum (last 2 bytes)
*
*   @param data Pointer to the buffer of bytes
*   @param count Number of bytes to use
*
*   @returns 1 if successful, 0 otherwise
*/
PDD_DECL int PDD_FixChecksum(const unsigned char* data, int count)
{
    unsigned short calcChecksum;
    PDD_Header* header = (PDD_Header*)data;
    calcChecksum = PDD_CalculateChecksum(data, count - 2);
    if (count < header->checksumOffset + 1)
    {
        return 0;
    }
    *(unsigned short*)(data + header->checksumOffset) = calcChecksum;
    return 1;
}

#ifndef DONT_USE_MALLOC
 /** @brief Create and initialize PDDecoder structure
 *
 *   @returns Pointer to PDDecoder structure
 */
PDD_DECL PDD_Decoder* PDD_CreateDecoder()
{
    PDD_Decoder* pDec = malloc(sizeof(PDD_Decoder));
    if (pDec)
    {
        PDD_InitializeDecoder(pDec);
    }
    return pDec;
}

/** @brief De-allocate memory for decoder.  Remember to set pDec to NULL after calling this function.
*/
PDD_DECL void PDD_DestroyDecoder(PDD_Decoder* pDec)
{
    if (pDec)
    {
        free(pDec);
    }
}

#endif

/** @brief Initialize decoder
*
*   @param pDec Pointer to the decoder structure
*/
PDD_DECL void PDD_InitializeDecoder(PDD_Decoder* pDec)
{
    if (pDec != 0)
    {
        memset(pDec, 0, sizeof(PDD_Decoder));
    }
}

/** @brief Internal function to move the data in the buffer
*
*   @param pDec Pointer to the decoder structure
*   @param index Position in the buffer
*/
static void InternalMoveData(PDD_Decoder* pDec, int index)
{
    if (pDec)
    {
        if (index > 0 && index < pDec->dataCount)
        {
            int i;
            for (i = index; i < pDec->dataCount; i++)
            {
                pDec->data[i - index] = pDec->data[i];
            }
        }

        pDec->dataCount -= index;
    }
}

/** @brief Adds data buffer to the decoder (call GetPD0Ensemble after until returns 0)
*
*   @param pDec Pointer to the decoder structure
*   @param data Pointer to the data buffer
*   @param count Number of bytes to be copied (must be less than MAX_DECODER_SIZE)
*
*   @returns 1 if successful, 0 otherwise
*/
PDD_DECL int PDD_AddDecoderData(PDD_Decoder* pDec, unsigned char* data, int count)
{
    if (pDec && data)
    {
        if (pDec->dataCount + count < MAX_DECODER_SIZE)
        {
            // Append data to non-processed data
            memcpy(pDec->data + pDec->dataCount, data, count);
            pDec->dataCount += count;
            return 1;
        }
        else if (count >= MAX_DECODER_SIZE)
        {
            // Not enough space in internal buffer but do the best you can
            memcpy(pDec->data, data, MAX_DECODER_SIZE);
            pDec->dataCount = MAX_DECODER_SIZE;
            return 0;
        }
    }

    return 0;
}

/** @brief Internal function to get generic ensemble from decoder
*
*   @param pDec Pointer to the decoder structure
*   @param ensembleID Each ensemble starts with ID
*   @param buffer Pointer to the buffer where ensemble data will be copied
*   @param size Buffer size
*   @param ensembleSize Returned value of found ensemble size
*
*   @returns 1 if successful, 0 otherwise
*/
static int InternalGetEnsemble(PDD_Decoder* pDec, unsigned short ensembleID, unsigned char* buffer, int size, int* ensembleSize)
{
    unsigned short ensSize;
    int idFound = 0;
    int pos = 0;
    int i = 0;
    *ensembleSize = 0;

    while (1)
    {
        unsigned short id;
        // Check if ID found
        if (!idFound)
        {
            // Check if enough data in the buffer
            if (i + PDD_MIN_ENSEMBLE_SIZE >= pDec->dataCount)
            {
                // Not enough data in the buffer - remove anything from the buffer that is not needed
                InternalMoveData(pDec, pos);
                return 0;
            }
        }

        id = *(unsigned short*)(pDec->data + i);
        if (id == ensembleID)
        {
            idFound = 1;
            pos = i;
            i += 2;
        }
        else
        {
            pos++;
            i = pos;
        }

        if (idFound)
        {
            ensSize = *(unsigned short*)(pDec->data + pos + 2) + 2;
            if (ensSize < PDD_MIN_ENSEMBLE_SIZE || ensSize > PDD_MAX_ENSEMBLE_SIZE || ensSize > size)
            {
                // Something is wrong - search for ID again
                pos++;
                i = pos;
                idFound = 0;
                continue;
            }

            if (pos + ensSize <= pDec->dataCount)
            {
                // Enough data in ensemble
                if (PDD_VerifyChecksum(pDec->data + pos, ensSize))
                {
                    memcpy(buffer, pDec->data + pos, ensSize);
                    InternalMoveData(pDec, pos + ensSize);
                    *ensembleSize = ensSize;
                    return 1;
                }
                else
                {
                    // Checksum invalid - resume the search
                    pos++;
                    i = pos;
                }
                idFound = 0;
            }
            else
            {
                // Not enough data in the buffer (remove anything not needed)
                InternalMoveData(pDec, pos);
                return 0;
            }
        }
    }
    return 0;
}

/** @brief Get PD0 ensemble (call AddDecoderData before this function)
*
*   @param pDec Pointer to the decoder structure
*   @param pEns Pointer to PD0 ensemble structure
*
*   @returns 1 if successful, 0 otherwise
*/
PDD_DECL int PDD_GetPD0Ensemble(PDD_Decoder* pDec, PDD_PD0Ensemble* pEns)
{
    int ensSize;
    int result = 0;
    if (pDec && pEns)
    {
        memset(pEns, 0, sizeof(PDD_PD0Ensemble));
        result = InternalGetEnsemble(pDec, PD0DataID, pEns->data, sizeof(pEns->data), &ensSize);
        if (result > 0)
        {
            PDD_DecodePD0(pEns->data, ensSize, pEns);
        }
    }
    return result;
}

/** @brief Get Waves ensemble packet (from special Waves packet file)
*
*   @param pDec Pointer to the decoder structure
*   @param pEns Pointer to waves ensemble structure
*
*   @returns 1 if successful, 0 otherwise
*/
PDD_DECL int PDD_GetWavesEnsemble(PDD_Decoder* pDec, PDD_WavesEnsemble* pEns)
{
    int ensSize;
    int result = 0;
    if (pDec && pEns)
    {
        memset(pEns, 0, sizeof(PDD_WavesEnsemble));
        result = InternalGetEnsemble(pDec, WavesDataID, pEns->data, sizeof(pEns->data), &ensSize);
        if (result > 0)
        {
            PDD_DecodeWaves(pEns->data, ensSize, pEns);
        }
    }
    return result;
}

#ifdef __cplusplus
}
#endif