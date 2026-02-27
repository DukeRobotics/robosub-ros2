/** @file PDD_Decoder.h
 *  @brief PDD_Decoder structure and functions definitions
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */
#ifndef PDD_DECODER_H
#define PDD_DECODER_H

#include "PDD_Definitions.h"
#include "PDD_Ensemble.h"

#ifdef __cplusplus
extern "C"
{
    namespace tdym
    {
#endif

        /** @brief Maximum decoder bytes in internal buffer
        */
#define MAX_DECODER_SIZE (65535) 

        /** @brief PD Decoder data structure
        */
        typedef struct PDD_Decoder
        {
            unsigned char data[MAX_DECODER_SIZE];               //!< Buffer to hold decoder data (size 65535)
            int dataCount;                                      //!< Number of valid bytes in decoder
        } PDD_Decoder;


        PDD_DECL unsigned short PDD_CalculateChecksum(const unsigned char* data, int count);    // Calculate checksum
        PDD_DECL int PDD_VerifyChecksum(const unsigned char* data, int count);                  // Verify checksum in ensemble
        PDD_DECL int PDD_FixChecksum(const unsigned char* data, int count);                     // Fix checksum in ensemble

#ifndef DONT_USE_MALLOC
        PDD_DECL PDD_Decoder* PDD_CreateDecoder();                                              // Create decoder
        PDD_DECL void PDD_DestroyDecoder(PDD_Decoder* pDec);                                    // Destroy decoder
#endif

        PDD_DECL void PDD_InitializeDecoder(PDD_Decoder* pDec);                                 // Initialize decoder
        PDD_DECL int PDD_AddDecoderData(PDD_Decoder* pDec, unsigned char* data, int count);     // Add data to the decoder
        PDD_DECL int PDD_GetPD0Ensemble(PDD_Decoder* pDec, PDD_PD0Ensemble* pEns);              // Get next PD0 ensemble
        PDD_DECL int PDD_GetWavesEnsemble(PDD_Decoder* pDec, PDD_WavesEnsemble* pEns);          // Get next Waves packet ensemble

#ifdef __cplusplus
    }
}
#endif

#endif 
