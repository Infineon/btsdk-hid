/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*******************************************************************************
*
* File Name: hidd_audio.c
*
* This file implements the microphone for voice using Audio ADC
*******************************************************************************/
#ifdef SUPPORT_AUDIO
#include "hidd_audio.h"
#include "string.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_event.h"
#include "wiced_bt_trace.h"

#define PCM_AUDIO_BUFFER_SIZE       120  // in 16-bit sample

// Definition of bitmap for for buffer state in hidd_voice_report_t
#define PCMBUFFSTATE_FIFO_MASK  0x0007  // 3 bits only
#define PCMBUFFSTATE_FIFO_INP_SHFT 0   // Bit 0-2
#define PCMBUFFSTATE_FIFO_OUT_SHFT 3   // Bit 3-5
#define PCMBUFFSTATE_FIFO_UNRFLOW_SHFT 6   // Bit 6-8
#define PCMBUFFSTATE_FIFO_OVRFLOW_SHFT 9   // Bit 9-11

static tMicAudio micAudioDriver;
uint32_t packetDelayCount;
uint16_t audioBufSize16; // in 16-bit sample
hidd_audio_encoding_t audioEncType;
uint8_t hidd_mic_stop_command_pending = 0;

//adc audio fifo number
#define BUFFER_ADC_AUDIO_FIFO_NUM	25

AdcAudioFifo wicedAdcAudioFifo[BUFFER_ADC_AUDIO_FIFO_NUM] = {0};
#ifdef SUPPORT_DIGITAL_MIC
AdcAudioFifo wicedAdcAudioFifo2[BUFFER_ADC_AUDIO_FIFO_NUM] = {0};
#else
#define wicedAdcAudioFifo2 NULL
#endif

#if AUDIO_DEBUG_ENABLE
#define AUDIO_DUMP_SIZE 16000
uint16_t audioDump[AUDIO_DUMP_SIZE];
uint32_t audioDumpIndex = 0;
#endif

extern void wiced_bt_allowPeripheralLatency(wiced_bool_t allow);

extern int32_t custom_gain_boost;
extern AdcAudioDrcSettings adc_audioDrcSettings;

#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
extern AdcAudioFilterCfg_t adcAudioFilterData;
#define eq_filter_enabled() (adcAudioFilterData.eqFilter.coeff[0])
#endif

#ifdef SBC_ENCODER
#include "sbc_encoder.h"
#include "sbc.h"

#define WBS_MSBC_BIT_POOL_VAL                26

SBC_ENC_PARAMS mSBCEncParams = {0};

extern SINT16 SBC_Encoder_Init(SBC_ENC_PARAMS *strEncParams);
extern uint16_t SBC_Encoder_encode(SBC_ENC_PARAMS *pstrEncParams, SINT16  * pcm_in, uint8_t * sbc_out, uint16_t len);
#endif
#ifdef CELT_ENCODER
#include "celt_encoder_api.h"

CELT_ENC_PARAMS enc_param = {0};
#endif

#ifdef ADPCM_ENCODER
#include "adpcm_codec.h"

CodecState adpcm_enc_state = {0};
#endif

#ifdef FATORY_TEST_SUPPORT
extern uint8_t factory_mode;

extern uint8_t *audio_byte_pool;
extern uint32_t audio_byte_pool_size;
extern uint32_t index_in_byte_pool;
#endif

extern void audio_drc_loop_init(UINT8 pgaGain, UINT32 sampleRate);
extern void adc_assignFilterData(AdcAudioFilterCfg_t * data);
extern void adc_stopAudio_fix(void);

extern UINT32 appUtils_cpuIntDisable(void);
extern void   appUtils_cpuIntEnable(UINT32 _newPosture);

void micAudio_micAudioCallback(UINT8 *audioData, UINT32 receivedLength, UINT32 adcFIFOStatus);

typedef struct {
    wiced_bool_t  enabled:1;
    wiced_bool_t  disable_level:1;
    uint8_t       gpio;
} pin_en_mic_t;
static pin_en_mic_t pin_en_mic={0};

#ifdef SUPPORT_DIGITAL_MIC
extern void adc_pdm_pinconfig(UINT8 ch1, UINT8 risingEdge1, UINT8 ch2, UINT8 risingEdge2, UINT8 clk);
static uint8_t gpioPDMInClk = WICED_P27;   // default PDM clk in case if app does not assign
static uint8_t gpioPDMInData = WICED_P26;   // default PDM data in case if app does not assign
#define pdm_in_ch1_rising_edge 1
#define pdm_in_ch2_rising_edge 0

////////////////////////////////////////////////////////////////////////////////
/// Configure Audio pdm pins
///
/// \param clk, data gpio pins
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void hidd_mic_assign_mic_pdm_pins(uint8_t clk, uint8_t data)
{
    extern uint8_t useWicedPDMAudio;
    useWicedPDMAudio = TRUE;

    gpioPDMInClk = clk;
    gpioPDMInData = data;
}
#endif

///////////////////////////////////////////////////////////////////////////////
/// Assgin MIC EN pin. By default is disabled until app assigns a pin
///
/// \param gpio
/// \param default level (disabled logic)
///
/// \return WICED_TRUE/WICED_FALSE
void hidd_mic_assign_mic_en_pin(uint8_t gpio, wiced_bool_t disable_level)
{
    pin_en_mic.enabled = TRUE;
    pin_en_mic.disable_level = disable_level;
    pin_en_mic.gpio = gpio;
    wiced_hal_gpio_configure_pin(gpio, (disable_level ? GPIO_PULL_UP : GPIO_PULL_DOWN) | GPIO_OUTPUT_ENABLE, disable_level);
    wiced_hal_gpio_slimboot_reenforce_cfg(gpio, GPIO_OUTPUT_ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// Configure Audio ADC for microphone
///
/// \param config - configuration
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void hidd_mic_audio_config(hidd_microphone_config_t * config)
{
    adc_config();
    adc_audioConfig(wicedAdcAudioFifo, BUFFER_ADC_AUDIO_FIFO_NUM, wicedAdcAudioFifo2);

    micAudioDriver.codec        = NULL;
    micAudioDriver.audioAdcData = config->audio_fifo;
    micAudioDriver.dataCnt      = config->data_count;
    micAudioDriver.fifo_cnt     = config->fifo_count;
    micAudioDriver.pin_en_audio = config->enable;
    micAudioDriver.audio_gain   = config->audio_gain;
    micAudioDriver.audio_boost  = config->audio_boost;
    micAudioDriver.audio_delay  = config->audio_delay;
    micAudioDriver.codec_sampling_freq = config->codec_sampling_freq;

}

///////////////////////////////////////////////////////////////////////////////
// Set DRC parameters
///////////////////////////////////////////////////////////////////////////////
void micAudio_setDrcParameters(uint8_t *drcCustomConfigBuf)
{
    memcpy(&adc_audioDrcSettings, drcCustomConfigBuf, sizeof(AdcAudioDrcSettings));
}

///////////////////////////////////////////////////////////////////////////////
/// Enhanced configure Audio ADC for microphone (DRC, eq filter etc)
///
/// \param pConfig - pointer to the configuration data
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void hidd_mic_audio_config_enhanced(uint8_t *pConfig)
{
    hidd_microphone_enhanced_config_t * pMicAudioConfiguration = (hidd_microphone_enhanced_config_t *)pConfig;

    audioEncType = pMicAudioConfiguration->audioEncType;
    if (audioEncType == HIDD_AUDIO_ENC_TYPE_PCM)
    {
        audioBufSize16 = PCM_AUDIO_BUFFER_SIZE;
    }
    else
    {
        audioBufSize16 = HIDD_MIC_AUDIO_BUFFER_SIZE;
    }

    custom_gain_boost = pMicAudioConfiguration->custom_gain_boost;  //Increase gain 3.5 dB  1.496 = 10^(3.5/20)

    micAudio_setDrcParameters((uint8_t *)&pMicAudioConfiguration->drcSettings);

#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
    adc_assignFilterData(&pMicAudioConfiguration->audioFilterData);
#endif

    audio_drc_loop_init(micAudioDriver.audio_gain, 16000);
}

////////////////////////////////////////////////////////////////////////////////
/// microphone init
///
/// \param callback - application callback function when audio data is generated and ready to send
/// \param context - pass back to the application as callback parameter as is passed in
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void  hidd_mic_audio_init(void (*callback)(void*), void* context)
{
#ifdef SBC_ENCODER
    SBC_ENC_PARAMS *p_mSBCEncParams =  &mSBCEncParams;
#endif
#ifdef CELT_ENCODER
    CELT_ENC_PARAMS* ptr_enc_param = &enc_param;
#endif

    // turn codec power off by default
    if (pin_en_mic.enabled)
    {
        wiced_hal_gpio_set_pin_output(pin_en_mic.gpio, pin_en_mic.disable_level);
    }

    micAudioDriver.userFn = callback;
    micAudioDriver.master = context;

    micAudioDriver.active = 0;


    micAudioDriver.voiceStarted = 1;
    hidd_mic_audio_stop();

#ifdef SBC_ENCODER
    //init SBC codec here
    p_mSBCEncParams->numOfSubBands = SBC_NUM_SUBBANDS_8;        //4,8
    p_mSBCEncParams->numOfBlocks = SBC_BLOCK_SIZE_15;               //4,8
    p_mSBCEncParams->allocationMethod  = SBC_ALLOC_METHOD_LOUDNESS;       //0,1
    p_mSBCEncParams->channelMode = SBC_CH_MODE_MONO;      //0,1,2,3
    p_mSBCEncParams->samplingFreq = SBC_SAMP_FREQ_16000;              //0,1,2,3
    p_mSBCEncParams->bitPool = WBS_MSBC_BIT_POOL_VAL;

    p_mSBCEncParams->sbc_mode = SBC_MODE_WB;
    //NOTE: we need to feed number of samples = numOfSubBands * numOfBlocks = 8*15=120 samples to the SBC encoder for it to function correctly.
#endif
#ifdef CELT_ENCODER
    ptr_enc_param->sampling_rate  = 16000;       //Supported sampling rates are 8000, 16000, 24000, 48000
    ptr_enc_param->channels       = 1;           // 1, 2
    ptr_enc_param->bitrate        = 32000;
    ptr_enc_param->complexity     = 3;
    ptr_enc_param->use_vbr        = 0;
    ptr_enc_param->use_cvbr       = 0;
    ptr_enc_param->frame_size     = 320;        // frame_size default to (sampling_rate / 50) = 16000/50=320 ;
#endif
}

///////////////////////////////////////////////////////////////////////////////
//  stop the audio and clear the buffer
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void hidd_mic_audio_stop()
{
    uint8_t i;

    hidd_mic_audio_set_active(0);

    if (micAudioDriver.voiceStarted)
    {
#ifdef SUPPORT_DIGITAL_MIC
        adc_stopAudio_fix();
#else
        adc_stopAudio();
#endif

        // clear all data buffer
        for (i=0; i<micAudioDriver.fifo_cnt; i++)
        {
            memset(&micAudioDriver.audioAdcData[i], 0, sizeof(hidd_voice_report_t));
            micAudioDriver.dataCnt[i] = 0;
        }

        micAudioDriver.fifoOutIndex = micAudioDriver.fifoInIndex = 0;
        micAudioDriver.voiceStarted = FALSE;
        if(hidd_mic_stop_command_pending == FALSE)
        {
            wiced_bt_allowPeripheralLatency(TRUE);
        }
    }

}

///////////////////////////////////////////////////////////////////////////////
//  start Audio ADC
///////////////////////////////////////////////////////////////////////////////
#ifdef SUPPORT_DIGITAL_MIC
#define MIC_TYPE TRUE    // PDM MIC
#else
#define MIC_TYPE FALSE   // Analog MIC
#endif
void micAudio_startAudio()
{
    wiced_bt_allowPeripheralLatency(FALSE);
    micAudioDriver.voiceStarted = 1;

//    WICED_BT_TRACE("\nmicAudio_startAudio");

    micAudioDriver.audioCounter = micAudioDriver.underflowCounter = micAudioDriver.overflowCounter = 0;

#ifdef SUPPORT_DIGITAL_MIC
    adc_pdm_pinconfig(gpioPDMInData, pdm_in_ch1_rising_edge, gpioPDMInData, pdm_in_ch2_rising_edge, gpioPDMInClk);
#endif
    if (micAudioDriver.codec_sampling_freq == HIDD_CODEC_SAMP_FREQ_8K)
    {
        packetDelayCount = micAudioDriver.audio_delay / 15;  //15 mSec per packet
        adc_startAudio(8000, 16, micAudioDriver.audio_gain, micAudio_micAudioCallback, NULL, MIC_TYPE);
    }
    else
    {
        packetDelayCount = micAudioDriver.audio_delay * 2 / 15;  //7.5 mSec per packet
        adc_startAudio(16000, 16, micAudioDriver.audio_gain, micAudio_micAudioCallback, NULL, MIC_TYPE);
    }

#ifdef FATORY_TEST_SUPPORT
    if ((factory_mode == 12) && audio_byte_pool)
    {
        packetDelayCount +=6;
    }
#endif

    micAudioDriver.voicePktTime = 0; //hiddcfa_currentNativeBtClk();

#ifdef SBC_ENCODER
    SBC_Encoder_Init(&mSBCEncParams);
#endif

#ifdef CELT_ENCODER
    CELT_Encoder_Init(&enc_param);
#endif

#ifdef ADPCM_ENCODER
    memset(&adpcm_enc_state, 0, sizeof(CodecState));
#endif

#if AUDIO_DEBUG_ENABLE
    //ADC Register Dump
    WICED_BT_TRACE("\nmia_adc_intf_ctl_adr %08x", REG32(mia_adc_intf_ctl_adr));
    WICED_BT_TRACE("\nadc_dma_ctl0_adr %08x", REG32(adc_dma_ctl0_adr));
    WICED_BT_TRACE("\nadc_dma_ctl1_adr %08x", REG32(adc_dma_ctl1_adr));
    WICED_BT_TRACE("\nlhl_adc_ctl_adr %08x", REG32(lhl_adc_ctl_adr));
    WICED_BT_TRACE("\nmia_adc_ctl0_adr %08x", REG32(mia_adc_ctl0_adr));
    WICED_BT_TRACE("\nadc_api_ctl0_adr %08x", REG32(adc_api_ctl0_adr));
    WICED_BT_TRACE("\nadc_api_ctl1_adr %08x", REG32(adc_api_ctl1_adr));
    WICED_BT_TRACE("\nadc_api_ctl2_adr %08x", REG32(adc_api_ctl2_adr));
    WICED_BT_TRACE("\nadc_api_ctl3_adr %08x", REG32(adc_api_ctl3_adr));
    WICED_BT_TRACE("\nadc_api_ctl4_adr %08x", REG32(adc_api_ctl4_adr));
    WICED_BT_TRACE("\nadc_filter_ctl0_adr %08x", REG32(adc_filter_ctl0_adr));
    WICED_BT_TRACE("\nadc_filter_ctl1_adr %08x", REG32(adc_filter_ctl1_adr));
    WICED_BT_TRACE("\nadc_filter_ctl2_adr %08x", REG32(adc_filter_ctl2_adr));
    WICED_BT_TRACE("\nadc_filter_ctl3_adr %08x", REG32(adc_filter_ctl3_adr));
    WICED_BT_TRACE("\nadc_filter_ctl4_adr %08x", REG32(adc_filter_ctl4_adr));
    WICED_BT_TRACE("\nadc_filter_ctl5_adr %08x", REG32(adc_filter_ctl5_adr));
    WICED_BT_TRACE("\nadc_filter_ctl6_adr %08x", REG32(adc_filter_ctl6_adr));
    WICED_BT_TRACE("\nadc_filter_ctl7_adr %08x", REG32(adc_filter_ctl7_adr));
    WICED_BT_TRACE("\nadc_filter_ctl8_adr %08x", REG32(adc_filter_ctl8_adr));
    WICED_BT_TRACE("\nadc_filter_ctl9_adr %08x", REG32(adc_filter_ctl9_adr));
    WICED_BT_TRACE("\nadc_filter_ctlA_adr %08x", REG32(adc_filter_ctlA_adr));
    WICED_BT_TRACE("\nadc_filter_ctlB_adr %08x", REG32(adc_filter_ctlB_adr));
    WICED_BT_TRACE("\nadc_filter_ctlC_adr %08x", REG32(adc_filter_ctlC_adr));
    WICED_BT_TRACE("\nadc_filter_ctlD_adr %08x", REG32(adc_filter_ctlD_adr));
    WICED_BT_TRACE("\nadc_filter_ctlE_adr %08x", REG32(adc_filter_ctlE_adr));
    WICED_BT_TRACE("\nadc_filter_ctlF_adr %08x", REG32(adc_filter_ctlF_adr));
    WICED_BT_TRACE("\nmia_adc_intf_ctl2_adr %08x", REG32(mia_adc_intf_ctl2_adr));
    WICED_BT_TRACE("\nmia_adc_ctl2_adr %08x", REG32(mia_adc_ctl2_adr));
    WICED_BT_TRACE("\nmia_adc_ctl1_adr %08x", REG32(mia_adc_ctl1_adr));

    WICED_BT_TRACE("\ncr_pulse_reset_peri %08x", cr_pulse_reset_peri);
    WICED_BT_TRACE("\ncr_mia_clk_cfg_adr %08x", REG32(cr_mia_clk_cfg_adr));
    WICED_BT_TRACE("\nmicAudio_startAudio");
#endif
}

///////////////////////////////////////////////////////////////////////////////
/// poll microphone for audio activity
///
/// \param dataPtr - application callback function when audio data is generated and ready to send
/// \param context - pass back to the application as callback parameter as is passed in
///
/// \return 0 - no audio activity
///           otherwise - audio activity available
///////////////////////////////////////////////////////////////////////////////
uint8_t hidd_mic_audio_poll_activity(HidEventUserDefine * dataPtr)
{
    //WICED_BT_TRACE("\nmicAudio_pollActivityUser");

    if (micAudioDriver.active == 1)
    {
        if (!micAudioDriver.voiceStarted)
        {
            micAudio_startAudio();
        }
        else if (micAudioDriver.audioAdcData[micAudioDriver.fifoOutIndex].reportId)
        {
            dataPtr->userDataPtr = &micAudioDriver.audioAdcData[micAudioDriver.fifoOutIndex];
            if (micAudioDriver.fifoInIndex == micAudioDriver.fifoOutIndex)
            {
                micAudioDriver.underflowCounter++;
            }

            if (++micAudioDriver.fifoOutIndex >= micAudioDriver.fifo_cnt)
            {
                micAudioDriver.fifoOutIndex = 0;
            }

            return (uint8_t)sizeof(hidd_voice_report_t);
        }
    }
    else
    {
        if (micAudioDriver.voiceStarted)
        {
            hidd_mic_audio_stop();
        }
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// callback to application to send audio
////////////////////////////////////////////////////////////////////////////////
int micAudio_sendAudio(void* unused)
{
    (micAudioDriver.userFn)(micAudioDriver.master);
    return 1;
}

///////////////////////////////////////////////////////////////////////////////
// Audio ADC callback Handler
///////////////////////////////////////////////////////////////////////////////
void micAudio_micAudioCallback(UINT8 *audioData, UINT32 receivedLength, UINT32 adcFIFOStatus)
{
    int index;
    uint32_t originalLength;

#ifdef FATORY_TEST_SUPPORT
    if (factory_mode)
    {
        if ((factory_mode == 12) && audio_byte_pool) //factory_mode == 12, i.e. audio capture test.
        {
            if(packetDelayCount == 0)
            {
                if (receivedLength + index_in_byte_pool > audio_byte_pool_size)
                {
                    receivedLength = audio_byte_pool_size - index_in_byte_pool;
                }

                //copy audio FIFO data to audio byte pool
                memcpy(&audio_byte_pool[index_in_byte_pool], audioData, receivedLength);
                index_in_byte_pool += receivedLength;

                //check if audio byte pool is full
                if (index_in_byte_pool == audio_byte_pool_size)
                {
                    // trigger App for activity
                    wiced_app_event_serialize(micAudio_sendAudio, NULL);
                    factory_mode = 1;
                }
            }
            else
            {
                packetDelayCount--;
            }
        }

        return;
    }
#endif

    receivedLength /= 2; //16 bits per sample

    if(packetDelayCount == 0)
    {
        while (receivedLength > 0)
        {
            originalLength = receivedLength;

            // make sure the next copy is not over buffer size
            if (receivedLength + micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] > audioBufSize16)
            {
                receivedLength = audioBufSize16 - micAudioDriver.dataCnt[micAudioDriver.fifoInIndex];
            }
            //Copy the data to our current buffer
            for(index=0; index<receivedLength; index++)
            {
                ((uint16_t *)&micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].dataBuffer[micAudioDriver.dataCnt[micAudioDriver.fifoInIndex]])[index] =
                    (audioData[2 * index + 1] << 8) + audioData[2 * index];

#if AUDIO_DEBUG_ENABLE
                audioDump[audioDumpIndex] = (audioData[2 * index + 1] << 8) + audioData[2 * index];
                audioDumpIndex++;
#endif
            }

#if AUDIO_DEBUG_ENABLE
            if(audioDumpIndex > (AUDIO_DUMP_SIZE - 400))
            {
                REG32(iocfg_fcn_p29_adr) = 0x0008;
                REG32(gpio_o_val_port1_adr) |= (1 << 13);

                //Dump the audio data to the UART
                for(index = 0; index < (AUDIO_DUMP_SIZE-400); index++)
                {
                    WICED_BT_TRACE("\n%04X", audioDump[index]);
                }

                audioDumpIndex = 0;

                REG32(gpio_o_val_port1_adr) &= ~(1 << 13);
            }
#endif

            //Read into local byte buffer with an offset of the previous length
            //Check if close to filled up local buffer
            micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] += receivedLength;
            audioData +=2*receivedLength;

            if(micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] == audioBufSize16)
            {
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].reportId  = HIDD_VOICE_REPORT_ID;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].format    = HIDD_AUDIO_DATA;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].sqn       = micAudioDriver.audioCounter++;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].ts        = 0; //(uint16_t) hiddcfa_BtClksSince(micAudioDriver.voicePktTime);

                //BitMap describing the state of the FIFO:
                // -------------------------------------------------------
                //FIFO_IN_INDEX:  Bit 0-2
                //FIFO_OUT_INDEX: Bit 3-5
                //OVERFLOW_COUNT: TBD
                //UNDERFLOW_COUNT: TBD
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state  = (micAudioDriver.fifoInIndex  & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_INP_SHFT;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state |= (micAudioDriver.fifoOutIndex & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_OUT_SHFT;
                if (((micAudioDriver.fifoInIndex+1)%micAudioDriver.fifo_cnt) == micAudioDriver.fifoOutIndex)
                {
                    micAudioDriver.overflowCounter++;
                }
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state |= (micAudioDriver.underflowCounter & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_UNRFLOW_SHFT;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state |= (micAudioDriver.overflowCounter  & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_OVRFLOW_SHFT;

                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].dataCnt   = micAudioDriver.dataCnt[micAudioDriver.fifoInIndex];
                if (++micAudioDriver.fifoInIndex >= micAudioDriver.fifo_cnt)
                {
                    micAudioDriver.fifoInIndex = 0;
                }

                // start fresh
                micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] = 0;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].reportId = 0;

                if(adc_audioDrcSettings.enable == 1)
                {   //FW DRC and EQ filtering already serialize it to MPAF thread
                    micAudio_sendAudio(NULL);
                }
                else
                {
                    wiced_app_event_serialize(micAudio_sendAudio, NULL);
                }
            }

            receivedLength = originalLength - receivedLength;
        }
    }
    else
    {
        packetDelayCount--;
    }
}


///////////////////////////////////////////////////////////////////////////////
/// set audio to active/inactive
///
/// \param active - active/inactive
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void hidd_mic_audio_set_active(wiced_bool_t active)
{
    if (micAudioDriver.active == active)
        return;

    micAudioDriver.active = active;

    if (pin_en_mic.enabled)
    {
        wiced_hal_gpio_set_pin_output(pin_en_mic.gpio, active ? !pin_en_mic.disable_level : pin_en_mic.disable_level);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// get audio codec
///
/// \param none
///
/// \return pointer point to external audio codec
///////////////////////////////////////////////////////////////////////////////
wiced_audio_codec_interface_func_tbl *hidd_mic_audio_getCodec(void)
{
    return micAudioDriver.codec;
}

////////////////////////////////////////////////////////////////////////////////
/// read audio codec settings
///
/// \param CntrlReport - pointer to the audio codec settings data read from audio codec
///
/// \return WICED_TRUE/WICED_FALSE
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_mic_audio_read_codec_setting(hidd_voice_control_report_t * CntrlReport)
{
    uint8_t op_success = WICED_TRUE;

    switch (CntrlReport->rsvd)
    {
        case HIDD_CODEC_SR:
            {
                    CntrlReport->dataCnt = 2;
                    if (micAudioDriver.codec_sampling_freq == HIDD_CODEC_SAMP_FREQ_8K)
                        CntrlReport->dataBuffer[0] = 0x20;      //sampling Frequency 8kHz
                    else
                        CntrlReport->dataBuffer[0] = 0x40;      //sampling Frequency 16kHz
                    CntrlReport->dataBuffer[1] = 0;
            }
            break;

        case HIDD_CODEC_PCM:
            {
                    CntrlReport->dataCnt = 2;
                    CntrlReport->dataBuffer[0] = 0;         //sample format is Linear PCM
                    CntrlReport->dataBuffer[1] = 0;
            }
            break;

        case HIDD_CODEC_BPS:
            {
                    CntrlReport->dataCnt = 2;
                    CntrlReport->dataBuffer[0] = 0x10;     //Bit per Sample 16bits
                    CntrlReport->dataBuffer[1] = 0;
            }
            break;

        case HIDD_CODEC_PGA:
            {
                    INT16 temp = adcConfig.adc_ctl2_reg.bitmap_adc_ctl2.micPgaGainCtl;
                    CntrlReport->dataCnt = 2;
                    CntrlReport->dataBuffer[0] = (uint8_t) (temp & 0x00FF);
                    CntrlReport->dataBuffer[1] = (uint8_t) (temp & 0xFF00)>>8;
            }
            break;

        case HIDD_CODEC_HPF:
            {
                    uint16_t temp = adcConfig.adc_filter_ctl1_reg.bitmap_adc_filter_ctl1.auxAdchpfNum;
                    CntrlReport->dataCnt = 2;
                    CntrlReport->dataBuffer[0] = (uint8_t) (temp & 0x00FF);
                    CntrlReport->dataBuffer[1] = (uint8_t) (temp & 0xFF00)>>8;
            }
            break;

        default:
            op_success = WICED_FALSE;
            break;

    }

    return (op_success);
}

////////////////////////////////////////////////////////////////////////////////
/// Configure codec sampling frequency
///
/// \param freq -- see sample_freq_e
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void hidd_mic_set_codec_sampling_freq(uint8_t freq)
{
    micAudioDriver.codec_sampling_freq = freq;
}

///////////////////////////////////////////////////////////////////////////////
/// write audio codec settings
///
/// \param codec_param - see #hidd_audio_codec_param_e
/// \param dataBuffer - pointer to the settings that needs to write to the audio codec
///
/// \return WICED_TRUE/WICED_FALSE
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_mic_audio_write_codec_setting(uint8_t codec_param, uint8_t *dataBuffer )
{
    uint8_t op_success = WICED_FALSE;

#if 0
        switch (codec_param)
        {
            case HIDD_CODEC_SR:
                op_success = codec->writeSR(dataBuffer[0]);
                break;

            case HIDD_CODEC_PCM:
                op_success = codec->writePCM(dataBuffer[0]);
                break;

            case HIDD_CODEC_BPS:
                op_success = codec->writeBPS(dataBuffer[0]);
                break;

            case HIDD_CODEC_PGA:
                op_success = codec->writePGA( dataBuffer[0] | (dataBuffer[1] << 8) );
                break;

            case HIDD_CODEC_HPF:
                op_success = codec->writeHPF( dataBuffer[0] | (dataBuffer[1] << 8)  );
                break;

            default:
                break;

        }
#endif
        return (op_success);
}

///////////////////////////////////////////////////////////////////////////////
/// is audio active?
///
/// \param none
///
/// \return WICED_TRUE/WICED_FALSE
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_mic_audio_is_active(void)
{
    return ((micAudioDriver.active || micAudioDriver.voiceStarted) ? WICED_TRUE : WICED_FALSE);
}


///////////////////////////////////////////////////////////////////////////////
/// get the total audio FIFO number
///
/// \param none
///
/// \return the total audio FIFO number
///////////////////////////////////////////////////////////////////////////////
uint8_t hidd_mic_audio_FIFO_count(void)
{
    return micAudioDriver.fifo_cnt;
}

///////////////////////////////////////////////////////////////////////////////
/// increase internal audio overflow counter
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void hidd_mic_audio_overflow(void)
{
    //Because interrupt could also update the overflow flag , we need to ensure no race condition here.
    uint32_t oldPosture = appUtils_cpuIntDisable();

    micAudioDriver.overflowCounter++;

    // NOTE: Don't forget to re-enable interrupts.
    appUtils_cpuIntEnable(oldPosture);
}

///////////////////////////////////////////////////////////////////////////////
/// is audio overflow for the current session?
///
/// \param none
///
/// \return 0 - no overflow. otherwise indicates the number of lost audio packages
///////////////////////////////////////////////////////////////////////////////
uint16_t hidd_mic_audio_is_overflow(void)
{
    return micAudioDriver.overflowCounter;
}

///////////////////////////////////////////////////////////////////////////////
/// get audio output data to send out
///
/// \param audio_In - pointer to audio activity data
/// \param audio_outData - pointer to the audio output data
///
/// \return the length (in byte) of the audio output data. If 0, no output data
///////////////////////////////////////////////////////////////////////////////
uint16_t hidd_mic_audio_get_audio_out_data(hidd_voice_report_t *audio_In, uint8_t *audio_outData)
{
    uint16_t len = 0;
    uint16_t audio_outData_len = 0;
    hidd_voice_report_t   outRpt = {0, };


    //NOTE: Must disable interrupts before sending audio.
    //Because interrupt is also updating the audio data in FIFO, we need to ensure no race condition here (i.e. send out the audio data that is partially new and partially old.)
    uint32_t oldPosture = appUtils_cpuIntDisable();

    if (audio_In->reportId)
    {
        // Copy the data into the buffer
        //memcpy(&outRpt, audio_In, (BRCM_VoiceReport_HeaderBytes+2*audio_In->dataCnt));
        memcpy(&outRpt, audio_In, sizeof(hidd_voice_report_t));

        // NOTE: Don't forget to re-enable interrupts.
        appUtils_cpuIntEnable(oldPosture);

        //if more voice events in the event queue than FIFO_CNT, the audio data inside older voice events is overwriten and out of sequence. Don't send.
        //if (app.audioPacketInQueue < audio->audioFIFOCnt())
        {
            if (HIDD_AUDIO_ENC_TYPE_PCM == audioEncType)
            {
                //first byte is sequence number
                audio_outData[0] = outRpt.sqn & 0xFF;
                //followed by PCM raw audio data
                memcpy(&audio_outData[1], outRpt.dataBuffer, PCM_AUDIO_BUFFER_SIZE*2);

                audio_outData_len = 1 + PCM_AUDIO_BUFFER_SIZE*2;
            }
#ifdef CELT_ENCODER
            else if (HIDD_AUDIO_ENC_TYPE_CELT == audioEncType)
            {
                //first byte is sequence number
                audio_outData[0] = outRpt.sqn & 0xFF;

                //followed by CELT encoded audio data
                enc_param.pcmBuffer = outRpt.dataBuffer;

                enc_param.packet = &audio_outData[1];
                len = CELT_Encoder(&enc_param); //encode one frame

                audio_outData_len = 1 + len;
            }
#endif
#ifdef ADPCM_ENCODER
            else if (HIDD_AUDIO_ENC_TYPE_ADPCM == audioEncType)
            {
                //set header first, then encode. so "Prev pred" and "index" reflect the correct value

                //header is big endian. i.e. 0x0001 => "00 01". Used by Android TV
                audio_outData[0] = (outRpt.sqn >> 8) & 0xFF;  //2 bytes seq number
                audio_outData[1] = outRpt.sqn & 0xFF;
                audio_outData[2] = 0; //Id. <reserved>
                audio_outData[3] = (adpcm_enc_state.valprev >> 8) & 0xFF; //2 bytes Prev pred
                audio_outData[4] = adpcm_enc_state.valprev & 0xFF;
                audio_outData[5] = adpcm_enc_state.index & 0xFF;

                encode(&adpcm_enc_state, outRpt.dataBuffer, HIDD_MIC_AUDIO_BUFFER_SIZE, &audio_outData[6]);

                audio_outData_len = 134;  //6 bytes header + 128 bytes ADPCM data
            }
#endif
#ifdef SBC_ENCODER
            else
            {
#ifdef ATT_MTU_SIZE_180
                //first byte is sequence number
                audio_outData[0] = outRpt.sqn & 0xFF;

                //followed by mSBC encoded audio data
                len = SBC_Encoder_encode(&mSBCEncParams, (SINT16 *)outRpt.dataBuffer, &audio_outData[1], outRpt.dataCnt);
                audio_outData_len = 1 + len;
#else
                //Add 2 bytes header and 1 byte padding
                audio_outData[0] = 0x01;

                if (outRpt.sqn % 4 == 0)
                {
                    audio_outData[1]=0x08;
                }
                else if (outRpt.sqn % 4 == 1)
                {
                    audio_outData[1]=0x38;
                }
                else if (outRpt.sqn % 4 == 2)
                {
                    audio_outData[1]=0xC8;
                }
                else
                {
                    audio_outData[1]=0xF8;
                }

                //followed by mSBC encoded audio data
                len = SBC_Encoder_encode(&mSBCEncParams, (SINT16 *)outRpt.dataBuffer, &audio_outData[2], outRpt.dataCnt);
                //Add 1 byte padding
                audio_outData[len+2] = 0;
                audio_outData_len = 3 + len;
#endif
            }
#endif
        }
    }
    else
    {
        // NOTE: Don't forget to re-enable interrupts.
        appUtils_cpuIntEnable(oldPosture);
    }
    return audio_outData_len;
}
#endif
