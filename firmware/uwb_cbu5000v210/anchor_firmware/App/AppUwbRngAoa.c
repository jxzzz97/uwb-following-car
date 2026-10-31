/**
 * @file    AppUwbRngAoa.c
 * @brief   Responder Fixed (Debug Enabled)
 */
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "AppUwbRngAoa.h"
#include "CB_Algorithm.h"
#include "CB_timer.h"
#include "CB_scr.h"
#include "NonLIB_sharedUtils.h"
#include "CB_uwbframework.h"
#include "CB_aoa.h"

//-------------------------------
// CONFIGURATION SECTION
//-------------------------------
#define DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER   APP_FALSE
#define APP_UWB_RNGAOA_UARTPRINT_ENABLE APP_TRUE

#if (APP_UWB_RNGAOA_UARTPRINT_ENABLE == APP_TRUE)
  #include "app_uart.h"
  #define app_uwb_rngaoa_print(...) app_uart_printf(__VA_ARGS__)
#else
  #define app_uwb_rngaoa_print(...)
#endif

//-------------------------------
// DEFINE SECTION
//-------------------------------
  #define DEF_RNGAOA_ENABLE_LOG           APP_TRUE
#define DEF_RNGAOA_PDOA_CAL_LOG         APP_FALSE
#define DEF_RESPONDER_RANGING_BIAS     0

#define DEF_SYNC_RX_PAYLOAD_SIZE       4
#define DEF_SYNC_ACK_TX_PAYLOAD_SIZE   3
#define DEF_PDOA_PD01_BIAS              (-82.91f)
#define DEF_PDOA_PD02_BIAS              (-5.81f) 
#define DEF_PDOA_PD12_BIAS              (77.09f)

typedef enum{
  EN_APP_STATE_IDLE = 0,
  EN_APP_STATE_SYNC_RECEIVE,
  EN_APP_STATE_SYNC_WAIT_RX_DONE,
  EN_APP_STATE_SYNC_TRANSMIT,
  EN_APP_STATE_SYNC_WAIT_TX_DONE,
  EN_APP_STATE_DSTWR_RECEIVE_POLL,
  EN_APP_STATE_DSTWR_RECEIVE_POLL_WAIT_RX_DONE,
  EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE,
  EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE_WAIT_TX_DONE,
  EN_APP_STATE_DSTWR_RECEIVE_FINAL,
  EN_APP_STATE_DSTWR_RECEIVE_FINAL_WAIT_RX_DONE,
  EN_APP_STATE_PDOA_PREPARE,
  EN_APP_STATE_PDOA_RECEIVE,   
  EN_APP_STATE_PDOA_WAIT_RX_DONE,
  EN_APP_STATE_PDOA_POSTINGPROCESSING,  
  EN_APP_STATE_RESULT_TRANSMIT,
  EN_APP_STATE_RESULT_WAIT_TX_DONE,
  EN_APP_STATE_RECEIVE_DIST,
  EN_APP_STATE_RECEIVE_DIST_WAIT_RX_DONE,
  EN_APP_STATE_TERMINATE,
} app_uwbrngaoa_state_en;

typedef struct
{
  volatile uint8_t TxDone;
  volatile uint8_t Rx0SfdDetected;
  volatile uint8_t Rx0Done;
  volatile uint8_t Rx1SfdDetected;
  volatile uint8_t Rx2SfdDetected;
}app_uwbrngaoa_irqstatus_st;

typedef struct
{
  cb_uwbframework_rangingdatacontainer_st rangingDataContainer;
  cb_uwbframework_pdoadatacontainer_st pdoaDataContainer;  
} app_rngaoa_responderdatacontainer_st;

static volatile uint8_t s_applicationTimeout = APP_FALSE;
static cb_uwbsystem_rx_dbb_config_st s_stRxCfg_CfoGainBypass  = { 0 };

static cb_uwbsystem_packetconfig_st s_stUwbPacketConfig = 
{
  .prfMode            = EN_PRF_MODE_BPRF_62P4,                 
  .psduDataRate       = EN_PSDU_DATA_RATE_6P81,           
  .bprfPhrDataRate    = EN_BPRF_PHR_DATA_RATE_0P85,       
  .preambleCodeIndex  = EN_UWB_PREAMBLE_CODE_IDX_9,       
  .preambleDuration   = EN_PREAMBLE_DURATION_64_SYMBOLS,  
  .sfdId              = EN_UWB_SFD_ID_2,                  
  .phrRangingBit      = 0x00,                             
  .rframeConfig       = EN_RFRAME_CONFIG_SP0,             
  .stsLength          = EN_STS_LENGTH_64_SYMBOLS,         
  .numStsSegments     = EN_NUM_STS_SEGMENTS_1,            
  .stsKey             = {0x14EB220FUL,0xF86050A8UL,0xD1D336AAUL,0x14148674UL},  
  .stsVUpper          = {0xD37EC3CAUL,0xC44FA8FBUL,0x362EEB34UL},               
  .stsVCounter        = 0x1F9A3DE4UL,                                           
  .macFcsType         = EN_MAC_FCS_TYPE_CRC16,            
};

static app_uwbrngaoa_irqstatus_st s_stIrqStatus;
static volatile app_uwbrngaoa_state_en s_enAppRngaoaState   = EN_APP_STATE_IDLE;
static volatile app_uwbrngaoa_state_en s_enAppRngAoaFailureState = EN_APP_STATE_IDLE;

static uint8_t s_syncExpectedRxPayload[DEF_SYNC_RX_PAYLOAD_SIZE] = {0x53,0x59,0x4E,0x43};
static uint8_t s_syncAckPayload[DEF_SYNC_ACK_TX_PAYLOAD_SIZE]    = {0x41,0x43,0x4B};
static cb_uwbsystem_txpayload_st s_stResultTxPayload;

static cb_uwbsystem_tx_tsutimestamp_st  s_stTxTsuTimestamp0;
static cb_uwbsystem_rx_tsutimestamp_st  s_stRxTsuTimestamp0;
static cb_uwbsystem_rx_tsutimestamp_st  rxTsuTimestamp1;

static uint8_t                            s_countOfPdoaScheduledRx   = 0;
static cb_uwbsystem_rx_signalinfo_st    s_stRssiResults            = {0};
static cb_uwbsystem_pdoaresult_st         s_stPdoaOutputResult       = {0};
static float                              s_pd01Bias                 = DEF_PDOA_PD01_BIAS;
static float                              s_pd02Bias                 = DEF_PDOA_PD02_BIAS;
static float                              s_pd12Bias                 = DEF_PDOA_PD12_BIAS;
static float                              s_aziResult                = 0.0f;
static float                              s_eleResult                = 0.0f;
static float                              s_receivedDistance         = 0.0f;
static uint32_t s_appCycleCount = 0; 
static uint32_t s_syncRestartCount = 0;
static float s_lastValidAziResult = 0.0f;
static uint8_t s_lastValidAziReady = APP_FALSE;
static uint8_t s_aziSpikeHoldCount = 0;

static cb_uwbframework_trx_scheduledconfig_st s_stDstwrTreply1Config = {
  .eventTimestampMask   = EN_UWBEVENT_TIMESTAMP_MASK_0,
  .eventIndex           = EN_UWBEVENT_17_RX0_SFD_DET,
  .absTimer             = EN_UWB_ABSOLUTE_TIMER_0,
  .timeoutValue         = 700,
  .eventCtrlMask        = EN_UWBCTRL_TX_START_MASK,
};

static cb_uwbframework_trx_scheduledconfig_st s_stDstwrTround2Config = {
  .eventTimestampMask   = EN_UWBEVENT_TIMESTAMP_MASK_0,
  .eventIndex           = EN_UWBEVENT_28_TX_DONE,
  .absTimer             = EN_UWB_ABSOLUTE_TIMER_0,
  .timeoutValue         = 500,
  .eventCtrlMask        = EN_UWBCTRL_RX0_START_MASK,
};

static app_rngaoa_responderdatacontainer_st s_stResponderDataContainer = 
{
  .rangingDataContainer = { .dstwrRangingBias   = DEF_RESPONDER_RANGING_BIAS,
                            .dstwrTroundTreply  = {0}},
  .pdoaDataContainer    = { .rx0_rx1 = 0.0f,
                            .rx0_rx2 = 0.0f,
                            .rx1_rx2 = 0.0f,
                            .elevationEst = 0.0f,
                            .azimuthEst = 0.0f }
};
  
// Overall RNGAOA timeout; keep some margin above the 50ms app cycle to avoid
// spurious terminate-on-timeout when the end-to-end exchange stretches slightly.
#define DEF_RNGAOA_OVERALL_PROCESS_TIMEOUT_MS    70
// ???????????????????????????????????????
#define DEF_RNGAOA_SYNC_RX_RESTART_TIMEOUT_MS    4
#define DEF_RNGAOA_APP_CYCLE_TIME_MS             50
#define DEF_DSTWR_RESPONSE_WAIT_TIME_MS          1
#define DEF_DSTWR_FINAL_WAIT_TIME_MS             0
#define DEF_DSTWR_RX_SOFT_TIMEOUT_MS             8
#define DEF_DSTWR_TX_SOFT_TIMEOUT_MS             8
#define DEF_NUMBER_OF_PDOA_REPEATED_RX           DEF_PDOA_NUMPKT_SUPERFRAME_MAX
#define DEF_PDOA_SFD_SOFT_TIMEOUT_MS             8
#define DEF_RNGAOA_RESULT_WAIT_TIME_MS           1
#define DEF_RNGAOA_WAIT_DIST_RX_TIMEOUT_MS       20
#define DEF_AZI_SPIKE_HOLD_DEG                   60.0f
#define DEF_AZI_SPIKE_HOLD_MAX_FRAMES            2u
  
void    app_rngaoa_reset                        (void);
void    app_rngaoa_timeout_error_message_print  (void);
uint8_t app_rngaoa_validate_sync_payload        (void);
void    app_rngaoa_log                          (void);
static void app_rngaoa_cleanup_trx              (void);
static void app_rngaoa_soft_timeout             (app_uwbrngaoa_state_en failureState);
static void app_rngaoa_update_azi_with_hold     (float candidateAzi);

void app_rngaoa_responder(void)
{  
  uint32_t startTime      = 0;
  uint32_t iterationTime  = 0;  
  
  cb_framework_uwb_init();
  
  cb_uwbsystem_txpayload_st stSyncAckPayloadPack = {0};
  cb_uwbsystem_txpayload_st stDstwrTxPayloadPack = {0};  
  stSyncAckPayloadPack.ptrAddress    = &s_syncAckPayload[0];
  stSyncAckPayloadPack.payloadSize   = sizeof(s_syncAckPayload);
  static uint8_t s_dstwrPayload[1]  = {0x1};
  stDstwrTxPayloadPack.ptrAddress   = &s_dstwrPayload[0];
  stDstwrTxPayloadPack.payloadSize  = sizeof(s_dstwrPayload);  

  cb_uwbsystem_tx_irqenable_st stTxIrqEnable = { CB_FALSE} ;
  stTxIrqEnable.txDone  = APP_TRUE;
  cb_uwbsystem_rx_irqenable_st stRxIrqEnable = { CB_FALSE } ;
  stRxIrqEnable.rx0Done = APP_TRUE;  
  
  cb_uwbsystem_rx_irqenable_st stPdoaRxIrqEnable = { CB_FALSE };   
  stPdoaRxIrqEnable.rx0SfdDetDone = CB_TRUE;  
  stPdoaRxIrqEnable.rx1SfdDetDone = CB_TRUE;  
  stPdoaRxIrqEnable.rx2SfdDetDone = CB_TRUE;  
  
  s_enAppRngaoaState = EN_APP_STATE_SYNC_RECEIVE;
  
  while(1)
  {
    if (s_applicationTimeout && s_enAppRngaoaState != EN_APP_STATE_TERMINATE)
    {
      s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
    }

    switch (s_enAppRngaoaState)
    {
      case EN_APP_STATE_IDLE:
        if (cb_hal_is_time_elapsed(iterationTime, DEF_RNGAOA_APP_CYCLE_TIME_MS))
          s_enAppRngaoaState = EN_APP_STATE_SYNC_RECEIVE;
        break;

      case EN_APP_STATE_SYNC_RECEIVE:
        cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_SYNC_WAIT_RX_DONE;
        startTime = cb_hal_get_tick();
        break;
      case EN_APP_STATE_SYNC_WAIT_RX_DONE:
        if (cb_hal_is_time_elapsed(startTime, DEF_RNGAOA_SYNC_RX_RESTART_TIMEOUT_MS))
        {
          s_syncRestartCount++;
          if ((s_syncRestartCount % 50u) == 0u)
          {
            app_uwb_rngaoa_print("WAIT SYNC... (%lu)\n", (unsigned long)s_syncRestartCount);
          }
          s_enAppRngaoaState = EN_APP_STATE_SYNC_RECEIVE;
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
        }
        else if (s_stIrqStatus.Rx0Done == APP_TRUE)
        {
          s_stIrqStatus.Rx0Done = APP_FALSE;
          s_stRssiResults = cb_framework_uwb_get_rx_rssi(EN_UWB_RX_0);
          s_syncRestartCount = 0;
          if (app_rngaoa_validate_sync_payload() == APP_TRUE) s_enAppRngaoaState = EN_APP_STATE_SYNC_TRANSMIT;
          else s_enAppRngaoaState = EN_APP_STATE_SYNC_RECEIVE;
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
        }
        break;

      case EN_APP_STATE_SYNC_TRANSMIT:
        cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &stSyncAckPayloadPack, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_SYNC_WAIT_TX_DONE;        
        break;
      case EN_APP_STATE_SYNC_WAIT_TX_DONE:
        if (s_stIrqStatus.TxDone == APP_TRUE)
        {
          s_stIrqStatus.TxDone = APP_FALSE;
          cb_framework_uwb_tx_end();
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_RECEIVE_POLL; 
        }          
        break;

      case EN_APP_STATE_DSTWR_RECEIVE_POLL:
        app_rngaoa_timer_init(DEF_RNGAOA_OVERALL_PROCESS_TIMEOUT_MS);  
        #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
        cb_framework_uwb_enable_scheduled_trx(s_stDstwrTreply1Config);
        #endif
        cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_DSTWR_RECEIVE_POLL_WAIT_RX_DONE;
        startTime = cb_hal_get_tick();
        break;
      case EN_APP_STATE_DSTWR_RECEIVE_POLL_WAIT_RX_DONE:
        if (s_stIrqStatus.Rx0Done == APP_TRUE)
        {        
          s_stIrqStatus.Rx0Done = APP_FALSE;
          cb_framework_uwb_get_rx_tsu_timestamp(&s_stRxTsuTimestamp0, EN_UWB_RX_0);
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE;
          startTime = cb_hal_get_tick();
        }
        else if (cb_hal_is_time_elapsed(startTime, DEF_DSTWR_RX_SOFT_TIMEOUT_MS))
        {
          app_rngaoa_soft_timeout(EN_APP_STATE_DSTWR_RECEIVE_POLL_WAIT_RX_DONE);
        }
        break;
           
      case EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE:
        #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_enable_scheduled_trx(s_stDstwrTround2Config);
          cb_framework_uwb_configure_scheduled_trx(s_stDstwrTreply1Config);
          cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &stDstwrTxPayloadPack, &stTxIrqEnable, EN_TRX_START_DEFERRED);
        #else
          if (!cb_hal_is_time_elapsed(startTime, DEF_DSTWR_RESPONSE_WAIT_TIME_MS)) break;
          cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &stDstwrTxPayloadPack, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
        #endif
        s_enAppRngaoaState = EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE_WAIT_TX_DONE;
        startTime = cb_hal_get_tick();
        break;
      case EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE_WAIT_TX_DONE:
        if (s_stIrqStatus.TxDone == APP_TRUE)
        {
          s_stIrqStatus.TxDone = APP_FALSE;
          #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_configure_scheduled_trx(s_stDstwrTround2Config);
          #endif
          cb_framework_uwb_get_tx_tsu_timestamp(&s_stTxTsuTimestamp0);
          cb_framework_uwb_tx_end();
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_RECEIVE_FINAL;
          startTime = cb_hal_get_tick();
        }
        else if (cb_hal_is_time_elapsed(startTime, DEF_DSTWR_TX_SOFT_TIMEOUT_MS))
        {
          app_rngaoa_soft_timeout(EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE_WAIT_TX_DONE);
        }
        break;
        
      case EN_APP_STATE_DSTWR_RECEIVE_FINAL:
        #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_DEFERRED);
        #else
          if (!cb_hal_is_time_elapsed(startTime, DEF_DSTWR_FINAL_WAIT_TIME_MS)) break;
          cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_NON_DEFERRED);
        #endif
        s_enAppRngaoaState = EN_APP_STATE_DSTWR_RECEIVE_FINAL_WAIT_RX_DONE;
        startTime = cb_hal_get_tick();
        break;
      case EN_APP_STATE_DSTWR_RECEIVE_FINAL_WAIT_RX_DONE:
        if (s_stIrqStatus.Rx0Done == APP_TRUE)
        {          
          s_stIrqStatus.Rx0Done = APP_FALSE;
          #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_disable_scheduled_trx(s_stDstwrTround2Config);
          #endif
          cb_framework_uwb_get_rx_tsu_timestamp(&rxTsuTimestamp1, EN_UWB_RX_0);
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
          s_enAppRngaoaState = EN_APP_STATE_PDOA_PREPARE;
        }
        else if (cb_hal_is_time_elapsed(startTime, DEF_DSTWR_RX_SOFT_TIMEOUT_MS))
        {
          app_rngaoa_soft_timeout(EN_APP_STATE_DSTWR_RECEIVE_FINAL_WAIT_RX_DONE);
        }
        break;

      case EN_APP_STATE_PDOA_PREPARE:
        s_stRxCfg_CfoGainBypass.stRxGain = (cb_uwbsystem_rx_dbb_gain_st){.enableBypass=APP_TRUE, .gainValue=s_stRssiResults.gainIdx};
        s_stRxCfg_CfoGainBypass.stRxCfo = (cb_uwbsystem_rx_dbb_cfo_st){.enableBypass=APP_TRUE, .cfoValue=s_stRssiResults.cfoEst};
        cb_framework_uwb_rxconfig_cfo_gain(EN_UWB_CFO_GAIN_SET, &s_stRxCfg_CfoGainBypass);
        s_enAppRngaoaState = EN_APP_STATE_PDOA_RECEIVE;
        break; 
      case EN_APP_STATE_PDOA_RECEIVE:
        cb_framework_uwb_rx_start(EN_UWB_RX_ALL, &s_stUwbPacketConfig, &stPdoaRxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_PDOA_WAIT_RX_DONE;
        startTime = cb_hal_get_tick();
        break;
      case EN_APP_STATE_PDOA_WAIT_RX_DONE:
        if (s_stIrqStatus.Rx0SfdDetected && s_stIrqStatus.Rx1SfdDetected && s_stIrqStatus.Rx2SfdDetected)
        {
          s_stIrqStatus.Rx0SfdDetected = APP_FALSE; s_stIrqStatus.Rx1SfdDetected = APP_FALSE; s_stIrqStatus.Rx2SfdDetected = APP_FALSE;
          cb_framework_uwb_pdoa_store_cir_data(s_countOfPdoaScheduledRx);          
          s_countOfPdoaScheduledRx++;
          if (s_countOfPdoaScheduledRx < DEF_NUMBER_OF_PDOA_REPEATED_RX) {
            cb_framework_uwb_rx_restart(EN_UWB_RX_ALL, &s_stUwbPacketConfig, &stPdoaRxIrqEnable, EN_TRX_START_NON_DEFERRED);
            startTime = cb_hal_get_tick();
          }
          else {
            cb_framework_uwb_rx_end(EN_UWB_RX_ALL);    
            s_countOfPdoaScheduledRx = 0;
            cb_framework_uwb_rxconfig_cfo_gain(EN_UWB_CFO_GAIN_RESET, NULL);
            s_enAppRngaoaState = EN_APP_STATE_PDOA_POSTINGPROCESSING;
          }
        }
        else if (cb_hal_is_time_elapsed(startTime, DEF_PDOA_SFD_SOFT_TIMEOUT_MS))
        {
          app_rngaoa_soft_timeout(EN_APP_STATE_PDOA_WAIT_RX_DONE);
        }
        break;
        
      case EN_APP_STATE_PDOA_POSTINGPROCESSING:
        cb_framework_uwb_pdoa_calculate_result(&s_stPdoaOutputResult,EN_PDOA_3D_CALTYPE, DEF_NUMBER_OF_PDOA_REPEATED_RX);
        /* 只用 PD12 (RX1-RX2 水平天线对) 算方位角，规避俯仰角串扰 */
        {
          float pd12_raw = s_stPdoaOutputResult.median.rx1_rx2;
          float pd12_comp = pd12_raw - s_pd12Bias;
          /* wrap to [-180, 180] */
          pd12_comp = fmodf(pd12_comp + 180.0f, 360.0f);
          if (pd12_comp < 0.0f) {
            pd12_comp += 360.0f;
          }
          pd12_comp -= 180.0f;
          /* path_diff = pd12 / 360 * lambda,  lambda = 3.753 cm (ch9) */
          float path_diff_cm = pd12_comp / 360.0f * 3.753f;
          /* azi = arcsin(path_diff / d),  d = 1.88 cm (天线物理间距) */
          float sinVal = path_diff_cm / 1.88f;
          if (sinVal > 1.0f) sinVal = 1.0f;
          if (sinVal < -1.0f) sinVal = -1.0f;
          app_rngaoa_update_azi_with_hold(asinf(sinVal) * 57.29577951f);  /* 180/PI */
          s_eleResult = 0.0f;
        }
        s_enAppRngaoaState = EN_APP_STATE_RESULT_TRANSMIT;
        startTime = cb_hal_get_tick();
        break;

      case EN_APP_STATE_RESULT_TRANSMIT:
        if (cb_hal_is_time_elapsed(startTime, DEF_RNGAOA_RESULT_WAIT_TIME_MS))
        {
          cb_framework_uwb_calculate_responder_tround_treply(&s_stResponderDataContainer.rangingDataContainer, s_stTxTsuTimestamp0, s_stRxTsuTimestamp0, rxTsuTimestamp1);
          s_stResponderDataContainer.pdoaDataContainer.rx0_rx1      = s_stPdoaOutputResult.median.rx0_rx1;
          s_stResponderDataContainer.pdoaDataContainer.rx0_rx2      = s_stPdoaOutputResult.median.rx0_rx2;
          s_stResponderDataContainer.pdoaDataContainer.rx1_rx2      = s_stPdoaOutputResult.median.rx1_rx2;
          s_stResponderDataContainer.pdoaDataContainer.azimuthEst   = s_aziResult;
          s_stResponderDataContainer.pdoaDataContainer.elevationEst = s_eleResult;
          s_stResultTxPayload.ptrAddress = (uint8_t*)(&s_stResponderDataContainer);
          s_stResultTxPayload.payloadSize = sizeof(s_stResponderDataContainer);
          
          cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &s_stResultTxPayload, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
          s_enAppRngaoaState = EN_APP_STATE_RESULT_WAIT_TX_DONE;
        }
        break;
      case EN_APP_STATE_RESULT_WAIT_TX_DONE:
        if (s_stIrqStatus.TxDone == APP_TRUE)
        {
          s_stIrqStatus.TxDone = APP_FALSE;
          cb_framework_uwb_tx_end();
          s_enAppRngaoaState = EN_APP_STATE_RECEIVE_DIST;
        }
        break;
      
      case EN_APP_STATE_RECEIVE_DIST:
        cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_RECEIVE_DIST_WAIT_RX_DONE;
        startTime = cb_hal_get_tick(); 
        break;

      case EN_APP_STATE_RECEIVE_DIST_WAIT_RX_DONE:
        if (cb_hal_is_time_elapsed(startTime, DEF_RNGAOA_WAIT_DIST_RX_TIMEOUT_MS)) {
           cb_framework_uwb_rx_end(EN_UWB_RX_0);
           s_applicationTimeout = APP_TRUE;
           s_enAppRngAoaFailureState = EN_APP_STATE_RECEIVE_DIST_WAIT_RX_DONE;
           s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
        }
        else if (s_stIrqStatus.Rx0Done == APP_TRUE) {
          s_stIrqStatus.Rx0Done = APP_FALSE;
          uint16_t rxSize = 0;
          uint8_t rxBuf[16] = {0};
          cb_framework_uwb_get_rx_payload(rxBuf, &rxSize, &s_stUwbPacketConfig);
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
          if (rxSize >= 5 && rxBuf[0] == 0x3) {
            memcpy(&s_receivedDistance, &rxBuf[1], sizeof(float));
          } else {
            s_applicationTimeout = APP_TRUE;
            s_enAppRngAoaFailureState = EN_APP_STATE_RECEIVE_DIST_WAIT_RX_DONE;
          }
          s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
        }
        break;

      case EN_APP_STATE_TERMINATE:
        #if (DEF_RNGAOA_ENABLE_LOG == APP_TRUE)
          app_rngaoa_log();
        #endif   
        app_rngaoa_cleanup_trx();
        // ????? ????:??????,?????????? ?????
        app_rngaoa_timeout_error_message_print();
        // ??????????????????????????????????????
        app_rngaoa_timer_off();
        app_rngaoa_reset();
        iterationTime = cb_hal_get_tick();
        s_enAppRngaoaState = EN_APP_STATE_IDLE;
        break;

      default:
        s_enAppRngAoaFailureState = s_enAppRngaoaState;
        s_applicationTimeout = APP_TRUE;
        s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
        break;
    }
  }  
}

void app_rngaoa_reset(void)
{
  s_stIrqStatus.TxDone = APP_FALSE; s_stIrqStatus.Rx0SfdDetected = APP_FALSE; s_stIrqStatus.Rx0Done = APP_FALSE;
  s_stIrqStatus.Rx1SfdDetected = APP_FALSE; s_stIrqStatus.Rx2SfdDetected = APP_FALSE;
  s_applicationTimeout = APP_FALSE;
  s_enAppRngAoaFailureState = EN_APP_STATE_IDLE;
  s_syncRestartCount = 0;
  memset(&s_stResponderDataContainer, 0, sizeof(s_stResponderDataContainer));
  s_stResponderDataContainer.rangingDataContainer.dstwrRangingBias = DEF_RESPONDER_RANGING_BIAS;
  s_aziResult = 0.0f; s_eleResult = 0.0f; s_receivedDistance = 0.0f;
  memset(&s_stPdoaOutputResult, 0, sizeof(s_stPdoaOutputResult));
  cb_framework_uwb_pdoa_reset_cir_data_container();
  app_rngaoa_cleanup_trx();
  cb_framework_uwb_tsu_clear();
  cb_framework_uwb_rxconfig_cfo_gain(EN_UWB_CFO_GAIN_RESET, NULL); 
}

static void app_rngaoa_soft_timeout(app_uwbrngaoa_state_en failureState)
{
  s_applicationTimeout = APP_TRUE;
  s_enAppRngAoaFailureState = failureState;
  s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
}

static float app_rngaoa_angle_diff_abs(float a, float b)
{
  float diff = fmodf(a - b + 180.0f, 360.0f);
  if (diff < 0.0f) {
    diff += 360.0f;
  }
  diff -= 180.0f;
  return fabsf(diff);
}

static void app_rngaoa_update_azi_with_hold(float candidateAzi)
{
  float diff;

  if (!(candidateAzi == candidateAzi)) {
    s_aziResult = s_lastValidAziReady ? s_lastValidAziResult : 0.0f;
    return;
  }

  if (s_lastValidAziReady == APP_FALSE) {
    s_lastValidAziReady = APP_TRUE;
    s_lastValidAziResult = candidateAzi;
    s_aziSpikeHoldCount = 0;
    s_aziResult = candidateAzi;
    return;
  }

  diff = app_rngaoa_angle_diff_abs(candidateAzi, s_lastValidAziResult);
  if (diff > DEF_AZI_SPIKE_HOLD_DEG &&
      s_aziSpikeHoldCount < DEF_AZI_SPIKE_HOLD_MAX_FRAMES) {
    s_aziSpikeHoldCount++;
    s_aziResult = s_lastValidAziResult;
    return;
  }

  s_aziSpikeHoldCount = 0;
  s_lastValidAziResult = candidateAzi;
  s_aziResult = candidateAzi;
}

static void app_rngaoa_cleanup_trx(void)
{
#if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
  cb_framework_uwb_disable_scheduled_trx(s_stDstwrTreply1Config);
  cb_framework_uwb_disable_scheduled_trx(s_stDstwrTround2Config);
#endif
  cb_framework_uwb_tx_end();
  cb_framework_uwb_rx_end(EN_UWB_RX_ALL);
}

void app_rngaoa_timer_init(uint16_t timeoutMs)
{
  cb_scr_timer0_module_on();
  cb_timer_disable_interrupt();
  NVIC_EnableIRQ(TIMER_0_IRQn);
  stTimerSetUp stTimerSetup;
  stTimerSetup.Timer = EN_TIMER_0; stTimerSetup.TimerMode = EN_TIMER_MODE_ONE_SHOT; stTimerSetup.TimeUnit = EN_TIMER_MS;
  stTimerSetup.stTimeOut.timeoutVal[0] = timeoutMs; stTimerSetup.stTimeOut.timeoutVal[1] = 0;
  stTimerSetup.stTimeOut.timeoutVal[2] = 0; stTimerSetup.stTimeOut.timeoutVal[3] = 0;
  stTimerSetup.stTimeOut.TimerTimeoutEvent = DEF_TIMER_TIMEOUT_EVENT_0; stTimerSetup.TimerEvtComMode = EN_TIMER_EVTCOM_MODE_00;
  stTimerSetup.AutoStartTimer = EN_START_TIMER_ENABLE; stTimerSetup.TimerEvtComEnable = EN_TIMER_EVTCOM_DISABLE;
  stTimerSetup.TimerInterrupt = EN_TIMER_INTERUPT_ENABLE; stTimerSetup.stPulseWidth.NumberOfCycles = 0;
  stTimerSetup.stPulseWidth.TimerPulseWidth = EN_TIMER_PULSEWIDTH_DISABLE;
  cb_timer_configure_timer(&stTimerSetup);
}

uint8_t app_rngaoa_validate_sync_payload(void)
{
  uint8_t  result = APP_TRUE;
  uint16_t rxPayloadSize = 0;
  uint8_t  syncRxPayload[DEF_SYNC_RX_PAYLOAD_SIZE] = {0};
  cb_uwbsystem_rxstatus_un rxStatus = cb_framework_uwb_get_rx_status();
  if (rxStatus.rx0_ok == CB_TRUE) {
    cb_framework_uwb_get_rx_payload(&syncRxPayload[0], &rxPayloadSize, &s_stUwbPacketConfig);
    for (uint16_t i = 0; i < DEF_SYNC_RX_PAYLOAD_SIZE; i++) if (syncRxPayload[i] != s_syncExpectedRxPayload[i]) result = APP_FALSE;
  } else result = APP_FALSE;
  return result;
}
    
void app_rngaoa_timer_off(void) { cb_timer_disable_timer(EN_TIMER_0); cb_scr_timer0_module_off(); NVIC_DisableIRQ(TIMER_0_IRQn); }
void cb_uwbapp_tx_done_irqhandler(void) { s_stIrqStatus.TxDone = APP_TRUE; }
void cb_uwbapp_rx0_done_irqcb(void) { s_stIrqStatus.Rx0Done = APP_TRUE; }
void cb_uwbapp_rx0_sfd_detected_irqcb(void) { s_stIrqStatus.Rx0SfdDetected = APP_TRUE; }
void cb_uwbapp_rx1_sfd_detected_irqcb(void) { s_stIrqStatus.Rx1SfdDetected = APP_TRUE; }
void cb_uwbapp_rx2_sfd_detected_irqcb(void) { s_stIrqStatus.Rx2SfdDetected = APP_TRUE; }

void cb_timer_0_app_irq_callback (void)
{
  s_applicationTimeout = APP_TRUE;
  s_enAppRngAoaFailureState = s_enAppRngaoaState;
}

void app_rngaoa_log(void)
{
  #ifndef M_PI
    #define M_PI 3.14159265358979323846
  #endif
  if (!s_applicationTimeout)
  {
    #if (DEF_RNGAOA_PDOA_CAL_LOG == APP_TRUE)
      app_uwb_rngaoa_print("PDOA: PD01(rx0_rx1)=%.2f, PD02(rx0_rx2)=%.2f, PD12(rx1_rx2)=%.2f, RSSI:%d\n",
                           s_stPdoaOutputResult.median.rx0_rx1,
                           s_stPdoaOutputResult.median.rx0_rx2,
                           s_stPdoaOutputResult.median.rx1_rx2,
                           (int)s_stRssiResults.rssiRx);
    #else
    /* 直接输出 s_aziResult，不做滤波；STM32 侧 UpdateFilteredAngle() 负责唯一滤波 */
    {
      double azi = (double)s_aziResult;
      double dist_cm = (double)s_receivedDistance;
      double azi_rad = azi * (M_PI / 180.0);
      double coord_x = dist_cm * sin(azi_rad);
      double coord_y = dist_cm * cos(azi_rad);
      app_uwb_rngaoa_print(
          "DIST:%.2f, X:%.2f, Y:%.2f, A:%.2f, PD01:%.2f, PD02:%.2f, PD12:%.2f, RSSI:%d\n",
          dist_cm,
          coord_x,
          coord_y,
          azi,
          s_stPdoaOutputResult.median.rx0_rx1,
          s_stPdoaOutputResult.median.rx0_rx2,
          s_stPdoaOutputResult.median.rx1_rx2,
          (int)s_stRssiResults.rssiRx);
    }
    #endif
  }
}

void app_rngaoa_timeout_error_message_print(void)
{
  if (!s_applicationTimeout) return; // ???????

  switch (s_enAppRngAoaFailureState)
  {
    case EN_APP_STATE_SYNC_WAIT_RX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:SYNC RX\n", s_appCycleCount++); break;
    case EN_APP_STATE_SYNC_WAIT_TX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:SYNC ACK\n", s_appCycleCount++); break;
    case EN_APP_STATE_DSTWR_RECEIVE_POLL_WAIT_RX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:DSTWR RX POLL\n", s_appCycleCount++); break;
    case EN_APP_STATE_DSTWR_TRANSMIT_RESPONSE_WAIT_TX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:DSTWR TX RESPONSE\n", s_appCycleCount++); break;
    case EN_APP_STATE_DSTWR_RECEIVE_FINAL_WAIT_RX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:DSTWR RX FINAL\n", s_appCycleCount++); break;
    case EN_APP_STATE_PDOA_WAIT_RX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:PDOA RX\n", s_appCycleCount++); break;
    case EN_APP_STATE_RESULT_WAIT_TX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:FINAL RESULT\n", s_appCycleCount++); break;
    // ????? ????,?????????,???? WAIT DIST ?????
    case EN_APP_STATE_RECEIVE_DIST_WAIT_RX_DONE: app_uwb_rngaoa_print("Cycle:%u, Timeout:WAIT DIST\n", s_appCycleCount++); break;
    default: break;
  }
}
