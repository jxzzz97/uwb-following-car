/**
 * @file    AppUwbRngAoa.c
 * @brief   
 * @details 
 * @author  Chipsbank
 * @date    2024
 */

//-------------------------------
// INCLUDE SECTION
//-------------------------------
#include <string.h>
#include "CB_gpio.h"
#include "CB_iomux.h"
#include "AppUwbRngAoa.h"
#include "CB_Algorithm.h"
#include "CB_timer.h"
#include "CB_scr.h"
#include "NonLIB_sharedUtils.h"
#include "CB_uwbframework.h"

//-------------------------------
// CONFIGURATION SECTION
//-------------------------------
#define DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER   APP_FALSE
#define APP_UWB_RNGAOA_UARTPRINT_ENABLE APP_FALSE

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
#define DEF_INITIATOR_RANGING_BIAS     0
#define DEF_TAG_LED_GPIO_PIN           EN_GPIO_PIN_3
#define DEF_TAG_BEEP_GPIO_PIN          EN_GPIO_PIN_0
#define DEF_TAG_BEEP_IOMUX_PIN         EN_IOMUX_GPIO_0
#define DEF_TAG_ALARM_DISTANCE_CM      300.0f   /* 距离 > 3m 报警 */
#define DEF_TAG_BEEP_ENABLE            APP_FALSE /* Keep buzzer disabled until PWM trigger logic is revalidated */
#define DEF_TAG_LED_SLOW_TOGGLE_MS     500U
#define DEF_TAG_LED_FAST_TOGGLE_MS     120U
#define DEF_TAG_DISTANCE_FRESH_MS      1500U
#define DEF_TAG_BEEP_PWM_TIMER         EN_TIMER_1
#define DEF_TAG_BEEP_PWM_PERIOD_US     250U     /* 4kHz, layout 指定频率 */
#define DEF_TAG_BEEP_PWM_HIGH_US       125U     /* 50% 占空比 */

#define DEF_SYNC_TX_PAYLOAD_SIZE       4
#define DEF_SYNC_ACK_RX_PAYLOAD_SIZE   3

//-------------------------------
// ENUM SECTION
//-------------------------------
typedef enum{
  
  //IDLE STATE
  EN_APP_STATE_IDLE = 0,
  //SYNC STATE
  EN_APP_STATE_SYNC_TRANSMIT,
  EN_APP_STATE_SYNC_WAIT_TX_DONE,
  EN_APP_STATE_SYNC_RECEIVE,
  EN_APP_STATE_SYNC_WAIT_RX_DONE,
  //DSTWR STATE
  EN_APP_STATE_DSTWR_TRANSMIT_POLL,
  EN_APP_STATE_DSTWR_TRANSMIT_POLL_WAIT_TX_DONE,
  EN_APP_STATE_DSTWR_RECEIVE_RESPONSE,
  EN_APP_STATE_DSTWR_RECEIVE_RESPONSE_WAIT_RX_DONE,
  EN_APP_STATE_DSTWR_TRANSMIT_FINAL,
  EN_APP_STATE_DSTWR_TRANSMIT_FINAL_WAIT_TX_DONE,
  // PDOA-TX STATE
  EN_APP_STATE_WAIT_RESPONDER_READY,             // Coordination with Responder
  EN_APP_STATE_PDOA_TRANSMIT,                    // PDOA-TX
  EN_APP_STATE_PDOA_WAIT_TX_DONE_N_REPEATED_TX,  // PDOA-TX  
  //RESULT SHARING STATE
  EN_APP_STATE_RESULT_RECEIVE,
  EN_APP_STATE_RESULT_WAIT_RX_DONE,
	EN_APP_STATE_RESULT_TX_DONE_WAIT,
  //TERMINATE STATE
  EN_APP_STATE_TERMINATE,
} app_uwbrngaoa_state_en;

//-------------------------------
// STRUCT/UNION SECTION
//-------------------------------
typedef struct
{
  volatile uint8_t TxDone;
  volatile uint8_t Rx0Done;
}app_uwbrngaoa_irqstatus_st;

typedef struct
{
  cb_uwbframework_rangingdatacontainer_st   rangingDataContainer;
  cb_uwbframework_pdoadatacontainer_st      pdoaDataContainer;  
} app_rngaoa_responderdatacontainer_st;

//-------------------------------
// GLOBAL VARIABLE SECTION
//-------------------------------
/* Default uwb packet configuration.*/
static cb_uwbsystem_packetconfig_st s_stUwbPacketConfig = 
{
  .prfMode            = EN_PRF_MODE_BPRF_62P4,                 // PRF mode selection
  .psduDataRate       = EN_PSDU_DATA_RATE_6P81,           // PSDU data rate
  .bprfPhrDataRate    = EN_BPRF_PHR_DATA_RATE_0P85,       // BPRF PHR data rate
  .preambleCodeIndex  = EN_UWB_PREAMBLE_CODE_IDX_9,       // Preamble code index (9-32)
  .preambleDuration   = EN_PREAMBLE_DURATION_64_SYMBOLS,  // Preamble duration (0-1)
  .sfdId              = EN_UWB_SFD_ID_2,                  // SFD identifier (0-4)
  .phrRangingBit      = 0x00,                             // PHR Ranging Bit (0-1)
  .rframeConfig       = EN_RFRAME_CONFIG_SP0,             // SP0, SP1, SP3
  .stsLength          = EN_STS_LENGTH_64_SYMBOLS,         // STS Length
  .numStsSegments     = EN_NUM_STS_SEGMENTS_1,            // Number of STS segments
  .stsKey             = {0x14EB220FUL,0xF86050A8UL,0xD1D336AAUL,0x14148674UL},  // PhyHrpUwbStsKey
  .stsVUpper          = {0xD37EC3CAUL,0xC44FA8FBUL,0x362EEB34UL},               // PhyHrpUwbStsVUpper96
  .stsVCounter        = 0x1F9A3DE4UL,                                           // PhyHrpUwbStsVCounter
  .macFcsType         = EN_MAC_FCS_TYPE_CRC16,            // CRC16
};

static app_uwbrngaoa_irqstatus_st          s_stIrqStatus             = { APP_FALSE };
static volatile app_uwbrngaoa_state_en     s_enAppRngaoaState        = EN_APP_STATE_IDLE;
static volatile app_uwbrngaoa_state_en     s_enAppRngAoaFailureState = EN_APP_STATE_IDLE;

//  SYNC TX Payload                                                 'S'  'Y'  'N'  'C'
static uint8_t s_syncTxPayload  [DEF_SYNC_TX_PAYLOAD_SIZE]      = {0x53,0x59,0x4E,0x43};
//  SYNC ACK payload                                                'A'  'C'  'K'
static uint8_t s_syncAckPayload [DEF_SYNC_ACK_RX_PAYLOAD_SIZE]  = {0x41,0x43,0x4B};

static volatile uint8_t s_applicationTimeout = APP_FALSE;

static cb_uwbsystem_rx_tsutimestamp_st       s_stRxTsuTimestamp0;
static cb_uwbsystem_tx_tsutimestamp_st       s_stTxTsuTimestamp0;
static cb_uwbsystem_tx_tsutimestamp_st       s_stTxTsuTimestamp1;
static cb_uwbsystem_rangingtroundtreply_st  s_stInitiatorTround1Treply2;

static cb_uwbframework_rangingdatacontainer_st  s_stInitiatorDataContainer = 
{
  .dstwrRangingBias = DEF_INITIATOR_RANGING_BIAS,
  .dstwrTroundTreply = {0}
};

static app_rngaoa_responderdatacontainer_st s_stResponderDataContainer = {
  .rangingDataContainer = { .dstwrRangingBias   = DEF_INITIATOR_RANGING_BIAS,
                            .dstwrTroundTreply  = {0}},
  .pdoaDataContainer    = { .rx0_rx1 = 0.0f,
                            .rx0_rx2 = 0.0f,
                            .rx1_rx2 = 0.0f,
                            .elevationEst = 0.0f,
                            .azimuthEst = 0.0f }
};

static double s_measuredDistance = 0.0; // Measured Distance
static uint32_t s_appCycleCount  = 0;   // Logging Purpose: cycle count
static uint8_t s_countOfPdoaScheduledTx = 0;
static float s_latestMeasuredDistance = 0.0f;
static uint32_t s_latestDistanceTick = 0U;
static uint8_t s_latestDistanceValid = APP_FALSE;
static uint32_t s_indicatorToggleTick = 0U;
static uint8_t s_indicatorPhaseOn = APP_FALSE;
static uint8_t s_buzzerPwmConfigured = APP_FALSE;
static uint8_t s_buzzerPwmEnabled = APP_FALSE;

static cb_uwbframework_trx_scheduledconfig_st s_stDstwrTround1Config = {
  .eventTimestampMask   = EN_UWBEVENT_TIMESTAMP_MASK_0,
  .eventIndex           = EN_UWBEVENT_28_TX_DONE,
  .absTimer             = EN_UWB_ABSOLUTE_TIMER_0,
  .timeoutValue         = 500,
  .eventCtrlMask        = EN_UWBCTRL_RX0_START_MASK,
};

static cb_uwbframework_trx_scheduledconfig_st s_stDstwrTreply2Config = {
  .eventTimestampMask   = EN_UWBEVENT_TIMESTAMP_MASK_0,
  .eventIndex           = EN_UWBEVENT_17_RX0_SFD_DET,
  .absTimer             = EN_UWB_ABSOLUTE_TIMER_0,
  .timeoutValue         = 700,
  .eventCtrlMask        = EN_UWBCTRL_TX_START_MASK,
};

static cb_uwbframework_trx_scheduledconfig_st s_stPdoaRepeatedTxConfig = {
  .eventTimestampMask   = EN_UWBEVENT_TIMESTAMP_MASK_0,
  .eventIndex           = EN_UWBEVENT_28_TX_DONE,
  .absTimer             = EN_UWB_ABSOLUTE_TIMER_0,
  .timeoutValue         = 250,
  .eventCtrlMask        = EN_UWBCTRL_TX_START_MASK,
};

static void app_rngaoa_cleanup_scheduled_trx(void);

//-------------------------------
// RNGAOA: INITIATOR SETUP
//-------------------------------
//-------------------------------------------------------
//    Initiator                         Responder
//     Idle                                Idle
//       |---------1. SYNC ----------------->|
//       |<------- 2. ACK  ------------------|
//     a |---------3. RNGAOA(POLL) --------->| d
//     b |<--------4. RNGAOA(RESPONSE) ------| e
//     c |---------5. RNGAOA(FINAL) -------->| f
//       |---------6. PDOA (n cycles) ------>| 
//       |<--------7. RESULT ----------------|
//     Terminate                         Terminate  
//
// DEF_RNGAOA_SYNC_ACK_TIMEOUT_MS       : 1 + 2
// DEF_RNGAOA_OVERALL_PROCESS_TIMEOUT_MS: 3 + 4 + 5 + 6
// DEF_RNGAOA_APP_CYCLE_TIME_MS         : Idle
// DEF_RNGAOA_POLL_WAIT_TIME_MS         : 3
// DEF_RNGAOA_RESPONSE_WAIT_TIME_MS     : 4
// DEF_RNGAOA_FINAL_WAIT_TIME_MS        : 5
// DEF_NUMBER_OF_PDOA_REPEATED_TX       : 6 (n cycles)
// DEF_PDOA_TX_START_WAIT_TIME_MS       : 6 (wait responder enter rx)
//
// Initiator: Tround_1 = b - a
//            Treply_2 = c - d     
// Responder: Treply_1 = e - d
//            Tround_2 = f - e
//
//  a: s_stTxTsuTimestamp0    d,e,f:   s_stInitiatorDataContainer.dstwrTroundTreply
//  b: s_stRxTsuTimestamp0    
//  c: s_stTxTsuTimestamp1    
//-------------------------------------------------------
#define DEF_RNGAOA_SYNC_ACK_TIMEOUT_MS           10
#define DEF_RNGAOA_OVERALL_PROCESS_TIMEOUT_MS    70
#define DEF_RNGAOA_APP_CYCLE_TIME_MS             50
#define DEF_DSTWR_POLL_WAIT_TIME_MS              1
#define DEF_DSTWR_RESPONSE_WAIT_TIME_MS          0
#define DEF_DSTWR_FINAL_WAIT_TIME_MS             1  
#define DEF_DSTWR_RESPONSE_SOFT_TIMEOUT_MS       10
#define DEF_NUMBER_OF_PDOA_REPEATED_TX           5
#define DEF_PDOA_TX_START_WAIT_TIME_MS           2

//-------------------------------
// FUNCTION PROTOTYPE SECTION
//-------------------------------
void    app_rngaoa_reset                        (void);
void    app_rngaoa_timeout_error_message_print  (void);
uint8_t app_rngaoa_validate_sync_ack_payload    (void);
void    app_rngaoa_log                          (void);
static void app_rngaoa_update_indicators        (void);
static void app_rngaoa_buzzer_pwm_enable        (void);
static void app_rngaoa_buzzer_pwm_disable       (void);
 
//-------------------------------
// FUNCTION BODY SECTION
//-------------------------------
void app_rngaoa_initiator(void)
{  
  uint32_t startTime      = 0;
  uint32_t iterationTime  = 0;  
  
  //--------------------------------
  // Init
  //--------------------------------
  cb_framework_uwb_init();
  
  //--------------------------------
  // Configure Payload
  //--------------------------------
  cb_uwbsystem_txpayload_st stSyncTxPayloadPack  = {0};
  cb_uwbsystem_txpayload_st stDstwrTxPayloadPack = {0};
  cb_uwbsystem_txpayload_st pdoaTxPayloadPack   = {0};
  
  stSyncTxPayloadPack.ptrAddress      = &s_syncTxPayload[0];
  stSyncTxPayloadPack.payloadSize     = sizeof(s_syncTxPayload);
  
  // RNGAOA Payload
  static uint8_t s_dstwrPayload[1]    = {0x1};
  stDstwrTxPayloadPack.ptrAddress     = &s_dstwrPayload[0];
  stDstwrTxPayloadPack.payloadSize    = sizeof(s_dstwrPayload);
  
  // PDOA Payload
  static uint8_t s_pdoaTxPayload[1]   = {0x2};
  pdoaTxPayloadPack.ptrAddress        = &s_pdoaTxPayload[0];
  pdoaTxPayloadPack.payloadSize       = sizeof(s_pdoaTxPayload);
  static uint8_t s_resultTxPayload[5] = {0x3, 0, 0, 0, 0}; 
  cb_uwbsystem_txpayload_st resultTxPayloadPack = {0};
  resultTxPayloadPack.ptrAddress      = &s_resultTxPayload[0];
  resultTxPayloadPack.payloadSize     = sizeof(s_resultTxPayload);
  //--------------------------------
  // Configure IRQ
  //--------------------------------
  // TX IRQ
  cb_uwbsystem_tx_irqenable_st stTxIrqEnable = { CB_FALSE} ;
  stTxIrqEnable.txDone  = APP_TRUE;
  // RX IRQ
  cb_uwbsystem_rx_irqenable_st stRxIrqEnable = { CB_FALSE } ;
  stRxIrqEnable.rx0Done = APP_TRUE;
  
  //--------------------------------
  // Configure absolute timer for scheduled TX start
  //--------------------------------
  //--------------------------------
  // Configure absolute timer for scheduled PDOA-TX start
  //--------------------------------
  s_enAppRngaoaState = EN_APP_STATE_SYNC_TRANSMIT;
  
  while(1)
  {
    app_rngaoa_update_indicators();

    if (s_applicationTimeout && s_enAppRngaoaState != EN_APP_STATE_TERMINATE)
    {
      s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
    }

    switch (s_enAppRngaoaState)
    {
      //-------------------------------------
      // IDLE
      //-------------------------------------        
      case EN_APP_STATE_IDLE:
        // Wait for next cycle
        if (cb_hal_is_time_elapsed(iterationTime, DEF_RNGAOA_APP_CYCLE_TIME_MS))
        {
          s_enAppRngaoaState = EN_APP_STATE_SYNC_TRANSMIT;
        }
        break;

      //-------------------------------------
      // SYNC: TX
      //-------------------------------------        
      case EN_APP_STATE_SYNC_TRANSMIT:
        cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &stSyncTxPayloadPack, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_SYNC_WAIT_TX_DONE;
        break;
      case EN_APP_STATE_SYNC_WAIT_TX_DONE:
        if (s_stIrqStatus.TxDone == APP_TRUE)
        {
          s_stIrqStatus.TxDone = APP_FALSE;
          cb_framework_uwb_tx_end();
          s_enAppRngaoaState = EN_APP_STATE_SYNC_RECEIVE;   
        }
        break;
        
      //-------------------------------------
      // SYNC: RX (ACK)
      //-------------------------------------        
      case EN_APP_STATE_SYNC_RECEIVE:
        cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_SYNC_WAIT_RX_DONE;
        startTime = cb_hal_get_tick();
        break;
      case EN_APP_STATE_SYNC_WAIT_RX_DONE:
        if (cb_hal_is_time_elapsed(startTime, DEF_RNGAOA_SYNC_ACK_TIMEOUT_MS))
        {
          // if SYNC-ACK not received from Responder within 10ms, send SYNC again.
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
          s_enAppRngaoaState = EN_APP_STATE_SYNC_TRANSMIT;
        }
        else if (s_stIrqStatus.Rx0Done == APP_TRUE)
        {
          s_stIrqStatus.Rx0Done = APP_FALSE;
          uint32_t ackValidateResult = app_rngaoa_validate_sync_ack_payload();
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
          if (ackValidateResult == APP_TRUE)
          {
            s_enAppRngaoaState = EN_APP_STATE_DSTWR_TRANSMIT_POLL;
            startTime = cb_hal_get_tick();
          }
          else
          {
            // if SYNC-ACK payload validation failed, send SYNC again.
            s_enAppRngaoaState = EN_APP_STATE_SYNC_TRANSMIT;
          }
        }        
        break;
        
      //-------------------------------------
      // DS-TWR: POLL
      //-------------------------------------    
      case EN_APP_STATE_DSTWR_TRANSMIT_POLL:
        if (cb_hal_is_time_elapsed(startTime, DEF_DSTWR_POLL_WAIT_TIME_MS))
        {
          app_rngaoa_timer_init(DEF_RNGAOA_OVERALL_PROCESS_TIMEOUT_MS);
          
          #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_enable_scheduled_trx(s_stDstwrTround1Config);
          #endif
          
          cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &stDstwrTxPayloadPack, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_TRANSMIT_POLL_WAIT_TX_DONE;
        }
        break;
      case EN_APP_STATE_DSTWR_TRANSMIT_POLL_WAIT_TX_DONE:
        if (s_stIrqStatus.TxDone == APP_TRUE)
        {
          s_stIrqStatus.TxDone = APP_FALSE;
          
          cb_framework_uwb_get_tx_tsu_timestamp(&s_stTxTsuTimestamp0);
          cb_framework_uwb_tx_end();
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_RECEIVE_RESPONSE;
          startTime = cb_hal_get_tick();  
        }
        break;
      //-------------------------------------
      // DS-TWR: RESPONSE
      //-------------------------------------       
      case EN_APP_STATE_DSTWR_RECEIVE_RESPONSE:
        #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_enable_scheduled_trx(s_stDstwrTreply2Config);
          cb_framework_uwb_configure_scheduled_trx(s_stDstwrTround1Config);
          cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_DEFERRED);

          s_enAppRngaoaState = EN_APP_STATE_DSTWR_RECEIVE_RESPONSE_WAIT_RX_DONE;
        #else
        if (cb_hal_is_time_elapsed(startTime, DEF_DSTWR_RESPONSE_WAIT_TIME_MS))
        {
          cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_NON_DEFERRED);
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_RECEIVE_RESPONSE_WAIT_RX_DONE;
        }
        #endif
        break;
			case EN_APP_STATE_DSTWR_RECEIVE_RESPONSE_WAIT_RX_DONE:
        if (s_stIrqStatus.Rx0Done == APP_TRUE)
        {
          s_stIrqStatus.Rx0Done = APP_FALSE;
          cb_framework_uwb_get_rx_tsu_timestamp(&s_stRxTsuTimestamp0, EN_UWB_RX_0);
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_TRANSMIT_FINAL;
          startTime = cb_hal_get_tick();
        }
        else if (cb_hal_is_time_elapsed(startTime, DEF_DSTWR_RESPONSE_SOFT_TIMEOUT_MS)) 
        {
           s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
        }
        break;
        case EN_APP_STATE_RESULT_WAIT_RX_DONE:
        if (s_stIrqStatus.Rx0Done == APP_TRUE)
        {        
          s_stIrqStatus.Rx0Done = APP_FALSE;
          uint16_t rxPayloadSize = 0;
          cb_framework_uwb_get_rx_payload((uint8_t*)(&s_stResponderDataContainer), &rxPayloadSize, &s_stUwbPacketConfig);
          cb_framework_uwb_calculate_initiator_tround_treply(&s_stInitiatorDataContainer, s_stTxTsuTimestamp0, s_stTxTsuTimestamp1, s_stRxTsuTimestamp0);
          s_measuredDistance = cb_framework_uwb_calculate_distance(s_stInitiatorDataContainer, s_stResponderDataContainer.rangingDataContainer);
          s_latestMeasuredDistance = (float)s_measuredDistance;
          s_latestDistanceTick = cb_hal_get_tick();
          s_latestDistanceValid = APP_TRUE;
          cb_framework_uwb_rx_end(EN_UWB_RX_0);
          /* Keep a short guard gap before the distance-share TX. If cycle
           * latency later becomes a concern, replace this blocking delay
           * with a state/timestamp based wait instead of sleeping here. */
          cb_hal_delay_in_us(1000);
          float dist_to_send = (float)s_measuredDistance;
          memcpy(&s_resultTxPayload[1], &dist_to_send, sizeof(float));
          cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &resultTxPayloadPack, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
          s_enAppRngaoaState = EN_APP_STATE_RESULT_TX_DONE_WAIT; 
        }
        break;
      case EN_APP_STATE_RESULT_TX_DONE_WAIT:
        if (s_stIrqStatus.TxDone == APP_TRUE)
        {
             s_stIrqStatus.TxDone = APP_FALSE;
             cb_framework_uwb_tx_end();
             s_enAppRngaoaState = EN_APP_STATE_TERMINATE;
        }
        break;        
      case EN_APP_STATE_DSTWR_TRANSMIT_FINAL:
        #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &stDstwrTxPayloadPack, &stTxIrqEnable, EN_TRX_START_DEFERRED);
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_TRANSMIT_FINAL_WAIT_TX_DONE;
        #else
        if (cb_hal_is_time_elapsed(startTime, DEF_DSTWR_FINAL_WAIT_TIME_MS))
        {
          cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &stDstwrTxPayloadPack, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
          s_enAppRngaoaState = EN_APP_STATE_DSTWR_TRANSMIT_FINAL_WAIT_TX_DONE;
        }
        #endif
        break;
      case EN_APP_STATE_DSTWR_TRANSMIT_FINAL_WAIT_TX_DONE:
        if (s_stIrqStatus.TxDone == APP_TRUE)
        {
          s_stIrqStatus.TxDone = APP_FALSE;
          #if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
          cb_framework_uwb_disable_scheduled_trx(s_stDstwrTreply2Config);
          #endif
          cb_framework_uwb_get_tx_tsu_timestamp(&s_stTxTsuTimestamp1);
          cb_framework_uwb_tx_end();
          s_enAppRngaoaState = EN_APP_STATE_WAIT_RESPONDER_READY;
          startTime = cb_hal_get_tick();
        }
        break;
        
      //-------------------------------------
      // PDOA-TX
      //-------------------------------------
      case EN_APP_STATE_WAIT_RESPONDER_READY:
        if(cb_hal_is_time_elapsed(startTime, DEF_PDOA_TX_START_WAIT_TIME_MS) == CB_PASS)
        {
          s_enAppRngaoaState = EN_APP_STATE_PDOA_TRANSMIT;
        }
        break;
        
      case EN_APP_STATE_PDOA_TRANSMIT:
        s_countOfPdoaScheduledTx = 0;
        cb_framework_uwb_enable_scheduled_trx(s_stPdoaRepeatedTxConfig);
        cb_framework_uwb_tx_start(&s_stUwbPacketConfig, &pdoaTxPayloadPack, &stTxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_PDOA_WAIT_TX_DONE_N_REPEATED_TX;
        break;
      
      case EN_APP_STATE_PDOA_WAIT_TX_DONE_N_REPEATED_TX:
        if (s_stIrqStatus.TxDone)
        {
          s_stIrqStatus.TxDone = APP_FALSE;  
          s_countOfPdoaScheduledTx++;          
          if (s_countOfPdoaScheduledTx < DEF_NUMBER_OF_PDOA_REPEATED_TX)
          {
            cb_framework_uwb_configure_scheduled_trx(s_stPdoaRepeatedTxConfig);
            cb_framework_uwb_tx_restart(&stTxIrqEnable, EN_TRX_START_DEFERRED);
          }
          else
          {
            cb_framework_uwb_disable_scheduled_trx(s_stPdoaRepeatedTxConfig);
            cb_framework_uwb_tx_end();
            s_countOfPdoaScheduledTx  = 0;  
            s_enAppRngaoaState   = EN_APP_STATE_RESULT_RECEIVE;  
          }
        }
        break;     
        
      //-------------------------------------
      // Ranging Result: RX
      //-------------------------------------         
      case EN_APP_STATE_RESULT_RECEIVE:
        cb_framework_uwb_rx_start(EN_UWB_RX_0, &s_stUwbPacketConfig, &stRxIrqEnable, EN_TRX_START_NON_DEFERRED);
        s_enAppRngaoaState = EN_APP_STATE_RESULT_WAIT_RX_DONE;
        break;
      //-------------------------------------
      // Terminate
      //-------------------------------------       
      case EN_APP_STATE_TERMINATE:
        #if (DEF_RNGAOA_ENABLE_LOG == APP_TRUE)
          app_rngaoa_log();
        #endif        
        app_rngaoa_cleanup_scheduled_trx();
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

/**
 * @brief   Resets all member variables
 * @details This function turns on resets all the member variables
*/
void app_rngaoa_reset(void)
{
  s_measuredDistance = 0.0;

  s_stIrqStatus.TxDone         = APP_FALSE;
  s_stIrqStatus.Rx0Done        = APP_FALSE;
  s_applicationTimeout         = APP_FALSE;
  s_enAppRngAoaFailureState    = EN_APP_STATE_IDLE;
  memset(&s_stInitiatorDataContainer,   0, sizeof(s_stInitiatorDataContainer)); 
  s_countOfPdoaScheduledTx  = 0;
  
  app_rngaoa_cleanup_scheduled_trx();
  cb_framework_uwb_tsu_clear();
  cb_framework_uwb_tx_end();            // ensure propoer TX end upon abnormal condition
  cb_framework_uwb_rx_end(EN_UWB_RX_0); // ensure propoer RX end upon abnormal condition
}

static void app_rngaoa_cleanup_scheduled_trx(void)
{
#if (DEF_APP_RNGAOA_USE_ABSOLUTE_TIMER == APP_TRUE)
  cb_framework_uwb_disable_scheduled_trx(s_stDstwrTround1Config);
  cb_framework_uwb_disable_scheduled_trx(s_stDstwrTreply2Config);
#endif
  cb_framework_uwb_disable_scheduled_trx(s_stPdoaRepeatedTxConfig);
}

/**
 * @brief   Initialize the application timer module.
 * @details This function turns on Timer 0 disables their interrupts, 
 *          and enables CPU Timer 0 interrupts.
 */
void app_rngaoa_timer_init(uint16_t timeoutMs)
{
  // Turn on Timer 0
  cb_scr_timer0_module_on();

  // Disable Timer's module interrupt
  cb_timer_disable_interrupt();
  
  // Enable CPU Timer 0,1,2,3 Interrupt
  NVIC_EnableIRQ(TIMER_0_IRQn);

  // Configure TIMER0-EVENT0 in One_shot mode that timeout every DEF_RNGAOA_OVERALL_PROCESS_TIMEOUT_MS ms.
  stTimerSetUp stTimerSetup;
  stTimerSetup.Timer                        = EN_TIMER_0;
  stTimerSetup.TimerMode                    = EN_TIMER_MODE_ONE_SHOT;
  stTimerSetup.TimeUnit                     = EN_TIMER_MS;

  stTimerSetup.stTimeOut.timeoutVal[0]      = timeoutMs;
  stTimerSetup.stTimeOut.timeoutVal[1]      = 0;
  stTimerSetup.stTimeOut.timeoutVal[2]      = 0;
  stTimerSetup.stTimeOut.timeoutVal[3]      = 0;
  stTimerSetup.stTimeOut.TimerTimeoutEvent  = DEF_TIMER_TIMEOUT_EVENT_0;
  stTimerSetup.TimerEvtComMode              = EN_TIMER_EVTCOM_MODE_00;
  stTimerSetup.AutoStartTimer               = EN_START_TIMER_ENABLE;
  stTimerSetup.TimerEvtComEnable            = EN_TIMER_EVTCOM_DISABLE;
  stTimerSetup.TimerInterrupt               = EN_TIMER_INTERUPT_ENABLE;
  stTimerSetup.stPulseWidth.NumberOfCycles  = 0;
  stTimerSetup.stPulseWidth.TimerPulseWidth = EN_TIMER_PULSEWIDTH_DISABLE;
  
  cb_timer_configure_timer(&stTimerSetup);
}

uint8_t app_rngaoa_validate_sync_ack_payload(void)
{
  uint8_t  result = APP_TRUE;
  uint16_t rxPayloadSize = 0;
  uint8_t  syncAckPayloadReceived[DEF_SYNC_ACK_RX_PAYLOAD_SIZE] = {0};
  
  cb_uwbsystem_rxstatus_un rxStatus = cb_framework_uwb_get_rx_status();
          
  if (rxStatus.rx0_ok   == CB_TRUE)
  {  
    cb_framework_uwb_get_rx_payload(&syncAckPayloadReceived[0], &rxPayloadSize, &s_stUwbPacketConfig);
    for (uint16_t i = 0; i < DEF_SYNC_ACK_RX_PAYLOAD_SIZE; i++)
    {
      if (syncAckPayloadReceived[i] != s_syncAckPayload[i])
        result = APP_FALSE;
    }
  }
  else
  {
    result = APP_FALSE;
  }  
  return result;
}

/**
 * @brief   Off the application timer module.
 * @details This function turns off Timer 0 disables their interrupts, 
 *          and disable CPU Timer 0 interrupts.
 */
void app_rngaoa_timer_off(void)
{
  // Disable Timer 0
  cb_timer_disable_timer(EN_TIMER_0);
  
  // Turn off Timer 0
  cb_scr_timer0_module_off();
  
  // Enable CPU Timer 0,1,2,3 Interrupt
  NVIC_DisableIRQ(TIMER_0_IRQn);
}

/**
 * @brief Callback function for the UWB TX Done IRQ.
 * 
 * This function is called when the UWB TX Done IRQ is triggered.
 */
void cb_uwbapp_tx_done_irqhandler(void)
{
  s_stIrqStatus.TxDone = APP_TRUE;
}

/**
 * @brief Callback function for the UWB RX0 Done IRQ.
 * 
 * This function is called when the UWB RX0 Done IRQ is triggered. 
 */
void cb_uwbapp_rx0_done_irqcb(void)
{
  s_stIrqStatus.Rx0Done = APP_TRUE; 
}

/**
 * @brief   Callback function for Timer 0 IRQ.
 *
 * This function is called when the Timer 0 IRQ is triggered.
 */
void cb_timer_0_app_irq_callback (void)
{
  s_applicationTimeout = APP_TRUE;
  s_enAppRngAoaFailureState = s_enAppRngaoaState;
}

/**
 * @brief Dstwr Logging function, at different verbosity 
 * @param DeviceRole The role of the current device.
 * @details Logging verbosity level is set globally, at file-scope. @see s_logOpt
 * 0: Simple Log (default)
 * 1: Simple Log + CIR log
 * 2: Full log (Simple log + CIR log + Status log)
 */
void app_rngaoa_log(void) 
{
  float azimuth = s_stResponderDataContainer.pdoaDataContainer.azimuthEst;

  if (!s_applicationTimeout)
  {
    app_uwb_rngaoa_print("SUCCESS! DIST:%.2f, A:%.2f\n", s_measuredDistance, azimuth);
  }
  else
  {
    app_uwb_rngaoa_print("TO:%d, STATE:%d, DIST:%.2f, A:%.2f\n",
                         s_applicationTimeout,
                         s_enAppRngAoaFailureState,
                         s_measuredDistance,
                         azimuth);
  }
}

static void app_rngaoa_update_indicators(void)
{
  uint32_t now = cb_hal_get_tick();
  uint32_t togglePeriodMs = DEF_TAG_LED_SLOW_TOGGLE_MS;
  uint8_t alarmActive = APP_FALSE;

  if ((s_latestDistanceValid == APP_TRUE) &&
      (cb_hal_is_time_elapsed(s_latestDistanceTick, DEF_TAG_DISTANCE_FRESH_MS) == CB_FAIL) &&
      (s_latestMeasuredDistance > DEF_TAG_ALARM_DISTANCE_CM))
  {
    alarmActive = APP_TRUE;
    togglePeriodMs = DEF_TAG_LED_FAST_TOGGLE_MS;
  }

  if (cb_hal_is_time_elapsed(s_indicatorToggleTick, togglePeriodMs) == CB_PASS)
  {
    s_indicatorToggleTick = now;
    s_indicatorPhaseOn = (s_indicatorPhaseOn == APP_TRUE) ? APP_FALSE : APP_TRUE;
  }

#if (DEF_TAG_BEEP_ENABLE == APP_TRUE)
  if (alarmActive == APP_TRUE)
  {
    app_rngaoa_buzzer_pwm_enable();
  }
  else
  {
    app_rngaoa_buzzer_pwm_disable();
  }
#else
  app_rngaoa_buzzer_pwm_disable();
#endif

  cb_gpio_write_pin(DEF_TAG_LED_GPIO_PIN, s_indicatorPhaseOn ? EN_GPIO_PIN_SET : EN_GPIO_PIN_RESET);
}

static void app_rngaoa_buzzer_pwm_enable(void)
{
  if (s_buzzerPwmConfigured == APP_FALSE)
  {
    stTimerSetUp timerSetup = {0};

    cb_scr_gpio_module_on();
    cb_iomux_config(DEF_TAG_BEEP_IOMUX_PIN, &(stIomuxGpioMode){EN_IOMUX_GPIO_MODE_GPIO, 0});
    cb_gpio_write_pin(DEF_TAG_BEEP_GPIO_PIN, EN_GPIO_PIN_RESET);

    timerSetup.Timer                        = DEF_TAG_BEEP_PWM_TIMER;
    timerSetup.TimerMode                    = EN_TIMER_MODE_FREERUN;
    timerSetup.TimeUnit                     = EN_TIMER_US;
    timerSetup.stTimeOut.timeoutVal[0]      = DEF_TAG_BEEP_PWM_HIGH_US;
    timerSetup.stTimeOut.timeoutVal[1]      = DEF_TAG_BEEP_PWM_PERIOD_US;
    timerSetup.stTimeOut.timeoutVal[2]      = 0;
    timerSetup.stTimeOut.timeoutVal[3]      = 0;
    timerSetup.stTimeOut.TimerTimeoutEvent  = DEF_TIMER_TIMEOUT_EVENT_0 | DEF_TIMER_TIMEOUT_EVENT_1;
    timerSetup.TimerEvtComMode              = EN_TIMER_EVTCOM_MODE_01;
    timerSetup.AutoStartTimer               = EN_START_TIMER_ENABLE;
    timerSetup.TimerEvtComEnable            = EN_TIMER_EVTCOM_ENABLE;
    timerSetup.TimerInterrupt               = EN_TIMER_INTERUPT_DISABLE;
    timerSetup.stPulseWidth.NumberOfCycles  = 0;
    timerSetup.stPulseWidth.TimerPulseWidth = EN_TIMER_PULSEWIDTH_DISABLE;

    cb_timer_init(DEF_TAG_BEEP_PWM_TIMER);
    cb_timer_configure_timer(&timerSetup);
    cb_iomux_config(DEF_TAG_BEEP_IOMUX_PIN,
                    &(stIomuxGpioMode){EN_IOMUX_GPIO_MODE_EVENT_LIST_0_OUTPUT, (uint8_t)EN_TIMER1_TOE0});
    s_buzzerPwmConfigured = APP_TRUE;
    s_buzzerPwmEnabled = APP_TRUE;
    return;
  }

  if (s_buzzerPwmEnabled == APP_FALSE)
  {
    cb_timer_resume_timer(DEF_TAG_BEEP_PWM_TIMER);
    cb_iomux_config(DEF_TAG_BEEP_IOMUX_PIN,
                    &(stIomuxGpioMode){EN_IOMUX_GPIO_MODE_EVENT_LIST_0_OUTPUT, (uint8_t)EN_TIMER1_TOE0});
    s_buzzerPwmEnabled = APP_TRUE;
  }
}

static void app_rngaoa_buzzer_pwm_disable(void)
{
  if (s_buzzerPwmConfigured == APP_TRUE && s_buzzerPwmEnabled == APP_TRUE)
  {
    cb_timer_pause_timer(DEF_TAG_BEEP_PWM_TIMER);
    s_buzzerPwmEnabled = APP_FALSE;
  }

  cb_scr_gpio_module_on();
  cb_gpio_write_pin(DEF_TAG_BEEP_GPIO_PIN, EN_GPIO_PIN_RESET);
  cb_iomux_config(DEF_TAG_BEEP_IOMUX_PIN, &(stIomuxGpioMode){EN_IOMUX_GPIO_MODE_GPIO, 0});
  cb_gpio_write_pin(DEF_TAG_BEEP_GPIO_PIN, EN_GPIO_PIN_RESET);
}

/**
 * @brief Prints a timeout error message based on the current process state.
 *
 * This function logs an error message with the current cycle count and the 
 * specific process state where a timeout occurred. It provides descriptive 
 * feedback for debugging and monitoring the RNGAOA protocol.
 *
 * @param[in] state  The current state of the RNGAOA process when the timeout occurred.
 *                   Possible values:
 *                   - ``EN_IDLE``: Idle state.
 *                   - ``EN_STATE_SYNCING``: Syncing state.
 *                   - ``EN_STATE_RNGAOA``: RNGAOA state.
 *                   - ``EN_STATE_RESULT``: Final result state.
 *                   - ``EN_STATE_DIST_OOB``: Distance out-of-bounds state.
 */
void app_rngaoa_timeout_error_message_print(void)
{
  switch (s_enAppRngAoaFailureState)
  {
    case EN_APP_STATE_IDLE:
      break;
    case EN_APP_STATE_SYNC_TRANSMIT:
      break;
    case EN_APP_STATE_SYNC_WAIT_TX_DONE:
      app_uwb_rngaoa_print("Cycle:%u, Timeout:SYNC TX\n", s_appCycleCount++);
      break;
    case EN_APP_STATE_SYNC_RECEIVE:
      break;
    case EN_APP_STATE_SYNC_WAIT_RX_DONE:
      app_uwb_rngaoa_print("Cycle:%u, Timeout:SYNC ACK\n", s_appCycleCount++);
      break;
    case EN_APP_STATE_DSTWR_TRANSMIT_POLL:
      break;
    case EN_APP_STATE_DSTWR_TRANSMIT_POLL_WAIT_TX_DONE:
      app_uwb_rngaoa_print("Cycle:%u, Timeout:RNGAOA TX POLL\n", s_appCycleCount++);
      break;
    case EN_APP_STATE_DSTWR_RECEIVE_RESPONSE:
      break;
    case EN_APP_STATE_DSTWR_RECEIVE_RESPONSE_WAIT_RX_DONE:
      app_uwb_rngaoa_print("Cycle:%u, Timeout:RNGAOA RX RESPONSE\n", s_appCycleCount++);
      break;
    case EN_APP_STATE_DSTWR_TRANSMIT_FINAL:
      break;
    case EN_APP_STATE_DSTWR_TRANSMIT_FINAL_WAIT_TX_DONE:
      app_uwb_rngaoa_print("Cycle:%u, Timeout:RNGAOA TX FINAL\n", s_appCycleCount++);
      break;
    case EN_APP_STATE_WAIT_RESPONDER_READY:             // Coordination with Responder
      break;
    case EN_APP_STATE_PDOA_TRANSMIT:                    // PDOA-TX
      break;
    case EN_APP_STATE_PDOA_WAIT_TX_DONE_N_REPEATED_TX:  // PDOA-TX
      break;
    case EN_APP_STATE_RESULT_RECEIVE:
      break;
    case EN_APP_STATE_RESULT_WAIT_RX_DONE:
      app_uwb_rngaoa_print("Cycle:%u, Timeout:FINAL RESULT\n", s_appCycleCount++);
      break;
		case EN_APP_STATE_RESULT_TX_DONE_WAIT:
			app_uwb_rngaoa_print("Cycle:%u, Timeout:DIST TX\n", s_appCycleCount++);
      break;
    case EN_APP_STATE_TERMINATE:
      break;    
  }
}
