#include "fdcan_app.h"
#include "fdcan.h"
#include "DRV8353.h"
#include "sense.h"
#include "foc.h"
#include "Protect.h"
#include "math.h"

extern volatile float bus_voltage;   // 来自 sense.c 的真实母线电压采样值(V)

// 接收缓存结构体
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    volatile uint8_t flag; // 1 表示收到新数据
} FDCAN_RxMsg_t;

extern float manual_speed;
extern float manual_voltage;

static FDCAN_RxMsg_t rx_msg;
static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[8];

// CAN 错误 / 总线关闭监测变量（回调中更新，主循环查询）
volatile uint32_t fdcan_error_flags  = 0;
volatile uint32_t fdcan_busoff_count = 0;
volatile uint8_t  fdcan_tec          = 0;
volatile uint8_t  fdcan_rec          = 0;
volatile uint32_t fdcan_proto_err    = 0;

/**
 * @brief FDCAN 应用层初始化
 */
void FDCAN_App_Init(void)
{
    // 配置接收过滤器：只接收本节点命令帧 ID（0x100 + NODE_ID），须在 Start 之前配置
    // 同一路总线上的 6 块板共享总线，靠硬件过滤器按帧 ID 区分，避免互相"串台"响应其他电机的命令
    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;     // 掩码模式：FilterID1=匹配值, FilterID2=掩码
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CMD_FRAME_ID;           // 只匹配本节点命令帧 ID
    filter.FilterID2 = 0x7FF;                  // 掩码 0x7FF = 11 位标准帧 ID 全位精确匹配

    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);

    // 中断线分工：把 Error Passive / 协议错误组分配到 IT1，IT0 负责接收 + Error Warning/Bus Off
    // （G4 的 EWE/BOE 无中断线选择位，固定走 IT0；此调用只设置 ILS 寄存器，须在 ActivateNotification 之前）
    HAL_FDCAN_ConfigInterruptLines(&hfdcan2,
                                   FDCAN_IT_GROUP_BIT_LINE_ERROR | FDCAN_IT_GROUP_PROTOCOL_ERROR,
                                   FDCAN_INTERRUPT_LINE1);

    // 启动 FDCAN 外设 (HAL 库已经由 CubeMX 初始化，只需调用 Start)
    HAL_FDCAN_Start(&hfdcan2);
    
    // 激活 FIFO0 接收中断 + 错误状态(EW/EP/BO) + 协议错误监测中断
    HAL_FDCAN_ActivateNotification(&hfdcan2,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                   FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
                                   FDCAN_IT_BUS_OFF |
                                   FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
                                   0);
    
    // 初始化 Tx 帧头（固定为标准帧，数据长度根据实际设置）
    tx_header.Identifier = 0;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    
    rx_msg.flag = 0;
}

/**
 * @brief FDCAN 接收回调 (在中断中执行，尽量快速处理)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        FDCAN_RxHeaderTypeDef rx_header;
        
        // 读取数据到缓存
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_msg.data) == HAL_OK) {
            rx_msg.id = rx_header.Identifier;
            rx_msg.dlc = rx_header.DataLength;
            rx_msg.flag = 1;   // 标记收到了新数据
            Protect_CommFeed();   // 通讯心跳：每收到一帧刷新失联计时
        }
    }
}

/**
 * @brief FDCAN 错误状态回调 (EW/EP/BO 状态变化, 中断上下文执行)
 * @note  只记录事件与错误计数，BusOff 恢复放在主循环 FDCAN_App_Process() 中处理，
 *        避免在中断里阻塞等待总线空闲(128 个连续隐性位)
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    FDCAN_ErrorCountersTypeDef ec;

    if (ErrorStatusITs & FDCAN_IT_ERROR_WARNING) {
        fdcan_error_flags |= FDCAN_ERR_FLAG_WARNING;
    }
    if (ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) {
        fdcan_error_flags |= FDCAN_ERR_FLAG_PASSIVE;
    }
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        fdcan_error_flags |= FDCAN_ERR_FLAG_BUSOFF;
        fdcan_busoff_count++;
    }

    // 记录当前收发错误计数，供上位机诊断 (TEC/REC > 96 即进入 Error Warning)
    if (HAL_FDCAN_GetErrorCounters(hfdcan, &ec) == HAL_OK) {
        fdcan_tec = ec.TxErrorCnt;
        fdcan_rec = ec.RxErrorCnt;
    }
}

/**
 * @brief FDCAN 错误回调 (协议错误/错误日志溢出等, 走 IT1 中断线)
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    fdcan_proto_err++;
}

/**
 * @brief 在主循环中调用，解析收到的指令并执行 + 总线关闭自动恢复
 */
void FDCAN_App_Process(void)
{
    // ---- CAN 总线关闭自动恢复 ----
    // 恢复流程：Stop(进入 INIT) -> Start(退出 INIT, 硬件等待总线空闲 128 个连续隐性位后恢复)
    // Start 失败(超时)说明总线仍持续繁忙(如短路)，下个循环重试
    if (fdcan_error_flags & FDCAN_ERR_FLAG_BUSOFF) {
        fdcan_error_flags &= ~FDCAN_ERR_FLAG_BUSOFF;   // 先清标志，防止死循环
        if ((HAL_FDCAN_Stop(&hfdcan2) == HAL_OK) && (HAL_FDCAN_Start(&hfdcan2) == HAL_OK)) {
            // 恢复成功，重新激活中断通知（IE 寄存器 Stop/Start 不清除，此处保险重设）
            HAL_FDCAN_ActivateNotification(&hfdcan2,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                           FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
                                           FDCAN_IT_BUS_OFF |
                                           FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
                                           0);
        } else {
            fdcan_error_flags |= FDCAN_ERR_FLAG_BUSOFF;  // 恢复失败，下个周期重试
        }
    }

    if (rx_msg.flag == 0) return;
    
    // 解析指令
    uint8_t cmd = rx_msg.data[0];          // 第一个字节是命令码
    int16_t val_int;                       // 用于接收 16 位数值
    float val_float;
    
    switch (cmd) {
        case CMD_SET_SPEED:
            // 上位机发送速度 (数据: byte1~2 为 int16, 单位 0.1 rad/s)
            val_int = (int16_t)(rx_msg.data[1] | (rx_msg.data[2] << 8));
            foc.target_speed = (float)val_int / 10.0f;
            // 如果当前在开环模式，外部修改的 manual_speed 也同步一下，防止下次覆盖
            manual_speed = foc.target_speed; 
            break;
            
        case CMD_SET_VOLTAGE:
            // 上位机发送电压 (数据: byte1~2 为 int16, 单位 0.01V)
            val_int = (int16_t)(rx_msg.data[1] | (rx_msg.data[2] << 8));
            manual_voltage = (float)val_int / 100.0f;
            FOC_SetOpenLoopVoltage(manual_voltage);
            break;
            
        case CMD_SET_MODE:
            // 控制模式切换 (数据: byte1)
            // 0: IDLE, 1: OPENLOOP, 2: CLOSEDLOOP(速度), 3: ALIGN, 4: CLOSEDLOOP(位置), 5: CLOSEDLOOP(力矩)
            switch (rx_msg.data[1]) {
                case 0: FOC_SetMode(FOC_STATE_IDLE); break;
                case 1: FOC_SetMode(FOC_STATE_OPENLOOP); break;
                case 2: FOC_SetMode(FOC_STATE_CLOSEDLOOP); FOC_SetClosedLoopMode(CL_MODE_SPEED); break;
                case 3: FOC_SetMode(FOC_STATE_ALIGN); break;
                case 4: FOC_SetMode(FOC_STATE_CLOSEDLOOP); FOC_SetClosedLoopMode(CL_MODE_POSITION); break;
                case 5: FOC_SetMode(FOC_STATE_CLOSEDLOOP); FOC_SetClosedLoopMode(CL_MODE_TORQUE); break;
                default: break;
            }
            break;

        case CMD_SET_TORQUE:
            // 设置目标转矩 (数据: byte1~2 为 int16, 单位 Nm * 100)
            val_int = (int16_t)(rx_msg.data[1] | (rx_msg.data[2] << 8));
            val_float = (float)val_int / 100.0f;
            FOC_SetTargetTorque(val_float);
            break;

        case CMD_SET_POSITION:
            // 设置目标位置 (数据: byte1~4 为 int32, 单位 rad * 10000)
            {
                int32_t pos_int = (int32_t)(rx_msg.data[1] | (rx_msg.data[2] << 8) |
                                            (rx_msg.data[3] << 16) | (rx_msg.data[4] << 24));
                float pos_float = (float)pos_int / 10000.0f;
                FOC_SetTargetPosition(pos_float);
            }
            break;
            
        case CMD_SET_DIRECTION:
            // 方向控制 (数据: byte1)
            //  0: 正转 (正速度), 1: 反转 (负速度)
            if (rx_msg.data[1] == 0) {
                manual_speed = fabs(manual_speed);      // 确保正数
                foc.target_speed = manual_speed;
            } else {
                manual_speed = -fabs(manual_speed);     // 确保负数
                foc.target_speed = manual_speed;
            }
            break;
            
        case CMD_GET_STATUS:
            // 上位机请求状态，立即打包发送反馈
            FDCAN_SendStatus();
            break;
            
        default:
            // 未识别命令，忽略
            break;
    }
    
    rx_msg.flag = 0;
}

/**
 * @brief 发送状态反馈给上位机
 *        反馈包：ID=0x20, 8字节数据
 *        Byte0: 系统状态 (0:空闲/就绪, 1:运行, 2:故障)
 *        Byte1-2: 当前转速 (int16, 单位 rad/s * 10)
 *        Byte3-4: A相电流 (int16, 单位 A * 100)
 *        Byte5: 当前控制模式
 *        Byte6-7: 母线电压 (int16, 单位 V * 10)
 */
void FDCAN_SendStatus(void)
{
    uint32_t id = FBK_FRAME_ID;   // 反馈帧 ID 携带节点号，上位机据此区分 12 个电机
    tx_header.Identifier = id;
    
    // 打包数据
    // 系统状态：基于 FOC 状态机，0=空闲/就绪, 1=运行(对齐/开环/闭环), 2=故障
    if (foc.mode == FOC_STATE_FAULT)       tx_data[0] = 2;
    else if (foc.mode == FOC_STATE_IDLE)   tx_data[0] = 0;
    else                                   tx_data[0] = 1;
    
    // 速度 (rad/s * 10)
    int16_t speed_int = (int16_t)(foc.speed * 10.0f);
    tx_data[1] = speed_int & 0xFF;
    tx_data[2] = (speed_int >> 8) & 0xFF;
    
    // 电流 (A * 100)
    int16_t cur_int = (int16_t)(current_A * 100.0f);
    tx_data[3] = cur_int & 0xFF;
    tx_data[4] = (cur_int >> 8) & 0xFF;
    
    // 控制模式
    tx_data[5] = (uint8_t)foc.mode;
    
    // 母线电压 (V * 10)，读取真实采样值
    int16_t bus_voltage_tx = (int16_t)(bus_voltage * 10.0f);
    tx_data[6] = bus_voltage_tx & 0xFF;
    tx_data[7] = (bus_voltage_tx >> 8) & 0xFF;
    
    // 发送
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data);
}