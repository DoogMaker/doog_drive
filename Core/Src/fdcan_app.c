#include "fdcan_app.h"
#include "fdcan.h"
#include "DRV8353.h"
#include "sense.h"
#include "foc.h"
#include "Protect.h"
#include "motor_control.h"   // MotorControl_CommandGate（硬件错误锁死门控）
#include "math.h"

extern volatile float bus_voltage;  

// 接收队列（中断生产 / 主循环消费，单生产者单消费者无锁环形缓冲）
#define FDCAN_RX_QUEUE_SIZE   16                          // 队列深度，必须是 2 的幂
#define FDCAN_RX_QUEUE_MASK   (FDCAN_RX_QUEUE_SIZE - 1)

typedef struct {
    uint32_t id;         // 帧 ID（标准帧 11 位）
    uint8_t  data[8];
    uint8_t  dlc;
} FDCAN_RxFrame_t;

static volatile FDCAN_RxFrame_t rx_queue[FDCAN_RX_QUEUE_SIZE];
static volatile uint16_t rx_head = 0;   // 写入索引：只在 CAN 接收中断里推进
static volatile uint16_t rx_tail = 0;   // 读取索引：只在主循环里推进

// 队列满而丢弃的帧数（诊断用：持续增长说明主循环处理速度跟不上上位机发帧速率）
volatile uint32_t fdcan_rx_drop_count = 0;

extern float manual_speed;
extern float manual_voltage;

// 角度换算常量（不依赖 PI 宏，避免头文件依赖差异）
#define FD_DEG2RAD   0.01745329252f   // 度 -> rad
#define FD_RAD2DEG   57.29577951308f  // rad -> 度

extern float KTH7823_Angle_Multiturn;   // 电机轴多圈机械角(未滤波)，与闭环/对齐同源

static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[8];

// CAN 错误 / 总线关闭监测变量（回调中更新，主循环查询）
volatile uint32_t fdcan_error_flags  = 0;
volatile uint32_t fdcan_busoff_count = 0;
volatile uint8_t  fdcan_tec          = 0;
volatile uint8_t  fdcan_rec          = 0;
volatile uint32_t fdcan_proto_err    = 0;

// 串口 can 诊断命令用的运行统计（见 fdcan_app.h）。
// 注意：刻意不在 FDCAN_App_Init() 里清零——BusOff 自动恢复会重新初始化外设，
//      统计值保留才能看出"历史上到底有没有收到过帧、有没有出过总线错误"。
volatile uint32_t fdcan_rx_count         = 0;
volatile uint32_t fdcan_rx_otherid_count = 0;
volatile uint32_t fdcan_tx_count         = 0;
volatile uint32_t fdcan_tx_fail_count    = 0;
volatile uint8_t  fdcan_last_rx_valid    = 0;
volatile uint32_t fdcan_last_rx_id       = 0;
volatile uint8_t  fdcan_last_rx_dlc      = 0;
volatile uint8_t  fdcan_last_rx_data[8]  = {0};
volatile uint32_t fdcan_last_rx_ms       = 0;

// 应用层需要使能的 FDCAN 中断集合（初始化与 BusOff 恢复共用一份，避免两处取值漂移）
//   RX FIFO0 新帧 + 错误状态(EW/EP/BO) + 协议错误(仲裁段/数据段，仅做诊断计数)
#define FDCAN_APP_IT_LIST   (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
                             FDCAN_IT_ERROR_WARNING        | \
                             FDCAN_IT_ERROR_PASSIVE        | \
                             FDCAN_IT_BUS_OFF              | \
                             FDCAN_IT_ARB_PROTOCOL_ERROR   | \
                             FDCAN_IT_DATA_PROTOCOL_ERROR)

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

    // 全局过滤器：不匹配上面任何过滤器的帧一律"拒绝"，含扩展帧与远程帧。
    // 为什么必须显式配置：RXGFC.ANFS 的复位值会把"不匹配的标准帧"收进 Rx FIFO0，
    // 而 6 块板共一条总线时，其他节点的命令帧（0x102 ~ 0x10C）就会被本板收到，
    // 造成"响应别人命令"的串台（同时还会顶掉本节点帧、误刷新通讯心跳）。
    // 这里与接收中断里的帧 ID 复核构成双重保护。
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                 FDCAN_REJECT, FDCAN_REJECT,        // 不匹配的标准帧 / 扩展帧：拒绝
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);  // 远程帧：拒绝

    // 中断线分工（此调用只写 ILS 寄存器，须在 ActivateNotification 之前执行）。
    // 注意 G4 的 ILS 分组与直觉不同，不能按"EW/BO 固定走 IT0"理解（分组原文见 HAL 头文件）：
    //   ILS.BERR(FDCAN_IT_GROUP_BIT_LINE_ERROR) = Error Passive + Error Logging Overflow
    //   ILS.PERR(FDCAN_IT_GROUP_PROTOCOL_ERROR) = ARA + PED + PEA + WDI + BusOff + ErrorWarning
    // 所以 BERR|PERR 划到 Line1 后，实际分工是：
    //   IT0 = RX FIFO0 接收
    //   IT1 = Error Warning / Error Passive / Bus Off / 协议错误（EW 与 BO 同样走 IT1）
    // 两个 IRQHandler 都调用 HAL_FDCAN_IRQHandler，且两条 NVIC 线均已使能，故功能上等价。
    HAL_FDCAN_ConfigInterruptLines(&hfdcan2,
                                   FDCAN_IT_GROUP_BIT_LINE_ERROR | FDCAN_IT_GROUP_PROTOCOL_ERROR,
                                   FDCAN_INTERRUPT_LINE1);

    // 启动 FDCAN 外设 (HAL 库已经由 CubeMX 初始化，只需调用 Start)
    HAL_FDCAN_Start(&hfdcan2);
    
    // 激活 FIFO0 接收中断 + 错误状态(EW/EP/BO) + 协议错误监测中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_APP_IT_LIST, 0);
    
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
    
    // 清空接收队列（静态变量本身已零初始化，这里显式复位，便于热重启/重复初始化）
    rx_head = 0;
    rx_tail = 0;
    fdcan_rx_drop_count = 0;
}

/**
 * @brief FDCAN 接收回调 (在中断中执行，尽量快速处理)
 * @note  一次中断把 FIFO0 里所有积压帧全部读入软件队列。
 *        HAL 的接收中断标志按"新帧到达"触发，若一次中断只读一帧，突发多帧时队头帧
 *        会积压在硬件 FIFO（G4 只有 3 格）里，靠后续新帧触发才被取出，容易溢出丢帧。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    // 循环读空 FIFO0；guard 只是防御性上限（硬件深度 3），避免异常情况下卡在中断里
    uint32_t guard = FDCAN_RX_QUEUE_SIZE;
    while ((guard-- > 0U) &&
           (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)) {
        FDCAN_RxHeaderTypeDef rx_header;
        // 显式补零：HAL 只填 DLC 对应的字节数，短帧时高字节必须是 0（而不是上一帧残留/栈上垃圾）
        uint8_t data[8] = {0};

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK) break;

        // 帧 ID 复核（第一重保护在全局过滤器）：只认发给本节点的命令帧。
        // 多节点共总线时，其他节点的帧既不执行、也不刷新本节点心跳，杜绝"串台"。
        if (rx_header.Identifier != CMD_FRAME_ID) {
            fdcan_rx_otherid_count++;   // 诊断：正常恒为 0，增长说明过滤器/全局过滤器配置被改坏
            continue;
        }

        Protect_CommFeed();   // 通讯心跳：收到属于本节点的帧才刷新失联计时

        // 诊断快照（串口 can 命令读取）：先记录"确实收到过这一帧"，再判队列是否满，
        // 这样即使随后因队列满被丢弃，串口也能看到上位机实际发来的字节，便于核对编码。
        fdcan_rx_count++;
        fdcan_last_rx_id    = rx_header.Identifier;
        fdcan_last_rx_dlc   = (uint8_t)rx_header.DataLength;
        fdcan_last_rx_ms    = HAL_GetTick();
        for (uint8_t i = 0; i < 8U; i++) {
            fdcan_last_rx_data[i] = data[i];
        }
        fdcan_last_rx_valid = 1;

        uint16_t next = (uint16_t)((rx_head + 1U) & FDCAN_RX_QUEUE_MASK);
        if (next == rx_tail) {
            // 队列满：丢弃最新帧，绝不覆盖尚未处理的历史命令（保持"先到先执行"的时序）
            fdcan_rx_drop_count++;
            continue;
        }

        rx_queue[rx_head].id  = rx_header.Identifier;
        rx_queue[rx_head].dlc = (uint8_t)rx_header.DataLength;
        for (uint8_t i = 0; i < 8U; i++) {
            rx_queue[rx_head].data[i] = data[i];
        }
        rx_head = next;   // 数据写完最后再推进写指针（保证主循环不会读到半成品帧）
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
 * @brief 从接收队列取出一帧（单生产者单消费者环形缓冲，只在主循环上下文调用）
 * @param out 输出的帧副本
 * @retval 1 = 取到一帧；0 = 队列为空
 * @note  槽位在 rx_tail 推进前绝不会被中断重写（生产者要等队列绕一整圈才会回到该槽），
 *        因此这里安全地把数据拷出来再推进读指针。
 */
static uint8_t FDCAN_Rx_Pop(FDCAN_RxFrame_t *out)
{
    if (rx_tail == rx_head) return 0;   // 队列空

    out->id  = rx_queue[rx_tail].id;
    out->dlc = rx_queue[rx_tail].dlc;
    for (uint8_t i = 0; i < 8U; i++) {
        out->data[i] = rx_queue[rx_tail].data[i];
    }

    rx_tail = (uint16_t)((rx_tail + 1U) & FDCAN_RX_QUEUE_MASK);
    return 1;
}

/**
 * @brief CAN 总线关闭(BusOff)自动恢复（主循环周期调用，非阻塞）
 * @note  恢复流程：Stop(置 CCCR.INIT 复位 CAN 核) -> Start(清 INIT，由硬件等待总线空闲后
 *        重新参与仲裁)。判断逻辑必须按 HAL 两个函数的真实返回值语义来写：
 *          - HAL_FDCAN_Stop  只在 State==BUSY 时成功，且内部超时会把它置成 ERROR；
 *          - HAL_FDCAN_Start 没有超时等待（只清 INIT 位后立即返回），仅在 State!=READY 时失败，
 *            因此它不会因为"总线繁忙"而失败，不能指望它失败后重试就能成功。
 *        据此：Stop 的返回值一律忽略（State 非 BUSY 时它必然失败），
 *        Start 失败则走 Init + FDCAN_App_Init 整外设重配，避免 State 卡在 ERROR 永久无法恢复。
 */
static void FDCAN_BusOffRecover(void)
{
    // ---- CAN 总线关闭自动恢复 ----
    if ((fdcan_error_flags & FDCAN_ERR_FLAG_BUSOFF) == 0U) return;

    fdcan_error_flags &= ~FDCAN_ERR_FLAG_BUSOFF;   // 先清标志，防止死循环

    // Stop 返回值忽略：State 非 BUSY（如已被置 ERROR）时它必然返回失败，
    // 但只要成功进入 INIT，随后的 Start 就能把外设拉回 BUSY 运行态。
    (void)HAL_FDCAN_Stop(&hfdcan2);

    if (HAL_FDCAN_Start(&hfdcan2) == HAL_OK) {
        // 恢复成功，重新激活中断通知（IE 寄存器 Stop/Start 不清除，此处保险重设）
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_APP_IT_LIST, 0);
    }
    else if (HAL_FDCAN_Init(&hfdcan2) == HAL_OK) {
        // State 异常（例如 Stop 超时被置为 ERROR）：整外设重新初始化，
        // 过滤器/全局过滤器/中断线由 FDCAN_App_Init() 一并重配。
        FDCAN_App_Init();
    }

    // 仍未进入 BUSY 运行态 → 重新置回 BO 标志，下个主循环周期继续重试
    if (hfdcan2.State != HAL_FDCAN_STATE_BUSY) {
        fdcan_error_flags |= FDCAN_ERR_FLAG_BUSOFF;
    }
}

/**
 * @brief 解析并执行一帧命令（主循环上下文；帧 ID 已在接收中断里复核为本节点命令帧）
 */
static void FDCAN_HandleCommand(const FDCAN_RxFrame_t *msg)
{
    // 解析指令
    uint8_t cmd = msg->data[0];            // 第一个字节是命令码
    int16_t val_int;                       // 用于接收 16 位数值
    float val_float;

    // 控制命令(0x01~0x07)门控：上电自检硬件错误锁死后拒绝执行
    // （心跳/查询 0x08/0x09/0x10 不受影响，锁死后仍可诊断）
    if (cmd >= CMD_SET_SPEED && cmd <= CMD_SET_OPENLOOP_ANGLE_HOLD) {
        if (!MotorControl_CommandGate()) return;
    }

    switch (cmd) {
        case CMD_SET_SPEED:
            // 开环门控：速度指令仅在开环模式生效，其余模式静默忽略（不驱动电机、不回反馈）
            if (foc.mode != FOC_STATE_OPENLOOP) break;
            // 上位机发送速度 (数据: byte1~2 为 int16, 单位 0.1 rad/s)
            val_int = (int16_t)(msg->data[1] | (msg->data[2] << 8));
            foc.target_speed = (float)val_int / 10.0f;
            // 如果当前在开环模式，外部修改的 manual_speed 也同步一下，防止下次覆盖
            manual_speed = foc.target_speed; 
            break;
            
        case CMD_SET_VOLTAGE:
            // 开环门控：电压指令仅在开环模式生效，其余模式静默忽略（不驱动电机、不回反馈）
            if (foc.mode != FOC_STATE_OPENLOOP) break;
            // 上位机发送电压 (数据: byte1~2 为 int16, 单位 0.01V)
            val_int = (int16_t)(msg->data[1] | (msg->data[2] << 8));
            manual_voltage = (float)val_int / 100.0f;
            if (val_int == 0) {
                // v=0：释放"转角度/悬停保持"动作（中止状态机并把电压、速度归零）
                FOC_StopOpenLoopMoveHold();
            } else {
                FOC_SetOpenLoopVoltage(manual_voltage);
            }
            break;
            
        case CMD_SET_MODE:
            // 控制模式切换 (数据: byte1)，编号直接等于 FOC_State_t 枚举值
            // 0: IDLE, 1: OPENLOOP, 2: ALIGN, 3: TORQUE, 4: SPEED, 5: POSITION
            // 6(FAULT) 不允许上位机设置（故障只能由保护触发）
            switch (msg->data[1]) {
                case 0: FOC_SetMode(FOC_STATE_IDLE);     break;
                case 1: FOC_SetMode(FOC_STATE_OPENLOOP); break;
                case 2: FOC_SetMode(FOC_STATE_ALIGN);    break;
                case 3: FOC_SetMode(FOC_STATE_TORQUE);   break;
                case 4: FOC_SetMode(FOC_STATE_SPEED);    break;
                case 5: FOC_SetMode(FOC_STATE_POSITION); break;
                default: break;
            }
            break;

        case CMD_SET_TORQUE:
            // 设置目标转矩 (数据: byte1~2 为 int16, 单位 Nm * 100)
            val_int = (int16_t)(msg->data[1] | (msg->data[2] << 8));
            val_float = (float)val_int / 100.0f;
            FOC_SetTargetTorque(val_float);
            break;

        case CMD_SET_POSITION:
            // 设置目标位置 (数据: byte1~4 为 int32, 单位 rad * 10000)
            {
                int32_t pos_int = (int32_t)(msg->data[1] | (msg->data[2] << 8) |
                                            (msg->data[3] << 16) | (msg->data[4] << 24));
                float pos_float = (float)pos_int / 10000.0f;
                FOC_SetTargetPosition(pos_float);
            }
            break;
            
        case CMD_SET_DIRECTION:
            // 方向控制 (数据: byte1)
            //  0: 正转 (正速度), 1: 反转 (负速度)
            if (msg->data[1] == 0) {
                manual_speed = fabs(manual_speed);      // 确保正数
                foc.target_speed = manual_speed;
            } else {
                manual_speed = -fabs(manual_speed);     // 确保负数
                foc.target_speed = manual_speed;
            }
            break;
            
        case CMD_SET_OPENLOOP_ANGLE_HOLD:
            // 开环转到指定输出端角度并悬停保持
            // 数据: byte1~2 为 int16，单位 度×10（0.1° 分辨率，有符号，正=正转，相对当前位置）
            // 运动速度/励磁电压复用 0x01/0x02 的设定值；未设定时用缺省值保证能转动并保持。
            // 开环门控：仅开环模式生效，其余模式静默忽略（不驱动电机、不回反馈）。
            if (foc.mode != FOC_STATE_OPENLOOP) break;
            if (msg->dlc < 3) break;     // 短帧保护：DLC<3 视为无效帧，直接忽略
            val_int = (int16_t)(msg->data[1] | (msg->data[2] << 8));
            if (val_int == 0) break;     // 0° 视为无效，直接忽略
            {
                float out_rad = ((float)val_int / 10.0f) * FD_DEG2RAD;   // 0.1度 -> rad

                float spd = fabsf(manual_speed);
                if (spd < 0.1f) spd = FD_OL_DEFAULT_SPEED;      // 速度未设定则用缺省
                manual_speed     = (out_rad >= 0.0f) ? spd : -spd;   // 方向取角度符号
                foc.target_speed = manual_speed;

                if (manual_voltage < 0.05f) manual_voltage = FD_OL_DEFAULT_VOLTAGE;  // 电压未设定则用缺省

                FOC_StartOpenLoopMoveHold(out_rad);       // 启动"运动 -> 到位悬停保持"状态机
                FOC_SetOpenLoopVoltage(manual_voltage);   // SetMode 会清零电压，这里重新施加运动电压
            }
            break;

        case CMD_HEARTBEAT:
            // 心跳：不做任何事、不改变任何运行状态。
            // 失联计时已在接收中断(HAL_FDCAN_RxFifo0Callback -> Protect_CommFeed)逐帧刷新。
            break;

        case CMD_GET_OL_STATUS:
            // 查询开环动作状态：不受开环门控限制，任意模式和状态均可查询，立即回一帧独立状态帧
            FDCAN_SendOLStatus();
            break;

        case CMD_GET_STATUS:
            // 上位机请求状态，立即打包发送反馈
            FDCAN_SendStatus();
            break;
            
        default:
            // 未识别命令，忽略
            break;
    }
}

/**
 * @brief 在主循环中调用：总线关闭自动恢复 + 按接收顺序处理队列中的全部命令帧
 * @note  一次调用把积压帧按到达顺序全部处理完：
 *        - 连发的多条命令（如 0x03 切模式 + 0x02 设电压 + 0x07 转角度）不会互相覆盖；
 *        - 顺序被严格保持，不会出现"0x03 丢失、只剩 0x07 非开环被静默忽略"的错序现象。
 */
void FDCAN_App_Process(void)
{
    // 总线关闭自动恢复（先恢复链路，再处理本轮收到的帧）
    FDCAN_BusOffRecover();

    // 按接收顺序处理队列中的全部命令帧（队列深度有限，不会长时间占用主循环）
    FDCAN_RxFrame_t frame;
    while (FDCAN_Rx_Pop(&frame)) {
        FDCAN_HandleCommand(&frame);
    }
}

/**
 * @brief 发送状态反馈给上位机
 *        反馈包：ID=0x20, 8字节数据
 *        Byte0: 系统状态 (0:空闲/就绪, 1:运行, 2:故障)
 *        Byte1-2: 当前转速 (int16, 单位 rad/s * 10)
 *        Byte3-4: A相电流 (int16, 单位 A * 100)
 *        Byte5: 当前控制模式（对外统一编号，与 0x03 命令 byte1 一致：0=IDLE 1=OPENLOOP 2=ALIGN 3=TORQUE 4=SPEED 5=POSITION 6=FAULT）
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
    
    // 控制模式：枚举值即对外统一编号（与 0x03 命令 byte1 完全一致）：
    //   0=IDLE 1=OPENLOOP 2=ALIGN 3=TORQUE 4=SPEED 5=POSITION 6=FAULT
    tx_data[5] = (uint8_t)foc.mode;
    
    // 母线电压 (V * 10)，读取真实采样值
    int16_t bus_voltage_tx = (int16_t)(bus_voltage * 10.0f);
    tx_data[6] = bus_voltage_tx & 0xFF;
    tx_data[7] = (bus_voltage_tx >> 8) & 0xFF;
    
    // 发送（返回值计入诊断：失败多为 TX FIFO 满或外设不在 BUSY 运行态，串口 can 命令可见）
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data) == HAL_OK) {
        fdcan_tx_count++;
    } else {
        fdcan_tx_fail_count++;
    }
}

// int16 饱和限幅：多字节数值写入前调用，避免浮点溢出后回绕
static int16_t fd_sat_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

/**
 * @brief 发送开环"运动/悬停保持"动作状态（命令 0x09 的响应帧）
 *        帧 ID = FBK_OL_FRAME_ID(0x300 + NODE_ID)，与 0x10 的反馈帧(0x200 + NODE_ID)用 ID 隔离，互不影响。
 *        byte0   : 状态码 0=非活动/已释放(IDLE) 1=转动中(RUN) 2=已到位悬停(HOLD) 3=回拉中(PULL)
 *        byte1-2 : int16 输出端相对角，单位 0.1°（相对本次动作起点，非 IDLE 时有效，IDLE 填 0）
 *        byte3-4 : int16 相对悬停点偏差，单位 0.01°（折算到输出端，仅 HOLD/PULL 有效，其余填 0）
 *        byte5-6 : int16 开环设定电压，单位 V×100
 *        byte7   : 保留 0
 *        注：输出端角度是由电机轴多圈机械角推算的相对量（输出端编码器未启用），非绝对零点。
 */
void FDCAN_SendOLStatus(void)
{
    const float gear = GEAR_RATIO_OUT2MOTOR;

    tx_header.Identifier = FBK_OL_FRAME_ID;   // 独立帧 ID，与 0x200 反馈帧区分

    tx_data[0] = (uint8_t)foc.openloop_move_state;

    int16_t rel_deg10  = 0;   // 输出端相对角 (0.1°)
    int16_t dev_deg100 = 0;   // 相对悬停点偏差 (0.01°，输出端)
    if (foc.openloop_move_state != FOC_OL_MOVE_IDLE) {
        // 输出端相对角：电机轴机械角差 / 减速比；×encoder_direction 保证"正=正转"
        float rel_mech = (KTH7823_Angle_Multiturn - foc.openloop_move_ref_mech) * (float)encoder_direction;
        rel_deg10 = fd_sat_i16((rel_mech / gear) * FD_RAD2DEG * 10.0f);

        if (foc.openloop_move_state == FOC_OL_MOVE_HOLD ||
            foc.openloop_move_state == FOC_OL_MOVE_PULL) {
            float dev_mech = (KTH7823_Angle_Multiturn - foc.openloop_move_hold_mech) * (float)encoder_direction;
            dev_deg100 = fd_sat_i16((dev_mech / gear) * FD_RAD2DEG * 100.0f);
        }
    }
    tx_data[1] = rel_deg10 & 0xFF;
    tx_data[2] = (rel_deg10 >> 8) & 0xFF;
    tx_data[3] = dev_deg100 & 0xFF;
    tx_data[4] = (dev_deg100 >> 8) & 0xFF;

    // 开环设定电压 (V ×100)
    int16_t volt_int = fd_sat_i16(foc.openloop_voltage * 100.0f);
    tx_data[5] = volt_int & 0xFF;
    tx_data[6] = (volt_int >> 8) & 0xFF;

    tx_data[7] = 0;

    // 发送：TX FIFO 满或总线异常时直接丢弃本帧，绝不阻塞主循环（自动重传已关闭）
    // 返回值计入诊断（串口 can 命令可见：TXfail 持续增长说明 FIFO 满或外设不在运行态）
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data) == HAL_OK) {
        fdcan_tx_count++;
    } else {
        fdcan_tx_fail_count++;
    }
}