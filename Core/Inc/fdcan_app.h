#ifndef __FDCAN_APP_H
#define __FDCAN_APP_H

#include "main.h"
#include "foc.h"

// ==================== 多节点配置（12 个关节电机，两路 CAN 总线）====================
// 上位机 RK3588S 只有两路 FDCAN，每路挂 6 个电机（单边前后两条腿 × 每条腿 3 关节）：
//   第 1 路 CAN0 ：节点 1 ~ 6    （一侧：前腿髋/膝/踝 = 1/2/3，后腿髋/膝/踝 = 4/5/6）
//   第 2 路 CAN1 ：节点 7 ~ 12   （另一侧：前腿 7/8/9，后腿 10/11/12）
// 所有驱动板硬件完全相同，仅此宏不同，分别编译烧录 NODE_ID = 1 ~ 12 十二份固件。
// 两路 CAN 物理隔离，节点号全局唯一（1~12），即使两路合并也不会冲突。
#define NODE_ID             1     // 本板节点号（烧录前按实际关节改成 1 ~ 12）

// ---------- CAN 帧 ID 编码（标准帧 11 位 ID，节点号写入 ID，硬件过滤器自动区分）----------
// 命令帧（上位机 -> 电机）：ID = 0x100 + node_id，每个电机只接收属于自己的这一个 ID
// 反馈帧（电机 -> 上位机）：ID = 0x200 + node_id，12 个电机反馈 ID 全局唯一、互不冲突
// ID 范围：命令 0x101~0x10C、反馈 0x201~0x20C，均在 11 位标准帧范围内（≤0x7FF），无越界。
#define CMD_FRAME_ID        (0x100 + NODE_ID)   // 本板接收的命令帧 ID（0x101 ~ 0x10C）
#define FBK_FRAME_ID        (0x200 + NODE_ID)   // 本板反馈帧 ID（0x201 ~ 0x20C）
#define FBK_OL_FRAME_ID     (0x300 + NODE_ID)   // 开环动作状态帧 ID（0x301 ~ 0x30C），与 0x200 反馈帧完全隔离
                                                //   仅 0x09 查询时回一帧，不增加总线负载

// ---------- 上位机命令 ID (命令帧数据区 byte0，上位机发送 -> 下位机接收) ----------
#define CMD_SET_SPEED      0x01   // 设置目标速度 (int16, 单位 rad/s * 10)
#define CMD_SET_VOLTAGE    0x02   // 设置开环电压 (int16, 单位 V * 100)
#define CMD_SET_MODE       0x03   // 设置控制模式 (0:IDLE 1:开环 2:对齐 3:闭环力矩 4:闭环速度 5:闭环位置)
#define CMD_SET_DIRECTION  0x04   // 设置方向 (0:正转, 1:反转)
#define CMD_SET_TORQUE     0x05   // 设置目标转矩 (int16, 单位 Nm * 100)
#define CMD_SET_POSITION   0x06   // 设置目标位置 (int32, 单位 rad * 10000)
#define CMD_SET_OPENLOOP_ANGLE_HOLD 0x07  // 开环转到指定输出端角度并悬停保持 (int16, 单位 度×10, 相对当前位置)
#define CMD_HEARTBEAT      0x08   // 心跳：无参数、不改变任何运行状态（接收中断已逐帧刷新失联计时）
#define CMD_GET_OL_STATUS  0x09   // 查询开环"转动/悬停"动作状态（回一帧 FBK_OL_FRAME_ID 状态帧）
#define CMD_GET_STATUS     0x10   // 请求获取状态反馈

// ---------- 开环"转角度+悬停"缺省参数（与 FOC.H 单一来源，避免取值漂移）----------
#define FD_OL_DEFAULT_SPEED     FOC_OL_DEFAULT_SPEED     // 未设转速时的默认开环转速 (rad/s，电机轴)
#define FD_OL_DEFAULT_VOLTAGE   FOC_OL_DEFAULT_VOLTAGE   // 未设电压时的默认开环电压 (V)

// ---------- FDCAN 错误 / 总线关闭监测（中断线由 HAL_FDCAN_ConfigInterruptLines 的 ILS 分组决定）----------
// 中断分工（两个 IRQHandler 都调用 HAL_FDCAN_IRQHandler，且两条 NVIC 线均已使能）：
//   IT0 = RX FIFO0 接收
//   IT1 = Error Warning / Error Passive / Bus Off / 协议错误
// 注意：G4 的 ILS.PERR 分组本身就覆盖 Warning Status 与 Bus_Off Status，
//       因此 EW/BO 并非"固定走 IT0"，而是随 ILS.PERR 一起落在 IT1。
#define FDCAN_ERR_FLAG_WARNING  0x01   // 错误警告 (EW)：TEC>96 或 REC>96
#define FDCAN_ERR_FLAG_PASSIVE  0x02   // 错误被动 (EP)：TEC>127 或 REC>127
#define FDCAN_ERR_FLAG_BUSOFF   0x04   // 总线关闭 (BO)：节点已离线，自动恢复后清除

extern volatile uint32_t fdcan_error_flags;   // 当前错误状态标志（bit 定义见上）
extern volatile uint32_t fdcan_busoff_count;  // 总线关闭累计次数
extern volatile uint8_t  fdcan_tec;           // 发送错误计数 (TEC)
extern volatile uint8_t  fdcan_rec;           // 接收错误计数 (REC)
extern volatile uint32_t fdcan_proto_err;     // 协议错误累计次数（仲裁/数据段）
extern volatile uint32_t fdcan_rx_drop_count; // 接收队列满而丢弃的帧数（持续增长=主循环处理不过来）

// ---------- 串口 can 诊断命令用的运行统计（由 CAN 中断/发送路径维护，只读）----------
// 用途：本地串口敲 can，一眼区分"一帧都没收到" / "收到了但被门控或保护吃掉" / "收发正常"。
extern volatile uint32_t fdcan_rx_count;         // 本节点命令帧接收总数（已过 ID 复核，含队列满被丢的）
extern volatile uint32_t fdcan_rx_otherid_count; // 进了 RX FIFO0 但 ID 不是本节点的帧数（过滤器异常时才增长）
extern volatile uint32_t fdcan_tx_count;         // 反馈帧成功入 TX FIFO 的次数
extern volatile uint32_t fdcan_tx_fail_count;    // 反馈帧入 TX FIFO 失败次数（TX FIFO 满/外设异常）
extern volatile uint8_t  fdcan_last_rx_valid;    // 1 = 下面这组字段有效（至少收到过一帧本节点命令帧）
extern volatile uint32_t fdcan_last_rx_id;       // 最后一帧 ID（原样）
extern volatile uint8_t  fdcan_last_rx_dlc;      // 最后一帧 DLC（原样）
extern volatile uint8_t  fdcan_last_rx_data[8];  // 最后一帧数据（原样，未按命令解析）
extern volatile uint32_t fdcan_last_rx_ms;       // 最后一帧到达时刻 (HAL_GetTick, ms)

// ---------- 接收队列（突发多帧不再互相覆盖）----------
// CAN 接收中断把每一帧放入队列（深度 FDCAN_RX_QUEUE_SIZE = 16），
// 主循环 FDCAN_App_Process() 一次按到达顺序处理完所有积压帧；
// 队列满时丢弃最新帧并累加 fdcan_rx_drop_count，绝不覆盖尚未处理的命令。

// ---------- FDCAN 初始化与任务函数 ----------
void FDCAN_App_Init(void);
void FDCAN_App_Process(void);    // 主循环中调用，处理接收到的指令 + 总线关闭自动恢复
void FDCAN_SendStatus(void);     // 发送状态反馈
void FDCAN_SendOLStatus(uint8_t evt);   // 发送开环动作状态反馈（命令 0x09 的响应帧，ID=FBK_OL_FRAME_ID）

#endif