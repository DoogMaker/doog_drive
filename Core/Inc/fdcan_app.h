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

// ---------- 上位机命令 ID (命令帧数据区 byte0，上位机发送 -> 下位机接收) ----------
#define CMD_SET_SPEED      0x01   // 设置目标速度 (int16, 单位 rad/s * 10)
#define CMD_SET_VOLTAGE    0x02   // 设置开环电压 (int16, 单位 V * 100)
#define CMD_SET_MODE       0x03   // 设置控制模式 (0:IDLE 1:开环 2:闭环速度 3:对齐 4:闭环位置 5:闭环力矩)
#define CMD_SET_DIRECTION  0x04   // 设置方向 (0:正转, 1:反转)
#define CMD_SET_TORQUE     0x05   // 设置目标转矩 (int16, 单位 Nm * 100)
#define CMD_SET_POSITION   0x06   // 设置目标位置 (int32, 单位 rad * 10000)
#define CMD_GET_STATUS     0x10   // 请求获取状态反馈

// ---------- FDCAN 错误 / 总线关闭监测（由 FDCAN2_IT1_IRQn + 固定走 IT0 的错误中断驱动）----------
// 中断分工：IT0 = FIFO0 接收 + Error Warning / Bus Off（G4 无 ILS 位，固定走 IT0）
//           IT1 = Error Passive + 协议错误（经 HAL_FDCAN_ConfigInterruptLines 分配到 IT1）
#define FDCAN_ERR_FLAG_WARNING  0x01   // 错误警告 (EW)：TEC>96 或 REC>96
#define FDCAN_ERR_FLAG_PASSIVE  0x02   // 错误被动 (EP)：TEC>127 或 REC>127
#define FDCAN_ERR_FLAG_BUSOFF   0x04   // 总线关闭 (BO)：节点已离线，自动恢复后清除

extern volatile uint32_t fdcan_error_flags;   // 当前错误状态标志（bit 定义见上）
extern volatile uint32_t fdcan_busoff_count;  // 总线关闭累计次数
extern volatile uint8_t  fdcan_tec;           // 发送错误计数 (TEC)
extern volatile uint8_t  fdcan_rec;           // 接收错误计数 (REC)
extern volatile uint32_t fdcan_proto_err;     // 协议错误累计次数（仲裁/数据段）

// ---------- FDCAN 初始化与任务函数 ----------
void FDCAN_App_Init(void);
void FDCAN_App_Process(void);    // 主循环中调用，处理接收到的指令 + 总线关闭自动恢复
void FDCAN_SendStatus(void);     // 发送状态反馈

#endif