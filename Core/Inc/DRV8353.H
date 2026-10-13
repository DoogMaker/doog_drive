#ifndef drv8353_H
#define drv8353_H
#include "stm32g4xx_hal.h"

// DRV8353 PWM 控制脚 (PB12)
#define DRV8353_PWM_ctrl_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)	
#define DRV8353_PWM_ctrl_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

// DRV8353F 使能脚 (PA11)
#define DRV8353_Enable() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
#define DRV8353_Disable() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)

// DRV8353F 关键寄存器地址
#define DRV8353_REG_STATUS1         0x00
#define DRV8353_REG_STATUS2         0x01
#define DRV8353_REG_DRIVER_CTRL     0x02
#define DRV8353_REG_GATE_DRIVE_HS   0x03
#define DRV8353_REG_GATE_DRIVE_LS   0x04
#define DRV8353_REG_OCP_CTRL        0x05
#define DRV8353_REG_CSA_CTRL        0x06  	// 使用 INA240 外部采样时禁用内部 CSA
#define DRV8353_REG_DRIVER_CONFIG   0x07  	// 仅DRV8353F

// 关键寄存器配置值

// DRIVER_CTRL (0x02): bit5-6 = PWM 模式, 01=3x PWM
#define DRV8353_PWM_MODE_3X         (0x01 << 5)

// DRIVER_CTRL (0x02): bit0 = CLR_FLT 清故障
#define DRV8353_CLR_FAULT           (0x01 << 0)

// DRIVER_CTRL (0x02): bit10 = OCP_ACT, 1=过流时关断所有半桥
#define DRV8353_OCP_ACT_ALL         (0x01 << 10)

// GATE_DRIVE_HS (0x03) / GATE_DRIVE_LS (0x04)
// 目标 MOSFET: Qg ≈ 37nC (typ), Vplateau ≈ 4.6V
// 计算: t_rise ≈ Qg / I_DRIVE → 期望开关时间 ~200~500ns
// 灌电流>拉电流以加快关断, 避免 HS/LS 直通
//   bit9-8 TDRIVE=01 (1000ns) 
//   bit7-4 IDRIVEP_HS/LS
//   bit3-0 IDRIVEN_HS/LS
//   0x0148 = 0001_0100_1000b -> TDRIVE=01(1000ns), IDRIVEP_HS=0100(300mA), IDRIVEN_HS=1000(1100mA)
#define DRV8353_DRIVE_STRENGTH_HS   0x0148  // HS: TDRIVE=1000ns, IDRIVEP=300mA, IDRIVEN=1100mA
//   0x0136 = 0001_0011_0110b -> TDRIVE=01(1000ns), IDRIVEP_LS=0011(150mA), IDRIVEN_LS=0110(800mA)
#define DRV8353_DRIVE_STRENGTH_LS   0x0136  // LS: TDRIVE=1000ns, IDRIVEP=150mA, IDRIVEN=800mA

// OCP_CTRL (0x05) = 0x002D
//   bit9-8 DEAD_TIME=00 (50ns 死区)
//   bit7-6 OCP_MODE=00  (锁存模式: 故障后持续关断, 必须由 CLR_FLT 清除; 与 main.c 的软件重试状态机配合)
//   bit5-4 OCP_DEG=10   (4µs 抗尖峰时间)
//   bit3-0 VDS_LVL=1101 (1.0V 过流阈值, typ)
//   0x002D = 0000_0000_0010_1101b -> OCP_MODE=00(锁存), DEAD_TIME=00, OCP_DEG=10, VDS_LVL=1101
#define DRV8353_OCP_CONFIG          0x002D

// CSA_CTRL (0x06): 使用 INA240 外部采样, 应禁用内部 CSA 的过流故障
#define DRV8353_CSA_DISABLE         (0x01 << 5)  // = 0x0020

// ==================== 驱动自检（DRV8353_SelfCheck）====================
// SPI 返回帧：低 11 位为寄存器数据，高 5 位为地址/读写回显，比对时只取低 11 位
#define DRV8353_REG_DATA_MASK       0x07FF

// 读回比对用的“可写位”掩码（避免只读/保留位造成误报）：
//   DRIVER_CTRL 只关心 PWM 模式(3x) 与 OCP_ACT，其余位不参与比对
#define DRV8353_DRIVER_CTRL_MASK    (DRV8353_PWM_MODE_3X | DRV8353_OCP_ACT_ALL)
//   STATUS1/STATUS2 低 11 位均为故障/告警位，任一置位即视为异常
#define DRV8353_STATUS_FAULT_MASK   0x07FF

// DRV8353_SelfCheck() 返回值（按位诊断），0 = 全部通过
#define DRV8353_SC_OK               0x0000
#define DRV8353_SC_DRIVER_CTRL_ERR  0x0001   // DRIVER_CTRL 读回不符（PWM 模式/OCP_ACT）
#define DRV8353_SC_GATE_HS_ERR      0x0002   // GATE_DRIVE_HS 读回不符
#define DRV8353_SC_GATE_LS_ERR      0x0004   // GATE_DRIVE_LS 读回不符
#define DRV8353_SC_OCP_ERR          0x0008   // OCP_CTRL 读回不符
#define DRV8353_SC_CSA_ERR          0x0010   // CSA_CTRL 读回不符
#define DRV8353_SC_STATUS1_FAULT    0x0020   // STATUS1 报告故障位（详情见 status1）
#define DRV8353_SC_STATUS2_FAULT    0x0040   // STATUS2 报告故障位（详情见 status2）

// 自检读回快照（低 11 位有效），供上层打印详细结果
typedef struct {
    uint16_t driver_ctrl;   // DRIVER_CTRL (0x02) 读回值
    uint16_t gate_hs;       // GATE_DRIVE_HS (0x03) 读回值
    uint16_t gate_ls;       // GATE_DRIVE_LS (0x04) 读回值
    uint16_t ocp_ctrl;      // OCP_CTRL (0x05) 读回值
    uint16_t csa_ctrl;      // CSA_CTRL (0x06) 读回值
    uint16_t status1;       // STATUS1 (0x00) 读回值
    uint16_t status2;       // STATUS2 (0x01) 读回值
    uint16_t result;        // 本次自检诊断位（DRV8353_SC_*）
} DRV8353_SelfCheck_Report_t;

extern DRV8353_SelfCheck_Report_t drv8353_selfcheck;   // 最近一次自检读回快照

// 函数声明	
void DRV8353_Init(void);
void DRV8353_WriteReg(uint8_t reg, uint16_t data);
uint16_t DRV8353_ReadReg(uint8_t reg);
void DRV8353_ClearFault(void);
void DRV8353_Recover(void);

// 驱动自检：读回配置寄存器按可写位掩码比对 + 检查 STATUS 故障位
// 返回按位诊断结果（DRV8353_SC_*，0 = 全部通过）；本函数只校验并刷新快照，不打印
uint16_t DRV8353_SelfCheck(void);

#endif
