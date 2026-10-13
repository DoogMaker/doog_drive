#include "main.h"
#include "spi.h"
#include "DRV8353.H"
#include "tim.h"
#include "dwt_delay.h"   // DWT_DelayMs


/**
 * @brief 组合 16 位 SPI 命令
 * @param reg   4 位寄存器地址
 * @param data  11 位数据
 * @param read  1=读操作，0=写操作
 */
 
static uint16_t status;

static uint16_t DRV8353_BuildCmd(uint8_t reg, uint16_t data, uint8_t read)
{
    return (read << 15) | ((reg & 0x0F) << 11) | (data & 0x07FF);
}

// 向 DRV8353F 写入寄存器
void DRV8353_WriteReg(uint8_t reg, uint16_t data)
{
    uint16_t cmd = DRV8353_BuildCmd(reg, data, 0);
    uint16_t tx = cmd;   // 16 位单帧，HAL 16 位模式按 sizeof(uint16_t) 步进
    uint16_t rx = 0;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
	DWT_DelayMs(1);
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&tx, (uint8_t *)&rx, 1, 100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_SET);
}


// 读取 DRV8353F 寄存器
uint16_t DRV8353_ReadReg(uint8_t reg)
{
    uint16_t cmd = DRV8353_BuildCmd(reg, 0, 1);
    uint16_t tx = cmd;   // 16 位单帧，HAL 16 位模式按 sizeof(uint16_t) 步进
    uint16_t rx = 0;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
    DWT_DelayMs(1);
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&tx, (uint8_t *)&rx, 1, 100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_SET);

    return rx;   // HAL 16 位模式已写入完整 16 位
}

// 向 DRV8353F 写入全部运行配置寄存器
// 注意：EN 拉低进入睡眠模式时所有寄存器恢复默认值，故 Init 与 Recover 都必须调用本函数
static void DRV8353_ConfigRegs(void)
{
	// 配置 3x PWM 模式及 OCP_ACT (OCP_ACT 属于 DRIVER_CTRL bit10, 过流时关断所有半桥)
	DRV8353_PWM_ctrl_OFF();
	DRV8353_WriteReg(DRV8353_REG_DRIVER_CTRL, DRV8353_PWM_MODE_3X | DRV8353_OCP_ACT_ALL);

	// 配置栅极驱动强度（HS/LS 分开，根据 MOSFET 选型调整）
	DRV8353_WriteReg(DRV8353_REG_GATE_DRIVE_HS, DRV8353_DRIVE_STRENGTH_HS);
	DRV8353_WriteReg(DRV8353_REG_GATE_DRIVE_LS, DRV8353_DRIVE_STRENGTH_LS);

	// 配置过流保护（OCP_CTRL: 锁存模式 + 死区 + 抗尖峰 + VDS阈值）
	DRV8353_WriteReg(DRV8353_REG_OCP_CTRL, DRV8353_OCP_CONFIG);

	// 禁用内部CSA
	DRV8353_WriteReg(DRV8353_REG_CSA_CTRL, DRV8353_CSA_DISABLE);
}

// DRV8353F 初始化
void DRV8353_Init(void)
{	
    DRV8353_Enable();

	// 等待芯片上电稳定
	DWT_DelayMs(100);

	DRV8353_ConfigRegs();

	// 读取状态寄存器确认配置成功
	status = DRV8353_ReadReg(DRV8353_REG_STATUS1);
	
    // 检查 FAULT 位 (STATUS1 bit10 = 0x0400)
    if (status & 0x0400) {
        // 清除故障（写 CLR_FLT 位）
		DRV8353_ClearFault();
     }
}

// 清除故障
void DRV8353_ClearFault(void)
{
    // 读取当前 DRIVER_CTRL 值，置位 CLR_FLT，再写回
    uint16_t ctrl = DRV8353_ReadReg(DRV8353_REG_DRIVER_CTRL);
    DRV8353_WriteReg(DRV8353_REG_DRIVER_CTRL, ctrl | DRV8353_CLR_FAULT);
    // 等待清除完成
	DWT_DelayMs(10);
}

/**
  * @brief  过流/故障后的驱动恢复动作（不含重试计数与锁定判断）
  *         清故障锁存位 -> 恢复 PWM 控制脚 -> 使能驱动 -> 重启 TIM1 三路 PWM
  * @note   调用前需确认故障源已排除；若需状态机/锁定逻辑，见 FOC.c 的 FOC_STATE_FAULT 状态机
  */
void DRV8353_Recover(void)
{
    // 故障恢复时必须重新写入所有配置寄存器：
    // EN 被拉低(睡眠)后 DRV8353F 的 DRIVER_CTRL/GATE_DRIVE/OCP/CSA 全部恢复默认，
    // 若不重写，恢复后将以默认配置(6x PWM、非锁存 OCP、内部 CSA 使能)吃 PWM，可能直通炸管。
    DRV8353_Enable();                   // PA11 拉高，退出睡眠(EN=1)
    DWT_DelayMs(2);                     // tREADY：EN 拉高到 SPI 就绪(数据手册典型 1ms)
    DRV8353_ConfigRegs();               // 重新写入完整运行配置，覆盖睡眠清掉的默认值
    DRV8353_ClearFault();               // 清 DRV8353 故障锁存位(需在 EN 有效时写)
    DRV8353_PWM_ctrl_ON();              // PB12 拉高，恢复 PWM 控制脚
    // 恢复前清零占空比，避免残留值导致恢复瞬间满电流冲击
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

// 最近一次自检读回快照（定义于此处，声明见 DRV8353.H）
DRV8353_SelfCheck_Report_t drv8353_selfcheck;

/**
  * @brief  驱动自检：读回配置寄存器按“可写位掩码”比对 + 检查 STATUS 故障位
  * @retval 按位诊断结果（DRV8353_SC_*），0 = 全部通过
  * @note   仅做校验与快照，不打印（保持本模块与串口解耦）；
  *         调用前应已由 DRV8353_Init() / DRV8353_Recover() 完成配置写入。
  */
uint16_t DRV8353_SelfCheck(void)
{
    uint16_t result = DRV8353_SC_OK;

    // 读回各配置/状态寄存器（只取低 11 位数据）
    uint16_t driver_ctrl = DRV8353_ReadReg(DRV8353_REG_DRIVER_CTRL)    & DRV8353_REG_DATA_MASK;
    uint16_t gate_hs     = DRV8353_ReadReg(DRV8353_REG_GATE_DRIVE_HS)  & DRV8353_REG_DATA_MASK;
    uint16_t gate_ls     = DRV8353_ReadReg(DRV8353_REG_GATE_DRIVE_LS)  & DRV8353_REG_DATA_MASK;
    uint16_t ocp_ctrl    = DRV8353_ReadReg(DRV8353_REG_OCP_CTRL)       & DRV8353_REG_DATA_MASK;
    uint16_t csa_ctrl    = DRV8353_ReadReg(DRV8353_REG_CSA_CTRL)       & DRV8353_REG_DATA_MASK;
    uint16_t status1     = DRV8353_ReadReg(DRV8353_REG_STATUS1)        & DRV8353_REG_DATA_MASK;
    uint16_t status2     = DRV8353_ReadReg(DRV8353_REG_STATUS2)        & DRV8353_REG_DATA_MASK;

    // 刷新快照，供上层打印详细结果
    drv8353_selfcheck.driver_ctrl = driver_ctrl;
    drv8353_selfcheck.gate_hs     = gate_hs;
    drv8353_selfcheck.gate_ls     = gate_ls;
    drv8353_selfcheck.ocp_ctrl    = ocp_ctrl;
    drv8353_selfcheck.csa_ctrl    = csa_ctrl;
    drv8353_selfcheck.status1     = status1;
    drv8353_selfcheck.status2     = status2;

    // 配置读回比对：DRIVER_CTRL 只比较可写位（PWM 模式/OCP_ACT 应全部置位）
    if ((driver_ctrl & DRV8353_DRIVER_CTRL_MASK) != DRV8353_DRIVER_CTRL_MASK) {
        result |= DRV8353_SC_DRIVER_CTRL_ERR;
    }
    // GATE_DRIVE/OCP/CSA：期望值为完整写入值，按低 11 位精确比对
    if (gate_hs  != (DRV8353_DRIVE_STRENGTH_HS & DRV8353_REG_DATA_MASK)) result |= DRV8353_SC_GATE_HS_ERR;
    if (gate_ls  != (DRV8353_DRIVE_STRENGTH_LS & DRV8353_REG_DATA_MASK)) result |= DRV8353_SC_GATE_LS_ERR;
    if (ocp_ctrl != (DRV8353_OCP_CONFIG       & DRV8353_REG_DATA_MASK)) result |= DRV8353_SC_OCP_ERR;
    if (csa_ctrl != (DRV8353_CSA_DISABLE      & DRV8353_REG_DATA_MASK)) result |= DRV8353_SC_CSA_ERR;

    // 状态寄存器：任一故障/告警位置位即标记（具体位由上层按 status1/status2 解析）
    if (status1 & DRV8353_STATUS_FAULT_MASK) result |= DRV8353_SC_STATUS1_FAULT;
    if (status2 & DRV8353_STATUS_FAULT_MASK) result |= DRV8353_SC_STATUS2_FAULT;

    drv8353_selfcheck.result = result;
    return result;
}
