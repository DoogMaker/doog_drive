#ifndef __UART_CMD_H
#define __UART_CMD_H

#include "main.h"

// 命令解析结果
typedef enum {
    CMD_OK = 0,
    CMD_UNKNOWN,
    CMD_PARAM_ERROR,
    CMD_BUFFER_OVERFLOW
} CmdResult_t;

// 初始化串口命令处理
void UART_Cmd_Init(void);

// 主循环中调用，处理接收到的命令
void UART_Cmd_Process(void);

// 发送状态反馈（故障寄存器、电流、速度等）
void UART_SendStatus(void);

// 自动停止（主循环调用）：v= 设为非0 后超时自动把电压归零，防测试时忘记停机
void UART_Cmd_AutoStop(void);

// 发送字符串（非阻塞：拷入发送环形缓冲后立即返回，可在中断中调用）
void UART_SendString(const char* str);

// 发送缓冲剩余可写入字节数（用于背压/丢包判断）
uint16_t UART_TxFree(void);

// 因发送缓冲满而被整条丢弃的消息数（正常调试应保持 0）
uint32_t UART_TxDropCount(void);

// 主循环调用：发送链路兜底恢复（正常由中断推进，无需手动干预）
void UART_Tx_Poll(void);

// 主循环调用：对齐完成结果上报（对齐逻辑在中断里置标志，本函数负责格式化+发送）
void UART_ReportAlignResult(void);

// 上电参数校验打印（启动序列调用）：逐项输出 PID/限位/阈值/母线电压/温度/关键常量及 PASS/WARN
void UART_PrintSystemReport(void);

#endif