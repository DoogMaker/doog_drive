---
name: kth7823-single-frame-pipeline
overview: 把 KTH7823 读取改为单帧流水线（每帧读取都利用"返回上一命令回读"），并从代码中彻底移除 TIM3 编码器刷新链路（初始化、中断、回调、KTH7823_UpdateISR），编码器统一由主循环 1ms 节拍刷新；同步改写 README 更新记录。
todos:
  - id: single-frame-read
    content: 改造 KTH7823.c/.h：单帧流水线读取，删除 UpdateISR 与开关宏并更新注释
    status: completed
  - id: refresh-unconditional
    content: main.c 与 motor_control.c 去掉条件编译，主循环无条件刷新并删除 TIM3 启动与回调分支
    status: completed
    dependencies:
      - single-frame-read
  - id: remove-tim3
    content: 删除 main.c 的 MX_TIM3_Init 调用及 tim.c/tim.h/stm32g4xx_it.c/.h 中全部 TIM3 代码
    status: completed
    dependencies:
      - single-frame-read
  - id: comment-cleanup
    content: 清理 Protect.h/.c、dbg_monitor.h、uart_cmd.c 中 TIM3 相关注释，并用 [skill:lsp-code-analysis] 核对无残留
    status: completed
    dependencies:
      - refresh-unconditional
      - remove-tim3
  - id: readme-update
    content: 改写 README 2026-10-19 条目为最终方案，并同步删除 TIM3 外设行、更新优先级与模块表
    status: completed
    dependencies:
      - refresh-unconditional
      - remove-tim3
---

## 用户需求
把 KTH7823 编码器读取改为**单帧流水线**方式，并从代码中**彻底移除 TIM3 编码器刷新链路**；改动完成后同步写入 README。

## 核心要求
- **单帧流水线读取**：协议为"当前帧发送命令、返回上一命令帧的回读"，因此每次刷新只发 1 帧、直接用返回值，无需"发 2 帧取第 2 帧"。保持 `HAL_SPI_TransmitReceive` 不变（本次不做寄存器级改造）。
- **不改 SPI 时钟**：KTH7823 最高 SCLK 为 10MHz，现有 9MHz（PCLK2 36MHz ÷ 4）保持不变。
- **CS 时序约束**：CS 不允许连续拉低，两帧之间必须有 ≥50ns 高电平；现有"每帧一拉低一拉高"的结构满足，改造后须保持。
- **彻底删除 TIM3**：`MX_TIM3_Init`、`htim3`、`TIM3_IRQHandler`、TIM3 NVIC 配置、定时器回调分支、`KTH7823_UpdateISR` 全部删除；`Drive.ioc` 中的 TIM3 配置由用户自行在 CubeMX 删除（本次不动 `.ioc`）。
- **刷新节拍保持 1ms**：主循环仍保留 `DWT_DelayMs(1)`，刷新率 ≈1kHz，本次只减半单次读取耗时。
- **README 直接改写**：把 2026-10-19 那条"测试开关"记录改写为最终方案，不保留中间过程。

## 预期效果
- 单次读取线时由 2 帧 3.56µs 降为 **1 帧 1.78µs**；含 HAL 开销的单次刷新由约 10~13µs 降到约 **6~8µs**。
- 刷新率仍为 ≈1kHz，角度样本额外滞后一个命令帧（≈1.78µs，可忽略）。
- 代码中不再出现任何 `TIM3` / `htim3` / `KTH7823_UpdateISR` 引用（含注释），编码器刷新路径唯一且清晰：主循环 → `KTH7823_UpdateAngle()` + `KTH7823_UpdateSpeed()`。
- 保护逻辑不变：`Protect_EncoderStallCheck()` 仍按 `kth7823_raw_fault_cnt ≥ 8` 判失效，`encoder_update_count` 仍由 `UpdateAngle` 递增。


## 技术栈
沿用现有工程：STM32G474 + STM32G4 HAL + Keil MDK-ARM（`MDK-ARM/Drive.uvprojx`），C 语言，无新增依赖、不新增文件。

## 实现方案
本次是"协议层 + 调度层"的收敛改造，核心是把"两帧读一次"改成"一帧读一次"，并把编码器刷新彻底固定到主循环，删除 TIM3 相关的一切代码，使全工程只剩一条刷新路径。

关键决策与理由：

1. **单帧流水线而不是保留双帧兼容**
   协议已明确"返回上一命令帧的回读"，且每次发送的都是同一个读命令（`0x0000`），因此连续单帧读取的返回值始终是"上一拍命令对应的角度"，语义与现有"发两帧取第二帧"完全等价，只是结果提前一拍交付。改造后单次线时减半，是本次唯一的速度收益来源（SPI 时钟已封顶 9MHz）。
   若保留双帧版本会留下无调用者的死代码，与"彻底移除"的目标冲突，故直接替换并补注释说明流水线原理。

2. **保持 `HAL_SPI_TransmitReceive` 与逐帧 CS 翻转**
   用户明确本次只改帧数。`spi_exchange_frame()` 的"CS 低 → 传输 16bit → CS 高"结构不变，天然满足"两帧之间 ≥50ns 高电平"（HAL 调用开销远大于 50ns）；也避免寄存器级改造可能把 CS 高电平压到 50ns 以下的风险。

3. **TIM3 全链路删除（按用户确认的范围）**
   删除分为三层，避免出现"调用还在、函数已删"的编译错误：
   - 运行时：`MotorControl_Init()` 的 `HAL_TIM_Base_Start_IT(&htim3)`、`HAL_TIM_PeriodElapsedCallback` 的 TIM3 分支；
   - 驱动层：`tim.c` 的 `htim3` 句柄与 `MX_TIM3_Init()`、`HAL_TIM_Base_MspInit/MspDeInit` 的 TIM3 分支、`tim.h` 的 extern 与原型；
   - 中断层：`stm32g4xx_it.c` 的 `extern htim3` 与 `TIM3_IRQHandler()`、`stm32g4xx_it.h` 的原型；
   - 调用点：`main.c` 的 `MX_TIM3_Init();`。
   `tim.c` 仍被工程引用（TIM1），无需改 MDK 工程。

4. **去掉编译期开关，路径唯一化**
   上轮引入的 `ENCODER_REFRESH_IN_MAIN_LOOP` 已完成使命，删除宏、其长注释以及 `main.c`/`motor_control.c` 中的全部 `#if` 包裹，主循环刷新改为无条件执行，避免"两种模式并存"的长期维护成本。

5. **注释与文档同步**
   `Protect.c`、`Protect.h`、`dbg_monitor.h`、`uart_cmd.c` 中"编码器刷新已由 TIM3 中断完成"一类注释会与最终代码矛盾，需一并改为"主循环刷新"，否则后续排查会被误导。

性能与可靠性说明：
- 单次刷新耗时 ≈ 1.78µs（线时）+ 4~8µs（HAL 开销）+ 1.5~3µs（`sinf/cosf/sqrtf/atan2f` 滤波）≈ **6~8µs**；主循环 1ms 节拍下 CPU 占用不足 1%。
- 20kHz 控制中断读取的角度最旧 ≈1ms（与改造前一致），控制动态特性无变化，便于与旧数据对比。
- 单帧流水线不改变"每拍都更新 `KTH7823_Angle_Rad`/`Angle_Multiturn`/心跳"的数据流，保护判据、解包裹野值剔除（`KTH7823_UNWRAP_MAX_DELTA`）均无需调整。
- 上电时序：`KTH7823_Init()` 保留一次 `KTH7823_ReadAngleRaw()`（用于丢弃上电无效值并建立流水线首拍）+ `DWT_DelayMs(10)`，因此第一次 `UpdateAngle` 拿到的是 Init 那一帧的回读（有效，滞后一帧）。

## 实施注意事项
- 删除 `tim.c` 的 TIM3 分支时注意：`HAL_TIM_Base_MspInit` 中 TIM1 分支的 `}` 与该分支的缩进保持现状，避免误删 TIM1 的 NVIC/时钟配置（TIM1 是 20kHz 控制中断的唯一来源，绝不能动）。
- `HAL_TIM_PeriodElapsedCallback` 删除 `else if (TIM3)` 后只剩 TIM1 分支，需确认括号闭合与缩进正确。
- `main.c` 主循环中的刷新块去掉 `#if/#endif` 后，`DWT_DelayMs(1)` 仍在 `HAL_IWDG_Refresh()` 之前，喂狗每轮都能执行。
- 不改：`spi.c`（9MHz）、`Drive.ioc`、`MDK-ARM` 工程、FOC/保护算法、滤波系数、1ms 节拍。
- 本机无 ARM 工具链，无法编译验证；需在 Keil 中编译确认，并可用逻辑分析仪抽查 CS 帧间高电平 ≥50ns（HAL 结构下天然满足）。

## 目录结构（本次改动文件）
```
doog_drive/
├─ Core/
│  ├─ Inc/
│  │  ├─ kth7823.h            # [MODIFY] 删除 ENCODER_REFRESH_IN_MAIN_LOOP 宏与长注释、
│  │  │                       #          KTH7823_UpdateISR 声明；更新 encoder_update_count 注释
│  │  ├─ tim.h                # [MODIFY] 删除 extern htim3 与 MX_TIM3_Init 原型
│  │  ├─ stm32g4xx_it.h       # [MODIFY] 删除 TIM3_IRQHandler 原型
│  │  ├─ Protect.h            # [MODIFY] 编码器监测注释改为"主循环刷新"
│  │  └─ dbg_monitor.h        # [MODIFY] encoder_update_count 注释去掉"TIM3 中断内"
│  └─ Src/
│     ├─ KTH7823.c            # [MODIFY] KTH7823_ReadAngleRaw 改单帧流水线并补原理/CS 时序注释；
│     │                       #          Init 首帧注释；删除 KTH7823_UpdateISR 定义；
│     │                       #          心跳注释去掉"中断内"
│     ├─ main.c               # [MODIFY] 删除 MX_TIM3_Init() 调用；主循环刷新块去掉 #if 改无条件执行
│     ├─ motor_control.c      # [MODIFY] 删除 TIM3 启动调用与回调中的 TIM3 分支
│     ├─ tim.c                # [MODIFY] 删除 htim3、MX_TIM3_Init()、
│     │                       #          MspInit/MspDeInit 的 TIM3 分支（保留 TIM1）
│     ├─ stm32g4xx_it.c       # [MODIFY] 删除 extern htim3 与 TIM3_IRQHandler()
│     ├─ Protect.c            # [MODIFY] 第 73、80 行注释改为"主循环刷新"
│     └─ uart_cmd.c           # [MODIFY] 第 222、239、313 行注释改为"主循环刷新"
└─ README.md                  # [MODIFY] 改写 2026-10-19 更新记录为最终方案；
                              #          删除 TIM3 外设行、更新中断优先级行、修正模块表描述
```


## Agent Extensions
### Skill
- **lsp-code-analysis**
  - Purpose: 在删除 TIM3 链路后做全局引用核对，确认 `TIM3` / `htim3` / `MX_TIM3_Init` / `KTH7823_UpdateISR` / `ENCODER_REFRESH_IN_MAIN_LOOP` 已无任何符号级残留（比文本搜索更可靠，能识别宏与声明引用）
  - Expected outcome: 输出一份"无残留符号"的核对结论；若仍有引用，给出精确文件与位置以便修复
