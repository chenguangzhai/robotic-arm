/**
  * 串口指令协议层：把 UART 字节流切成「一行一条命令」，解析后交给 Motion 执行
  *
  * 分层关系（三层互不知道对方内部）：
  *   uart_app.c   只负责搬字节（环形缓冲 + DMA，不知道什么叫命令）
  *   protocol.c   只负责切行 + 解析 + 分发（不知道 DMA 怎么工作，也不驱动舵机）
  *   motion.c     只负责「从当前点沿直线走到目标点」（不知道命令从哪来）
  *   main.c       主循环，按键和串口在这里汇合成同一份目标
  *
  * 为什么按 '\n' 切行，而不是按 DMA 的 IDLE 标志切帧：
  *   IDLE 只代表「总线静默了」，不代表「一条命令结束了」。连发
  *   "M 0.3 0.1 0\r\nM 0.2 0.1 0\r\n" 时两条命令很可能落在同一帧里，
  *   按帧切会把它们当成一条而解析失败。行缓冲天然处理跨帧的半条命令，
  *   所以这里完全不依赖 IDLE 语义。
  *
  * 指令集（大小写均可，参数之间空格/逗号都行，结尾 \n 或 \r\n）：
  *
  *   M x y z     移动到 (x,y,z)，单位【米】。例：M 0.28 0.10 0
  *               受理后立即回 OK，随后末端以 MOTION_SPEED 沿直线走过去，
  *               到位时主动推一行 DONE。注意 OK 的含义是「已受理」，
  *               不是「已到位」——要看 DONE。
  *   G 0|1       夹爪开合：0 = 张开（0°），1 = 收紧（默认 60°，见 protocol.c
  *               的 GRIP_ANGLE_CLOSE，需按实际工件微调）。
  *               只驱动 PA7（舵机 6）。PA6（舵机 5）是旋转爪子，
  *               固定在中位 90°，暂无指令控制。
  *               立即到位、不走插值（插值要 3 秒，夹爪等不起）。
  *               回 OK 表示已下发，不是「夹到位」——夹爪没有位置反馈。
  *   R <角度>    夹爪旋转（PA6 / 舵机 5），0~180 度。
  *               0°  = 夹爪平行于地面（上电初始姿态）
  *               90° = 夹爪垂直于地面
  *               例：R 90（转垂直）、R 45（斜 45°）、R 0（回平行）
  *               立即到位；超范围回 ERR:RANGE。
  *   ?           查询状态，回一行：
  *               S <x_mm> <y_mm> <z_mm> <底座> <肩> <肘> <腕> <夹爪> <旋转> <busy>
  *               busy = 1 表示还在运动，此时应等 DONE 再发下一条
  *
  * 错误码：
  *   ERR:CMD      无法识别的命令字
  *   ERR:ARG      参数个数不对或不是数字
  *   ERR:RANGE    坐标绝对值超过上限（挡住「把毫米当米发」）
  *   ERR:OUT      目标点超出机械臂工作空间
  *   ERR:PATH     目标点可达，但当前点到目标的直线中途穿出工作空间
  *   ERR:NOTIMPL  指令已识别但功能尚未实现
  *   ERR:LONG     单行超过 PROTO_LINE_MAX，整行丢弃
  *
  * 用法：
  *   Uart_Init(&huart1);      // 先起传输层
  *   Protocol_Init();
  *   while (1) { Protocol_Poll(); Motion_Tick(); ... }   // 每轮调一次，非阻塞
  */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

/* 一条命令的最大长度（含结尾 '\0'）。
 * 最长合法命令 "M -0.999 -0.999 -0.999" 只有 24 字节，64 留足余量。 */
#define PROTO_LINE_MAX      64

/* 协议挂在哪个串口上。换口只改这一行。 */
#define PROTO_UART          (&huart1)

/* 初始化：清空行缓冲和状态。必须在 Uart_Init() 之后调用。 */
void Protocol_Init(void);

/* 轮询：把缓冲里的字节取干净并逐字节喂给行解析器；
 * 顺便检查「上一条 M 是否已经走完」，走完就推一行 DONE。
 * 非阻塞、可每轮主循环调用，没有数据时开销只有一次 Uart_Read。 */
void Protocol_Poll(void);

/* 向 PROTO_UART 回一行（自动补 \r\n）。
 * 注意：内部用 vsnprintf，不要传 %f——本工程没开浮点 printf 支持，
 *       需要打印坐标请先换成整数毫米。 */
void Protocol_Reply(const char *fmt, ...);

#endif
