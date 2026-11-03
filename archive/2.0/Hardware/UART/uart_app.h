/**
  * 串口收发封装：环形缓冲 + DMA 循环接收，DMA 永不停止
  *
  * 设计目标：一套代码管所有串口。将来接 WiFi / LoRa / NB-IoT / 4G 都直接调这里，
  *           换模块只改协议层，收发层一个字不用动。
  *
  * 和「收一帧停一次」的区别：
  *   旧方案一帧收完 DMA 就停，主循环来不及取，这段时间的字节全丢；
  *   本方案 DMA 一直在跑，收和取彻底解耦，主循环慢一点只是数据在缓冲里多待一会儿。
  *
 * 用法：
 *   Uart_Init(&huart1);                          // 注册并启动接收
 *   uint8_t buf[64];
 *   uint16_t n = Uart_Read(&huart1, buf, sizeof(buf));   // 按需取 N 字节
 *
 * 本工程只有 USART1（PA9=TX / PA10=RX，115200，DMA1_Channel5 循环接收），
 * 业务口和日志口是同一个，所以回显调试必须关闭（见下方 UART_ECHO_DEBUG）。
 */
#ifndef __UART_APP_H
#define __UART_APP_H

#include "main.h"
#include "usart.h"
#include "dma.h"
#include <stdint.h>

/* 每个串口接收环形缓冲的字节数。
   必须是 2 的幂（内部用 & 取模，比 % 快）。
   1024 足够放一条几百字节的 MQTT 报文，RAM 紧张时可以降到 512。 */
#define UART_RX_BUF_SIZE    1024

/* 同时管理的串口数量上限。USART1 一般留给日志，所以 2~3 就够。
   每多一个端口多占 UART_RX_BUF_SIZE 字节 RAM。 */
#define UART_PORT_MAX       3

/* 发送超时（毫秒） */
#define UART_TX_TIMEOUT     1000

/* ==================== 调试回显 ====================
   1 = 把 Uart_Read() 取到的数据原样转发到 UART_ECHO_PORT 打印
   0 = 关闭，这段代码在编译期整段消失，零运行时开销

   本工程只有一个 USART1，业务口和日志口是同一个端口，
   开着回显既没意义（自己发给自己）也会浪费带宽，所以固定为 0。
   将来如果加了第二个串口做日志，把它改回 1 并指定 UART_ECHO_PORT。 */
#define UART_ECHO_DEBUG     0

/* 回显输出到哪个串口（仅当 UART_ECHO_DEBUG = 1 时有效） */
#define UART_ECHO_PORT      (&huart1)

/* 函数返回值 */
#define UART_OK             0
#define UART_ERR_PARAM      (-1)    /* 参数为空 */
#define UART_ERR_NOBUF      (-2)    /* 端口表已满，UART_PORT_MAX 不够用 */
#define UART_ERR_STATE      (-3)    /* HAL 状态异常，启动接收失败 */

int8_t   Uart_Init(UART_HandleTypeDef *huart);
void     Uart_DeInit(UART_HandleTypeDef *huart);

uint16_t Uart_Available(UART_HandleTypeDef *huart);
uint8_t  Uart_FrameReady(UART_HandleTypeDef *huart);
uint16_t Uart_Read(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len);
void     Uart_FlushRx(UART_HandleTypeDef *huart);
uint32_t Uart_GetOverflow(UART_HandleTypeDef *huart);

void     Uart_Write(UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t len);
void     Uart_Printf(UART_HandleTypeDef *huart, const char *format, ...);

#endif
