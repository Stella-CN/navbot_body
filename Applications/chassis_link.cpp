#include "chassis_link.h"
#include "link_protocol.h"
#include "imu_diagnostics.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "c30d_safety.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#define RX_SIZE 4096U
#define DMA_SIZE 512U
#define RESPONSE_COUNT 4U
/* ISR owns producer and DMA cursor; task owns consumer, protocol and TX.
 * DMA never addresses CCM. Overflow drops new bytes and invalidates parser. */
static uint8_t dma_rx[DMA_SIZE], rx[RX_SIZE], tx[1024];
static volatile uint32_t produced, consumed, rx_overflows, uart_errors, rx_restart;
static uint16_t dma_cursor;
static LinkParser parser;
static LinkFrame incoming, outgoing, responses[RESPONSE_COUNT];
static uint32_t response_head, response_tail, response_drops, tx_drops, tx_errors;
static uint32_t session, sequence, last_rx_seq, have_rx_seq, rejected, accepted;
static uint32_t last_telemetry, last_attitude, last_status, tx_started, last_tick, tick_high;
static uint64_t boot_id;
static size_t tx_used;
static uint64_t now_us;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart != &huart3 || size > DMA_SIZE)
        return;
    unsigned count = size >= dma_cursor ? (unsigned)size - dma_cursor : DMA_SIZE - dma_cursor + size;
    if (count > RX_SIZE - (uint32_t)(produced - consumed))
    {
        ++rx_overflows;
    }
    else
    {
        uint32_t end = produced;
        for (unsigned i = 0; i < count; ++i)
            rx[end++ % RX_SIZE] = dma_rx[(dma_cursor + i) % DMA_SIZE];
        __DMB();
        produced = end;
    }
    dma_cursor = size;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart3)
    {
        ++uart_errors;
        rx_restart = 1;
    }
}
static uint32_t random_word(void)
{
    for (unsigned i = 0; i < 1000000U; ++i)
    {
        uint32_t status = RNG->SR;
        if ((status & (RNG_SR_CECS | RNG_SR_SECS)) != 0)
            C30D_TemplatePanic();
        if ((status & RNG_SR_DRDY) != 0)
            return RNG->DR;
    }
    C30D_TemplatePanic();
}
static void receive_start(void)
{
    taskENTER_CRITICAL();
    (void)HAL_UART_AbortReceive(&huart3);
    consumed = produced = 0;
    dma_cursor = 0;
    parser.used = 0;
    rx_restart = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_rx, DMA_SIZE) != HAL_OK)
        rx_restart = 1;
    taskEXIT_CRITICAL();
}
void Navbot_LinkInit(void)
{
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG->CR = RNG_CR_RNGEN;
    uint32_t low = random_word();
    boot_id = (uint64_t)random_word() << 32 | low;
    RNG->CR = 0;
    __HAL_RCC_RNG_CLK_DISABLE();
    last_tick = osKernelGetTickCount();
    receive_start();
}
static void append(LinkFrame *f)
{
    f->session = session;
    f->sequence = sequence++;
    size_t n = 24U + f->length;
    if (tx_used + n <= sizeof(tx))
        tx_used += Link_Encode(tx + tx_used, f);
    else
        ++tx_drops;
}
static LinkFrame *response(const LinkFrame *request, uint8_t id, uint16_t length)
{
    if (response_head - response_tail == RESPONSE_COUNT)
    {
        ++response_drops;
        return NULL;
    }
    LinkFrame *f = &responses[response_head++ % RESPONSE_COUNT];
    memset(f, 0, sizeof(*f));
    f->id = id;
    f->length = length;
    f->timestamp_us = now_us;
    /* Responses carry request sequence explicitly; stream sequence stays independent. */
    Link_Put(f->payload, request->sequence, 4);
    return f;
}
static void ack(const LinkFrame *request, uint8_t result)
{
    LinkFrame *f = response(request, 0x7f, 6);
    if (f)
    {
        f->payload[4] = request->id;
        f->payload[5] = result;
    }
}
static void handle(const LinkFrame *f)
{
    if (f->id == 1 && f->length == 0 && f->session != 0)
    {
        if (session != f->session)
        {
            session = f->session;
            have_rx_seq = 0;
            response_head = response_tail = 0;
        }
    }
    if (session == 0 || f->session != session)
    {
        ++rejected;
        return;
    }
    uint32_t delta = f->sequence - last_rx_seq;
    if (have_rx_seq && (delta == 0 || delta >= 0x80000000U))
    {
        ++rejected;
        ack(f, 3);
        return;
    }
    have_rx_seq = 1;
    last_rx_seq = f->sequence;
    ++accepted;
    if (f->id == 1)
    {
        if (f->length != 0)
        {
            ack(f, 2);
            return;
        }
        LinkFrame *reply = response(f, 1, 36);
        if (reply)
        {
            Link_Put(reply->payload + 4, boot_id, 8);
            Link_Put(reply->payload + 12, 0x00010000U, 4); /* bench firmware 1.0.0 */
            Link_Put(reply->payload + 16, 0x0f, 4);        /* raw, attitude, time, bench-disabled */
            Link_Put(reply->payload + 20, 1000, 4);        /* timestamp resolution us */
            Link_Put(reply->payload + 24, 100, 2);
            Link_Put(reply->payload + 26, 50, 2);
            Link_Put(reply->payload + 28, 100, 2);
            Link_Put(reply->payload + 30, 10, 2);
            reply->payload[32] = 0xea;
            reply->payload[33] = 0; /* IMU expected ID; no motors */
            reply->payload[34] = 1; /* chip axes, no mounting transform */
            reply->payload[35] = 1; /* timestamp denotes read-transaction start */
        }
    }
    else if (f->id == 2)
    {
        if (f->length != 8)
        {
            ack(f, 2);
            return;
        }
        LinkFrame *reply = response(f, 2, 28);
        if (reply)
        {
            memcpy(reply->payload + 4, f->payload, 8);
            Link_Put(reply->payload + 12, now_us, 8);
            /* t3 filled immediately before batching for DMA. */
        }
    }
    else if (f->id == 0x12)
    {
        if (f->length != 0)
        {
            ack(f, 2);
            return;
        }
        C30D_ForceMotorIdle();
        ack(f, 0);
    }
    else if (f->id == 0x10 || f->id == 0x11 || f->id == 0x13 || f->id == 0x31)
    {
        ack(f, 4); /* bench mode never accepts motion or configuration */
    }
    else
    {
        ack(f, 1);
    }
}
static uint64_t sample_time(uint32_t now, uint32_t sampled)
{
    uint64_t age = (uint64_t)(uint32_t)(now - sampled) * 1000U;
    return age <= now_us ? now_us - age : 0;
}
static void telemetry(uint32_t now)
{
    C30D_ImuSnapshot imu;
    Navbot_AttitudeSnapshot attitude;
    Navbot_ImuSnapshot(&imu, &attitude);
    LinkFrame *f = &outgoing;
    memset(f, 0, sizeof(*f));
    f->id = 0x20;
    f->length = 52;
    f->timestamp_us = now_us;
    /* Wheel valid mask zero: no configured/connected motor encoders. */
    append(f);
    f->id = 0x21;
    f->length = 28;
    f->timestamp_us = sample_time(now, imu.timestamp_ms);
    Link_Put(f->payload, imu.valid && (uint32_t)(now - imu.timestamp_ms) < 30U ? 1U : 0U, 4);
    for (unsigned i = 0; i < 3; ++i)
    {
        Link_Float(f->payload + 4 + 4 * i, imu.gyro_rad_s[i]);
        Link_Float(f->payload + 16 + 4 * i, imu.accel_m_s2[i]);
    }
    append(f);
    if ((uint32_t)(now - last_attitude) >= 20U)
    {
        last_attitude = now;
        f->id = 0x22;
        f->length = 20;
        f->timestamp_us = sample_time(now, attitude.timestamp_ms);
        Link_Put(f->payload, imu.valid && attitude.valid && (uint32_t)(now - imu.timestamp_ms) < 30U && (uint32_t)(now - attitude.timestamp_ms) < 30U ? 1U : 0U, 4);
        for (unsigned i = 0; i < 4; ++i)
            Link_Float(f->payload + 4 + i * 4, attitude.q[(i + 1) % 4]);
        append(f);
    }
    if ((uint32_t)(now - last_status) >= 100U)
    {
        last_status = now;
        f->id = 0x23;
        f->length = 80;
        f->timestamp_us = now_us;
        const uint32_t values[20] = {
            0, 0, imu.health, imu.last_error, imu.sequence, imu.timestamp_ms, attitude.calibration_state,
            attitude.heading_epoch, parser.crc_errors, parser.length_errors, parser.version_errors,
            parser.timeouts, rx_overflows, uart_errors, tx_drops, response_drops, rejected, accepted,
            imu.recovery_count, tx_errors};
        for (unsigned i = 0; i < 20; ++i)
            Link_Put(f->payload + i * 4, values[i], 4);
        append(f);
    }
}
void Navbot_LinkPoll(uint32_t now)
{
    if (now < last_tick)
        ++tick_high;
    last_tick = now;
    now_us = (((uint64_t)tick_high << 32) | now) * 1000U;
    if (rx_restart)
        receive_start();
    static uint32_t seen_overflows;
    if (seen_overflows != rx_overflows)
    {
        taskENTER_CRITICAL();
        consumed = produced;
        parser.used = 0;
        seen_overflows = rx_overflows;
        taskEXIT_CRITICAL();
    }
    /* Bound work even when a sender continuously floods the port. */
    for (unsigned budget = 0; budget < 512U && consumed != produced; ++budget)
    {
        __DMB();
        uint8_t byte = rx[consumed % RX_SIZE];
        __DMB();
        ++consumed;
        Link_Feed(&parser, byte, now);
        while (Link_Next(&parser, &incoming, now))
            handle(&incoming);
    }
    while (Link_Next(&parser, &incoming, now))
        handle(&incoming);
    if (huart3.gState != HAL_UART_STATE_READY)
    {
        if ((uint32_t)(now - tx_started) > 100U)
        {
            (void)HAL_UART_AbortTransmit(&huart3);
            ++tx_errors;
        }
        return;
    }
    tx_used = 0;
    while (response_tail != response_head)
    {
        LinkFrame *f = &responses[response_tail++ % RESPONSE_COUNT];
        if (f->id == 2)
            Link_Put(f->payload + 20, now_us, 8);
        append(f);
    }
    if (session && (uint32_t)(now - last_telemetry) >= 10U)
    {
        last_telemetry = now;
        telemetry(now);
    }
    if (tx_used)
    {
        tx_started = now;
        if (HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)tx_used) != HAL_OK)
            ++tx_errors;
    }
}
