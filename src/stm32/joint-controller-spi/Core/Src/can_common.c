/*
 * can_common.c
 *
 * CAN API transport layer implementation.
 * Handles initialization, filter config, transmit, receive, and ISR callbacks.
 * Supersedes the old can_bus.c.
 */

#include "can_common.h"
#include "can/ak70_9.h"
#include <string.h>

/* ── Internal State ── */
static CAN_HandleTypeDef *g_hcan = NULL;
static uint8_t g_my_node_id = 0;
/* ── Init ── */

int can_common_init(CAN_HandleTypeDef *hcan, uint8_t my_node_id) {
    g_hcan = hcan;
    g_my_node_id = my_node_id;


    CAN_FilterTypeDef f;

    /* Filter 0: Accept all ESTOP messages (FIFO0).
     * Mask must include IDE=0 — without it, every extended frame whose
     * ExtID[28:25] = 0 (which is essentially every VESC packet, since their
     * packet IDs stay below ~30) accidentally matches the type=0 condition
     * here and leaks into FIFO0. */
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (CAN_BUILD_ID(CAN_MSG_ESTOP, 0, 0) << 5);
    f.FilterIdLow          = 0x0000;        /* IDE bit (bit 2) = 0 -> standard frame */
    f.FilterMaskIdHigh     = (0x0780 << 5);  /* match type bits [10:7] */
    f.FilterMaskIdLow      = 0x0006;        /* Standard data frames only */
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    /* Filter 1: Accept TORQUE_CMD for my node (FIFO0).
     * Same IDE=0 requirement as Filter 0 — without it the filter rejects
     * VESC frames only by accidental bit alignment, which is fragile. */
    uint16_t torque_id = CAN_BUILD_ID(CAN_MSG_TORQUE_CMD, 0, my_node_id);
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 1;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (torque_id << 5);
    f.FilterIdLow          = 0x0000;        /* IDE = 0 -> standard frame */
    f.FilterMaskIdHigh     = (0x07FF << 5);  /* match type + dest bits */
    f.FilterMaskIdLow      = 0x0006;        /* Standard data frames only */
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return 0;

    return HAL_CAN_Start(hcan) == HAL_OK;

}

/* ── Send ── */

int can_send_std(uint16_t std_id, const uint8_t *data, uint8_t dlc) {
    if (!g_hcan) return 0;
    if (std_id > 0x7FF) return 0;
    if (dlc > 8) return 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0) return 0;

    CAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.StdId = std_id;
    hdr.DLC   = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return (HAL_CAN_AddTxMessage(g_hcan, &hdr, (uint8_t *)data, &mailbox) == HAL_OK);
}

int can_send_ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc) {
    if (!g_hcan) return 0;
    if (ext_id > 0x1FFFFFFFU) return 0;
    if (dlc > 8) return 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0) return 0;

    CAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.IDE   = CAN_ID_EXT;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.ExtId = ext_id;
    hdr.DLC   = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return (HAL_CAN_AddTxMessage(g_hcan, &hdr, (uint8_t *)data, &mailbox) == HAL_OK);
}

/* ── Recv ── */

int can_recv(CanFrame *out) {
    CAN_RxHeaderTypeDef hdr;
    uint32_t fifo = HAL_CAN_GetRxFifoFillLevel(g_hcan, CAN_RX_FIFO0)
                  ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    if (!HAL_CAN_GetRxFifoFillLevel(g_hcan, fifo)) return 0;
    if (HAL_CAN_GetRxMessage(g_hcan, fifo, &hdr, out->data) != HAL_OK) return 0;
    out->is_extended = hdr.IDE == CAN_ID_EXT;
    out->id = out->is_extended ? hdr.ExtId : hdr.StdId;
    out->dlc = hdr.DLC;
    return hdr.RTR == CAN_RTR_DATA;
}

uint8_t can_get_my_node_id(void) {
    return g_my_node_id;
}

int can_set_motor_filter(uint8_t motor_id) {
    if (!g_hcan) return 0;

    /* bxCAN places the extended ID in bits 31:3 and IDE in bit 2. */
    uint32_t filter_word = (ak70_9_feedback_id(motor_id) << 3) | CAN_ID_EXT;
    CAN_FilterTypeDef f;
    memset(&f, 0, sizeof(f));
    f.FilterBank           = 2;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = filter_word >> 16;
    f.FilterIdLow          = filter_word & 0xFFFFU;
    f.FilterMaskIdHigh     = 0xFFFF;
    f.FilterMaskIdLow      = 0xFFFE;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    return (HAL_CAN_ConfigFilter(g_hcan, &f) == HAL_OK);
}
