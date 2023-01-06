/******************************************************************************
 * @file Filter_MsgConcerned.c
 * @brief Mock function Filter_MsgConcerned()
 * @author Luos
 * @version 1.0.0
 ******************************************************************************/
#include "context.h"
#include "luos_hal.h"
/*******************************************************************************
 * Function
 ******************************************************************************/
/******************************************************************************
 * @brief Mock : Redefine function Filter_MsgConcerned
 * @param header of message
 * @return Always returns LOCALHOST
 ******************************************************************************/
luos_localhost_t Filter_MsgConcerned(header_t *header)
{
    // Right now, Mock is unable to emulate external Nodes.
    // So context is initialized to "localhost" to launch a detection without any trouble.
    uint16_t i = 0;
    switch (header->target_mode)
    {
        case SERVICEIDACK:
        case NODEIDACK:
            ctx.rx.status.rx_error = false;
            break;
        default:
            break;
    }
    return LOCALHOST;
}
