/******************************************************************************
 * @file filter.c
 * @brief Calculate filter for Phy and compare filter
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/

#include <filter.h>
#include <string.h>
#include <stdbool.h>
#include "luos_hal.h"
#include "context.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Functions
 ******************************************************************************/

/******************************************************************************
 * @brief Init the interface file.
 * @param None
 * @return None
 ******************************************************************************/
void Filter_Init(void)
{
}
/******************************************************************************
 * @brief Parse msg to find a service concerne
 * @param header of message
 * @return None
 * _CRITICAL function call in IRQ
 ******************************************************************************/
_CRITICAL error_return_t Filter_ServiceIDCompare(uint16_t service_id)
{
    //--------------------------->|__________|
    //	      Shift byte		  byte Mask of bit address
    // In an node, service ID are consecutive
    // MaskID is byte field wich have the size of MAX_SERVICE_NUMBER
    // Shift depend od ID of first service in Node (shift = NodeID/8)

    uint16_t compare = 0;

    if ((service_id > (8 * ctx.IDShiftMask))) // IDMask aligned byte
    {
        // Calcul ID mask for ID receive
        compare = ((service_id - 1) - ((8 * ctx.IDShiftMask)));
        // check if compare and internal mask match
        if ((ctx.IDMask[compare / 8] & (1 << (compare % 8))) != 0)
        {
            return SUCCEED;
        }
    }
    return FAILED;
}
/******************************************************************************
 * @brief Parse multicast mask to find if target exists
 * @param target of message
 * @return None
 * _CRITICAL function call in IRQ
 ******************************************************************************/
_CRITICAL error_return_t Filter_TopicCompare(uint16_t topic_id)
{
    uint8_t compare = 0;
    // make sure there is a topic that can be received by the node
    if (topic_id <= LAST_TOPIC)
    {
        compare = topic_id - ((topic_id / 8) * 8);
        // search if topic exists in mask
        if ((ctx.TopicMask[(topic_id / 8)] & (1 << compare)) != 0)
        {
            return SUCCEED;
        }
    }
    return FAILED;
}
/******************************************************************************
 * @brief Parse msg to find a service concerne
 * @param header of message
 * @return None
 * warning : this function can be redefined only for mock testing purpose
 * _CRITICAL function call in IRQ
 ******************************************************************************/
_WEAKED luos_localhost_t Filter_MsgConcerned(header_t *header)
{
    uint16_t i = 0;

    // Find if we are concerned by this message.
    // check if we need to filter all the messages

    switch (header->target_mode)
    {
        case SERVICEIDACK:
            ctx.rx.status.rx_error = false;
        case SERVICEID:
            // Check all ll_service id
            if (Filter_ServiceIDCompare(header->target) == SUCCEED)
            {
                return ctx.verbose;
            }
            if (ctx.filter_state == false)
            {
                // check if it is message comes from service that demanded the filter desactivation
                if (ctx.filter_id != header->source)
                {
                    // if there is a service that deactivated the filtering occupy the message
                    return MULTIHOST;
                }
            }
            break;
        case TYPE:
            // Check all ll_service type
            for (i = 0; i < ctx.ll_service_number; i++)
            {
                if (header->target == ctx.ll_service_table[i].type)
                {
                    return MULTIHOST;
                }
            }
            if (ctx.filter_state == false)
            {
                // check if it is message comes from service that demanded the filter desactivation
                if (ctx.filter_id != header->source)
                {
                    // if there is a service that deactivated the filtering occupy the message
                    return MULTIHOST;
                }
            }
            break;
        case BROADCAST:
            if (header->target == BROADCAST_VAL)
            {
                return MULTIHOST;
            }
            break;
        case NODEIDACK:
            ctx.rx.status.rx_error = false;
        case NODEID:
            if ((header->target == 0) && (ctx.port.activ != NBR_PORT) && (ctx.port.keepLine == false))
            {
                return ctx.verbose; // discard message if ID = 0 and no Port activ
            }
            else
            {
                if ((header->target == ctx.node.node_id) && ((header->target != 0)))
                {
                    return ctx.verbose;
                }
                else if (ctx.filter_state == false)
                {
                    // check if it is message comes from service that demanded the filter desactivation
                    if (ctx.filter_id != header->source)
                    {
                        // if there is a service that deactivated the filtering occupy the message
                        return MULTIHOST;
                    }
                }
            }
            break;
        case TOPIC:
            if ((Filter_TopicCompare(header->target) == SUCCEED) || (ctx.filter_state == false))
            {
                return MULTIHOST;
            }
            break;
        default:
            return EXTERNALHOST;
            break;
    }
    return EXTERNALHOST;
}
