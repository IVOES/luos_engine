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
#include "luos_utils.h"
#include "topic.h"

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
 * @brief Reset Masks
 * @param None
 * @return None
 ******************************************************************************/
void Filter_IDMaskInit(void)
{
    ctx.IDShiftMask = 0;
    for (uint16_t i = 0; i < ID_MASK_SIZE; i++)
    {
        ctx.IDMask[i] = 0;
    }
}
/******************************************************************************
 * @brief Reset Masks
 * @param None
 * @return None
 ******************************************************************************/
void Filter_TopicMaskInit(void)
{
    for (uint16_t i = 0; i < TOPIC_MASK_SIZE; i++)
    {
        ctx.TopicMask[i] = 0;
    }
}

/******************************************************************************
 * @brief Function that changes the filter value
 * @param uint8_t value, 1 if we want to disable, 0 to enable
 * @return None
 ******************************************************************************/
void Filter_SetFilterState(uint8_t state, ll_service_t *service)
{
    ctx.filter_state = state;
    ctx.filter_id    = service->id;
}
/******************************************************************************
 * @brief ID Mask calculation
 * @param ID and Number of service
 * @return None
 ******************************************************************************/
void Filter_IDMaskCalculation(uint16_t service_id, uint16_t service_number)
{
    // 4096 bit address 512 byte possible
    // Create a mask of only possibility in the node
    //--------------------------->|__________|
    //	Shift byte		            byte Mask of bit address

    LUOS_ASSERT(service_id > 0);
    LUOS_ASSERT(service_id <= 4096 - MAX_SERVICE_NUMBER);
    uint16_t tempo  = 0;
    ctx.IDShiftMask = (service_id - 1) / 8; // aligned to byte

    // create a mask of bit corresponding to ID number in the node
    for (uint16_t i = 0; i < service_number; i++)
    {
        tempo = (((service_id - 1) + i) - (8 * ctx.IDShiftMask));
        ctx.IDMask[tempo / 8] |= 1 << ((tempo) % 8);
    }
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
/******************************************************************************
 * @brief Add new mutlicast topic to service bank and node mask
 * @param ll_service
 * @param topic
 * @return Error
 ******************************************************************************/
error_return_t Filter_TopicSubscribe(ll_service_t *ll_service, uint16_t topic_id)
{
    // assert if we add a topic that is greater than the max topic value
    LUOS_ASSERT(topic_id <= LAST_TOPIC);
    // add 1 to the bit corresponding to the topic in multicast mask
    ctx.TopicMask[(topic_id / 8)] |= 1 << (topic_id - ((int)(topic_id / 8)) * 8);
    // add multicast topic to service
    if (ll_service == 0)
    {
        return Topic_Subscribe((ll_service_t *)(&ctx.ll_service_table[0]), topic_id);
    }
    return Topic_Subscribe(ll_service, topic_id);
}
/******************************************************************************
 * @brief Remove mutlicast topic to service bank and node mask
 * @param ll_service
 * @param topic
 * @return Error
 ******************************************************************************/
error_return_t Filter_TopicUnsubscribe(ll_service_t *ll_service, uint16_t topic_id)
{
    error_return_t err;

    // delete topic from service list
    if (ll_service == 0)
    {
        err = Topic_Unsubscribe((ll_service_t *)(&ctx.ll_service_table[0]), topic_id);
    }
    else
    {
        err = Topic_Unsubscribe(ll_service, topic_id);
    }

    if (err == SUCCEED)
    {
        for (uint16_t i = 0; i < ctx.ll_service_number; i++)
        {
            if (Topic_IsTopicSubscribed((ll_service_t *)(&ctx.ll_service_table[i]), topic_id) == true)
            {
                return err;
            }
        }
        // calculate mask after topic deletion
        ctx.TopicMask[(topic_id / 8)] -= 1 << (topic_id - ((int)(topic_id / 8)) * 8);
    }
    return err;
}
