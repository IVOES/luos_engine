/******************************************************************************
 * @file robus
 * @brief User functionalities of the robus communication protocol
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#include <robus.h>

#include <string.h>
#include <stdbool.h>
#include "transmission.h"
#include "reception.h"
#include "port_manager.h"
#include "context.h"
#include "topic.h"
#include "robus_hal.h"
#include "luos_hal.h"
#include "msg_alloc.h"
#include "luos_utils.h"
#include "timestamp.h"

#include <filter.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct __attribute__((__packed__))
{
    union
    {
        struct __attribute__((__packed__))
        {
            uint16_t prev_nodeid;
            uint16_t nodeid;
        };
        uint8_t unmap[sizeof(uint16_t) * 2];
    };
} node_bootstrap_t;

static error_return_t Robus_MsgHandler(msg_t *input);
static error_return_t Robus_DetectNextNodes(ll_service_t *ll_service);
static error_return_t Robus_ResetNetworkDetection(ll_service_t *ll_service);
static inline uint16_t Robus_CtxIndexFromID(uint16_t id);
static inline void Robus_DoubleAlloc(msg_t *msg);
static inline ll_service_t *Robus_GetConcernedLLService(header_t *header);
static inline void Robus_InterpretMsgProtocol(msg_t *msg);
/*******************************************************************************
 * Variables
 ******************************************************************************/
// Creation of the robus context. This variable is used in all files of this lib.
volatile context_t ctx;
uint32_t baudrate; /*!< System current baudrate. */
volatile uint16_t last_node = 0;

/*******************************************************************************
 * Function
 ******************************************************************************/

/******************************************************************************
 * @brief Initialisation of the Robus communication protocole
 * @param None
 * @return None
 ******************************************************************************/
void Robus_Init(memory_stats_t *memory_stats)
{
    // Init the number of created  virtual service.
    ctx.ll_service_number = 0;
    // Set default service id. This id is a void id used if no service is created.
    ctx.node.node_id = DEFAULTID;
    // By default node are not certified.
    ctx.node.certified = false;
    // set node_info value
    ctx.node.node_info = 0;
#ifdef NO_RTB
    ctx.node.node_info |= 1 << 0;
#endif
    // no transmission lock
    ctx.tx.lock = false;
    // Init collision state
    ctx.tx.collision = false;
    // Init Tx status
    ctx.tx.status = TX_DISABLE;
    // Save luos baudrate
    baudrate = DEFAULTBAUDRATE;
    // mask
    Filter_IDMaskInit();
    Filter_TopicMaskInit();

    // Init reception
    Recep_Init();

    // Clear message allocation buffer table
    MsgAlloc_Init(memory_stats);

    // Init hal
    RobusHAL_Init();

    // init detection structure
    PortMng_Init();

    // Initialize the robus service status
    ctx.rx.status.unmap      = 0;
    ctx.rx.status.identifier = 0xF;

    Luos_SetNodeDetected(NO_DETECTION);
}
/******************************************************************************
 * @brief Loop of the Robus communication protocole
 * @param None
 * @return None
 ******************************************************************************/
void Robus_Loop(void)
{
    // Execute message allocation tasks
    MsgAlloc_loop();
    // Interpreat received messages and create luos task for it.
    msg_t *msg = NULL;
    while (MsgAlloc_PullMsgToInterpret(&msg) == SUCCEED)
    {
        // Check if this message is a protocol one
        if (Robus_MsgHandler(msg) == FAILED)
        {
            // If not create luos tasks.
            Robus_InterpretMsgProtocol(msg);
        }
    }
}
/******************************************************************************
 * @brief Formalize message Set tx task and send
 * @param service to send
 * @param msg to send
 * @return error_return_t
 ******************************************************************************/
error_return_t Robus_SetTxTask(ll_service_t *ll_service, msg_t *msg)
{
    error_return_t error = SUCCEED;
    uint8_t ack          = 0;
    uint16_t data_size   = 0;
    uint16_t crc_val     = 0xFFFF;
    // ***************************************************
    // don't send luos messages if network is down
    // ***************************************************
    if ((msg->header.cmd >= LUOS_LAST_RESERVED_CMD) && (Luos_NodeDetectedState() != DETECTION_OK))
    {
        return PROHIBITED;
    }

    // Compute the full message size based on the header size info.
    if (msg->header.size > MAX_DATA_MSG_SIZE)
    {
        data_size = MAX_DATA_MSG_SIZE;
    }
    else
    {
        data_size = msg->header.size;
    }
    // Add the CRC to the total size of the message
    uint16_t full_size = sizeof(header_t) + data_size + CRC_SIZE;

    uint16_t crc_max_index = full_size;

    if (Timestamp_IsTimestampMsg(msg) == true)
    {
        full_size += sizeof(time_luos_t);
    }
    // Compute the CRC
    crc_val = ll_crc_compute(&msg->stream[0], crc_max_index - CRC_SIZE, 0xFFFF);

    // Check the localhost situation
    luos_localhost_t localhost = Filter_MsgConcerned(&msg->header);
    // Check if ACK needed
    if (((msg->header.target_mode == SERVICEIDACK) || (msg->header.target_mode == NODEIDACK)) && ((localhost && (msg->header.target != DEFAULTID)) || (ctx.verbose == MULTIHOST)))
    {
        // This is a localhost message and we need to transmit a ack. Add it at the end of the data to transmit
        ack = ctx.rx.status.unmap;
        full_size++;
    }

    // ********** Allocate the message ********************
    if (MsgAlloc_SetTxTask(ll_service, (uint8_t *)msg->stream, crc_val, full_size, localhost, ack) == FAILED)
    {
        error = FAILED;
    }
// **********Try to send the message********************
#ifndef VERBOSE_LOCALHOST
    if (localhost != LOCALHOST)
    {
#endif
        Transmit_Process();
#ifndef VERBOSE_LOCALHOST
    }
#endif
    return error;
}
/******************************************************************************
 * @brief Send Msg to a service
 * @param service to send
 * @param msg to send
 * @return none
 ******************************************************************************/
error_return_t Robus_SendMsg(ll_service_t *ll_service, msg_t *msg)
{
    // ********** Prepare the message ********************
    if (ll_service->id != 0)
    {
        msg->header.source = ll_service->id;
    }
    else
    {
        msg->header.source = ctx.node.node_id;
    }
    if (Robus_SetTxTask(ll_service, msg) == FAILED)
    {
        return FAILED;
    }
    return SUCCEED;
}
/******************************************************************************
 * @brief Start a topology detection procedure
 * @param ll_service pointer to the detecting ll_service
 * @return The number of detected node.
 ******************************************************************************/
uint16_t Robus_TopologyDetection(ll_service_t *ll_service)
{
    uint8_t redetect_nb = 0;
    bool detect_enabled = true;

    // if a detection is in progress,
    // Don't do an another detection and return 0
    if (Luos_NodeDetectedState() >= LOCAL_DETECTION)
    {
        return 0;
    }

    while (detect_enabled)
    {
        detect_enabled = false;

        // Reset all detection state of services on the network
        Robus_ResetNetworkDetection(ll_service);
        // Make sure that the detection is not interrupted
        if (Luos_NodeDetectedState() == EXTERNAL_DETECTION)
        {
            return 0;
        }
        // setup local node
        ctx.node.node_id = 1;
        last_node        = 1;
        // setup sending ll_service
        ll_service->id = 1;

        if (Robus_DetectNextNodes(ll_service) == FAILED)
        {
            // check the number of retry we made
            LUOS_ASSERT((redetect_nb <= 4));
            // Detection fail, restart it
            redetect_nb++;
            detect_enabled = true;
        }
    }

    return last_node;
}
/******************************************************************************
 * @brief reset all service port states
 * @param ll_service pointer to the detecting ll_service
 * @return The number of detected node.
 ******************************************************************************/
static error_return_t Robus_ResetNetworkDetection(ll_service_t *ll_service)
{
    msg_t msg;
    uint8_t try_nbr = 0;

    msg.header.config      = BASE_PROTOCOL;
    msg.header.target      = BROADCAST_VAL;
    msg.header.target_mode = BROADCAST;
    msg.header.cmd         = START_DETECTION;
    msg.header.size        = 0;

    do
    {
        // if a detection is in progress,
        // Don't do an another detection and return 0
        if (Luos_NodeDetectedState() >= LOCAL_DETECTION)
        {
            return 0;
        }
        // msg send not blocking
        Robus_SendMsg(ll_service, &msg);
        // need to wait until tx msg before clear msg alloc
        while (MsgAlloc_TxAllComplete() != SUCCEED)
            ;

        MsgAlloc_Init(NULL);

        // wait for some 2ms to be sure all previous messages are received and treated
        uint32_t start_tick = LuosHAL_GetSystick();
        while (LuosHAL_GetSystick() - start_tick < 2)
            ;
        try_nbr++;
    } while ((MsgAlloc_IsEmpty() != SUCCEED) || (try_nbr > 5));

    ctx.node.node_id = 0;
    PortMng_Init();
    if (try_nbr < 5)
    {
        Luos_SetNodeDetected(LOCAL_DETECTION);
        return SUCCEED;
    }

    return FAILED;
}
/******************************************************************************
 * @brief run the procedure allowing to detect the next nodes on the next port
 * @param ll_service pointer to the detecting ll_service
 * @return None.
 ******************************************************************************/
static error_return_t Robus_DetectNextNodes(ll_service_t *ll_service)
{
    // Lets try to poke other nodes
    while (PortMng_PokeNextPort() == SUCCEED)
    {
        // There is someone here
        // Clear spotted dead service detection
        ll_service->dead_service_spotted = 0;
        // Ask an ID  to the detector service.
        msg_t msg;
        msg.header.config      = BASE_PROTOCOL;
        msg.header.target_mode = NODEIDACK;
        msg.header.target      = 1;
        msg.header.cmd         = WRITE_NODE_ID;
        msg.header.size        = 0;
        Robus_SendMsg(ll_service, &msg);
        // Wait the end of transmission
        while (MsgAlloc_TxAllComplete() == FAILED)
            ;
        // Check if there is a failure on transmission
        if (ll_service->dead_service_spotted != 0)
        {
            // Message transmission failure
            // Consider this port unconnected
            ctx.node.port_table[ctx.port.activ] = 0xFFFF;
            ctx.port.activ                      = NBR_PORT;
            ctx.port.keepLine                   = false;
            continue;
        }

        // when Robus loop will receive the reply it will store and manage the new node_id and send it to the next node.
        // We just have to wait the end of the treatment of the entire branch
        uint32_t start_tick = LuosHAL_GetSystick();
        while (ctx.port.keepLine)
        {
            Robus_Loop();
            if (LuosHAL_GetSystick() - start_tick > NETWORK_TIMEOUT)
            {
                // topology detection is too long, we should abort it and restart
                return FAILED;
            }
        }
    }
    return SUCCEED;
}
/******************************************************************************
 * @brief check if received messages are protocols one and manage it if it is.
 * @param msg pointer to the reeived message
 * @return error_return_t SUCCEED if the message have been consumed.
 ******************************************************************************/
static error_return_t Robus_MsgHandler(msg_t *input)
{
    uint32_t baudrate;
    msg_t output_msg;
    node_bootstrap_t node_bootstrap;
    ll_service_t *ll_service = Robus_GetConcernedLLService(&input->header);
    switch (input->header.cmd)
    {
        case WRITE_NODE_ID:
            // Depending on the size of the received data we have to do different things
            switch (input->header.size)
            {
                case 0:
                    // Someone asking us a new node id (we are the detecting service)
                    // Increase the number of node_nb and send it back
                    last_node++;
                    output_msg.header.config      = BASE_PROTOCOL;
                    output_msg.header.cmd         = WRITE_NODE_ID;
                    output_msg.header.size        = sizeof(uint16_t);
                    output_msg.header.target      = input->header.source;
                    output_msg.header.target_mode = NODEIDACK;
                    memcpy(output_msg.data, (void *)&last_node, sizeof(uint16_t));
                    Robus_SendMsg(ll_service, &output_msg);
                    break;
                case 2:
                    // This is a node id for the next node.
                    // This is a reply to our request to generate the next node id.
                    // This node_id is the one after the currently poked branch.
                    // We need to save this ID as a connection on a port
                    memcpy((void *)&ctx.node.port_table[ctx.port.activ], (void *)&input->data[0], sizeof(uint16_t));
                    // Now we can send it to the next node
                    memcpy((void *)&node_bootstrap.nodeid, (void *)&input->data[0], sizeof(uint16_t));
                    node_bootstrap.prev_nodeid    = ctx.node.node_id;
                    output_msg.header.config      = BASE_PROTOCOL;
                    output_msg.header.cmd         = WRITE_NODE_ID;
                    output_msg.header.size        = sizeof(node_bootstrap_t);
                    output_msg.header.target      = 0;
                    output_msg.header.target_mode = NODEIDACK;
                    memcpy((void *)&output_msg.data[0], (void *)&node_bootstrap.unmap[0], sizeof(node_bootstrap_t));
                    Robus_SendMsg(ll_service, &output_msg);
                    break;
                case sizeof(node_bootstrap_t):
                    if (ctx.node.node_id != 0)
                    {
                        ctx.node.node_id = 0;
                        MsgAlloc_Init(NULL);
                    }
                    // This is a node bootstrap information.
                    memcpy((void *)&node_bootstrap.unmap[0], (void *)&input->data[0], sizeof(node_bootstrap_t));
                    ctx.node.node_id                    = node_bootstrap.nodeid;
                    ctx.node.port_table[ctx.port.activ] = node_bootstrap.prev_nodeid;
                    // Continue the topology detection on our other ports.
                    Robus_DetectNextNodes(ll_service);
                default:
                    break;
            }
            return SUCCEED;
            break;
        case START_DETECTION:
            return SUCCEED;
            break;
        case END_DETECTION:
            // Detect end of detection
            Luos_SetNodeDetected(DETECTION_OK);
            return FAILED;
            break;
        case SET_BAUDRATE:
            // We have to wait the end of transmission of all the messages we have to transmit
            while (MsgAlloc_TxAllComplete() == FAILED)
                ;
            memcpy(&baudrate, input->data, sizeof(uint32_t));
            RobusHAL_ComInit(baudrate);
            return SUCCEED;
            break;
        default:
            return FAILED;
            break;
    }
    return FAILED;
}
/******************************************************************************
 * @brief get node structure
 * @param None
 * @return Node pointer
 ******************************************************************************/
node_t *Robus_GetNode(void)
{
    return (node_t *)&ctx.node;
}
/******************************************************************************
 * @brief Parse msg to find a service concerned
 * @param header of message
 * @return ll_service pointer
 ******************************************************************************/
ll_service_t *Robus_GetConcernedLLService(header_t *header)
{
    uint16_t i = 0;
    // Find if we are concerned by this message.
    switch (header->target_mode)
    {
        case SERVICEIDACK:
        case SERVICEID:
            // Check all ll_service id
            for (i = 0; i < ctx.ll_service_number; i++)
            {
                if (header->target == ctx.ll_service_table[i].id)
                {
                    return (ll_service_t *)&ctx.ll_service_table[i];
                }
            }
            break;
        case TYPE:
            // Check all ll_service type
            for (i = 0; i < ctx.ll_service_number; i++)
            {
                if (header->target == ctx.ll_service_table[i].type)
                {
                    return (ll_service_t *)&ctx.ll_service_table[i];
                }
            }
            break;
        case BROADCAST:
        case NODEIDACK:
        case NODEID:
            return (ll_service_t *)&ctx.ll_service_table[0];
            break;
        case TOPIC:
        default:
            return NULL;
            break;
    }
    return NULL;
}
/******************************************************************************
 * @brief Double Allocate msg_task in case of desactivated filter
 * @param msg pointer
 * @return None
 ******************************************************************************/
static inline void Robus_DoubleAlloc(msg_t *msg)
{
    // if there is a service that deactivated the filter we also allocate a message for it
    if (ctx.filter_state == false)
    {
        // find the position of this service in the node
        uint16_t idx = Robus_CtxIndexFromID(ctx.filter_id);
        // check if it is message for the same service that demanded the filter desactivation
        switch (msg->header.target_mode)
        {
            case (SERVICEID):
                if (ctx.filter_id != msg->header.target)
                {
                    // store the message if it is not so that we dont have double messages in memory
                    MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[idx], msg);
                }
                break;
            case (TYPE):
                if (ctx.ll_service_table[idx].type != msg->header.target)
                {
                    // store the message if it is not so that we dont have double messages in memory
                    MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[idx], msg);
                }
                break;
            case (TOPIC):
                if (Topic_IsTopicSubscribed((ll_service_t *)&ctx.ll_service_table[idx], msg->header.target) == false)
                {
                    // store the message if it is not so that we dont have double messages in memory
                    MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[idx], msg);
                }
                break;
            default:
                break;
        }
    }
}
/******************************************************************************
 * @brief Parse msg to find all services concerned and create
 * @param msg pointer
 * @return None
 ******************************************************************************/
void Robus_InterpretMsgProtocol(msg_t *msg)
{
    uint16_t i = 0;

    // Find if we are concerned by this message.
    switch (msg->header.target_mode)
    {
        case SERVICEIDACK:
        case SERVICEID:
            // Check all ll_service id
            for (i = 0; i < ctx.ll_service_number; i++)
            {
                if (msg->header.target == ctx.ll_service_table[i].id)
                {
                    MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[i], msg);
                    break;
                }
            }
            // check if we need to double allocate msg_task
            Robus_DoubleAlloc(msg);
            return;
            break;
        case TYPE:
            // Check all ll_service type
            for (i = 0; i < ctx.ll_service_number; i++)
            {
                if (msg->header.target == ctx.ll_service_table[i].type)
                {
                    MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[i], msg);
                }
            }
            // check if we need to double allocate msg_task
            Robus_DoubleAlloc(msg);
            return;
            break;
        case BROADCAST:
            for (i = 0; i < ctx.ll_service_number; i++)
            {
                MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[i], msg);
            }
            return;
            break;
        case TOPIC:
            for (i = 0; i < ctx.ll_service_number; i++)
            {
                if (Topic_IsTopicSubscribed((ll_service_t *)&ctx.ll_service_table[i], msg->header.target))
                {
                    // TODO manage multiple slave concerned
                    MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[i], msg);
                }
            }
            // check if we need to double allocate msg_task
            Robus_DoubleAlloc(msg);
            return;
            break;
        case NODEIDACK:
        case NODEID:
            if (msg->header.target == DEFAULTID) // on default ID it's always a luos command create only one task
            {
                MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[0], msg);
                return;
            }
            // check if the message is really for the node or it is a service that has no filter
            if (msg->header.target == ctx.node.node_id)
            {
                for (i = 0; i < ctx.ll_service_number; i++)
                {
                    MsgAlloc_LuosTaskAlloc((ll_service_t *)&ctx.ll_service_table[i], msg);
                }
            }
            // check if we need to double allocate msg_task
            Robus_DoubleAlloc(msg);
            return;
            break;
        default:
            break;
    }
}
/******************************************************************************
 * @brief returns the index in context table from the service id
 * @param id
 * @return index
 ******************************************************************************/
static inline uint16_t Robus_CtxIndexFromID(uint16_t id)
{
    return (id - ctx.ll_service_table[0].id);
}
/******************************************************************************
 * @brief Set verbose mode
 * @param mode true or false
 * @return None
 * _CRITICAL function call in IRQ
 ******************************************************************************/
_CRITICAL void Robus_SetVerboseMode(uint8_t mode)
{
    // verbose is localhost or multihost
    ctx.verbose = mode + 1;
}
