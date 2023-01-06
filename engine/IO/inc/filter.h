/******************************************************************************
 * @file filter.c
 * @brief Calculate filter for Phy and compare filter
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef _FILTER_H_
#define _FILTER_H_

#include <robus.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Functions
 ******************************************************************************/

// generic functions
void Filter_Init(void);
void Filter_IDMaskInit(void);
void Filter_TopicMaskInit(void);
luos_localhost_t Filter_MsgConcerned(header_t *header);
error_return_t Filter_ServiceIDCompare(uint16_t service_id);
error_return_t Filter_TopicCompare(uint16_t topic_id);
void Filter_IDMaskCalculation(uint16_t service_id, uint16_t service_number);

#endif /* _FILTER_H_ */
