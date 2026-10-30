/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "can_console_motor_session.h"
#include "driver/twai.h"
/* Transport holds the gate through queue submission, preventing a writer
 * from slipping a frame past release/acquire. Standalone demos remain valid. */
bool motor_session_lock_tx(int channel,can_motor_client_t client,const twai_message_t *frame);
void motor_session_unlock_tx(void);
