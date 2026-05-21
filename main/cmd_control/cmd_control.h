#ifndef CMD_CONTROL_H
#define CMD_CONTROL_H

/* CAN protocol state flags — bit positions in the state_flags field of can_master_response_t */
#define CAN_STATE_FLAG_ARMED           (1U << 0)
#define CAN_STATE_FLAG_REFERENCED      (1U << 2)
#define CAN_STATE_FLAG_TCP_EST         (1U << 3)
#define CAN_STATE_FLAG_PROGRAM_RUNNING (1U << 6)
#define CAN_STATE_FLAG_ERROR           (1U << 7)

/* Robot FSM state identifiers reported in the robot_state field */
#define ROBOT_STATE_DISARMED       0U
#define ROBOT_STATE_UNREFERENCED   1U
#define ROBOT_STATE_READY          2U
#define ROBOT_STATE_RUNNING        3U
#define ROBOT_STATE_READY_FOR_SYNC 4U
#define ROBOT_STATE_ERROR          5U

/* Sentinel value meaning "no slot active" */
#define CAN_SLOT_NONE 0xFFU

/* Relay orchestrator timing (milliseconds) */
#define RELAY_STATUS_POLL_MS         100U
#define RELAY_READY_GRACE_MS         1200U
#define RELAY_READY_TIMEOUT_MS       120000U
#define RELAY_HOME_SETTLE_TIMEOUT_MS 120000U
#define RELAY_TASK_STACK_SIZE        6144U
#define RELAY_TASK_PRIORITY          5U

/* Program slot assignments — both nodes share the same slot numbering */
#define RELAY_FORWARD_SLOT 0U
#define RELAY_RETURN_SLOT  1U

/* run_sync default target slot */
#define RUN_SYNC_DEFAULT_SLOT 0U

void cmd_control_start(void);

#endif /* CMD_CONTROL_H */
