/**
  ******************************************************************************
  * @file           : microros_imu.c
  * @brief          : Implementation of micro-ROS (Jazzy) client node on STM32_IMU
  ******************************************************************************
  */

#include "microros_imu.h"
#include "custom_transport.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>
#include <sensor_msgs/msg/imu.h>
#include <string.h>

#pragma GCC diagnostic ignored "-Wunused-result"

#define PI_CONST                3.14159265358979323846
#define DEG_TO_RAD(deg)         ((deg) * (PI_CONST / 180.0))
#define SCALE_QUAT              (1.0 / 16384.0)
#define SCALE_GYRO_DPS          (1.0 / 16.0)
#define SCALE_ACCEL_MPS2        (1.0 / 100.0)

#define PING_INTERVAL_MS        1000
#define PUBLISH_INTERVAL_MS     10      /* 100 Hz */
#define MAX_CONSECUTIVE_ERRORS  10

static MicroRosState_t g_state = MICROROS_STATE_WAITING_AGENT;
static rcl_allocator_t g_allocator;
static rclc_support_t  g_support;
static rcl_node_t      g_node;
static rcl_publisher_t g_publisher;
static sensor_msgs__msg__Imu g_imu_msg;

static uint32_t g_last_ping_tick = 0;
static uint32_t g_last_pub_tick = 0;
static uint32_t g_error_count = 0;
static const char g_frame_id[] = "imu_link";

void MicroRosIMU_Init(UART_HandleTypeDef *huart)
{
  custom_transport_init(huart);

  /* Configure micro-ROS custom transport with XRCE stream framing */
  rmw_uros_set_custom_transport(
    MICROROS_TRANSPORTS_FRAMING_MODE,
    NULL,
    custom_transport_open,
    custom_transport_close,
    custom_transport_write,
    custom_transport_read
  );

  memset(&g_imu_msg, 0, sizeof(sensor_msgs__msg__Imu));

  /* Setup static frame_id string */
  g_imu_msg.header.frame_id.data = (char *)g_frame_id;
  g_imu_msg.header.frame_id.size = strlen(g_frame_id);
  g_imu_msg.header.frame_id.capacity = strlen(g_frame_id) + 1;

  /* Default Measurement Covariance matrices (diagonal) */
  g_imu_msg.angular_velocity_covariance[0] = 1e-4;
  g_imu_msg.angular_velocity_covariance[4] = 1e-4;
  g_imu_msg.angular_velocity_covariance[8] = 1e-4;

  g_imu_msg.orientation_covariance[0] = 1e-3;
  g_imu_msg.orientation_covariance[4] = 1e-3;
  g_imu_msg.orientation_covariance[8] = 1e-3;

  g_imu_msg.linear_acceleration_covariance[0] = 1e-2;
  g_imu_msg.linear_acceleration_covariance[4] = 1e-2;
  g_imu_msg.linear_acceleration_covariance[8] = 1e-2;

  g_state = MICROROS_STATE_WAITING_AGENT;
  g_last_ping_tick = 0;
  g_last_pub_tick = 0;
  g_error_count = 0;
}

volatile int g_create_step = 0;
volatile rcl_ret_t g_last_rc = 0;
volatile rcl_ret_t g_step1_rc = 0;
volatile rcl_ret_t g_step2_rc = 0;
volatile rcl_ret_t g_step3_rc = 0;
char g_error_string[128] = {0};

static void DestroyEntities(void)
{
  if (g_create_step >= 1)
  {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&g_support.context);
    if (rmw_context)
    {
      (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }
  }
  if (g_create_step >= 3)
  {
    (void)rcl_publisher_fini(&g_publisher, &g_node);
  }
  if (g_create_step >= 2)
  {
    (void)rcl_node_fini(&g_node);
  }
  if (g_create_step >= 1)
  {
    (void)rclc_support_fini(&g_support);
  }
  memset(&g_publisher, 0, sizeof(rcl_publisher_t));
  memset(&g_node, 0, sizeof(rcl_node_t));
  memset(&g_support, 0, sizeof(rclc_support_t));
  g_create_step = 0;
}

static bool CreateEntities(void)
{
  g_allocator = rcl_get_default_allocator();

  /* Ensure structs are cleanly zero-initialized */
  memset(&g_support, 0, sizeof(rclc_support_t));
  g_node = rcl_get_zero_initialized_node();
  g_publisher = rcl_get_zero_initialized_publisher();
  memset(g_error_string, 0, sizeof(g_error_string));

  g_step1_rc = rclc_support_init(&g_support, 0, NULL, &g_allocator);
  g_last_rc = g_step1_rc;
  if (g_step1_rc != RCL_RET_OK)
  {
    if (rcl_error_is_set())
    {
      strncpy(g_error_string, rcl_get_error_string().str, sizeof(g_error_string) - 1);
      rcl_reset_error();
    }
    return false;
  }
  g_create_step = 1;

  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&g_support.context);
  if (rmw_ctx != NULL)
  {
    (void)rmw_uros_set_context_entity_creation_session_timeout(rmw_ctx, 5000);
  }

  g_step2_rc = rclc_node_init_default(&g_node, "stm32_imu_node", "", &g_support);
  g_last_rc = g_step2_rc;
  if (g_step2_rc != RCL_RET_OK)
  {
    if (rcl_error_is_set())
    {
      strncpy(g_error_string, rcl_get_error_string().str, sizeof(g_error_string) - 1);
      rcl_reset_error();
    }
    return false;
  }
  g_create_step = 2;

  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
  g_step3_rc = rcl_publisher_init(
        &g_publisher,
        &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu",
        &pub_options
      );
  g_last_rc = g_step3_rc;
  if (g_step3_rc != RCL_RET_OK)
  {
    if (rcl_error_is_set())
    {
      strncpy(g_error_string, rcl_get_error_string().str, sizeof(g_error_string) - 1);
      rcl_reset_error();
    }
    return false;
  }
  g_create_step = 3;

  /* Synchronize clock with Host ROS 2 daemon */
  rmw_uros_sync_session(100);

  return true;
}

void MicroRosIMU_Process(const BNO055_Data_t *sensor_data)
{
  uint32_t now = HAL_GetTick();

  switch (g_state)
  {
    case MICROROS_STATE_WAITING_AGENT:
    {
      if ((now - g_last_ping_tick) >= PING_INTERVAL_MS)
      {
        g_last_ping_tick = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
        {
          g_state = MICROROS_STATE_AGENT_AVAILABLE;
        }
      }
      break;
    }

    case MICROROS_STATE_AGENT_AVAILABLE:
    {
      if (CreateEntities())
      {
        g_state = MICROROS_STATE_AGENT_CONNECTED;
        g_error_count = 0;
        g_last_pub_tick = now;
      }
      else
      {
        DestroyEntities();
        g_state = MICROROS_STATE_WAITING_AGENT;
      }
      break;
    }

    case MICROROS_STATE_AGENT_CONNECTED:
    {
      if ((now - g_last_pub_tick) >= PUBLISH_INTERVAL_MS)
      {
        g_last_pub_tick = now;

        if (sensor_data != NULL && sensor_data->valid)
        {
          /* Precompute normalized orientation Quaternion (w, x, y, z) */
          g_imu_msg.orientation.w = (double)sensor_data->quat.w * SCALE_QUAT;
          g_imu_msg.orientation.x = (double)sensor_data->quat.x * SCALE_QUAT;
          g_imu_msg.orientation.y = (double)sensor_data->quat.y * SCALE_QUAT;
          g_imu_msg.orientation.z = (double)sensor_data->quat.z * SCALE_QUAT;

          /* Precompute Angular Velocity in rad/s */
          double gx_dps = (double)sensor_data->gyro.x * SCALE_GYRO_DPS;
          double gy_dps = (double)sensor_data->gyro.y * SCALE_GYRO_DPS;
          double gz_dps = (double)sensor_data->gyro.z * SCALE_GYRO_DPS;
          g_imu_msg.angular_velocity.x = DEG_TO_RAD(gx_dps);
          g_imu_msg.angular_velocity.y = DEG_TO_RAD(gy_dps);
          g_imu_msg.angular_velocity.z = DEG_TO_RAD(gz_dps);

          /* Precompute Linear Acceleration in m/s^2 (gravity removed by BNO055) */
          g_imu_msg.linear_acceleration.x = (double)sensor_data->linear_acc.x * SCALE_ACCEL_MPS2;
          g_imu_msg.linear_acceleration.y = (double)sensor_data->linear_acc.y * SCALE_ACCEL_MPS2;
          g_imu_msg.linear_acceleration.z = (double)sensor_data->linear_acc.z * SCALE_ACCEL_MPS2;

          /* Populate timestamp synchronized with ROS 2 clock */
          int64_t nanos = rmw_uros_epoch_nanos();
          if (nanos > 0)
          {
            g_imu_msg.header.stamp.sec = (int32_t)(nanos / 1000000000LL);
            g_imu_msg.header.stamp.nanosec = (uint32_t)(nanos % 1000000000LL);
          }
          else
          {
            g_imu_msg.header.stamp.sec = (int32_t)(now / 1000);
            g_imu_msg.header.stamp.nanosec = (uint32_t)((now % 1000) * 1000000);
          }

          /* Publish message */
          rcl_ret_t rc = rcl_publish(&g_publisher, &g_imu_msg, NULL);
          if (rc == RCL_RET_OK)
          {
            g_error_count = 0;
          }
          else
          {
            g_error_count++;
            if (g_error_count >= MAX_CONSECUTIVE_ERRORS)
            {
              g_state = MICROROS_STATE_AGENT_DISCONNECTED;
            }
          }
        }
      }
      break;
    }

    case MICROROS_STATE_AGENT_DISCONNECTED:
    {
      DestroyEntities();
      g_state = MICROROS_STATE_WAITING_AGENT;
      g_last_ping_tick = now;
      break;
    }

    default:
      g_state = MICROROS_STATE_WAITING_AGENT;
      break;
  }
}

MicroRosState_t MicroRosIMU_GetState(void)
{
  return g_state;
}
