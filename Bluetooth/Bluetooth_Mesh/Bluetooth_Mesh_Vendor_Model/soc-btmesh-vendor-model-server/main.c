/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#include "retargetserial.h"
#include "vm_def.h"

#define BUTTON0_PIN         (6)
#define BUTTON0_PORT        (gpioPortF)
#define BUTTON1_PIN         (7)
#define BUTTON1_PORT        (gpioPortF)

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

static uint16_t uni_addr = 0;
static opcodes_t my_opc[] = { pub_addr_get, pub_addr_st, user_name, congrat };

/* Timer Id used in this project */
#define TIMER_ID_FACTORY_RESET          62

#define TIMER_ID_PROVISIONING     66
#define TIMER_ID_NODE_CONFIGURED  30
#define TIMER_ID_RESTART          78

/*******************************************************************************
 *  State of the LEDs is updated by calling LED_set_state().
 *  The new state is passed as parameter, possible values are defined below.
 ******************************************************************************/
#define LED_STATE_OFF    0   ///< light off (both LEDs turned off)
#define LED_STATE_ON     1   ///< light on (both LEDs turned on)
#define LED_STATE_PROV   3   ///< provisioning (LEDs blinking)

#define IS_UNICAST_ADDR(x)        ((x) > 0 && (x) < 0x8000)
#define IS_GROUP_ADDR(x)        ((x) >= 0xC000 && (x) <= 0xFFEF)

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .max_timers = 16,
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,   // Enable antenna configuration.
  .rf.antenna = GECKO_RF_ANTENNA,   // Select antenna path!
};

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

static void button_init(void)
{
  // configure pushbutton PB0 and PB1 as inputs, with pull-up enabled
  GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);
  GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);
}

int main()
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  // Bt stack initialization
  gecko_stack_init(&config);
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  gecko_bgapi_class_gatt_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  gecko_bgapi_class_sm_init();

  // Bt Mesh stack initialization
  gecko_bgapi_class_mesh_node_init();
  gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  gecko_bgapi_class_mesh_proxy_client_init();
  gecko_bgapi_class_mesh_generic_client_init();
  gecko_bgapi_class_mesh_generic_server_init();
  gecko_bgapi_class_mesh_vendor_model_init();
  gecko_bgapi_class_mesh_health_client_init();
  gecko_bgapi_class_mesh_health_server_init();
  gecko_bgapi_class_mesh_test_init();
  gecko_bgapi_class_mesh_lpn_init();
  gecko_bgapi_class_mesh_friend_init();
  gecko_bgapi_class_mesh_sensor_client_init();
  gecko_bgapi_class_mesh_sensor_server_init();
  gecko_bgapi_class_mesh_sensor_setup_server_init();

  // Coex initialization
  gecko_initCoexHAL();

  button_init();
  RETARGET_SerialInit();

  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    bool pass = mesh_bgapi_listener(evt);
    if (pass) {
      handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
    }
  }
}

/* LED GPIO is active-low */
#define TURN_LED_OFF   GPIO_PinOutSet
#define TURN_LED_ON    GPIO_PinOutClear
/***************************************************************************//**
 * Update the state of LEDs.
 *
 * @param[in] state  New state defined as LED_STATE_xxx.
 ******************************************************************************/
static void LED_set_state(int state)
{
  switch (state) {
    case LED_STATE_OFF:
      TURN_LED_OFF(BSP_LED0_PORT, BSP_LED0_PIN);
      TURN_LED_OFF(BSP_LED1_PORT, BSP_LED1_PIN);
      break;
    case LED_STATE_ON:
      TURN_LED_ON(BSP_LED0_PORT, BSP_LED0_PIN);
      TURN_LED_ON(BSP_LED1_PORT, BSP_LED1_PIN);
      break;
    case LED_STATE_PROV:
      GPIO_PinOutToggle(BSP_LED0_PORT, BSP_LED0_PIN);
      GPIO_PinOutToggle(BSP_LED1_PORT, BSP_LED1_PIN);
      break;

    default:
      break;
  }
}

void initiate_factory_reset(void)
{
  printf("factory reset\r\n");

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  uint16 result;

  switch (evt_id) {
    case gecko_evt_system_boot_id:
      printf("System boot: Vendor mode Server\r\n");
      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
      if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
        initiate_factory_reset();
      }
      else
      {
        // Initialize Mesh stack in Node operation mode, it will generate initialized event
        result = gecko_cmd_mesh_node_init()->result;
        if (result) {
          printf("init failed (0x%x)", result);
        }
      }
      break;

    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {
        case TIMER_ID_FACTORY_RESET:
          // reset the device to finish factory reset
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_RESTART:
          // restart timer expires, reset the device
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_PROVISIONING:
          // toggle LED to indicate the provisioning state
          LED_set_state(LED_STATE_PROV);
          break;

        case TIMER_ID_NODE_CONFIGURED:
          break;

        break;

        default:
          break;
      }

      break;

    case gecko_evt_mesh_node_initialized_id:
    {
      struct gecko_msg_mesh_node_initialized_evt_t *ret = (struct gecko_msg_mesh_node_initialized_evt_t *)&evt->data;
      printf("Node Initialized...\r\n");

      /* Derive the unicast address from the LSB 2 bytes from the BD_ADDR */
      struct gecko_msg_system_get_bt_address_rsp_t *bd_ret = gecko_cmd_system_get_bt_address();
//      uni_addr = ((bd_ret->address.addr[1] << 8) | bd_ret->address.addr[0]) & 0x7FFF;
      printf("Unicast Address = 0x%x%x\r\n", bd_ret->address.addr[1],bd_ret->address.addr[0]);

      /* Provision itself if not provisioned yet */
      if (ret->provisioned) {
        printf("Node provisioned already.\r\n");
      } else {
        printf("node is unprovisioned\r\n");
        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
//        break;
      }

      /* Initialize the vendor model */
      printf("Initialize Vendor Model\r\n");
      result = gecko_cmd_mesh_vendor_model_init(0, VENDOR_ID, SERVER_MODEL_ID, true, sizeof(my_opc), my_opc)->result;
      if (result) {
        printf("initialize vendor model failed (0x%x)", result);
      }

#if 0
      /* Set the publication and subscription */
      struct gecko_msg_mesh_test_get_local_model_pub_rsp_t *pub_setting = gecko_cmd_mesh_test_get_local_model_pub(ELEMENT_ID, VENDOR_ID, SERVER_MODEL_ID);
      if (!pub_setting->result && pub_setting->pub_address == SERVER_PUB_ADDR) {
        printf("Configuration done already.\r\n");
      } else {
        printf("Pub setting result = 0x%04X, pub setting address = 0x%04X\r\n", pub_setting->result, pub_setting->pub_address);
        printf("Add local app key...\r\n");
        gecko_cmd_mesh_test_add_local_key(1, enc_key, APP_KEY_IDX, NET_KEY_IDX);
        printf("Bind local app key...\r\n");
        gecko_cmd_mesh_test_bind_local_model_app(ELEMENT_ID, APP_KEY_IDX, VENDOR_ID, SERVER_MODEL_ID);
        printf("Set local model pub...\r\n");
        gecko_cmd_mesh_test_set_local_model_pub(ELEMENT_ID, APP_KEY_IDX, VENDOR_ID, SERVER_MODEL_ID, SERVER_PUB_ADDR, DEFAULT_TTL, 0, 0, 0);
        printf("Add local model sub...\r\n");
        gecko_cmd_mesh_test_add_local_model_sub(ELEMENT_ID, VENDOR_ID, SERVER_MODEL_ID, SERVER_SUB_ADDR);
        printf("Set relay...\r\n");
        gecko_cmd_mesh_test_set_relay(1, 0, 0);
        printf("Set Network tx state.\r\n");
        gecko_cmd_mesh_test_set_nettx(2, 4);
        //	printf("add local model sub...\r\n");
        //	gecko_cmd_mesh_test_add_local_model_sub(0, VENDOR_ID, MODEL_ID, DUT_SUB_ADDR);
      }
#endif

      break;
    }

    case gecko_evt_mesh_node_provisioning_started_id:
      printf("Started provisioning\r\n");
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      led_init(); /* shared GPIO pins used as LED output */
#endif
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
//      _elem_index = 0;   // index of primary element is zero. This example has only one element.

      // stop LED blinking when provisioning complete
      gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_PROVISIONING, 0);
      LED_set_state(LED_STATE_OFF);
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
      printf("provisioning failed, code %x\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
      /* start a one-shot timer that will trigger soft reset after small delay */
      gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
      break;

    case gecko_evt_mesh_node_key_added_id:
      printf("got new %s key with index %x\r\n",
             evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
             evt->data.evt_mesh_node_key_added.index);
      // try to init lpn 5 seconds after adding key
      result = gecko_cmd_hardware_set_soft_timer(32768*5,
                                                 TIMER_ID_NODE_CONFIGURED,
                                                 1)->result;
      if (result) {
        printf("timer failure?!  %x\r\n", result);
      }
      break;

    case gecko_evt_mesh_node_model_config_changed_id:
      printf("model config changed\r\n");
      // try to init lpn 5 seconds after configuration change
      result = gecko_cmd_hardware_set_soft_timer(32768*5,
                                                 TIMER_ID_NODE_CONFIGURED,
                                                 1)->result;
      if (result) {
        printf("timer failure?!  %x\r\n", result);
      }
      break;

    case gecko_evt_mesh_node_config_set_id:
      printf("model config set\r\n");
      // try to init lpn 5 seconds after configuration set
      result = gecko_cmd_hardware_set_soft_timer(32768*5,
                                                 TIMER_ID_NODE_CONFIGURED,
                                                 1)->result;
      if (result) {
        printf("timer failure?!  %x\r\n", result);
      }
      break;

    case gecko_evt_mesh_vendor_model_receive_id:
    {
      printf("\r\nevt: gecko_evt_mesh_vendor_model_receive_id \r\n");
      struct gecko_msg_mesh_vendor_model_receive_evt_t *re_evt = (struct gecko_msg_mesh_vendor_model_receive_evt_t *)&evt->data;
      /* Get publication address && Use unicast (Destination address is not group address) */
      if (re_evt->opcode == user_name) {
    	  printf("Received the user name message %s from destination 0x%04x \r\n", re_evt->payload.data, re_evt->destination_address);
      }
    }
    break;

    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      break;
    default:
      break;
  }
}
