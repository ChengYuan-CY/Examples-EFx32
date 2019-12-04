/***************************************************************************//**
 * @file
 * @brief Application code
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

/* C Standard Library headers */
#include <stdio.h>

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"

/* GPIO peripheral library */
#include <em_gpio.h>


/* Display Interface header */
#include "display_interface.h"

/* Own header */
#include "app.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/*******************************************************************************
 * Timer handles defines.
 * System control Timer up to 32 (0-31)
 * Provisioning process Timer up to 64 (32 - 95)
 * Remote control Timer up to 32  (96 - 127)
 * Reserved for future Timer 128  (128 - 255)
 ******************************************************************************/

// uncomment this to enable configuration of vendor model
#define CONFIGURE_VENDOR_MODEL

#ifdef CONFIGURE_VENDOR_MODEL
#define VM_VENDOR_ID	0x02FF
#define VM_SERVER_MODEL_ID	0x1111
#define VM_CLIENT_MODEL_ID	0x2222
#define VENDOR_GRP_SUB_ADDR         0xC003
#define VENDOR_GRP_PUB_ADDR         0xC004

typedef enum {
	pub_addr_get = 0x1,
	pub_addr_st,
	user_name,
	congrat
}opcodes_t;

/* List of the opcodes client model suppports */
static opcodes_t my_opc[] = {pub_addr_get, pub_addr_st, user_name, congrat};

#endif

// uncomment this to enable provisioning using PB-GATT (PB-ADV is used by default)
//#define PROVISION_OVER_GATT

// uncomment this to use fixed network and application keys (for debugging only)
#define USE_FIXED_KEYS

// uncomment this to control the generic model/vendor model from provisioner
#define REMOTE_CTL_FROM_PROV

#define TIMER_ID_GET_DCD        20
#define TIMER_ID_APPKEY_ADD     21
#define TIMER_ID_APPKEY_BIND    22
#define TIMER_ID_PUB_SET        23
#define TIMER_ID_SUB_ADD        24

#define TIMER_ID_SEND_PUB_MSG   25

#define TIMER_ID_SYS_FACTORY_RESET  1
#define TIMER_ID_SYS_RESTART        2
#define TIMER_ID_SYS_BUTTON_POLL    3

/// Flag for indicating DFU Reset must be performed
static uint8_t boot_to_dfu = 0;
/// Address of the Primary Element of the Node
static uint16 _my_address = 0;
/// Number of active Bluetooth connections
static uint8 num_connections = 0;
/// Handle of the last opened LE connection
static uint8 conn_handle = 0xFF;
/// Flag for indicating that initialization was performed
static uint8 init_done = 0;


/** global variables */

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

enum {
  init,
  scanning,
  connecting,
  provisioning,
  provisioned,
  waiting_dcd,
  waiting_appkey_ack,
  waiting_bind_ack,
  waiting_pub_ack,
  waiting_sub_ack
} state;

struct mesh_generic_state current, target;

uint8_t netkey_id = 0xff;
uint8_t appkey_id = 0xff;
uint8_t ask_user_input = false;
uint16 provisionee_address = 0xFFFF;

uint8_t config_retrycount = 0;
uint8_t _uuid_copy[16];

#ifdef PROVISION_OVER_GATT
bd_addr             bt_address;
uint8               bt_address_type;
#endif

#ifdef USE_FIXED_KEYS
const uint8 fixed_netkey[16] = {0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03};
const uint8 fixed_appkey[16] = {0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03};;
#endif

typedef struct
{
  uint16 err;
  const char *pShortDescription;
} tsErrCode;

#define STATUS_OK                      0
#define STATUS_BUSY                    0x181

typedef struct
{
  uint16 model_id;
  uint16 vendor_id;
} tsModel;

typedef struct
{
  uint16 loc;
  uint8 numS;
  uint8 numV;

  // reserve space for up to 11 SIG models for each element
  uint16 SIG_models[11];

  // reserve space for up to 4 vendor models for each element
  tsModel vendor_models[4];
} tsElement;

typedef struct
{
  uint16 cid;
  uint16 pid;
  uint16 vid;
  uint16 crpl;
  uint16 features;
  uint8  numElement;  // Not included in the DCD event after v1.4.0 sdk, need to calculate it.
  // reserve space for up to 2 elements
  tsElement element[2];
}tsDCD;

// DCD of the last provisioned device:
tsDCD _sDCD;

typedef struct
{
  uint16 model_id;
  uint16 vendor_id;
  uint8 element_idx;
} provModel;

typedef struct
{
  // model bindings to be done. for simplicity, all models are bound to same appkey in this example
  // (assuming there is exactly one appkey used and the same appkey is used for all model bindings)
  provModel bind_model[4];
  uint8 num_bind;
  uint8 num_bind_done;

  // publish addresses for up to 4 models
  provModel pub_model[4];
  uint16 pub_address[4];
  uint8 num_pub;
  uint8 num_pub_done;

  // subscription addresses for up to 4 models
  provModel sub_model[4];
  uint16 sub_address[4];
  uint8 num_sub;
  uint8 num_sub_done;
}tsConfig;

// config data to be sent to last provisioned node:
tsConfig _sConfig;

#define LIGHT_CTRL_GRP_ADDR     0xC001
#define LIGHT_STATUS_GRP_ADDR   0xC002

#define LIGHT_CTL_CTRL_TEMPERATURE   0xC005
#define LIGHT_CTL_STATUS_TEMPERATURE   0xC006

#define SENSOR_GRP_ADDR			0xC007

/* models used by simple light example (on/off only)
 * The beta SDK 1.0.1 and 1.1.0 examples are based on these
 * */
#define LIGHT_MODEL_ID            0x1000 // Generic On/Off Server
#define SWITCH_MODEL_ID           0x1001 // Generic On/Off Client

/*
 * Lightness models used in the dimming light example of 1.2.0 SDK
 * */
#define DIM_LIGHT_MODEL_ID              0x1300 // Light Lightness Server
#define DIM_SWITCH_MODEL_ID             0x1302 // Light Lightness Client

#define LIGHT_CTL_TEMPERATURE_SERVER    0x1306

#define SENSOR_CLIENT_MODEL_ID            0x1102
#define SENSOR_SERVER_MODEL_ID            0x1100

/*
 * Look-up table for mapping error codes to strings. Not a complete
 * list, for full description of error codes, see
 * Bluetooth LE and Mesh Software API Reference Manual */

tsErrCode _sErrCodes[] = {
    {0x0c01, "already_exists"},
    {0x0c02, "does_not_exist"},
    {0x0c03, "limit_reached"},
    {0x0c04, "invalid_address"},
    {0x0c05, "malformed_data"},
};

const char err_unknown[] = "<?>";

const char * res2str(uint16 err)
{
  int i;

  for(i=0;i<sizeof(_sErrCodes)/sizeof(tsErrCode);i++)
  {
    if(err == _sErrCodes[i].err)
    {
      return _sErrCodes[i].pShortDescription;
    }
  }

  // code was not found in the lookup table
  return err_unknown;
}


/***************************************************************************//**
 * This function is called to initiate factory reset. Factory reset may be
 * initiated by keeping one of the WSTK pushbuttons pressed during reboot.
 * Factory reset is also performed if it is requested by the provisioner
 * (event gecko_evt_mesh_node_reset_id).
 ******************************************************************************/
static void initiate_factory_reset(void)
{
  printf("factory reset\r\n");
  DI_Print("\n***\nFACTORY RESET\n***", DI_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_SYS_FACTORY_RESET, 1);
}


/***************************************************************************//**
 * This function prints debug information for mesh server state change event.
 *
 * @param[in] pEvt  Pointer to mesh_lib_generic_server_state_changed event.
 ******************************************************************************/
static void server_state_changed(struct gecko_msg_mesh_generic_server_state_changed_evt_t *pEvt)
{
  int i;

  printf("state changed: ");
  printf("model ID %4.4x, type %2.2x ", pEvt->model_id, pEvt->type);
  for (i = 0; i < pEvt->parameters.len; i++) {
    printf("%2.2x ", pEvt->parameters.data[i]);
  }
  printf("\r\n");
}

/*******************************************************************************
 * Initialise used bgapi classes.
 ******************************************************************************/
void gecko_bgapi_classes_init(void)
{
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  gecko_bgapi_class_gatt_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  //gecko_bgapi_class_sm_init();
  //mesh_native_bgapi_init();
//  gecko_bgapi_class_mesh_node_init();   //
#ifdef REMOTE_CTL_FROM_PROV
  gecko_bgapi_class_mesh_generic_server_init();   // If need to sub the message from group, need to initialize BGAPI class mesh_generic_server
  gecko_bgapi_class_mesh_generic_client_init();   // If need to pub the message to group, need to initialize BGAPI class mesh_generic_client
  gecko_bgapi_class_mesh_vendor_model_init();
  gecko_bgapi_class_mesh_test_init();       // If need the provisioner to pub/sub to/from group, initialize BGAPI class mesh_test
#endif
  gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_config_client_init();


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
  mesh_native_bgapi_init();
  gecko_bgapi_class_mesh_config_client_init();
  gecko_bgapi_class_mesh_prov_init();

//  gecko_bgapi_class_mesh_proxy_init();
//  gecko_bgapi_class_mesh_proxy_server_init();
  //gecko_bgapi_class_mesh_proxy_client_init();

  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  //gecko_bgapi_class_mesh_lpn_init();
//  gecko_bgapi_class_mesh_friend_init();
}

/*
 * Add one publication setting to the list of configurations to be done
 * */
static void config_pub_add(uint16 model_id, uint16 vendor_id, uint8_t element_idx, uint16 address)
{
  _sConfig.pub_model[_sConfig.num_pub].model_id = model_id;
  _sConfig.pub_model[_sConfig.num_pub].vendor_id = vendor_id;
  _sConfig.pub_model[_sConfig.num_pub].element_idx = element_idx;
  _sConfig.pub_address[_sConfig.num_pub] = address;
  _sConfig.num_pub++;
}

/*
 * Add one subscription setting to the list of configurations to be done
 * */
static void config_sub_add(uint16 model_id, uint16 vendor_id, uint8_t element_idx, uint16 address)
{
  _sConfig.sub_model[_sConfig.num_sub].model_id = model_id;
  _sConfig.sub_model[_sConfig.num_sub].vendor_id = vendor_id;
  _sConfig.sub_model[_sConfig.num_sub].element_idx = element_idx;
  _sConfig.sub_address[_sConfig.num_sub] = address;
  _sConfig.num_sub++;
}

/*
 * Add one appkey/model bind setting to the list of configurations to be done
 * */
static void config_bind_add(uint16 model_id, uint16 vendor_id, uint8_t element_idx, uint16 netkey_id, uint16 appkey_id)
{
  _sConfig.bind_model[_sConfig.num_bind].model_id = model_id;
  _sConfig.bind_model[_sConfig.num_bind].vendor_id = vendor_id;
  _sConfig.bind_model[_sConfig.num_bind].element_idx = element_idx;
  _sConfig.num_bind++;
}

/*
 * This function scans for the SIG models in the DCD that was read from a freshly provisioned node.
 * Based on the models that are listed, the publish/subscribe addresses are added into a configuration list
 * that is later used to configure the node.
 *
 * This example configures generic on/off client and lightness client to publish
 * to "light control" group address and subscribe to "light status" group address.
 *
 * Similarly, generic on/off server and lightness server (= the light node) models
 * are configured to subscribe to "light control" and publish to "light status" group address.
 *
 * Alternative strategy for automatically filling the configuration data would be to e.g. use the product ID from the DCD.
 *
 *
 * */

static void config_check()
{
  uint8 i,elementIdx;

  memset(&_sConfig, 0, sizeof(_sConfig));

  printf("\n------------ add models to sub/pub/bind list ------------\r\n");

  for(elementIdx=0; elementIdx<_sDCD.numElement;elementIdx++)
  {
    for(i=0; i<_sDCD.element[elementIdx].numS;i++)
      printf("SIG model ID: 0x%04x\r\n", _sDCD.element[elementIdx].SIG_models[i]);

    for(i=0; i<_sDCD.element[elementIdx].numV;i++)
      printf("Vendor model ID: vendorID 0x%04x modelID 0x%04x\r\n",
          _sDCD.element[elementIdx].vendor_models[i].vendor_id,
          _sDCD.element[elementIdx].vendor_models[i].model_id);

    printf("\r\n");

    // scan the SIG models in the DCD data
    for(i=0;i<_sDCD.element[elementIdx].numS;i++)
    {
      if(_sDCD.element[elementIdx].SIG_models[i] == SWITCH_MODEL_ID)
      {
        config_pub_add(SWITCH_MODEL_ID, 0xFFFF, elementIdx, LIGHT_CTRL_GRP_ADDR);
        config_sub_add(SWITCH_MODEL_ID, 0xFFFF, elementIdx, LIGHT_STATUS_GRP_ADDR);
        config_bind_add(SWITCH_MODEL_ID, 0xFFFF, elementIdx, 0, 0);
        printf("add the Generic On/Off Client model to sub/pub/bind list, elementIdx = %d\r\n",elementIdx);
      }
      else if(_sDCD.element[elementIdx].SIG_models[i] == LIGHT_MODEL_ID)
      {
        config_pub_add(LIGHT_MODEL_ID, 0xFFFF, elementIdx, LIGHT_STATUS_GRP_ADDR);
        config_sub_add(LIGHT_MODEL_ID, 0xFFFF, elementIdx, LIGHT_CTRL_GRP_ADDR);
        config_bind_add(LIGHT_MODEL_ID, 0xFFFF, elementIdx, 0, 0);
        printf("add the Generic On/Off Server model to sub/pub/bind list, elementIdx = %d\r\n",elementIdx);
      }
    // Just consider the Generic On/Off model for test
//      else if(_sDCD.element[elementIdx].SIG_models[i] == DIM_SWITCH_MODEL_ID)
//      {
//        config_pub_add(DIM_SWITCH_MODEL_ID, 0xFFFF,elementIdx, LIGHT_CTRL_GRP_ADDR);
//        config_sub_add(DIM_SWITCH_MODEL_ID, 0xFFFF,elementIdx, LIGHT_STATUS_GRP_ADDR);
//        config_bind_add(DIM_SWITCH_MODEL_ID, 0xFFFF,elementIdx, 0, 0);
//        printf("add the Light Lightness Client model to sub/pub/bind list\r\n");
//      }
//      else if(_sDCD.element[elementIdx].SIG_models[i] == DIM_LIGHT_MODEL_ID)
//      {
//        config_pub_add(DIM_LIGHT_MODEL_ID, 0xFFFF,elementIdx, LIGHT_STATUS_GRP_ADDR);
//        config_sub_add(DIM_LIGHT_MODEL_ID, 0xFFFF,elementIdx, LIGHT_CTRL_GRP_ADDR);
//        config_bind_add(DIM_LIGHT_MODEL_ID, 0xFFFF, elementIdx, 0, 0);
//        printf("add the Light Lightness Server model to sub/pub/bind list\r\n");
//      }
//      else if(_sDCD.element[elementIdx].SIG_models[i] == LIGHT_CTL_TEMPERATURE_SERVER)
//      {
//        config_pub_add(LIGHT_CTL_TEMPERATURE_SERVER, 0xFFFF, elementIdx, LIGHT_CTL_STATUS_TEMPERATURE);
//        config_sub_add(LIGHT_CTL_TEMPERATURE_SERVER, 0xFFFF, elementIdx, LIGHT_CTL_CTRL_TEMPERATURE);
//        config_bind_add(LIGHT_CTL_TEMPERATURE_SERVER, 0xFFFF, elementIdx, 0, 0);
//        printf("add the CTL temperature Server model to sub/pub/bind list, elementIdx = %d\r\n", elementIdx);
//      }
//		else if(_sDCD.element[elementIdx].SIG_models[i] == SENSOR_CLIENT_MODEL_ID)
//		{
//			config_pub_add(SENSOR_CLIENT_MODEL_ID, 0xFFFF, elementIdx, SENSOR_GRP_ADDR);
//			config_sub_add(SENSOR_CLIENT_MODEL_ID, 0xFFFF, elementIdx, SENSOR_GRP_ADDR);
//			config_bind_add(SENSOR_CLIENT_MODEL_ID, 0xFFFF, elementIdx, 0, 0);
//			printf("add the sensor client model to sub/pub/bind list, elementIdx = %d\r\n",elementIdx);
//		}
//		else if(_sDCD.element[elementIdx].SIG_models[i] == SENSOR_SERVER_MODEL_ID)
//		{
//			config_pub_add(SENSOR_SERVER_MODEL_ID, 0xFFFF, elementIdx, SENSOR_GRP_ADDR);
//			config_sub_add(SENSOR_SERVER_MODEL_ID, 0xFFFF, elementIdx, SENSOR_GRP_ADDR);
//			config_bind_add(SENSOR_SERVER_MODEL_ID, 0xFFFF, elementIdx, 0, 0);
//			printf("add the sensor server model to sub/pub/bind list, elementIdx = %d\r\n",elementIdx);
//		}

    }

  #ifdef CONFIGURE_VENDOR_MODEL
    printf("---------- add Vendor models to sub/pub/bind list ----------\r\n");
    // scan the vendor models found in the DCD
    for(i=0;i<_sDCD.element[elementIdx].numV;i++)
    {
      // this example only handles vendor model with vendor ID 0x02FF (Silabs) and model ID 0xABCD.
      // if such model found, configure it to publish/subscribe to a single group address
      if((_sDCD.element[elementIdx].vendor_models[i].model_id == VM_SERVER_MODEL_ID) && (_sDCD.element[elementIdx].vendor_models[i].vendor_id == VM_VENDOR_ID))
      {
        config_pub_add(VM_SERVER_MODEL_ID, VM_VENDOR_ID, elementIdx, VENDOR_GRP_SUB_ADDR);
        config_sub_add(VM_SERVER_MODEL_ID, VM_VENDOR_ID, elementIdx, VENDOR_GRP_SUB_ADDR);
        // using single appkey to bind all models. It could be also possible to use different appkey for the
        // vendor models
        config_bind_add(VM_SERVER_MODEL_ID, VM_VENDOR_ID, elementIdx, 0, 0);
        printf("add the vendor model 0x%4.4X to sub/pub/bind list\r\n",_sDCD.element[elementIdx].vendor_models[i].model_id);
      }
      else if((_sDCD.element[elementIdx].vendor_models[i].model_id == VM_CLIENT_MODEL_ID) && (_sDCD.element[elementIdx].vendor_models[i].vendor_id == VM_VENDOR_ID))
      {
        config_pub_add(VM_CLIENT_MODEL_ID, VM_VENDOR_ID, elementIdx, VENDOR_GRP_PUB_ADDR);
        config_sub_add(VM_CLIENT_MODEL_ID, VM_VENDOR_ID, elementIdx, VENDOR_GRP_PUB_ADDR);
        // using single appkey to bind all models. It could be also possible to use different appkey for the
        // vendor models
        config_bind_add(VM_CLIENT_MODEL_ID, VM_VENDOR_ID, elementIdx, 0, 0);
        printf("add the vendor model 0x%4.4X to sub/pub/bind list\r\n",_sDCD.element[elementIdx].vendor_models[i].model_id);
      }
    }
  #endif
  }
}


static void  button_poll()
{
  int key;
  if(ask_user_input == false){
    return;
  }

  /* provisioning can be accepted/rejected either by pushbuttons PB1/PB0
   * or alternatively by entering 'y' / 'n' on keyboard */
  key = RETARGET_ReadChar();

  if (GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0 || key == 'y') {
    ask_user_input = false;
    printf("Sending prov request\r\n");

#ifndef PROVISION_OVER_GATT
    // provisioning using ADV bearer (this is the default)
    struct gecko_msg_mesh_prov_provision_device_rsp_t *prov_resp_adv;
    prov_resp_adv = gecko_cmd_mesh_prov_provision_device(netkey_id, 16, _uuid_copy);

    if (prov_resp_adv->result == 0) {
      printf("Successful call of gecko_cmd_mesh_prov_provision_device\r\n");
      state = provisioning;
    } else {
      printf("Failed call to provision node. %x\r\n", prov_resp_adv->result);
    }
#else
    // provisioning using GATT bearer. First we must open a connection to the remote device
    if(gecko_cmd_le_gap_open(bt_address, bt_address_type)->result == 0)
    {
      printf("trying to open a connection\r\n");
    }
    else
    {
      printf("le_gap_open failed\r\n");
    }
    state = connecting;

#endif
  }
  else if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || key == 'n') {
    ask_user_input = false;
  }

}


static void DCD_decode(struct gecko_msg_mesh_config_client_dcd_data_evt_t *pDCD)
{
  uint16 *pu16;
  uint8 i,j;

  if(pDCD->page)
    return;

  printf("--------------------- Decode the DCD ---------------------- \r\n");

  // Clear the _sDCD structure for each node DCD decode
  memset(&_sDCD, 0, sizeof(_sDCD));

  pu16 = (uint16 *)&(pDCD->data.data[0]);
  printf("pDCD->data.data[0] = 0x%2.2o \r\n",pDCD->data.data[0]);
  printf("pDCD->data.len = %4d \r\n",pDCD->data.len);
  _sDCD.cid = *pu16++;
  _sDCD.pid = *pu16++;
  _sDCD.vid = *pu16++;
  _sDCD.crpl = *pu16++;
  _sDCD.features = *pu16++;

  printf("DCD: company ID 0x%4.4x, Product ID 0x%4.4x, Vendor ID 0x%4.4x, CRPL 0x%4.4x, features 0x%4.4x\r\n",
      _sDCD.cid,
      _sDCD.pid,
      _sDCD.vid,
      _sDCD.crpl,
      _sDCD.features);

  // calculate the number of elements
  // CID, PID, VID, CRPL, Features total 10 octets, grab elements from 11 octets
  for(i = 10; i < pDCD->data.len;)
  {
    i = i + ((4 + pDCD->data.data[i+2]*2 + pDCD->data.data[i+3]*4));
    _sDCD.numElement++;
  }

  printf("DCD: elements number %d \r\n", _sDCD.numElement);

  for(i = 0; i<_sDCD.numElement; i++)
  {
    _sDCD.element[i].loc = *pu16++;

    _sDCD.element[i].numS = (uint8)(*pu16 & 0x00FF);
    _sDCD.element[i].numV = (uint8)((*pu16 & 0xFF00) >> 8);
    pu16++;

    printf(" ->->-> element %d information ->->-> \r\n",i);
    // grab the SIG models from the DCD data
    for(j=0; j<_sDCD.element[i].numS; j++)
    {
      _sDCD.element[i].SIG_models[j] = *pu16++;
      printf("SIG model ID: 0x%04X\r\n", _sDCD.element[i].SIG_models[j]);
    }

    // grab the vendor models from the DCD data
    for(j=0; j<_sDCD.element[i].numV; j++)
    {
      _sDCD.element[i].vendor_models[j].vendor_id = *pu16++;
      _sDCD.element[i].vendor_models[j].model_id = *pu16++;
      printf("Vendor model ID: 0x%04X, model ID: 0x%04X\r\n", _sDCD.element[i].vendor_models[j].vendor_id, _sDCD.element[i].vendor_models[j].model_id);
    }

  }

}


#ifdef REMOTE_CTL_FROM_PROV
void send_onoff_request(int retrans)
{
  static uint8_t onOffCtl = 0;
  static uint8_t trid = 0;
  uint16 resp;
  uint16 delay = 0;
  struct mesh_generic_request req;
  const uint32 transtime = 0; /* using zero transition time by default */

  req.kind = mesh_generic_request_on_off;
  req.on_off = onOffCtl ? MESH_GENERIC_ON_OFF_STATE_ON : MESH_GENERIC_ON_OFF_STATE_OFF;
  onOffCtl = onOffCtl?0:1;

  // increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    trid++;
  }

  /* delay for the request is calculated so that the last request will have a zero delay and each
   * of the previous request have delay that increases in 50 ms steps. For example, when using three
   * on/off requests per button press the delays are set as 100, 50, 0 ms
   */
  resp = gecko_cmd_mesh_generic_client_publish(
    MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
    0,
    trid,
    transtime,   // transition time in ms
    delay,
    0,     // flags
    mesh_generic_request_on_off,     // type
    1,     // param len
    &req.on_off     /// parameters data
    )->result;

  if (resp) {
    printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
  } else {
    printf("request sent, delay = %d\r\n", delay);
  }

}

void vendor_model_publish(void)
{
	uint8_t my_name[] = "EATON";
	uint16 result;
	/* Set up the publication data with User Name */
	result = gecko_cmd_mesh_vendor_model_set_publication(0, VM_VENDOR_ID, VM_CLIENT_MODEL_ID, user_name, 1, sizeof(my_name)-1, my_name)->result;
	printf("Set vendor model publication message, result %d\r\n", result);

	/* Publish once, the data was set before */
	result = gecko_cmd_mesh_vendor_model_publish(0, VM_VENDOR_ID, VM_CLIENT_MODEL_ID)->result;
	printf("Publish vendor model publication message, result %d\r\n", result);
}


#endif

/*******************************************************************************
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] evt     Pointer to incoming event.
 ******************************************************************************/
void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  if (NULL == evt) {
    return;
  }

  switch (evt_id) {
    case gecko_evt_system_boot_id:
      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
      if ((GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0) || (GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0)) {
        initiate_factory_reset();
      } else {
        printf("Initializing as provisioner\r\n");

        state = init;
        // init as provisioner
        struct gecko_msg_mesh_prov_init_rsp_t *prov_init_rsp = gecko_cmd_mesh_prov_init();
        if (prov_init_rsp->result == 0) {
          printf("Successfully initialized\r\n");
        } else {
          printf("Error initializing node as provisioner. Error %x\r\n", prov_init_rsp->result);
          gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(20), TIMER_ID_SYS_RESTART, 1);
        }
      }
      break;

    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {

        case TIMER_ID_SYS_BUTTON_POLL:
          button_poll();
          break;

        case TIMER_ID_GET_DCD:
        {
  //        struct gecko_msg_mesh_prov_get_dcd_rsp_t*  get_dcd_result = gecko_cmd_mesh_prov_get_dcd(provisionee_address, 0xFF);

          struct gecko_msg_mesh_config_client_get_dcd_rsp_t* get_dcd_result = gecko_cmd_mesh_config_client_get_dcd(netkey_id, provisionee_address, 0xFF);

          if (get_dcd_result->result == 0x0181) {
            printf(".");
            fflush(stdout);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(1000), TIMER_ID_GET_DCD, 1);
          } else if(get_dcd_result->result != STATUS_OK){
            printf("gecko_cmd_mesh_config_client_get_dcd failed with result 0x%X (%s) addr %x\r\n", get_dcd_result->result, res2str(get_dcd_result->result), provisionee_address);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(1000), TIMER_ID_GET_DCD, 1);
          }
          else
          {
            printf("requesting DCD from the node...\r\n");
            state = waiting_dcd;
          }

        }
        break;

        case TIMER_ID_APPKEY_ADD:
        {
  //        struct gecko_msg_mesh_prov_appkey_add_rsp_t *appkey_deploy_evt;
  //            appkey_deploy_evt = gecko_cmd_mesh_prov_appkey_add(provisionee_address, netkey_id, appkey_id);

          struct gecko_msg_mesh_config_client_add_appkey_rsp_t *appkey_deploy_evt;
          appkey_deploy_evt = gecko_cmd_mesh_config_client_add_appkey(netkey_id, provisionee_address, appkey_id, netkey_id);

          if (appkey_deploy_evt->result == STATUS_OK) {
            printf("\r\nAdd an application key to device, address 0x%x, waiting for the response\r\n", provisionee_address);
            state = waiting_appkey_ack;
          }
          else{
            printf("\r\nAppkey deployment failed. addr 0x%04x, error: 0x%X\r\n", provisionee_address, appkey_deploy_evt->result);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_ADD, 1);
          }
        }
        break;

        case TIMER_ID_APPKEY_BIND:
        {
          uint16 vendor_id;
          uint16 model_id;
          uint8 element_idx = 0;

          // take the next model from the list of models to be bound with application key.
          // for simplicity, the same appkey is used for all models but it is possible to also use several appkeys
          model_id = _sConfig.bind_model[_sConfig.num_bind_done].model_id;
          vendor_id = _sConfig.bind_model[_sConfig.num_bind_done].vendor_id;
          element_idx = _sConfig.bind_model[_sConfig.num_bind_done].element_idx;

          printf("\r\nAPP_BIND, config %d/%d:: model 0x%4.4X vendor_id 0x%4.4X, element_idx %d key index %x\r\n", _sConfig.num_bind_done+1, _sConfig.num_bind, model_id, vendor_id, element_idx, appkey_id);

          //        struct gecko_msg_mesh_prov_model_app_bind_rsp_t *model_app_bind_result = gecko_cmd_mesh_prov_model_app_bind(provisionee_address,
  //            provisionee_address,
  //          netkey_id,
  //          appkey_id,
  //          vendor_id,
  //          model_id);

          struct gecko_msg_mesh_config_client_bind_model_rsp_t *model_app_bind_result = gecko_cmd_mesh_config_client_bind_model(netkey_id,
              provisionee_address,
              element_idx,    // only bind the models for the element 0 by default
              appkey_id,
              vendor_id,
              model_id);

          if (model_app_bind_result->result == STATUS_BUSY)
          {
            printf(".");
            fflush(stdout);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_BIND, 1);
          }
          else if(model_app_bind_result->result != STATUS_OK)
          {
            printf("prov_model_app_bind failed with result 0x%X\r\n", model_app_bind_result->result);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_BIND, 1);
          }
          else if(model_app_bind_result->result  == STATUS_OK)
          {
            printf("Bind an application key successfully, waiting bind ack\r\n");
            state = waiting_bind_ack;
          }
        }
        break;

        case TIMER_ID_PUB_SET:
        {
          uint16 vendor_id;
          uint16 model_id;
          uint16 pub_address;
          uint8 element_idx = 0;

          // get the next model/address pair from the configuration list:
          model_id = _sConfig.pub_model[_sConfig.num_pub_done].model_id;
          vendor_id = _sConfig.pub_model[_sConfig.num_pub_done].vendor_id;
          element_idx = _sConfig.pub_model[_sConfig.num_pub_done].element_idx;
          pub_address = _sConfig.pub_address[_sConfig.num_pub_done];

          printf("\r\npublish set, config %d/%d: model_id 0x%4.4X vendor_id 0x%4.4X, element_idx %d -> address %4.4x\r\n", _sConfig.num_pub_done+1, _sConfig.num_pub, model_id, vendor_id, element_idx, pub_address);

          struct gecko_msg_mesh_config_client_set_model_pub_rsp_t *model_pub_set_result = gecko_cmd_mesh_config_client_set_model_pub(netkey_id,
              provisionee_address,
              element_idx,                // element_index
              vendor_id,
              model_id,
              pub_address,
              appkey_id,
              0,              // friendship credential flag
              3,              // Publication time-to-live value
              0,              // period = NONE
              0,              // model publication retransmissions
              50               // publication retransmissions interval, range 50 to 1600ms
              );

          if (model_pub_set_result->result == STATUS_BUSY)
          {
            printf(".");
            fflush(stdout);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_PUB_SET, 1);
          }
          else if(model_pub_set_result->result != STATUS_OK)
          {
            printf("prov_model_pub_set failed with result 0x%04X\r\n", model_pub_set_result->result);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_PUB_SET, 1);
          }
          else if (model_pub_set_result->result == STATUS_OK)
          {
            printf("Set model publication state success - waiting pub ack\r\n");
            state = waiting_pub_ack;
          }
        }
        break;

        case TIMER_ID_SUB_ADD:
        {
          uint16 vendor_id = 0xFFFF;
          uint16 model_id;
          uint16 sub_address;
          uint8 element_idx = 0;

          // get the next model/address pair from the configuration list:
          model_id = _sConfig.sub_model[_sConfig.num_sub_done].model_id;
          vendor_id = _sConfig.sub_model[_sConfig.num_sub_done].vendor_id;
          element_idx = _sConfig.sub_model[_sConfig.num_sub_done].element_idx;

          sub_address = _sConfig.sub_address[_sConfig.num_sub_done];

          printf("\r\nsubscription add, config %d/%d: model %4.4x -> address %4.4x\r\n", _sConfig.num_sub_done+1, _sConfig.num_sub, model_id, sub_address);

          struct gecko_msg_mesh_config_client_add_model_sub_rsp_t *model_sub_add_result = gecko_cmd_mesh_config_client_add_model_sub(netkey_id,
              provisionee_address,
              element_idx,
              vendor_id,
              model_id,
              sub_address);

          if(model_sub_add_result->result == STATUS_BUSY)
          {
            printf(".");
            fflush(stdout);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_SUB_ADD, 1);
          }
          else if(model_sub_add_result->result != STATUS_OK)
          {
            printf("prov_model_sub_add failed with result 0x%X\r\n",model_sub_add_result->result);
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_SUB_ADD, 1);
          }
          else if (model_sub_add_result->result == STATUS_OK)
          {
            printf("Add address to model subscription list success - waiting sub ack\r\n");
            state = waiting_sub_ack;
          }
        }
        break;

  #ifdef REMOTE_CTL_FROM_PROV
        case TIMER_ID_SEND_PUB_MSG:
          // send the pub message from provisioner
          printf("evt TIMER_ID_SEND_PUB_MSG send the message from provisioner\r\n");
//          send_onoff_request(0);

          vendor_model_publish();
          break;
  #endif

        case TIMER_ID_SYS_FACTORY_RESET:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_SYS_RESTART:
          gecko_cmd_system_reset(0);
          break;

        default:
          break;
        }

      break;

    case gecko_evt_mesh_config_client_dcd_data_id:
    {
      struct gecko_msg_mesh_config_client_dcd_data_evt_t *pDCD = (struct gecko_msg_mesh_config_client_dcd_data_evt_t *)&(evt->data);
      printf("evt: gecko_evt_mesh_config_client_dcd_data_id, request handle: 0x%08lx, page: 0x%02x\r\n", pDCD->handle, pDCD->page);

      if(pDCD->page == 0)   // page 0 contains the DCD
      {
        // decode the DCD content
        DCD_decode(pDCD);

        // check the desired configuration settings depending on what's in the DCD
        config_check();
      }

    }
      break;

    case gecko_evt_mesh_config_client_dcd_data_end_id:
    {
      struct gecko_msg_mesh_config_client_dcd_data_end_evt_t *pDcdEnd = (struct gecko_msg_mesh_config_client_dcd_data_end_evt_t *)&(evt->data);
      printf("evt: gecko_evt_mesh_config_client_dcd_data_end_id, request handle: 0x%08lx, result 0x%X\r\n",
          pDcdEnd->handle,
          pDcdEnd->result);

      if(pDcdEnd->result != STATUS_OK)
      {
        // if config failed, try again up to 5 times
        if (config_retrycount++ < 5) {
          printf("Get DCD not successful, will try again\r\n");
          gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_GET_DCD, 1);
        }
        else
        {
          printf("ERROR: get DCD failed, retry limit reached. stuck at state %d\r\n", state);
        }
      }
      else
      {
        config_retrycount = 0;
        // next step : send appkey to device
        gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_ADD, 1);
      }
    }
      break;

    case gecko_evt_mesh_config_client_appkey_status_id:
    {
      struct gecko_msg_mesh_config_client_appkey_status_evt_t *pRsp = (struct gecko_msg_mesh_config_client_appkey_status_evt_t *)&(evt->data);
      if(state == waiting_appkey_ack)
      {
        if(pRsp->result != STATUS_OK)
        {
          printf("evt: gecko_evt_mesh_config_client_appkey_status_id, Add an application key failed, error 0x%04X \r\n", pRsp->result);

          // if config failed, try again up to 5 times
          if (config_retrycount++ < 5) {
            printf("Not successful, will try again\r\n");
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_ADD, 1);
          }
          else
          {
            printf("ERROR: config failed, retry limit reached. stuck at state %d\r\n", state);
          }
        }
        else
        {
          config_retrycount = 0;
          printf("evt: gecko_evt_mesh_config_client_appkey_status_id, Add an application key successfully \r\n");
          gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_BIND, 1);
        }
      }
    }
      break;

    case gecko_evt_mesh_config_client_binding_status_id:
    {
      struct gecko_msg_mesh_config_client_binding_status_evt_t *model_app_bind_status = (struct gecko_msg_mesh_config_client_binding_status_evt_t *)&(evt->data);
      if(state == waiting_bind_ack)
      {
        if(model_app_bind_status->result != STATUS_OK)
        {
          printf("evt: gecko_evt_mesh_config_client_binding_status_id, Bind an application key failed, error 0x%04x \r\n", model_app_bind_status->result);

          // if config failed, try again up to 5 times
          if (config_retrycount++ < 5) {
            printf("Not successful, will try again\r\n");
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_BIND, 1);
          }
          else
          {
            printf("ERROR: config failed, retry limit reached. stuck at state %d\r\n", state);
          }

        }
        else
        {
          config_retrycount = 0;
          printf("evt: gecko_evt_mesh_config_client_binding_status_id, Bind an application key successfully \r\n");

          _sConfig.num_bind_done++;

          if(_sConfig.num_bind_done < _sConfig.num_bind)
          {
            // more model<->appkey bindings to be done
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_BIND, 1);
          }
          else
          {
            // start PUB after finishing the appkey bind
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_PUB_SET, 1);
          }
        }
      }
    }
      break;


    case gecko_evt_mesh_config_client_model_pub_status_id:
    {
      struct gecko_msg_mesh_config_client_model_pub_status_evt_t *model_pub_status = (struct gecko_msg_mesh_config_client_model_pub_status_evt_t *)&(evt->data);

      if(state == waiting_pub_ack)
      {

        if(model_pub_status->result != STATUS_OK)
        {
          printf("evt: gecko_evt_mesh_config_client_model_pub_status_id, Set model publication addr failed, error 0x%04x \r\n", model_pub_status->result);

          // if config failed, try again up to 5 times
          if (config_retrycount++ < 5) {
            printf("Not successful, will try again\r\n");
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_PUB_SET, 1);
          }
          else
          {
            printf("ERROR: config failed, retry limit reached. stuck at state %d\r\n", state);
          }
        }
        else
        {
          config_retrycount = 0;

          printf("PUB complete\r\n");
          _sConfig.num_pub_done++;

          if(_sConfig.num_pub_done < _sConfig.num_pub)
          {
            // more publication settings to be done
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_PUB_SET, 1);
          }
          else
          {
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_SUB_ADD, 1);
          }
        }
      }

    }
      break;

    case gecko_evt_mesh_config_client_model_sub_status_id:
    {
      struct gecko_msg_mesh_config_client_model_sub_status_evt_t *model_sub_status = (struct gecko_msg_mesh_config_client_model_sub_status_evt_t *)&(evt->data);
      if(state == waiting_sub_ack)
      {
        if(model_sub_status->result != STATUS_OK)
        {
          printf("evt: gecko_evt_mesh_config_client_model_sub_status_id, Add model subscription failed, error 0x%04x \r\n", model_sub_status->result);

          // if config failed, try again up to 5 times
          if (config_retrycount++ < 5) {
            printf("Not successful, will try again\r\n");
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_SUB_ADD, 1);
          }
          else
          {
            printf("ERROR: config failed, retry limit reached. stuck at state %d\r\n", state);
          }

        }
        else
        {
          config_retrycount = 0;

          printf("SUB complete\r\n");
          _sConfig.num_sub_done++;
          if(_sConfig.num_sub_done < _sConfig.num_sub)
          {
            // more subscription settings to be done
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_SUB_ADD, 1);
          }
          else
          {
            printf("***\r\n configuration complete\r\n***\r\n");
            state = scanning;

#ifdef REMOTE_CTL_FROM_PROV
            // If need to subscribe the message from switch node, bind and sub the group address here.
            uint8_t i = 0;

            /* If need to subscribe message from switch node or pub message to light node, bind and sub/pub the group address here.
            *
            * Below are the subscription and publication address Information
            *         		PubAddr   SubAddr
            * Light   		0xC002    0xC001
            * Switch  		0xC001    0xC002
            * Vendor model	0xC003    None
            * If need to subscribe the message from Switch node, the provisioner's server model need to sub to 0xC001
            * If need to send control message to Light node, the provisioner's client model need to pub to 0xC001
            * If need to send message with vendor model, the provisioner's vendor client model need to pub to 0xC003
            */

              /* Set the subscription of Generic On Off server model*/
            struct gecko_msg_mesh_generic_server_init_rsp_t *genericServerInitRsp = gecko_cmd_mesh_generic_server_init();
            printf("gecko_cmd_mesh_generic_server_init %x\n\r", genericServerInitRsp->result);

            struct gecko_msg_mesh_test_get_local_model_sub_rsp_t *sub_setting = gecko_cmd_mesh_test_get_local_model_sub(0, 0xFFFF, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_sub result: %x sub_add_len: %d\r\n", sub_setting->result, sub_setting->addresses.len);

            for(i=0; i<sub_setting->addresses.len; i++)
              printf("gecko_cmd_mesh_test_get_local_model_sub, model 0x%4.4X sub_address: %x\r\n", MESH_GENERIC_ON_OFF_SERVER_MODEL_ID, sub_setting->addresses.data[i]);

            if (!sub_setting->result && (sub_setting->addresses.len != 0)) {
              // In fact, need to check the subscription address list here.
              printf("Subscription Configuration done already.\n");
            }
            else{
              //struct gecko_msg_mesh_test_add_local_key_rsp_t *addLocalKeyRsp = gecko_cmd_mesh_test_add_local_key(1, appKey, appkey_id, netkey_id);
              //printf("gecko_cmd_mesh_test_add_local_key %x\n\r", addLocalKeyRsp->result);

              struct gecko_msg_mesh_test_bind_local_model_app_rsp_t *bindLocalModelRsp = gecko_cmd_mesh_test_bind_local_model_app(0, appkey_id, 0xffff, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID);
              printf("gecko_cmd_mesh_test_bind_local_model_app %x\n\r", bindLocalModelRsp->result);

              struct gecko_msg_mesh_test_add_local_model_sub_rsp_t *addLocalModelRsp = gecko_cmd_mesh_test_add_local_model_sub(0, 0xffff, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID, 0xC001);
              printf("gecko_cmd_mesh_test_add_local_model_sub %x\n\r", addLocalModelRsp->result);
            }

            /* Set the publication of Generic On Off client model*/
            struct gecko_msg_mesh_generic_client_init_rsp_t *genericClientInitRsp = gecko_cmd_mesh_generic_client_init();
            printf("\r\ngecko_cmd_mesh_generic_client_init %x\n\r", genericClientInitRsp->result);

            struct gecko_msg_mesh_test_get_local_model_pub_rsp_t *pub_setting = gecko_cmd_mesh_test_get_local_model_pub(0, 0xFFFF, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_pub result: %x\r\n", pub_setting->result);
            printf("gecko_cmd_mesh_test_get_local_model_pub, model 0x%4.4X pub_address: 0x%04x\r\n", MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID, pub_setting->pub_address);

            if (!sub_setting->result && (pub_setting->pub_address != 0)) {
              printf("Publication Configuration done already.\n");
            }
            else{
              struct gecko_msg_mesh_test_bind_local_model_app_rsp_t *bindLocalModelRsp = gecko_cmd_mesh_test_bind_local_model_app(0, appkey_id, 0xffff, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID);
              printf("gecko_cmd_mesh_test_bind_local_model_app %x\n\r", bindLocalModelRsp->result);

              struct gecko_msg_mesh_test_set_local_model_pub_rsp_t *setLocalModelPubRsp = gecko_cmd_mesh_test_set_local_model_pub(0, appkey_id, 0xffff, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID, 0xC001, 5, 0, 0, 0);
              printf("gecko_cmd_mesh_test_set_local_model_pub %x\n\r", setLocalModelPubRsp->result);
            }

            /* Set the publication of vendor client model*/

            struct gecko_msg_mesh_vendor_model_init_rsp_t *vendorModelInitRsp = gecko_cmd_mesh_vendor_model_init(0, VM_VENDOR_ID, VM_CLIENT_MODEL_ID, true, sizeof(my_opc), my_opc);;
            printf("\r\ngecko_cmd_mesh_vendor_model_init %x\n\r", vendorModelInitRsp->result);

            struct gecko_msg_mesh_test_get_local_model_pub_rsp_t *vm_pub_setting = gecko_cmd_mesh_test_get_local_model_pub(0, VM_VENDOR_ID, VM_CLIENT_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_pub result: %x\r\n", vm_pub_setting->result);
            printf("gecko_cmd_mesh_test_get_local_model_pub, model 0x%4.4X pub_address: 0x%04x\r\n", VM_CLIENT_MODEL_ID, vm_pub_setting->pub_address);

            if (!sub_setting->result && (pub_setting->pub_address != 0)) {
              printf("Publication Configuration done already.\n");
            }
            else{
              struct gecko_msg_mesh_test_bind_local_model_app_rsp_t *vmBindLocalModelRsp = gecko_cmd_mesh_test_bind_local_model_app(0, appkey_id, VM_VENDOR_ID, VM_CLIENT_MODEL_ID);
              printf("gecko_cmd_mesh_test_bind_local_model_app %x\n\r", vmBindLocalModelRsp->result);

              struct gecko_msg_mesh_test_set_local_model_pub_rsp_t *vmSetLocalModelPubRsp = gecko_cmd_mesh_test_set_local_model_pub(0, appkey_id, VM_VENDOR_ID, VM_CLIENT_MODEL_ID, 0xC003, 5, 0, 2, 0);
              printf("gecko_cmd_mesh_test_set_local_model_pub %x\n\r", vmSetLocalModelPubRsp->result);
            }
            // ------------------------------------------ //
            printf("\r\n---------------- Dump sub/pub infor here ----------------\r\n");
            sub_setting = gecko_cmd_mesh_test_get_local_model_sub(0, 0xFFFF, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID);
            for(i=0; i<sub_setting->addresses.len; (i=i+2))
              printf("gecko_cmd_mesh_test_get_local_model_sub, SIG model 0x%4.4X, sub_address: 0x%02x%02x\r\n", MESH_GENERIC_ON_OFF_SERVER_MODEL_ID, sub_setting->addresses.data[i+1], sub_setting->addresses.data[i]);

            sub_setting = gecko_cmd_mesh_test_get_local_model_sub(0, VM_VENDOR_ID, VM_SERVER_MODEL_ID);
            for(i=0; i<sub_setting->addresses.len; (i=i+2))
              printf("gecko_cmd_mesh_test_get_local_model_sub, vendor model 0x%4.4X, sub_address: 0x%02x%02x\r\n", VM_CLIENT_MODEL_ID, sub_setting->addresses.data[i+1], sub_setting->addresses.data[i]);

            pub_setting = gecko_cmd_mesh_test_get_local_model_pub(0, 0xFFFF, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_pub, model 0x%4.4X, pub_address: 0x%04x\r\n", MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID, pub_setting->pub_address);

            pub_setting = gecko_cmd_mesh_test_get_local_model_pub(0, VM_VENDOR_ID, VM_CLIENT_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_pub, vendor model 0x%4.4X, pub_address: 0x%04x\r\n", VM_CLIENT_MODEL_ID, pub_setting->pub_address);
            printf("---------------------------------------------------------\r\n");

            // uncomment it if need to control the light node from the provisioner
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000), TIMER_ID_SEND_PUB_MSG, 0);
  #endif

          }
        }
      }
    }
      break;

#ifdef API_MESH_V13
    case gecko_evt_mesh_prov_dcd_status_id:
    {
      struct gecko_msg_mesh_prov_dcd_status_evt_t *pDCD = (struct gecko_msg_mesh_prov_dcd_status_evt_t *)&(evt->data);
      printf("DCD status event. result = %x\r\n", pDCD->result);

      if(pDCD->result == 0)
      {
        // decode the DCD content
        DCD_decode(pDCD);

        // check the desired configuration settings depending on what's in the DCD
        config_check();

        // next step : send appkey to device
        gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_ADD, 1);
      }
      else
      {
        printf("DCD status: %x\r\n",  pDCD->result);
      }

    }
    break;


    case gecko_evt_mesh_prov_config_status_id:
    {
      struct gecko_msg_mesh_prov_config_status_evt_t *conf_status_evt = (struct gecko_msg_mesh_prov_config_status_evt_t *)&evt->data;

      printf("mesh_prov_config_status: addr = 0x%X, id = 0x%X, status = 0x%X\r\n",
          conf_status_evt->address,
          conf_status_evt->id,
          conf_status_evt->status);

      if (conf_status_evt->status) {

//        // if config failed, try again up to 5 times
//        if (config_retrycount < 5) {
//          printf("Not successful, will try again\r\n");
//          config_retrycount++;
//          config_retry();
//        } else {
//          printf("ERROR: config failed, retry limit reached. stuck at state %d\r\n", state);
//        }
      }
//      else
//      {
//        config_retrycount = 0;
        // move to next phase in configuration

//        if(state == waiting_appkey_ack)
//        {
//          gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_BIND, 1);
//        }
//        else
//          if(state == waiting_bind_ack)
//        {
//          printf("bind complete\r\n");
//          _sConfig.num_bind_done++;
//
//          if(_sConfig.num_bind_done < _sConfig.num_bind)
//          {
//            // more model<->appkey bindings to be done
//            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_APPKEY_BIND, 1);
//          }
//          else
//          {
//            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_PUB_SET, 1);
//          }
//        }
//        else
//          if(state == waiting_pub_ack)
//        {
//          printf("PUB complete\r\n");
//          _sConfig.num_pub_done++;
//
//          if(_sConfig.num_pub_done < _sConfig.num_pub)
//          {
//            // more publication settings to be done
//            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_PUB_SET, 1);
//          }
//          else
//          {
//            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_SUB_ADD, 1);
//          }
//        }
//        else
//          if(state == waiting_sub_ack)
//        {
//          printf("SUB complete\r\n");
//          _sConfig.num_sub_done++;
//          if(_sConfig.num_sub_done < _sConfig.num_sub)
//          {
//            // more subscription settings to be done
//            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_SUB_ADD, 1);
//          }
//          else
//          {
//            printf("***\r\nconfiguration complete\r\n***\r\n");

#ifdef REMOTE_CTL_FROM_PROV
            // If need to subscribe the message from switch node, bind and sub the group address here.
            uint8_t i = 0;

            /* If need to subscribe message from switch node or pub message to light node, bind and sub/pub the group address here.
            *
            * Below are the subscription and publication address Information
            *         PubAddr   SubAddr
            * Light   0xC002    0xC001
            * Switch  0xC001    0xC002
            *
            * If need to subscribe the message from Switch node, the provisioner's server model need to sub to 0xC001
            * If need to send control message to Light node, the provisioner's client model need to pub to 0xC001
            */

              /* Set the subscription */
            struct gecko_msg_mesh_generic_server_init_rsp_t *genericServerInitRsp = gecko_cmd_mesh_generic_server_init();
            printf("gecko_cmd_mesh_generic_server_init %x\n\r", genericServerInitRsp->result);

            struct gecko_msg_mesh_test_get_local_model_sub_rsp_t *sub_setting = gecko_cmd_mesh_test_get_local_model_sub(0, 0xFFFF, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_sub result: %x sub_add_len: %d\r\n", sub_setting->result, sub_setting->addresses.len);

            for(i=0; i<sub_setting->addresses.len; i++)
              printf("gecko_cmd_mesh_test_get_local_model_sub sub_address: %x\r\n", sub_setting->addresses.data[i]);

            if (!sub_setting->result && (sub_setting->addresses.len != 0)) {
              // In fact, need to check the subscription address list here.
              printf("Subscription Configuration done already.\n");
            }
            else{
              //struct gecko_msg_mesh_test_add_local_key_rsp_t *addLocalKeyRsp = gecko_cmd_mesh_test_add_local_key(1, appKey, appkey_id, netkey_id);
              //printf("gecko_cmd_mesh_test_add_local_key %x\n\r", addLocalKeyRsp->result);

              struct gecko_msg_mesh_test_bind_local_model_app_rsp_t *bindLocalModelRsp = gecko_cmd_mesh_test_bind_local_model_app(0, appkey_id, 0xffff, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID);
              printf("gecko_cmd_mesh_test_bind_local_model_app %x\n\r", bindLocalModelRsp->result);

              struct gecko_msg_mesh_test_add_local_model_sub_rsp_t *addLocalModelRsp = gecko_cmd_mesh_test_add_local_model_sub(0, 0xffff, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID, 0xC001);
              printf("gecko_cmd_mesh_test_add_local_model_sub %x\n\r", addLocalModelRsp->result);
            }

            /* Set the publication */
            struct gecko_msg_mesh_generic_client_init_rsp_t *genericClientInitRsp = gecko_cmd_mesh_generic_client_init();
            printf("gecko_cmd_mesh_generic_client_init %x\n\r", genericClientInitRsp->result);

            struct gecko_msg_mesh_test_get_local_model_pub_rsp_t *pub_setting = gecko_cmd_mesh_test_get_local_model_pub(0, 0xFFFF, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_pub result: %x\r\n", pub_setting->result);
            printf("gecko_cmd_mesh_test_get_local_model_pub pub_address: 0x%04x\r\n", pub_setting->pub_address);

            if (!sub_setting->result && (pub_setting->pub_address != 0)) {
              printf("Publication Configuration done already.\n");
            }
            else{
              struct gecko_msg_mesh_test_bind_local_model_app_rsp_t *bindLocalModelRsp = gecko_cmd_mesh_test_bind_local_model_app(0, appkey_id, 0xffff, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID);
              printf("gecko_cmd_mesh_test_bind_local_model_app %x\n\r", bindLocalModelRsp->result);

              struct gecko_msg_mesh_test_set_local_model_pub_rsp_t *setLocalModelPubRsp = gecko_cmd_mesh_test_set_local_model_pub(0, appkey_id, 0xffff, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID, 0xC001, 5, 0, 0, 0);
              printf("gecko_cmd_mesh_test_set_local_model_pub %x\n\r", setLocalModelPubRsp->result);
            }

            // ------------------------------------------ //
            printf("---------------- Dump sub/pub infor here ----------------\r\n");
            sub_setting = gecko_cmd_mesh_test_get_local_model_sub(0, 0xFFFF, MESH_GENERIC_ON_OFF_SERVER_MODEL_ID);
            for(i=0; i<sub_setting->addresses.len; (i=i+2))
              printf("gecko_cmd_mesh_test_get_local_model_sub sub_address: 0x%02x%02x\r\n", sub_setting->addresses.data[i+1], sub_setting->addresses.data[i]);

            pub_setting = gecko_cmd_mesh_test_get_local_model_pub(0, 0xFFFF, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID);
            printf("gecko_cmd_mesh_test_get_local_model_pub pub_address: 0x%04x\r\n", pub_setting->pub_address);
            printf("---------------------------------------------------------\r\n");

            // uncomment it if need to control the light node from the provisioner
//            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000), TIMER_ID_SEND_PUB_MSG, 0);
  #endif

//            state = scanning;
//          }
//        }
        else {
          printf("unexpected prov conf status: state = %d\r\n", state);
        }
      }
    break;
#endif

    case gecko_evt_le_gap_adv_timeout_id:
      // adv timeout events silently discarded
      break;

    case gecko_evt_le_connection_opened_id:
      printf("evt:gecko_evt_le_connection_opened_id\r\n");
      num_connections++;
      conn_handle = evt->data.evt_le_connection_opened.connection;
      LCD_write("connected", LCD_ROW_CONNECTION);

      if(state == connecting)
      {
        printf("connection opened, proceeding provisioning over GATT\r\n");
        struct gecko_msg_mesh_prov_provision_gatt_device_rsp_t *prov_resp_adv;
        prov_resp_adv = gecko_cmd_mesh_prov_provision_gatt_device(netkey_id, conn_handle, 16, _uuid_copy);

        if (prov_resp_adv->result == 0) {
          printf("Successful call of gecko_cmd_mesh_prov_provision_gatt_device\r\n");
          state = provisioning;
        } else {
          printf("Failed call to gatt provision node. %x\r\n", prov_resp_adv->result);
        }
      }
      break;

    case gecko_evt_le_connection_parameters_id:
      printf("evt:gecko_evt_le_connection_parameters_id\r\n");
      break;

    case gecko_evt_le_connection_closed_id:
      printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          LCD_write("", LCD_ROW_CONNECTION);
        }
      }
      break;

    case gecko_evt_gatt_server_user_write_request_id:
      break;

    case gecko_evt_system_external_signal_id:
      {

      }
      break;

    case gecko_evt_mesh_prov_initialized_id:
    {
      struct gecko_msg_mesh_prov_initialized_evt_t *initialized_evt;
      initialized_evt = (struct gecko_msg_mesh_prov_initialized_evt_t *)&(evt->data);

      printf("gecko_cmd_mesh_prov_init_id\r\n");
      printf("networks: %x\r\n", initialized_evt->networks);
      printf("address: %x\r\n", initialized_evt->address);
      printf("ivi: %x\r\n", (unsigned int)initialized_evt->ivi);

      LCD_write("provisioner", LCD_ROW_STATUS);

      if (initialized_evt->networks > 0) {
        printf("network keys already exist\r\n");
        netkey_id = 0;
        appkey_id = 0;
      } else {
        printf("Creating a new netkey\r\n");

        struct gecko_msg_mesh_prov_create_network_rsp_t *new_netkey_rsp;
#ifdef USE_FIXED_KEYS
        new_netkey_rsp = gecko_cmd_mesh_prov_create_network(16, fixed_netkey);
#else
        new_netkey_rsp = gecko_cmd_mesh_prov_create_network(0, (const uint8 *)"");
#endif

        if (new_netkey_rsp->result == 0) {
          netkey_id = new_netkey_rsp->network_id;
          printf("Success, netkey id = %x\r\n", netkey_id);
        } else {
          printf("Failed to create new netkey. Error: %x", new_netkey_rsp->result);
        }

        printf("Creating a new appkey\r\n");

        struct gecko_msg_mesh_prov_create_appkey_rsp_t *new_appkey_rsp;

#ifdef USE_FIXED_KEYS
        new_appkey_rsp = gecko_cmd_mesh_prov_create_appkey(netkey_id, 16, fixed_appkey);
#else
        new_appkey_rsp = gecko_cmd_mesh_prov_create_appkey(netkey_id, 0, (const uint8 *)"");
#endif

        if (new_netkey_rsp->result == 0) {
          appkey_id = new_appkey_rsp->appkey_index;
          printf("Success, appkey_id = %x\r\n", appkey_id);
          printf("Appkey: ");
          for (uint32_t i = 0; i < new_appkey_rsp->key.len; ++i) {
            printf("%02x ", new_appkey_rsp->key.data[i]);
          }
          printf("\r\n");
        } else {
          printf("Failed to create new appkey. Error: %x", new_appkey_rsp->result);
        }
      }

      printf("List all of the nodes known by the provisioner \r\n");
      struct gecko_msg_mesh_prov_ddb_list_devices_rsp_t *listDDB_rsp;
      listDDB_rsp = gecko_cmd_mesh_prov_ddb_list_devices();
      if (listDDB_rsp->result == 0) {
        printf("Success - list nodes\r\n");
      } else {
        printf("Failure - list nodes, result 0x%04x\r\n", listDDB_rsp->result);
      }

      printf("Starting to scan for unprovisioned device beacons\r\n");

      struct gecko_msg_mesh_prov_scan_unprov_beacons_rsp_t *scan_rsp;
      scan_rsp = gecko_cmd_mesh_prov_scan_unprov_beacons();

      if (scan_rsp->result == 0) {
        printf("Success - initializing unprovisioned beacon scan\r\n");
        state = scanning;
      } else {
        printf("Failure initializing unprovisioned beacon scan. Result: %x\r\n", scan_rsp->result);
      }

      // start timer for button polling
       gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(100), TIMER_ID_SYS_BUTTON_POLL, 0);

      break;
    }

    case gecko_evt_mesh_prov_ddb_list_id:
      printf("evt: gecko_evt_mesh_prov_ddb_list_id address 0x%04x, element %d\r\n", evt->data.evt_mesh_prov_ddb_list.address, evt->data.evt_mesh_prov_ddb_list.elements);
      break;

    case gecko_evt_mesh_prov_unprov_beacon_id:
    {
      struct gecko_msg_mesh_prov_unprov_beacon_evt_t *beacon_evt = (struct gecko_msg_mesh_prov_unprov_beacon_evt_t *)&(evt->data);
      int i;

  	// this example can handle only one bearer type at a time: either PB-GATT or PB-ADV
#ifdef PROVISION_OVER_GATT
#define bearer_type 1
#else
#define bearer_type 0
#endif

      if ((state == scanning) && (ask_user_input == false) && (evt->data.evt_mesh_prov_unprov_beacon.bearer == bearer_type)) {
		  printf("gecko_evt_mesh_prov_unprov_beacon_id\r\n");

		  for(i=0;i<beacon_evt->uuid.len;i++)
		  {
			printf("%2.2x", beacon_evt->uuid.data[i]);
		  }

		  // show also the same shortened version that is used on the LCD of switch / light examples
		  printf(" (%2.2x %2.2x)", beacon_evt->uuid.data[11], beacon_evt->uuid.data[10]);
		  printf("\r\n");

		  memcpy(_uuid_copy, beacon_evt->uuid.data, 16);
#ifdef PROVISION_OVER_GATT
		  bt_address = beacon_evt->address;
		  bt_address_type = beacon_evt->address_type;
#endif
		  printf("confirm? Press Button 1 or enter 'y'\r\n");
		  // suspend reporting of unprov beacons until user has rejected or accepted this one using buttons PB0 / PB1
		  ask_user_input = true;
      }
      break;
    }

    case gecko_evt_mesh_prov_provisioning_failed_id:
    {
      struct gecko_msg_mesh_prov_provisioning_failed_evt_t *fail_evt = (struct gecko_msg_mesh_prov_provisioning_failed_evt_t*)&(evt->data);

      printf("Provisioning failed. Reason: %x\r\n", fail_evt->reason);
      state = scanning;

      break;
    }

    case gecko_evt_mesh_prov_device_provisioned_id:
    {
      struct gecko_msg_mesh_prov_device_provisioned_evt_t *prov_evt = (struct gecko_msg_mesh_prov_device_provisioned_evt_t*)&(evt->data);

      printf("Node successfully provisioned. Address: %4.4x\r\n", prov_evt->address);
      state = provisioned;

      printf("provisioning done - uuid 0x");
      for (uint8_t i = 0; i < prov_evt->uuid.len; i++) printf("%02X", prov_evt->uuid.data[i]);
      printf("\r\n");

      provisionee_address = prov_evt->address;

      /* kick of next phase which is reading DCD from the newly provisioned node */
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(500), TIMER_ID_GET_DCD, 1);

      break;
    }

    case gecko_evt_mesh_generic_server_client_request_id:
    {
      struct gecko_msg_mesh_generic_server_client_request_evt_t *genericRequestEvt = (struct gecko_msg_mesh_generic_server_client_request_evt_t*)&(evt->data);
      printf("evt gecko_evt_mesh_generic_server_client_request_id: client_addr 0x%04x server_addr 0x%04x\r\n", genericRequestEvt->client_address, genericRequestEvt->server_address);
      break;
    }

    case gecko_evt_mesh_generic_server_state_changed_id:
    {
      // uncomment following line to get debug prints for each server state changed event
      server_state_changed(&(evt->data.evt_mesh_generic_server_state_changed));

      break;
    }

    default:
      printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", (unsigned int)evt_id, (unsigned int)((evt_id >> 16) & 0xFF), (unsigned int)((evt_id >> 24) & 0xFF));
      break;
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
