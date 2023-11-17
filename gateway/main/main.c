#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "board.h"
#include "ble_mesh_example_init.h"

#include "esp_bt.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

//mqtt
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

#define TAG "ESP_MESH_MQTT"

#define CID_ESP 0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001
#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0X06, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x07, CID_ESP)

extern struct _led_state led_state[3];

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

esp_mqtt_client_handle_t mqtt_client;

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(vendor_pub, 8, ROLE_NODE); //The publication buffer size must be big enough to fit the longest message to be published.
static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 1),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 1),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER, 
    vnd_op, &vendor_pub, NULL),
};

//Allocating memory for publishing messages.
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_1, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_1 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_2, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_2 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
};

static esp_ble_mesh_model_t extend_model_0[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_1, &onoff_server_1),
};

static esp_ble_mesh_model_t extend_model_1[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_2, &onoff_server_2),
}; 

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_0, vnd_models),
    ESP_BLE_MESH_ELEMENT(0, extend_model_1, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};


/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_actions = ESP_BLE_MESH_PUSH,
    .input_size = 4,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t net_key[16], uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
    ESP_LOGI(TAG, "net_key/ %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
    net_key[0], net_key[1], net_key[2], net_key[3],net_key[4], net_key[5], net_key[6], net_key[7],
    net_key[8], net_key[9], net_key[10], net_key[11],net_key[12], net_key[13], net_key[14], net_key[15]);
    board_led_operation(LED_B, LED_OFF);
    ESP_LOGW(TAG, "LEDs_OFF provisioning completed");
}

static void example_change_led_state(esp_ble_mesh_model_t *model,
                                     esp_ble_mesh_msg_ctx_t *ctx, uint8_t onoff)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    struct _led_state *led = NULL;
    uint8_t i;

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                led = &led_state[i];
                board_led_operation(led->pin, onoff);
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            led = &led_state[model->element->element_addr - primary_addr];
            board_led_operation(led->pin, onoff);
        }
    } else if (ctx->recv_dst == ESP_BLE_MESH_ADDR_ALL_NODES) {
        led = &led_state[model->element->element_addr - primary_addr];
        board_led_operation(led->pin, onoff);
    }
}

static void example_handle_gen_onoff_msg(esp_ble_mesh_model_t *model,
                                         esp_ble_mesh_msg_ctx_t *ctx,
                                         esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    esp_ble_mesh_gen_onoff_srv_t *srv = model->user_data;

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
            ESP_LOGI(TAG, "srv->state.onoff %d", srv->state.onoff);
            uint8_t actual_state;
            if (srv->state.onoff == 1) {
                actual_state = 0x01;
                ESP_LOGI(TAG, "status %d", actual_state);
                esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, 0x0001, &actual_state);
                ESP_LOGI(TAG, "Status send to src");
            } else {
                actual_state = 0x00;
                ESP_LOGI(TAG, "status %d", actual_state);
                esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, 0x0001, &actual_state);
                ESP_LOGI(TAG, "Status send to src");
            }
            esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            0x0001, &actual_state, ROLE_NODE);
            ESP_LOGI(TAG, "Status published (as it was a GET)");
            ESP_LOGI(TAG, "sizeof(srv->state.onoff) %d", sizeof(srv->state.onoff));
        break;
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        if (set->op_en == false) {
            srv->state.onoff = set->onoff;
        } else {
            /* TODO: Delay and state transition */
            srv->state.onoff = set->onoff;
        }
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            ESP_LOGI(TAG, "It is an Ack Set Message****");
            esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
            ESP_LOGI(TAG, "Status msg sent Back to src as SET is ACK");
        }   
        ESP_LOGI(TAG, "ONOFF status published (to suscribers)");
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(srv->state.onoff), &srv->state.onoff, ROLE_NODE);
            ESP_LOGI(TAG, "role node {0-1-2} %d", ROLE_NODE);
        example_change_led_state(model, ctx, srv->state.onoff);
        break;
    default:
        break;
    }
}

void send_to_dimmer(uint8_t payload)
{
    printf("payload received by send_to_dimmer() 0x%02x", payload);
    esp_ble_mesh_msg_ctx_t ctx = {0};
    esp_err_t err = ESP_OK;

    ctx.net_idx = 0x0000;
    ctx.app_idx = 0x0000;
    ctx.addr = 0x3401;   /* dimmer */
    ctx.send_ttl = ESP_BLE_MESH_TTL_DEFAULT;
    ctx.send_rel = false;

    err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],&ctx,
     ESP_BLE_MESH_VND_MODEL_OP_SEND, sizeof(payload), &payload);
    if (err) {
        ESP_LOGI(TAG, "Send message to dimmer failed KO");
        return;
    }
    else {ESP_LOGI(TAG, "Message sent to dimmer OK");}
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{   

    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "Initialize BLE Mesh provisioning capabilities and internal data information completion event (esp_ble_mesh_init)");
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "Enable node provisioning functionality completion event");
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "Establish a BLE Mesh link event");
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "Close a BLE Mesh link event");
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "Provisioning done event");
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr, param->node_prov_complete.net_key,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "Provisioning reset event");
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "Set the unprovisioned device name completion event");
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "\n\n****CallBack Gen OnOff Server****");
    esp_ble_mesh_gen_onoff_srv_t *srv;
    ESP_LOGI(TAG, "\nInfos param->model");
    ESP_LOGI(TAG, "model_id 0x%04x", param->model->model_id);
    ESP_LOGI(TAG, "opcode 0x%04" PRIx32, param->model->op->opcode);
    ESP_LOGI(TAG, "op min length %d", param->model->op->min_len);
    ESP_LOGI(TAG, "publish_addr 0x%04x", param->model->pub->publish_addr);
    ESP_LOGI(TAG, "dev_role %d", param->model->pub->dev_role);
    ESP_LOGI(TAG, "\nInfos param->ctx");
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x, ttl value 0x%02x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst, param->ctx.recv_ttl);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT: //set_auto_rsp is set to ESP_BLE_MESH_SERVER_AUTO_RSP
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            example_change_led_state(param->model, &param->ctx, param->value.state_change.onoff_set.onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT: //get_auto_rsp is set to ESP_BLE_MESH_SERVER_RSP_BY_APP
        ESP_LOGI(TAG, "\n\nGET Message received****");
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            srv = param->model->user_data; // srv = état du server
            ESP_LOGI(TAG, "Server State onoff 0x%02x", srv->state.onoff);
            example_handle_gen_onoff_msg(param->model, &param->ctx, NULL);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT: //set_auto_rsp is set to ESP_BLE_MESH_SERVER_RSP_BY_APP
        ESP_LOGI(TAG, "SET Message received****");
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "SET Message content****");
            ESP_LOGI(TAG, "onoff 0x%02x, tid 0x%02x, optional_param %d", param->value.set.onoff.onoff, param->value.set.onoff.tid, param->value.set.onoff.op_en);
            if (param->value.set.onoff.op_en) {
                ESP_LOGI(TAG, "trans_time 0x%02x, delay 0x%02x",
                    param->value.set.onoff.trans_time, param->value.set.onoff.delay);
            }
            example_handle_gen_onoff_msg(param->model, &param->ctx, &param->value.set.onoff);
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        ESP_LOGI(TAG, "****CallBack Config Server****");
        ESP_LOGI(TAG, "Params de l'event");
        ESP_LOGI(TAG, "model_id 0x%04x", param->model->model_id);
        ESP_LOGI(TAG, "opcode 0x%04" PRIx32, param->model->op->opcode);
        ESP_LOGI(TAG, "op min length %d", param->model->op->min_len);
        ESP_LOGI(TAG, "source 0x%04x", param->ctx.addr);
        ESP_LOGI(TAG, "destination 0x%04x", param->ctx.recv_dst);
        ESP_LOGI(TAG, "ttl value 0x%02x", param->ctx.recv_ttl);
        ESP_LOGI(TAG, "received op 0x%04" PRIx32, param->ctx.recv_op);
        switch (param->ctx.recv_op) { // uint32_t recv_op - Opcode of a received message. Not used for sending message.
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD => Config AppKey Add");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x, app_key %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx,
                param->value.state_change.appkey_add.app_key[0],
                param->value.state_change.appkey_add.app_key[1],
                param->value.state_change.appkey_add.app_key[2],
                param->value.state_change.appkey_add.app_key[3],
                param->value.state_change.appkey_add.app_key[4],
                param->value.state_change.appkey_add.app_key[5],
                param->value.state_change.appkey_add.app_key[6],
                param->value.state_change.appkey_add.app_key[7],
                param->value.state_change.appkey_add.app_key[8],
                param->value.state_change.appkey_add.app_key[9],
                param->value.state_change.appkey_add.app_key[10],
                param->value.state_change.appkey_add.app_key[11],
                param->value.state_change.appkey_add.app_key[12],
                param->value.state_change.appkey_add.app_key[13],
                param->value.state_change.appkey_add.app_key[14],
                param->value.state_change.appkey_add.app_key[15]
                );
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND => Config Model App Bind");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD => Config Model Subscription Add");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    ESP_LOGI(TAG, "\n\n****CallBack Vendor Server****");
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        ESP_LOGI(TAG, "\n\nESP_BLE_MESH_MODEL_OPERATION_EVT = msg received");
        ESP_LOGI(TAG, "\nInfos param->model_operation");
        ESP_LOGI(TAG, "Length of the received message 0x%04x", param->model_operation.length);
        ESP_LOGI(TAG, "Value of the received message %d", *param->model_operation.msg);
        ESP_LOGI(TAG, "model_id 0x%04x", param->model_operation.model->model_id);
        ESP_LOGI(TAG, "Opcode of the received message 0x%04" PRIx32, param->model_operation.opcode);
        ESP_LOGI(TAG, "op min length %d", param->model_operation.model->op->min_len);
        ESP_LOGI(TAG, "publish_addr 0x%04x", param->model_operation.model->pub->publish_addr);
        ESP_LOGI(TAG, "dev_role %d", param->model_operation.model->pub->dev_role);

        ESP_LOGI(TAG, "\nInfos param->ctx");
        ESP_LOGI(TAG, "event 0x%02x, Opcode of a received message 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x, ttl value 0x%02x",
            event, param->model_operation.ctx->recv_op, param->model_operation.ctx->addr, param->model_operation.ctx->recv_dst, param->model_operation.ctx->recv_ttl);
            if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
                // uint16_t tid = *(uint16_t *)param->model_operation.msg;
                ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", payload 0x%d", param->model_operation.opcode, *param->model_operation.msg);
                esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                        param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                        sizeof(*param->model_operation.msg), param->model_operation.msg);

                if (err) {
                    ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
                } else {ESP_LOGI(TAG, "Status Message sent back to src");}
                esp_err_t err2 = esp_ble_mesh_model_publish(&vnd_models[0],
                        ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                        sizeof(*param->model_operation.msg), param->model_operation.msg, ROLE_NODE);
                        
                if (err2) {
                    ESP_LOGE(TAG, "Failed to publish message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
                } else {ESP_LOGI(TAG, "Status Message published");}

                send_to_dimmer(*param->model_operation.msg); // forward received message to dimmer
            }

            if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_STATUS) {
                ESP_LOGI(TAG, "Status Message received");
                ESP_LOGI(TAG, "Recv 0x%06" PRIx32, param->model_operation.opcode);
                ESP_LOG_BUFFER_HEX("status msg", param->model_operation.msg, 6);
                char str[12]; char str2[12];
                sprintf(str, "%d", (int) param->model_operation.msg[4]);
                sprintf(str2, "%d", (int) param->model_operation.msg[5]);
                strcat(str,"-"); strcat(str,str2);
                esp_mqtt_client_publish(mqtt_client, "Dimmer_status", str, 0, 0, 0);
            }
        break;

    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
        if (param->model_publish_comp.err_code) {
            ESP_LOGE(TAG, "Failed to send message %d", param->model_publish_comp.err_code);
            break;
        }
        ESP_LOGI(TAG, "\n\nESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT = msg published");
        ESP_LOGI(TAG, "\nInfos param->model_publish_comp");
        ESP_LOGI(TAG, "model_id 0x%04x", param->model_publish_comp.model->model_id);
        ESP_LOGI(TAG, "op min length %d", param->model_publish_comp.model->op->min_len);
        ESP_LOGI(TAG, "publish_addr 0x%04x", param->model_publish_comp.model->pub->publish_addr);
        ESP_LOGI(TAG, "dev_role %d", param->model_publish_comp.model->pub->dev_role);
        break;

    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        ESP_LOGI(TAG, "\n\ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT = receive publish messages event");
        ESP_LOGI(TAG, "\nInfos param->client_recv_publish_msg");
        ESP_LOGI(TAG, "Length of the received message 0x%04x", param->client_recv_publish_msg.length);
        ESP_LOGI(TAG, "Value of the received message %d", *param->client_recv_publish_msg.msg);
        ESP_LOGI(TAG, "model_id 0x%04x", param->client_recv_publish_msg.model->model_id);
        ESP_LOGI(TAG, "opcode 0x%04" PRIx32, param->client_recv_publish_msg.opcode);
        ESP_LOGI(TAG, "op min length %d", param->client_recv_publish_msg.model->op->min_len);
        ESP_LOGI(TAG, "publish_addr 0x%04x", param->client_recv_publish_msg.model->pub->publish_addr);
        ESP_LOGI(TAG, "dev_role %d", param->client_recv_publish_msg.model->pub->dev_role);
        ESP_LOGI(TAG, "\nInfos param->ctx");
        ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x, ttl value 0x%02x",
            event, param->client_recv_publish_msg.ctx->recv_op, param->client_recv_publish_msg.ctx->addr, param->client_recv_publish_msg.ctx->recv_dst, param->client_recv_publish_msg.ctx->recv_ttl);
        break;

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI(TAG, "\n\ESP_BLE_MESH_MODEL_SEND_COMP_EVT = send messages completion event");
        ESP_LOGI(TAG, "\nInfos param->model_send_comp");
        ESP_LOGI(TAG, "model_id 0x%04x", param->model_send_comp.model->model_id);
        ESP_LOGI(TAG, "Opcode of the message 0x%04" PRIx32, param->model_send_comp.opcode);
        ESP_LOGI(TAG, "op min length %d", param->model_send_comp.model->op->min_len);
        ESP_LOGI(TAG, "publish_addr 0x%04x", param->model_send_comp.model->pub->publish_addr);
        ESP_LOGI(TAG, "dev_role %d", param->model_send_comp.model->pub->dev_role);
        ESP_LOGI(TAG, "\nInfos param->ctx");
        ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x, ttl value 0x%02x",
            event, param->model_send_comp.ctx->recv_op, param->model_send_comp.ctx->addr, param->model_send_comp.ctx->recv_dst, param->model_send_comp.ctx->recv_ttl);
        break;
    default:
        break;
    }
}

int str2int(const char* str, int len)
{
    int i;
    int ret = 0;
    for(i = 0; i < len; ++i)
    {
        ret = ret * 10 + (str[i] - '0');
    }
    return ret;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    mqtt_client = event->client;
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, "BTmesh_dimmer", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            msg_id = esp_mqtt_client_subscribe(client, "Dimmer", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:            
            if (strncmp("BTmesh_dimmer", event->topic, 13) == 0) {
                // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
                printf("DATA=%.*s\r\n", event->data_len, event->data);
                ESP_LOGI(TAG, "BTmesh_dimmer OK");
                int data_as_int = str2int(event->data, event->data_len);
                printf("typecast of data_as_int for send_to_dimmer 0x%02x\n", (uint8_t) data_as_int);
                send_to_dimmer((uint8_t) data_as_int);
            }
            else if (strncmp("Dimmer", event->topic, 6) == 0) {
                esp_mqtt_client_publish(client, "Dimmer_back", "ok", 0, 0, 0);
            }

            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
           
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static esp_err_t ble_mesh_init(void)
{   
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb); 
    //* `esp_ble_mesh_register_prov_callback(esp_ble_mesh_prov_cb)`: registers the provisioning callback function in the BLE Mesh stack.
    //* This callback function gets executed during the BLE Mesh network configuration process. It allows the BLE Mesh stack to generate
    //* events and notify the application layer about important network configuration processes.

    // => cb with serv_cb_event (ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT), param => model (id, op, user_data), ctx(addr=source, app, net, dst=this), value(state_change) 
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb); 
    //* `esp_ble_mesh_register_custom_model_callback(esp_ble_mesh_model_cb)`: registers the model operation callback function.
    //* This callback function is used when the target peer operates the model state of the source peer after BLE Mesh has completed network
    //* configuration. 
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);
   
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }
    ESP_LOGW(TAG, "BLE Mesh stack initialized (prov & comp settings)");

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    ESP_LOGW(TAG, "BLE Mesh Node initialized: visible to Provisioners");

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }
    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    board_led_operation(LED_B, LED_ON);
    ESP_LOGW(TAG, "LEDs_ON waiting for provisioning to complete");

    return err;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    //TODO: cleanup code
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "PMixel",
            .password = "esp32ktu",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    board_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }
    ESP_LOGI(TAG, "Bluetooth initialised OK\n");
    ble_mesh_get_dev_uuid(dev_uuid);
    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
    ESP_LOGI(TAG, "Mesh initialised OK\n");
    mqtt_app_start();
}