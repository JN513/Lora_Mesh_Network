#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <vector>
#include <map>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lora.h"

#define LORA_BAND 915e6
#define NETWORK_BROADCAST_ADDRESS 0xFF
#define MAX_LORA_PAYLOAD_SIZE 255
#define LORA_RECEIVE_TIMEOUT 1000
#define NETWORK_SYNC_WORD 0x12
#define QTD_MESSAGE_RESYNC 10

typedef struct {
    u_int8_t id;
    u_int8_t type;
    u_int8_t distance;
} Node;

std::map <u_int8_t, Node> nodes;
std::vector <u_int8_t> node_tree[256];

char *NodeFunction[] = {"Server", "Ponte", "Solo"};
char *TAG = "Lora_Mesh_ESP32";

u_int8_t id = 3;
u_int8_t mytype = 0;
u_int8_t distance = 0;
u_int8_t messages = 0;
bool sending = false;


/*
Tipos de mensagens:
0 - Mensagem de broadcast
1 - Mensagem de broadcast com a lista de nós
2 - Mensagem para internet
3 - Mensagem para outro nó

message type = 0

Envia, uma menssagem de sinclonização para todos os nós proximos

formato da mensagem:

Endereço de broadcast, tipo de mensagem, id do nó, tipo do no, distancia, se existirm caso não e null

message type = 1

Envia, uma menssagem de sinclonização para todos os nós proximos, com todos os filhos

formato da mensagem:

Endereço de broadcast, tipo de mensagem, id do nó, tipo do no, distancia, quantidade de filhos, lista de filhos, se existirm caso não e null

message type = 2

Envia, para o nó com menor distancia de um server, destino server, ou 0x00

formato da mensagem:

destino, type, id do nó, payload

message tipe = 3

Calcula a distancia entre o nó e o destino, e alista dos mesmos

formato da mensagem:

destino, type, id do nó, nó final, payload

*/

void send_message(u_int8_t dest, u_int8_t type, u_int8_t node_id, u_int8_t *payload, u_int8_t payload_size) {
    u_int8_t message[MAX_LORA_PAYLOAD_SIZE];
    u_int8_t message_size = 0;

    message[message_size++] = dest;
    message[message_size++] = type;
    message[message_size++] = node_id;

    if (payload_size > 0) {
        if(payload_size > MAX_LORA_PAYLOAD_SIZE - message_size) {
            ESP_LOGE(TAG, "Payload size is bigger than the maximum size");
            payload_size = MAX_LORA_PAYLOAD_SIZE - message_size - 1;
        }

        memcpy(message + message_size, payload, payload_size);
        message_size += payload_size;
    }

    lora_send_packet(message, message_size);

    ESP_LOGI(TAG, "Mensagem de %d bytes, do tipo %d enviada para %d", payload_size, type, dest);

    sending = false;
}

void send_message_to_broadcast(){
    sending = true;

    u_int8_t message[3];

    message[0] = mytype;
    message[1] = distance;

    send_message(NETWORK_BROADCAST_ADDRESS, 0, id, message, 2);
}

void send_children_list_to_broadcast(){
    sending = true;

    u_int8_t children = (u_int8_t)nodes.size()+2;

    u_int8_t message[children+1];

    message[0] = mytype;
    message[1] = distance;
    message[2] = children;

    u_int8_t i = 3;

    for(auto it = nodes.begin(); it != nodes.end(); it++){
        message[i++] = it->first;

        if(i == 249){
            break;
        }
    }

    send_message(NETWORK_BROADCAST_ADDRESS, 1, id, message, children);
}

void send_message_to_server(u_int8_t node_id, u_int8_t *payload, u_int8_t payload_size){
    sending = true;

    int min_distance = 0xFFFF;
    u_int8_t min_node = 0;

    for(std::map<u_int8_t, Node>::iterator it = nodes.begin(); it != nodes.end(); it++) {
        if(it->second.distance < min_distance && (it->second.type == 0 or it->second.type == 1)) {
            min_distance = it->second.distance;
            min_node = it->first;
        }
    }

    send_message(min_node, 2, node_id, payload, payload_size);
}

void send_message_to_node(u_int8_t node_id, u_int8_t *payload, u_int8_t payload_size) {
    sending = true;

    u_int8_t *message = (u_int8_t *)malloc(payload_size+1);
    
    message[0] = node_id;

    memcpy(message + 1, payload, payload_size);

    u_int8_t first_node = 0; // vai uma logica ae

    payload_size++;

    send_message(first_node, 3, id, message, (u_int8_t)payload_size);
}

void OnReceive(){
    messages++;

    u_int8_t message[MAX_LORA_PAYLOAD_SIZE];
    u_int8_t x = 0;

    while(lora_received()){
        x = lora_receive_packet(message, MAX_LORA_PAYLOAD_SIZE);
        message[x] = 0;
        lora_receive();
    }

    printf("Recebido: %s\n", message);

    u_int8_t to = message[0];
    u_int8_t type = message[1];
    u_int8_t from = message[2];

    if(to == NETWORK_BROADCAST_ADDRESS) {
        ESP_LOGI(TAG, "Mensagem de sinconizacao recebida");

        if(nodes.find(from) == nodes.end()) {
            u_int8_t node_type = message[3];
            u_int8_t node_distance = message[4];

            nodes[from] = {from, node_type, node_distance};

            node_tree[id].push_back(from);

            if(mytype != 0 or distance > node_distance) {
                if(node_type == 0){
                    distance = 1;
                    mytype = 1;
                } else {
                    distance = node_distance + 1;
                    mytype = 2;
                }

                printf("Meu novo type: %d, minha nova distancia: %d", mytype, distance);
            }

            if(node_distance > distance){
                nodes[from].distance = distance+1;
            }

            if(type == 0) {
                vTaskDelay(100 / portTICK_PERIOD_MS);

                send_message_to_broadcast();
            }
        } else {
            for(std::map<u_int8_t, Node>::iterator it = nodes.begin(); it != nodes.end(); it++) {
                printf("Node: %d, distancia: %d, tipo: %d\n", it->second.id, it->second.distance, it->second.type);
            }
        }

        if(type == 1) {
            u_int8_t children = message[5];
            for(u_int8_t i = 0; i < children; i++) {
                u_int8_t child = message[i+5];

                node_tree[from].push_back(child);
            }

            vTaskDelay(500 / portTICK_PERIOD_MS);

            send_children_list_to_broadcast();
        } 

    } else if(type == 2) {
        ESP_LOGI(TAG, "Mensagem para internet recebida, ID do nó: %d", from);

        if(from == id) {
            ESP_LOGI(TAG, "Mensagem do nó %d para ele mesmo", from);

            return;
        }
/*
        if(nodes.find(from) == nodes.end()) {
            ESP_LOGI(TAG, "Nó não participante da rede, envie mensagem no broadcast, para entrar na rede");

            return;
        }
*/
        if(to == id or (mytype == 0 and to == 0)){
            if(mytype == 0){
                /*
                    Envia via MQTT de alguma forma ainda, não implementado
                */

                ESP_LOGI(TAG, "Mensagem para internet enviada");
            } else {
                ESP_LOGI(TAG, "Mensagem para internet recebida.\nEncaminhando Mensagem...");
                vTaskDelay(10 / portTICK_PERIOD_MS);
                send_message_to_server(from, message + 3, x - 3);
            }
        } else {
            ESP_LOGI(TAG, "Essa Menssagem não e para mim");
        }
    } else if(type == 3) {
        ESP_LOGI(TAG, "Mensagem para outro nó recebida");

        if(from == id) {
            ESP_LOGI(TAG, "Mensagem do nó %d para ele mesmo", from);

            return;
        }


    } else {
        ESP_LOGE(TAG, "Mensagem de tipo %d recebida", type);
    }
}

void task_rx(void *p){
    for(;;){
        if(messages == QTD_MESSAGE_RESYNC){
            ESP_LOGI(TAG, "Resync");
            messages = 0;
            send_children_list_to_broadcast();
        }

        if(sending == false){
            lora_receive();

            if(lora_received()){
                OnReceive();
            }
        } else {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1);
    }
}

void task_tx(void *p){
    for(;;){
        char message[MAX_LORA_PAYLOAD_SIZE-3];

        float temp = 25.0;
        float hum = 50.0;

        sprintf(message, "{'temp':%f,'humi'%f}", temp, hum);    

        //send_message(0, 2, id, (u_int8_t *)message, (u_int8_t)strlen(message));

        ESP_LOGI(TAG, "Enviando mensagem para internet");

        vTaskDelay(12000 / portTICK_PERIOD_MS);
    }
}


extern "C" void app_main(void)
{
    lora_init();
    lora_set_frequency(LORA_BAND);
    lora_enable_crc();
    lora_set_sync_word(NETWORK_SYNC_WORD);

    printf("Lora Initialized: %d\n", lora_initialized());

    send_message_to_broadcast();

    xTaskCreate(&task_rx, "task_rx", 8192, NULL, 5, NULL);
    xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
}
