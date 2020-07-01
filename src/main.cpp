/*
 * Serial port over ESP-NOW (wireless serial link).
 * Connections:
 *   PC <= serial wires or usb to serial => ESP8266(MAC 36:33:33:33:33:34) <=ESP_NOW=> ESP8266(MAC 36:33:33:33:33:33) <= serial wires or usb to serial => PC
 */
#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
  #include "user_interface.h"
}

//====================================================================================
//===============================--- DEFINITIONS --===================================
//====================================================================================

//#define DEVICE_1
/* Do not realy send data to uart but debug messages. - only for debbuging */
//#define DEBUG
/* Try to fill ESP-NOW buffer */
#define COLLECT_MORE_CHARACTERS
/* Send only if fifo space is available */
#define SEND_IF_FILO_EMPTY
/* Maximum packet size in bytes (no more than 128 because HW serial write fifo is limited to 128 bytes and values above 128 will stall loop() ) */
#define ESP_NOW_PACKET_SIZE (100)
/* RX/TX queue size in packets (MUST BE POWER OF 2) */
#define ESP_NOW_QUEUE_SIZE (128)
/* RX/TX queue mask */
#define ESP_NOW_QUEUE_MASK (ESP_NOW_QUEUE_SIZE-1)

/* Serial port speed bps 2Mb works fine on WebMos D1 */
#define serialSpeed        (2000000ul)
//#define serialSpeed        (115200ul)
/* Character timeout for end of frame detection */
#define characterBreakTime (5ul + 2*(10000000ul/serialSpeed))
/* Wifi channel to use */
#define wifiChannel  12
/* MAC adresses for first and second device */
#ifdef DEVICE_1
uint8_t myAddress[]     = {0x36, 0x33, 0x33, 0x33, 0x33, 0x34};
uint8_t remoteAddress[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
#else
uint8_t myAddress[]     = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
uint8_t remoteAddress[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x34};
#endif
//====================================================================================

//====================================================================================
//===============================--- PACKET QUEUE --==================================
//====================================================================================

/*!
 * \brief Single ESP NOW packet structure.
 */
 typedef struct {
  uint8_t buf[ESP_NOW_PACKET_SIZE];           /*!< Packet data            */
  int len;                                    /*!< Packe t size in bytes. */
} esp_now_packet_t;

/*!
 * \brief ESP NOW packet queue.
 */
typedef struct {
  esp_now_packet_t data[ESP_NOW_QUEUE_SIZE];  /*!< Queue data             */
  int rd;                                     /*!< Read pointer           */
  int wr;                                     /*!< Write pointer          */
} esp_now_queue_t;

void esp_now_queue_init(esp_now_queue_t *q) { q->rd = q->wr = 0; }
esp_now_packet_t *esp_now_queue_get_rx(esp_now_queue_t *q) {return &q->data[q->rd];}
esp_now_packet_t *esp_now_queue_get_tx(esp_now_queue_t *q) {return &q->data[q->wr];}
void esp_now_queue_pull(esp_now_queue_t *q) {int rd = q->rd; rd++; rd&=ESP_NOW_QUEUE_MASK; q->rd=rd;} 
void esp_now_queue_tx_commit(esp_now_queue_t *q) {int wr = q->wr; wr++; wr&=ESP_NOW_QUEUE_MASK; q->wr=wr;} 
#define esp_now_queue_not_empty(q) ((q)->rd != (q)->wr) 
void esp_now_queue_push(esp_now_queue_t *q,uint8_t *data, uint8_t len ) 
{
  esp_now_packet_t *p = esp_now_queue_get_tx(q);
  memcpy(p->buf, data, len);
  p->len = len;
  esp_now_queue_tx_commit(q);
}
//====================================================================================

//====================================================================================
//============================--- GLOBAL VARIABLES --=================================
//====================================================================================

const char* dummy             = "ESPNOW";   /*!< Dummy Wifi SSID                               */
volatile int send_in_progress = 0;          /*!< Send in progress flag 1 - sending in progress */
uint64_t previousMicros       = 0;          /*!< Previous micros for timeout computations      */
int blink,lastBytes           = 0;          /*!< Last avail bytes for timeout computations     */
esp_now_queue_t tx_queue;                   /*!< Transmit queue                                */
esp_now_queue_t rx_queue;                   /*!< Receive queue                                 */

//====================================================================================
//=================================--- MAIN CODE --===================================
//====================================================================================

/*!
 * \brief Setup output power and MAC address. 
 */
void initVariant() {
  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(20.5);
  wifi_set_macaddr(STATION_IF, &myAddress[0]);
}
//====================================================================================

/*!
 * \bief Callback when data is sent over ESP_NOW.
 * Get next packet from TX packet queue or clear send_in_progress flag if no more packets to send.
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {  
  esp_now_packet_t *p = nullptr;
  if (sendStatus == 0) {
    /* Send success - get next packet from TX queue */
    esp_now_queue_pull(&tx_queue);
    if (esp_now_queue_not_empty(&tx_queue)) {
      p = esp_now_queue_get_rx(&tx_queue);
    }
  } else {
    /* Send error  - try to send again and again ... !! */
    p = esp_now_queue_get_rx(&tx_queue);    
  }
  if (p) {
    esp_now_send(remoteAddress, p->buf, p->len);
  } else {
    /* Queue is empty - clear send_in_progress flag */
    send_in_progress = 0;
  }
}
//====================================================================================


/*!
 * \brief Callback when data is received from ESP_NOW.
 * Push received data to RX queue and blink the LED.
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  if (len <= ESP_NOW_PACKET_SIZE) {
    esp_now_queue_push(&rx_queue, incomingData, len); 
  }
  /* Blink led */
  digitalWrite(D4, (blink&1));blink++;
}
//====================================================================================

/*!
 * \bref Configure IO pins, serial speed and ESP_NOW parameters.
 */
void setup() {
  pinMode(D4,OUTPUT);
  Serial.begin(serialSpeed);
  esp_now_queue_init(&tx_queue);
  esp_now_queue_init(&rx_queue);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  //         SSID   pass   channel       BSSID  connect
  //WiFi.begin(dummy, nullptr, wifiChannel, nullptr, false);
  wifi_set_channel(wifiChannel);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(remoteAddress, ESP_NOW_ROLE_COMBO, wifiChannel, NULL, 0);
  esp_now_register_recv_cb(OnDataRecv);
#ifdef DEBUG
  Serial.printf("Character break time = %d\n",characterBreakTime);
#endif
}
//====================================================================================

/*!
 * \brief The main loop.
 * 1) Send packets from RX queue over uart port.
 * 2) Collect data from uart and push it to TX queue.
 */
void loop() {
  esp_now_packet_t *p;
  uint64_t currentMicros;
  int avail;

  /* Send data from RX queue */
  if (esp_now_queue_not_empty(&rx_queue)) {    
    p = esp_now_queue_get_rx(&rx_queue);
#ifdef SEND_IF_FILO_EMPTY    
    if (Serial.availableForWrite() >= p->len) {
#else
    {
#endif
#ifdef DEBUG
      Serial.printf("Write to serial %d bytes, free fifo space = %d \n", p->len, Serial.availableForWrite());
#else
      Serial.write(p->buf, p->len);
#endif      
      esp_now_queue_pull(&rx_queue);      
    }
  }
  
  /* Collect data and push it to TX queue */
  p             = nullptr;
  currentMicros = micros64();  
  avail         = Serial.available();
  if (avail) {
#ifdef COLLECT_MORE_CHARACTERS
    if (avail < ESP_NOW_PACKET_SIZE) {
      if (avail > lastBytes) {
        previousMicros = currentMicros;
        lastBytes      = avail;
      } else if ((currentMicros - previousMicros) > characterBreakTime) {
          /* Transmision break timeout  */
          p = esp_now_queue_get_tx(&tx_queue);
      }
    } else {
      /* Buffer is full */
      p = esp_now_queue_get_tx(&tx_queue);
    }
#else
    p = esp_now_queue_get_tx(&tx_queue);
#endif 
    /* Read data from serial */   
    if (p) {
      p->len = Serial.read((char *)p->buf, (size_t)ESP_NOW_PACKET_SIZE);
      lastBytes = 0;
    }
  } else if (send_in_progress == 0) {
    if (currentMicros - previousMicros >= 500000) {
      previousMicros = currentMicros;
      /* Send dummy packet on inactivity */
      p = esp_now_queue_get_tx(&tx_queue);
      p->len = (ESP_NOW_PACKET_SIZE+1);
    }
  }
  if (p) {
#ifdef DEBUG
    Serial.printf("Got %d bytes from serial \n", p->len);
#endif
    esp_now_queue_tx_commit(&tx_queue);
    if (send_in_progress == 0) {
      send_in_progress = 1;
      OnDataSent(nullptr, 1); 
    }
    previousMicros = currentMicros;
  }
}
//====================================================================================
