#include <Rak3172_Canopus.h>

long startTime;
bool rx_done = false;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;

uint8_t MAC[4] = {0x12, 0x34, 0x56, 0x78}; //Giả sử địa chỉ mac address
uint8_t data[2] = {0xAB, 0xCD}; //data gom 2 fuction - Giả sử giá trị 

/*
Ham tinh gia tri CRC
Hàm tính giá trị CRC giúp kiểm tra tính toàn vẹn của dữ liệu.
*/
uint8_t calculateCRC(uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}


// Hàm hexDump để hiển thị nội dung buffer
void hexDump(uint8_t *buf, uint16_t len) {
  for (uint16_t i = 0; i < len; i += 16) { //Duyet qua buffer theo tung nhom 16bytes
    char s[len]; //Mang s: Luu tru ky tu ASCII
    uint8_t iy = 0;
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        if (c > 31 && c < 128)
          s[iy++] = c;
      }
    }
    String msg = String(s);
    Serial.println(msg);
  }
  Serial.println("Buffer!");
}

// Callback khi nhận dữ liệu
void recv_cb(rui_lora_p2p_recv_t data) {
  //Chuyển Rx true để sẵn sàng nhận tín hiệu response từ việc callback
  rx_done = true;
  if (data.BufferSize == 0) {
    Serial.println("Empty buffer.");
    return;
  }
  char buff[92];
  sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d",
          data.BufferSize, data.Rssi, data.Snr);
  Serial.println(buff);
  hexDump(data.Buffer, data.BufferSize);
  digitalWrite(LED_RECV, !digitalRead(LED_RECV));
}

// Callback khi gửi dữ liệu thành công
void send_cb(void) {
  Serial.printf("P2P set Rx mode %s\r\n",
                api.lora.precv(65534) ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  Serial.println("RAK3172_Canopus lora P2P Example");
  Serial.println("------------------------------------------------------");
  init_io();
  startTime = millis();

  if (api.lora.nwm.get() != 0) {
    Serial.printf("Set Node device work mode %s\r\n",
                  api.lora.nwm.set() ? "Success" : "Fail");
    api.system.reboot();
  }

  Serial.println("P2P Start");
  Serial.printf("Hardware ID: %s\r\n", api.system.chipId.get().c_str());
  Serial.printf("Model ID: %s\r\n", api.system.modelId.get().c_str());
  Serial.printf("RUI API Version: %s\r\n",
                api.system.apiVersion.get().c_str());
  Serial.printf("Firmware Version: %s\r\n",
                api.system.firmwareVersion.get().c_str());
  Serial.printf("AT Command Version: %s\r\n",
                api.system.cliVersion.get().c_str());
  Serial.printf("Set P2P mode frequency %3.3f: %s\r\n", (myFreq / 1e6),
                api.lora.pfreq.set(myFreq) ? "Success" : "Fail");
  Serial.printf("Set P2P mode spreading factor %d: %s\r\n", sf,
                api.lora.psf.set(sf) ? "Success" : "Fail");
  Serial.printf("Set P2P mode bandwidth %d: %s\r\n", bw,
                api.lora.pbw.set(bw) ? "Success" : "Fail");
  Serial.printf("Set P2P mode code rate 4/%d: %s\r\n", (cr + 5),
                api.lora.pcr.set(cr) ? "Success" : "Fail");
  Serial.printf("Set P2P mode preamble length %d: %s\r\n", preamble,
                api.lora.ppl.set(preamble) ? "Success" : "Fail");
  Serial.printf("Set P2P mode tx power %d: %s\r\n", txPower,
                api.lora.ptp.set(txPower) ? "Success" : "Fail");
  api.lora.registerPRecvCallback(recv_cb);
  api.lora.registerPSendCallback(send_cb);
  Serial.printf("P2P set Rx mode %s\r\n",
                api.lora.precv(65534) ? "Success" : "Fail");
}

/*
_____________________________CAU HINH FRAME TRUYEN_______________________________________
 [BEGIN]    [LEN]   [FRAME TYPE]    [FUNCTION]    [MAC ADDR]    [DATA]    [CRC]   [END]

BEGIN: OXFE
END  : OXEF
MAC  : MAC ADDRESS NODE
LEN  : Length frame
CRC  : CRC 8 bit frame
*/
void loop() {
// Cấu hình frame để gửi dữ liệu ADAM
  uint8_t frame[15];
  frame[0] = 0xFE; // BEGIN
  frame[1] = 0x09; // LEN . 9 bytes (FRAME TYPE + FUNCTION + MAC (4 bytes) + DATA (2 bytes))
  frame[2] = 0x01; // FRAME TYPE :0x01 (request)
  frame[3] = 0x05; // FUNCTION: 0x05 (ADAM)
  memcpy(&frame[4], MAC, 4); // MAC: 0x12345678 (4 bytes) -- Nội dung của frame được cập nhật từ vị trí array[4]
  memcpy(&frame[8], data, 2); // DATA:0xABCD (2 bytes) - 16 bits
  frame[10] = calculateCRC(&frame[2], 9); // CRC:Tính giá trị CRC cho frame - Don't have begin, end frame
  frame[11] = 0xEF; // END
   
  bool send_result = false;
  Serial.print  f("P2P send Success\r\n");
  delay(5000);
  digitalWrite(LED_SEND,(!digitalRead(LED_SEND)));
    if (rx_done) { //Nếu nhận được tín hiệu Rx - flag of rx_done: true
        rx_done = false; //reassign
        while (!send_result) {
            send_result = api.lora.psend(sizeof(frame), frame);
            Serial.printf("P2P send %s\r\n", send_result ? "Success" : "Fail");
            if (!send_result) {
                Serial.printf("P2P finish Rx mode %s\r\n", api.lora.precv(0) ? "Success" : "Fail");
                //delay(5000);
            }
        }
    }
}

