/*LoRa - Node Adam - 20/6/2024 */
#include <Arduino.h> 
#include <Rak3172_Canopus.h>

// Định nghĩa các chân và các hàm hỗ trợ
#define ONEWIRE_PIN_DATA GPIO_PIN_11
#define ONEWIRE_PORT_DATA GPIOA


long startTime;
bool rx_done = false;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;

uint8_t MAC[4] = {0x12, 0x34, 0x56, 0x78}; //Giả sử địa chỉ mac address

uint8_t data[2] = {0x02, 0x05}; //data gom 2 fuction - Giả sử giá trị 

//________________________________OneWire - DS18B20 __________________________
 // Định nghĩa các chân và các hàm hỗ trợ
#define ONEWIRE_PIN_DATA GPIO_PIN_11
#define ONEWIRE_PORT_DATA GPIOA

void set_pin_as_output(GPIO_TypeDef *port, uint16_t pin) {
  // Cấu hình chân GPIO thành chế độ xuất (output).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void set_pin_as_input(GPIO_TypeDef *port, uint16_t pin) {
    // Cấu hình chân GPIO thành chế độ nhập (input).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

// Đặt lại DS18B20
void onewire_reset(void) {
    // Đặt lại cảm biến DS18B20.
    set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);  // Cấu hình chân dữ liệu thành chế độ xuất.
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);  // Kéo chân dữ liệu xuống mức thấp.
    delayMicroseconds(480);  // Chờ 480 microseconds.
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);  // Kéo chân dữ liệu lên mức cao.
    delayMicroseconds(480);  // Chờ thêm 480 microseconds.
    set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);  // Cấu hình chân dữ liệu thành chế độ nhập.
    delayMicroseconds(480);  // Chờ thêm 480 microseconds.
}

// Ghi 1 bit vào DS18B20
void onewire_write_bit(uint8_t bit) {
    set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    if (bit) { //Nếu bit la 1
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
        delayMicroseconds(1);
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
        delayMicroseconds(60);
    } else {
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
        delayMicroseconds(60);
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
        delayMicroseconds(1);
    }
    set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
}

// Đọc 1 bit từ DS18B20
uint8_t onewire_read_bit(void) {
    uint8_t bit = 0;
    set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
    delayMicroseconds(1);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
    set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    delayMicroseconds(15);
    bit = HAL_GPIO_ReadPin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    delayMicroseconds(45);
    return bit;
}

// Ghi 1 byte vào DS18B20
void onewire_write_byte(uint8_t data) {
    // Ghi một byte vào DS18B20.
    for (uint8_t i = 0; i < 8; i++) {
        onewire_write_bit(data & 0x01);  // Ghi bit thấp nhất của byte.
        data >>= 1;  // Dịch chuyển byte sang phải 1 bit.
    }
}

// Đọc 1 byte từ DS18B20
uint8_t onewire_read_byte(void) {
    // Đọc một byte từ DS18B20.
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        data |= (onewire_read_bit() << i);  // Đọc từng bit và dịch chuyển vào byte.
    }
    return data;  // Trả về giá trị byte đọc được.
}

// Đọc nhiệt độ từ DS18B20
float read_temperature(void) {
    uint8_t temp_LSB, temp_MSB;
    int16_t temp;

    // Khởi động lại cảm biến
    onewire_reset();

    // Gửi lệnh Skip ROM
    onewire_write_byte(0xCC);

    // Gửi lệnh Convert T
    onewire_write_byte(0x44);

    // Chờ cảm biến hoàn thành quá trình chuyển đổi nhiệt độ
    delay(750);

    // Khởi động lại cảm biến
    onewire_reset();

    // Gửi lệnh Skip ROM
    onewire_write_byte(0xCC);

    // Gửi lệnh Read Scratchpad
    onewire_write_byte(0xBE);

    // Đọc 2 byte dữ liệu nhiệt độ
    temp_LSB = onewire_read_byte();
    temp_MSB = onewire_read_byte();

    // Chuyển đổi dữ liệu nhiệt độ
    temp = ((int16_t)temp_MSB << 8) | temp_LSB;

    // Trả về giá trị nhiệt độ dạng float
    return (float)temp / 16.0;
}

//_________________________________END SETUP ONEWIRE___________________________

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

//________________________________Sleep mode --- RAK3172___________________________
void sleep_mode() {
    Serial.print("The timestamp before sleeping: ");
    Serial.print(millis());
    Serial.println(" ms");
    Serial.println("(Wait 10 seconds or Press any key to wakeup)");
    api.system.sleep.all(10000);
    Serial.print("The timestamp after sleeping: ");
    Serial.print(millis());
    Serial.println(" ms");
}

void setup() {
  Serial.begin(115200);
  Serial.println("RAK3172_Canopus lora P2P");
  Serial.println("------------------------------------------------------");
  Serial.println("RAKwireless System Powersave");
  Serial.println("------------------------------------------------------");
  init_io(); //Enable I/O
  enable_Vss3();
  startTime = millis();
  //_______________________ONE WIRE____________________--
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
  Serial.printf("P2P send Success\r\n");
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

  float temperature = read_temperature();
  Serial.print("Temperature: ");
  Serial.print(temperature, 2);  
  Serial.println(" *C");
  delay(100); 
}

