#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <EasyNextionLibrary/EasyNextionLibrary.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <math.h>
#include <mcp_can.h>
#include <utility/imumaths.h>

// #define DEBUG_CAN_INIT_STATS
// #define DEBUG_IMU_INIT_STATS

#define Jetson Serial
#define Nextion Serial1

constexpr uint32_t BAUD_CAN = CAN_1000KBPS;
constexpr uint32_t BAUD_JETSON = 516000;
constexpr uint32_t BAUD_NEXTION = 516000;
constexpr uint8_t PIN_GENERAL_SS = 10;
constexpr uint8_t PIN_CAN1_SS = 11;
constexpr uint8_t PIN_CAN2_SS = 12;
constexpr uint8_t PIN_INTERRUPT = 8;
constexpr uint8_t BNO055_SENSOR_ID = 55;
constexpr uint8_t BNO055_ADDRESS = 0x28;
constexpr uint16_t BNO055_SAMPLERATE_DELAY_MS = 1;

Adafruit_BNO055 IMU = Adafruit_BNO055(BNO055_SENSOR_ID, BNO055_ADDRESS);

MCP_CAN CAN1(PIN_CAN1_SS);
MCP_CAN CAN2(PIN_CAN2_SS);

uint8_t CAN_TX_ID[20];
uint8_t CAN_TX_BUFFER[20][8];

EasyNex display(Nextion);
void display_initialize()
{
    display.begin(BAUD_NEXTION);
    display.writeStr("page 0"); // for change current page of nextion display (overview)
    delay(1000);
    display.writeStr("page 1");
}

void IMU_initialize()
{
    if (!IMU.begin())
    {
#ifdef DEBUG_IMU_INIT_STATS
        Jetson.print("no IMU055 detected\r\n");
#endif
    }
}

void can_bus_initialize()
{
    // init CAN1 bus, baudrate: 1000k@16MHz
    // Board Oscillator freq: 16MHz
    // init CAN2 bus, baudrate: 1000k@16MHz
    // Board Oscillator freq: 16MHz

    if (CAN1.begin(MCP_STDEXT, BAUD_CAN, MCP_16MHZ) == CAN_OK)
    {
        CAN1.setMode(MCP_NORMAL);
#ifdef DEBUG_CAN_INIT_STATS
        Jetson.print("CAN1: Init Success\r\n");
#endif
    }
    else
    {
#ifdef DEBUG_CAN_INIT_STATS
        Jetson.print("CAN1: Init Fail\r\n");
#endif
    }

    if (CAN2.begin(MCP_STDEXT, BAUD_CAN, MCP_16MHZ) == CAN_OK)
    {
        CAN2.setMode(MCP_NORMAL);
#ifdef DEBUG_CAN_INIT_STATS
        Jetson.print("CAN2: Init Success\r\n");
#endif
    }
    else
    {
#ifdef DEBUG_CAN_INIT_STATS
        Jetson.print("CAN2: Init Fail\r\n");
#endif
    }
}

void setup()
{
    Jetson.begin(BAUD_JETSON);
    while (!Jetson)
        ;
    can_bus_initialize();
    display_initialize();
    IMU_initialize();
}

const uint8_t max_CAN_in_ID = 20;
static uint8_t CAN_ID_in_num = 0;
static uint8_t CAN_ID_in_list[max_CAN_in_ID] = {0};
char advertise_topic_name[max_CAN_in_ID][25]; // "can_out/ID" の文字数に準拠（余裕あり）

const uint8_t max_CAN_out_ID = 20;
static uint8_t CAN_ID_out_num = 0;
static uint8_t CAN_ID_out_list[max_CAN_out_ID] = {0};
char subscribe_topic_name[max_CAN_out_ID][25]; // "can_in/ID" の文字数に準拠（余裕あり）

void CAN_send(uint8_t id, uint8_t *data)
{
    CAN1.sendMsgBuf(id, 8, data);
}

void ROS_update()
{
    if (Jetson.available() > 11)
    {
        uint8_t frame_prefix = Jetson.read();
        // Jetson.println(String(frame_prefix));
        // static uint8_t count = 20;
        // String nex_text_id = "t" + String(count++) + ".txt";  //以前にTX分のテキストボックスを使用しているからその数（CAN_ID_out_num）を足す
        // display.writeStr(nex_text_id, String(frame_prefix));
        if (frame_prefix != 0xff) // frame prefix must be "0xff"
            return;
        frame_prefix = Jetson.read();
        if (frame_prefix != 0xfe) // next prefix must be "0xfe"
            return;
        uint8_t can_id = Jetson.read();
        uint8_t can_data[8];
        uint16_t can_data_sum = 0;
        for (uint8_t i = 0; i < 8; i++)
        {
            can_data[i] = Jetson.read();
            can_data_sum += can_data[i];
        }
        uint8_t frame_checksum = Jetson.read();
        uint8_t calc_checksum = 255 - (uint16_t(can_id + can_data_sum) % 256);
        if (frame_checksum != calc_checksum) // checksum error
            return;
        CAN_send(can_id, can_data);
    }
}

void ROS_send(uint8_t id, uint8_t *data)
{
    uint16_t can_data_sum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];
    uint8_t calc_checksum = 255 - (uint16_t(id + can_data_sum) % 256);
    Jetson.write(0xff);
    Jetson.write(0xfe);
    Jetson.write(id);
    Jetson.write(data[0]);
    Jetson.write(data[1]);
    Jetson.write(data[2]);
    Jetson.write(data[3]);
    Jetson.write(data[4]);
    Jetson.write(data[5]);
    Jetson.write(data[6]);
    Jetson.write(data[7]);
    Jetson.write(calc_checksum);
}

// BNOを４００KHZで制御している
// 不具合を確認したらすぐに速度を低下させること！
// [Line 80] Rev3_ROS_MAIN_BOARD/src/Adafruit_BNO055.cpp
void bno_data_compose(uint8_t *data_array)
{
    sensors_event_t rotation_data;
    union rotation
    {                         // store roll, pitch rotation
        float data_float[2];  // float   size -> 4UL => 8UL
        uint8_t data_byte[8]; // uint8_t size -> 1UL => 8UL
    } union_rotation;

    IMU.getEvent(&rotation_data, Adafruit_BNO055::VECTOR_EULER);
    union_rotation.data_float[0] = rotation_data.orientation.z; // roll
    union_rotation.data_float[1] = rotation_data.orientation.y; // pitch
    for (uint8_t i = 0; i < 8; i++)
    {
        data_array[i] = union_rotation.data_byte[i];
    }
}

void CAN_update()
{
    constexpr uint8_t max_can_id = 50;
    static uint8_t can_data[max_can_id][8]; // CANデータ保存用大規模配列
    static uint8_t can_id[max_can_id];      //いっていしゅうきそうしんようCANID
    static uint8_t can_id_num = 0;

    // fake data
    // uint8_t test_case = 10;
    // for (uint8_t i = 0; i < test_case; i++) {
    //     can_id[i] = i + 10;  // 10 is a just biasing number
    // }
    // static double index = 0;
    // index += 0.01;
    // can_id_num = test_case;
    // for (uint8_t k = 0; k < can_id_num; k++) {
    //     for (uint8_t i = 0; i < 8; i++) {
    //         can_data[can_id[k]][i] = i * 10 + k * 5 + (sin(index) * 20 + 20);
    //     }
    // }

    static bool IMU_disguise_init = true;
    static uint8_t IMU_disguise_can_id = 33;
    if (IMU_disguise_init)
    {
        can_id[can_id_num] = IMU_disguise_can_id;
        can_id_num += 1;
        IMU_disguise_init = false;
    }
    static long long prev_bno_read = 0;
    static constexpr uint16_t bno_update_interval = 10;
    if ((millis() - prev_bno_read) > bno_update_interval)
    {
        bno_data_compose(can_data[IMU_disguise_can_id]);
        prev_bno_read = millis();
    }

    static long long prev_ROS_send = 0;
    static constexpr uint16_t can_send_interval = 0;
    if (((micros() - prev_ROS_send) > can_send_interval) && can_id_num > 0)
    {
        static uint8_t id_index = 0;
        ROS_send(can_id[id_index], can_data[can_id[id_index]]);
        id_index++;
        if (id_index >= can_id_num)
            id_index = 0;
        prev_ROS_send = micros();
    }

    static long long prev_display_send = 0;
    static constexpr uint16_t display_send_interval = 100;
    if (((millis() - prev_display_send) > display_send_interval) && can_id_num > 0)
    {
        static uint8_t id_index = 0;
        String text = String(can_id[id_index]) + " : " + String(can_data[can_id[id_index]][0]) + " " + String(can_data[can_id[id_index]][1]) + " " + String(can_data[can_id[id_index]][2]) + " " + String(can_data[can_id[id_index]][3]) + " " + String(can_data[can_id[id_index]][4]) + " " + String(can_data[can_id[id_index]][5]) + " " + String(can_data[can_id[id_index]][6]) + " " + String(can_data[can_id[id_index]][7]);
        String text_id = "t" + String(20 + can_id[id_index] - 6) + ".txt"; //以前にTX分のテキストボックスを使用しているからその数（CAN_ID_out_num）を足す
        display.writeStr(text_id, text);
        id_index++;
        if (id_index >= can_id_num)
            id_index = 0;
        prev_display_send = millis();
    }

    INT32U rxID;
    INT8U message_length, rxBuffer[8];
    if (CAN1.readMsgBuf(&rxID, &message_length, rxBuffer) != CAN_OK)
    {
        return; // failed to receive can data
    }

    bool new_can_id_flag = true;
    for (uint8_t i = 0; i < can_id_num + 1; i++)
    {
        if (can_id[i] == rxID && rxID != 0)
        {
            new_can_id_flag = false;
        }
    }
    if (new_can_id_flag == true)
    {
        can_id[can_id_num] = rxID;
        can_id_num += 1;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        can_data[rxID][i] = rxBuffer[i];
    }
}

void loop()
{
    ROS_update();
    CAN_update();
}
