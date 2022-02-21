#include <CAN.h>
#include <QEI.hpp>
#include <WS2812B_Controller.h>
#include <algorithm>
#include <cstdint>
#include <iterator>
#include <mbed.h>
#include <vector>

class CAN_Transmitter {
private:
  CAN *CAN_Obj;
  struct Data_Stc {
    uint32_t ID;
    uint8_t Data[8];
  };
  vector<Data_Stc> Datas;
  uint16_t Default_Minimum_Interval_Hz;
  uint16_t Default_Maximum_Interval_Hz;
  vector<uint16_t> Minimum_Interval_Ms;
  vector<uint16_t> Maximum_Interval_Ms;
  vector<uint32_t> IDs;
  vector<double> Previous_Out1, Previous_Out2;

  template <class T> bool isDataEqual(uint8_t size, T *data1, T *data2) {
    for (uint8_t i = 0; i < size; i++) {
      if (data1[i] != data2[i])
        return false;
    }
    return true;
  }

  uint64_t getSec() { return (TIM_MST->CNT / 1e6); }
  uint64_t getMillis() { return (TIM_MST->CNT / 1e3); }
  uint64_t getMicros() { return (TIM_MST->CNT); }

  uint16_t getIDsIndexFromID(uint32_t ID) {
    std::vector<uint32_t>::iterator Itr = std::find(IDs.begin(), IDs.end(), ID);
    const uint16_t ID_Index = std::distance(IDs.begin(), Itr);
    return ID_Index;
  }

  int8_t isEligibleToOut(uint8_t Size, uint8_t *Data_Out, Data_Stc *Data_Set) {
    // Kernel系列(std::chrono)は激遅やから使用禁止！
    // auto now_ms = time_point_cast<milliseconds>(Kernel::Clock::now());
    bool Is_Data_Equal = isDataEqual(Size, Data_Out, Data_Set->Data);
    for (int i = 0; i < 8; i++) {
      Data_Set->Data[i] = Data_Out[i];
    }
    if (Is_Data_Equal) {
      if (getMillis() - Previous_Out1.at(getIDsIndexFromID(Data_Set->ID)) >=
          Maximum_Interval_Ms.at(getIDsIndexFromID(Data_Set->ID))) {
        Previous_Out1.at(getIDsIndexFromID(Data_Set->ID)) = getMillis();
        Previous_Out2.at(getIDsIndexFromID(Data_Set->ID)) = getMillis();
        return true;
      }
    } else {
      if (getMillis() - Previous_Out2.at(getIDsIndexFromID(Data_Set->ID)) >=
          Minimum_Interval_Ms.at(getIDsIndexFromID(Data_Set->ID))) {
        Previous_Out1.at(getIDsIndexFromID(Data_Set->ID)) = getMillis();
        Previous_Out2.at(getIDsIndexFromID(Data_Set->ID)) = getMillis();
        return true;
      }
    }
    return false;
  }

public:
  CAN_Transmitter(PinName rd, PinName td, uint32_t BusSpeed) {
    CAN_Obj = new CAN(rd, td, BusSpeed);
    CAN_Obj->mode(CAN::Normal);
    CAN_Obj->frequency(BusSpeed);
    Default_Minimum_Interval_Hz = 10;
    Default_Maximum_Interval_Hz = 200;
  }
  ~CAN_Transmitter();

  void setIntervalHz(uint32_t ID, uint16_t Minimum_Interval_Hz,
                     uint16_t Maximum_Interval_Hz) {
    // Search ID in the IDs vector and compose ID as new one
    if (std::find(IDs.begin(), IDs.end(), ID) == IDs.end()) {
      // ID not found in the IDs vector
      IDs.push_back(ID);
      Data_Stc _data;
      _data.ID = ID;
      Datas.push_back(_data);
      this->Maximum_Interval_Ms.resize(this->Maximum_Interval_Ms.size() + 1);
      this->Minimum_Interval_Ms.resize(this->Minimum_Interval_Ms.size() + 1);
      this->Previous_Out1.resize(this->Previous_Out1.size() + 1);
      this->Previous_Out2.resize(this->Previous_Out2.size() + 1);
    }

    this->Maximum_Interval_Ms.at(getIDsIndexFromID(ID)) =
        1e3 / Minimum_Interval_Hz;
    this->Minimum_Interval_Ms.at(getIDsIndexFromID(ID)) =
        1e3 / Maximum_Interval_Hz;
  }

  int8_t busOut(uint32_t ID, uint8_t *Data_Out) {
    this->busOut(ID, Data_Out, 8);
    return true;
  }

  int8_t busOut(uint32_t ID, uint8_t *Data_Out, uint8_t Length) {
    if (isEligibleToOut(Length, Data_Out, &Datas.at(getIDsIndexFromID(ID)))) {
      CANMessage Message =
          CANMessage(ID, Data_Out, Length, CANData, CANStandard);
      int state = CAN_Obj->write(Message);
      return state ? true : false;
    }
    return false;
  }

  int8_t busIn(uint8_t *ID, uint8_t *Data_In) {
    CANMessage Message;
    int state = CAN_Obj->read(Message);
    if (state) {
      Data_In = Message.data;
      *ID = Message.id;
    }
    return state ? true : false;
  }
};

void GPIO_HAL_init() {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

Timer _timer_encoder;
QEI encoder_1(PA_0, PA_1, NC, 100, &_timer_encoder, QEI::X2_ENCODING);
QEI encoder_2(PA_2, PA_3, NC, 100, &_timer_encoder, QEI::X2_ENCODING);
QEI encoder_3(PA_4, PA_5, NC, 100, &_timer_encoder, QEI::X2_ENCODING);
QEI encoder_4(PA_6, PA_7, NC, 100, &_timer_encoder, QEI::X2_ENCODING);
QEI encoder_5(PA_8, PA_9, NC, 100, &_timer_encoder, QEI::X2_ENCODING);
DigitalIn switch_1(PB_1);
DigitalIn switch_2(PB_0);
DigitalIn switch_3(PB_7);
DigitalIn switch_4(PB_6);
DigitalIn switch_5(PB_5);
CAN_Transmitter CANBus(PA_11, PA_12, 1e6);

#define CAN_ID1 (6)
#define CAN_ID2 (7)

int main() {
  GPIO_HAL_init();
  uint8_t CAN_Bus1_Data[8], CAN_Bus2_Data[8];
  CANBus.setIntervalHz(CAN_ID1, 50, 200);
  CANBus.setIntervalHz(CAN_ID2, 50, 200);

  switch_1.mode(PullDown);
  switch_2.mode(PullDown);
  switch_3.mode(PullDown);
  switch_4.mode(PullDown);
  switch_5.mode(PullDown);

  while (1) {

    CAN_Bus1_Data[0] = uint8_t((encoder_1.getPulses() >> 8) & 0xff);
    CAN_Bus1_Data[1] = uint8_t(encoder_1.getPulses() & 0xff);
    CAN_Bus1_Data[2] = uint8_t((encoder_2.getPulses() >> 8) & 0xff);
    CAN_Bus1_Data[3] = uint8_t(encoder_2.getPulses() & 0xff);
    CAN_Bus1_Data[4] = uint8_t((encoder_3.getPulses() >> 8) & 0xff);
    CAN_Bus1_Data[5] = uint8_t(encoder_3.getPulses() & 0xff);
    CAN_Bus1_Data[6] = uint8_t((encoder_4.getPulses() >> 8) & 0xff);
    CAN_Bus1_Data[7] = uint8_t(encoder_4.getPulses() & 0xff);
    CAN_Bus2_Data[0] = uint8_t((encoder_5.getPulses() >> 8) & 0xff);
    CAN_Bus2_Data[1] = uint8_t(encoder_5.getPulses() & 0xff);
    CAN_Bus2_Data[2] = switch_1.read();
    CAN_Bus2_Data[3] = switch_2.read();
    CAN_Bus2_Data[4] = switch_3.read();
    CAN_Bus2_Data[5] = switch_4.read();
    CAN_Bus2_Data[6] = switch_5.read();
    CAN_Bus2_Data[7] = 0;

    CANBus.busOut(CAN_ID1, CAN_Bus1_Data);
    CANBus.busOut(CAN_ID2, CAN_Bus2_Data);
  }
}
