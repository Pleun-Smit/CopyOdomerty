#include "hardware/EncoderSensor.h"

int channelA_;
int channelB_;

EncoderSensor::EncoderSensor(int channelA, int channelB) : channelA(channelA_), channelB(channelB_), encoder(channelA_, channelB_){
    encoder.Reset(); // reset form WPILib to start counting from 0
}

double EncoderSensor::getValue() const{
    // returns distance using WPILib encoder, but you process it self elsewhere
    return encoder.GetDistance();
}

void EncoderSensor::reset(){
    encoder.Reset();
}