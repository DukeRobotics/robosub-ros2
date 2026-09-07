#ifndef FRAME_PARSER_HPP
#define FRAME_PARSER_HPP

constexpr uint8_t START_FLAG[] = {0xFF, 0xFF};
constexpr uint8_t THRUSTER_BYTE = 0x01;
constexpr uint8_t SERVO_BYTE = 0x02;

enum class IngestState{
    Invalid,
    Start1,
    Start2,
    Type,
    Data
};

enum class DeviceType{
    Invalid,
    Thruster,
    Servo
};

class FrameParser{
    IngestState currState = IngestState::Start1;
    DeviceType currType = DeviceType::Invalid;
    uint16_t frameBuffer[32];
    uint8_t frameIndex = 0;
    uint8_t numThrusters = 8;

public:
    FrameParser(uint8_t numThrusters = 8) : numThrusters(numThrusters) {};

    bool processByte(uint8_t data){
        switch(currState){
            case IngestState::Start1:
                // Handle start state 1
                currType = DeviceType::Invalid;
                currState = IngestState::Start1;
                frameIndex = 0;

                if (data == START_FLAG[0]) {
                    currState = IngestState::Start2;
                } else {
                    currState = IngestState::Start1;
                }
                break;
            case IngestState::Start2:
                // Handle start state 2
                if (data == START_FLAG[1]) {
                    currState = IngestState::Type;
                } else {
                    currState = IngestState::Start1;
                }
                break;
            case IngestState::Type:
                // Handle type state
                if (data == THRUSTER_BYTE){
                    currType = DeviceType::Thruster;
                    currState = IngestState::Data;
                } else if (data == SERVO_BYTE) {
                    currType = DeviceType::Servo;
                    currState = IngestState::Data;
                } else {
                    currState = IngestState::Start1;
                }
                break;
            case IngestState::Data:
                // Handle data state
                if (currType == DeviceType::Thruster) {
                    // Process thruster data
                    frameBuffer[frameIndex++] = data;                    
                    if (frameIndex >= 2 * numThrusters) {
                        currState = IngestState::Start1;
                        return true;
                        // processFrame(currType);
                        
                    }
                } else if (currType == DeviceType::Servo) {
                    // Process servo data
                    frameBuffer[frameIndex++] = data;
                    if (frameIndex >= 3) {
                        currState = IngestState::Start1;
                        return true;
                        // processFrame(currType);
                        
                    }
                } else {
                    currState = IngestState::Start1;
                }
                
                break;
        }
        return false;
    }

    bool getFrame(DeviceType& type, uint16_t* buffer) {
        type = currType;
        if (type == DeviceType::Thruster) {
            // Process thruster frame
            for (int i = 0; i < numThrusters; i++) {
                uint16_t pwm = frameBuffer[i * 2 + 1] | (frameBuffer[i * 2] << 8);
                buffer[i] = pwm;                
            }
            return true;
        } else if (type == DeviceType::Servo) {
            // Process servo frame        
            uint16_t servoID = frameBuffer[0];
            uint16_t pwm = frameBuffer[2] | (frameBuffer[1] << 8);
            buffer[0] = servoID;
            buffer[1] = pwm;
            return true;
        }
        return false;
    }
};

#endif