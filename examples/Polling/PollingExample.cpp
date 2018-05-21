#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Rotary.h>
#include <RotaryEncOverMCP.h>

/* Our I2C MCP23017 GPIO expanders */
Adafruit_MCP23017 mcp;

//Array of pointers of all MCPs if there is more than one
Adafruit_MCP23017* allMCPs[] = { &mcp };
constexpr int numMCPs = (int)(sizeof(allMCPs) / sizeof(*allMCPs));

/* function prototypes */
void RotaryEncoderChanged(bool clockwise, int id);

/* Array of all rotary encoders and their pins */
RotaryEncOverMCP rotaryEncoders[] = {
        // outputA,B on GPA7,GPA6, register with callback and ID=1
        RotaryEncOverMCP(&mcp, 7, 6, &RotaryEncoderChanged, 1)
};
constexpr int numEncoders = (int)(sizeof(rotaryEncoders) / sizeof(*rotaryEncoders));

void RotaryEncoderChanged(bool clockwise, int id) {
    Serial.println("Encoder " + String(id) + ": "
            + (clockwise ? String("clockwise") : String("counter-clock-wise")));
}

void setup(){
    Serial.begin(115200);
    Serial.println("MCP23007 Polling Test");

    mcp.begin();      // use default address 0

    //Initialize input encoders (pin mode, interrupt)
    for(int i=0; i < numEncoders; i++) {
        rotaryEncoders[i].init();
    }
}

void pollAll() {
    //We could also call ".poll()" on each encoder,
    //however that will cause an I2C transfer
    //for every encoder.
    //It's faster to only go through each MCP object,
    //read it, and then feed it into the encoder as input.
    for(int j = 0; j < numMCPs; j++) {
        uint16_t gpioAB = allMCPs[j]->readGPIOAB();
        for (int i=0; i < numEncoders; i++) {
            //only feed this in the encoder if this
            //is coming from the correct MCP
            if(rotaryEncoders[i].getMCP() == allMCPs[j])
                rotaryEncoders[i].feedInput(gpioAB);
        }
    }
}

void loop() {
    pollAll();
}
