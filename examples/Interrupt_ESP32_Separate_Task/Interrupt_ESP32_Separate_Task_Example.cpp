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

/* the INT pin of the MCP can only be connected to
 * an interrupt capable pin on the ESP32,
 * Here we use IO23
 */
int arduinoIntPin = 23;

/* semaphore that will be used for reading of the rotary encoder */
SemaphoreHandle_t rotaryISRSemaphore = nullptr;

/* function prototypes */
void intCallBack();
void cleanInterrupts();
void handleInterrupt();
void RotaryEncoderChanged(bool clockwise, int id);
void rotaryReaderTask(void* pArgs);

/* Array of all rotary encoders and their pins */
RotaryEncOverMCP rotaryEncoders[] = {
    // outputA,B on GPA6,GPB1, register with callback and ID=1
    RotaryEncOverMCP(&mcp, 6, 9, &RotaryEncoderChanged, 1),
};
constexpr int numEncoders = (int)(sizeof(rotaryEncoders) / sizeof(*rotaryEncoders));

void RotaryEncoderChanged(bool clockwise, int id) {
    Serial.println("Encoder " + String(id) + ": "
            + (clockwise ? String("clockwise") : String("counter-clock-wise")));
}

void setup(){

    Serial.begin(115200);
    Serial.println("MCP23007 Interrupt Test");

    //initialize semaphore for reader task
    rotaryISRSemaphore = xSemaphoreCreateBinary();

    pinMode(arduinoIntPin,INPUT);

    mcp.begin();      // use default address 0
    mcp.readINTCAPAB(); //read this so that the interrupt is cleared

    //initialize all rotary encoders

    //Setup interrupts, OR INTA, INTB together on both ports.
    //thus we will receive an interrupt if something happened on
    //port A or B with only a single INT connection.
    mcp.setupInterrupts(true,false,LOW);

    //Initialize input encoders (pin mode, interrupt)
    for(int i=0; i < numEncoders; i++) {
        rotaryEncoders[i].init();
    }

    xTaskCreatePinnedToCore(&rotaryReaderTask, "rotary reader", 2048, NULL, 20, NULL, 1);

    attachInterrupt(arduinoIntPin, intCallBack, FALLING);

}

// The int handler will just signal that the int has happened
// we will do the work from a task.
void IRAM_ATTR intCallBack() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(rotaryISRSemaphore, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR ();
    }
}

void rotaryReaderTask(void* pArgs) {
    (void)pArgs;
    Serial.println("Started rotary reader task.");
    while(true) {
        if(xSemaphoreTake(rotaryISRSemaphore, portMAX_DELAY) == pdPASS) {
            //printing slows processing down, detaching and re-attaching interrupts
            //doesn't seem to needed here because of the high speed of the ESP32.
            //Serial.println("Got Signal from interrupt");
            //detachInterrupt(arduinoIntPin);
            handleInterrupt();
            //attachInterrupt(arduinoIntPin, intCallBack, FALLING);
        }
    }
}

void handleInterrupt(){
    //Read the entire state when the interrupt occurred

    //An interrupt occurred on some MCP object.
    //since all of them are ORed together, we don't
    //know exactly which one has fired.
    //just read all of them, pre-emptively.

    for(int j = 0; j < numMCPs; j++) {
        uint16_t gpioAB = allMCPs[j]->readINTCAPAB();
        // we need to read GPIOAB to clear the interrupt actually.
        volatile uint16_t dummy = allMCPs[j]->readGPIOAB();
        (void)dummy;
        for (int i=0; i < numEncoders; i++) {
            //only feed this in the encoder if this
            //is coming from the correct MCP
            if(rotaryEncoders[i].getMCP() == allMCPs[j])
                rotaryEncoders[i].feedInput(gpioAB);
        }
    }
}

void loop() {
    //loop() actually does nothing here, all the rotary encoder reading happens in other task!
    delay(1000);
}
