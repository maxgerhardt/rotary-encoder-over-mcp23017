#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Rotary.h>
#include <RotaryEncOverMCP.h>
#include <map>

/* Our I2C MCP23017 GPIO expanders */
Adafruit_MCP23017 mcp;

//Array of pointers of all MCPs if there is more than one
Adafruit_MCP23017* allMCPs[] = { &mcp };
constexpr int numMCPs = (int)(sizeof(allMCPs) / sizeof(*allMCPs));

enum class RotaryEvent : uint8_t {
    RotationClockwise = 0,
    RotationCounterClockwise  = 1,
};

/* can record up to 64 events */
struct RotaryEncoderEventCollection {
    int id; /* stores rotary encoder ID */
    uint64_t events; /* bit mask stores event. 0 = clockwise, 1 = coutner-clock */
    size_t numEvents; /* stores number of valid event bits */
    bool dirty; /* true if the value has changed in regards to the last readout */
    bool overflow; /* true if more than the maximum number of allowed events were reached */
};

class RotaryEncodersEventBufferer {
public:
    void registerEncoder(int encoderId) {
        rotaryIDToEvents[encoderId] = RotaryEncoderEventCollection{ encoderId };
    }

    void registerEncoders(RotaryEncOverMCP* encoders, int numEncoders) {
        for(int i=0; i < numEncoders; i++) {
            registerEncoder(encoders[i].getID());
        }
    }

    /* initializes a mutex to prevent data races */
    void init() { dataLock = xSemaphoreCreateRecursiveMutex(); }
    void lock() { xSemaphoreTakeRecursive(dataLock, portMAX_DELAY); }
    void unlock() { xSemaphoreGiveRecursive(dataLock); }

    void recordEvent(bool clockwise, int id) {
        lock();
        if(!rotaryIDToEvents.count(id)) {
            unlock();
            return;
        }
        auto record = rotaryIDToEvents[id];
        if(record.numEvents == 64) {
            //couldn't save it
            record.overflow = true;
        } else {
            if(!clockwise) 
                record.events |= (uint64_t) (1ull << record.numEvents);
            record.numEvents++;
            record.dirty = true;
        }
        //save back
        rotaryIDToEvents[id] = record;
        unlock();
        Serial.println("[Bufferer] Recorded event for ID = " + String(id) 
            + ": " + (((record.events & (1ull << (record.numEvents - 1))) == 0) ? String("clockwise") : String("counter-clock-wise")));
    }

    std::vector<RotaryEvent> getEvents(int id, bool& overflow) {
        lock();
        std::vector<RotaryEvent> events {};
        if(!rotaryIDToEvents.count(id)) {
            unlock();
            return events;
        }
        auto record = rotaryIDToEvents[id];
        if(record.dirty) {
            for(int i=0; i < record.numEvents; i++) {
                RotaryEvent evt = ((record.events & (1ull << i)) == 0) 
                    ? RotaryEvent::RotationClockwise : RotaryEvent::RotationCounterClockwise;  
                events.push_back(evt);
            }
            //reset read info.
            record.dirty = false;
            overflow = record.overflow;
            record.overflow = false;
            record.numEvents = 0;
            record.events = 0ul;
            rotaryIDToEvents[id] = record;
        }
        unlock();
        return events;
    }
    //funciton without overflow info (shortcut)
    std::vector<RotaryEvent> getEvents(int id) { bool dummy = false; return getEvents(id, dummy); }

    std::vector<std::tuple<int, std::vector<RotaryEvent>>> getAllEvents() {
        std::vector<std::tuple<int, std::vector<RotaryEvent>>> events {};
        lock();
        for (const auto &p : rotaryIDToEvents)
        {
            auto new_events = getEvents(p.first);
            if(new_events.size() != 0)
                events.push_back(std::make_tuple(p.first, new_events));
        } 
        unlock();
        return events;
    }
private:
    std::map<int, RotaryEncoderEventCollection> rotaryIDToEvents;
    SemaphoreHandle_t dataLock;
};

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

RotaryEncodersEventBufferer rotaryEncodersEventBufferer;

void RotaryEncoderChanged(bool clockwise, int id) {
    //Serial.println("Encoder " + String(id) + ": "
    //        + (clockwise ? String("clockwise") : String("counter-clock-wise")));
    rotaryEncodersEventBufferer.recordEvent(clockwise, id);
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

    rotaryEncodersEventBufferer.init();
    rotaryEncodersEventBufferer.registerEncoders(rotaryEncoders, numEncoders);

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
    //check on buffered events prepared by the other task
    auto bufferedEvents = rotaryEncodersEventBufferer.getAllEvents();
    Serial.println("[Main Task] Checking number of changed encoders: " + String(bufferedEvents.size()) 
            + String(" buffered events: ") + String(
            std::accumulate(bufferedEvents.begin(),bufferedEvents.end(), 0, 
            [&](int b, std::tuple<int, std::vector<RotaryEvent>> a){ return std::get<1>(a).size()+b;})
    ));

    for(std::tuple<int, std::vector<RotaryEvent>>& event : bufferedEvents) {
        int encoderId = std::get<0>(event);
        for(RotaryEvent turnEvent : std::get<1>(event)) {
            Serial.println("(Buffered) Encoder " + String(encoderId) + ": "
                + (turnEvent == RotaryEvent::RotationClockwise ? String("clockwise") : String("counter-clock-wise")));
            //can do application logic here
        }
    }

    //we can do work here that is time-intensive and blocking, since all encoder events
    // will be buffered by the managed rotary encoder class.
    Serial.println("[Main Task] Doing some work for 2 seconds...");
    delay(2000);
}
