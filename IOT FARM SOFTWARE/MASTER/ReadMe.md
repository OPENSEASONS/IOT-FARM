# Build Instructions

1. Delete previous builds and create new build directory:
```bash
mkdir build
```

2. Change to build directory:
```bash
cd build
```

3. Configure CMake with Unix Makefiles:
```bash
cmake .. -G "Unix Makefiles"
```

4. Build the project:
```bash
cmake --build .
```

can i have the following improvemmnts
Timing Improvements

unsigned long previousMillis = 0;
const long interval = 500; // ms

void loop() {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        // Your data collection and transmission code here
    }
    
    // You can add other non-blocking tasks here
}


 Configure low-power modes when power is less than 30%
    set_sleep_mode(SLEEP_MODE_IDLE);
add a noise filter to the data filter_factor
since we know the data is going to be linear any incosistent data in between 2 correctly linear data should be filtered off otherwise if consistently unlinear should be left alone 

Watchdog Timer
that resest the atmega 328p after a long period and say 24 hours of operations then restarts again 

Configuration Management
i need  thing parametised since the device will be unique for each soo check the files and see if it allows parameters to be passed from the spi communication and added in the next loop funcitons 

 Task Scheduling
 Have it as task sheduled please 