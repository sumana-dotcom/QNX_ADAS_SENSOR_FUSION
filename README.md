# QNX ADAS Sensor Fusion on Raspberry Pi 4

A small hardware-based ADAS (Advanced Driver-Assistance System) prototype using two physically connected sensors and QNX Neutrino RTOS on a Raspberry Pi 4 Model B.

The project reads distance from an ultrasonic sensor and obstacle state from a digital IR sensor. A sensor-fusion thread checks the readings for obstacle indications, stale data, sensor faults, and disagreement between the sensors.

> **Project scope:** This is an educational prototype, not a production or safety-certified ADAS system.

## Hardware

- Raspberry Pi 4 Model B running QNX SDP 8
- HC-SR04-style ultrasonic distance sensor
- Digital IR obstacle-detection module
- Warning LED and appropriate current-limiting resistor (if the LED is externally wired)
- Common ground between the Pi and sensor circuits
- A suitable voltage divider or logic-level shifter for the ultrasonic ECHO signal when using a 5 V HC-SR04 module

### GPIO connections

 Ultrasonic TRIG - GPIO23 - Output - Starts a distance measurement 
 Ultrasonic ECHO - GPIO24 - Input - Pulse width is used to calculate distance 
 Digital IR OUT  - GPIO25 - Input - LOW is interpreted as obstacle detected 
 Warning LED     - GPIO18 - Output - Indicates degraded mode by blinking 

**Electrical safety:** Raspberry Pi GPIO is 3.3 V logic and is not 5 V tolerant. Do not connect a 5 V ECHO output directly to GPIO24. Confirm the output voltage and polarity of your particular IR module before wiring it.

## Software and implementation

- QNX SDP 8 / QNX Neutrino RTOS
- C and POSIX threads (`pthread`)
- Mutexes for protecting shared sensor state and the event queue
- Condition variable for waking the background logger
- `clock_gettime(CLOCK_MONOTONIC)` for elapsed-time measurements
- Memory-mapped BCM2711 GPIO registers in the supplied implementation


## Thread architecture

There are **three main processing threads and one main thread**:

1. **Ultrasonic thread** - triggers the ultrasonic sensor, measures the ECHO pulse, calculates distance, filters recent readings, and updates the shared state.
2. **IR thread** - samples the digital IR output, applies a small sample-window filter, and updates the shared obstacle state.
3. **Sensor-fusion thread** - reads the latest shared sensor state and decides whether the system can operate normally or should enter degraded mode.

The supplied source also defines **three supporting threads**, so the complete program creates six threads in total:

- **Watchdog thread** - checks task heartbeat timestamps for stalled tasks.
- **Logger thread** - writes queued fault and recovery events to a file.
- **LED thread** - blinks the warning LED when degraded mode is active.

### Shared data and synchronization

The threads exchange sensor readings through a shared global `SensorData` structure. A `pthread_mutex_t` protects access to this structure so that one thread does not read partially updated fields while another thread is writing them.

This is **shared data between threads in one process**, not POSIX shared memory between separate processes. The `mmap()` call in this code maps physical GPIO registers; it does not create a shared-memory sensor-data object.

## Sensor processing and fusion

### Ultrasonic measurement

1. Drive TRIG LOW briefly.
2. Send a HIGH trigger pulse of approximately 10 microseconds.
3. Wait for ECHO to rise and then fall, with a timeout.
4. Convert the measured pulse duration to distance in centimetres.
5. Reject measurements outside the configured 2-400 cm range and smooth accepted measurements using a moving window.

The code's distance conversion is approximately:

distance_cm = pulse_duration_ns * 0.00001715;


Because this method polls the GPIO in a busy-wait loop, scheduling delays can affect timing accuracy. It is suitable as a learning prototype but should not be treated as a precision ranging implementation.

### IR obstacle detection

The IR thread samples GPIO25. The program interprets **LOW as obstacle detected** and uses a short sample window to reduce the effect of brief signal changes. This polarity must match the actual module's output behavior.

### Fusion decisions

The fusion logic compares the latest valid readings. Example disagreement conditions configured in the code include:

- IR reports an obstacle while ultrasonic distance is greater than the configured far threshold (30 cm).
- IR reports clear while ultrasonic distance is below the configured near threshold (10 cm).

The near/far thresholds should be calibrated for the sensor mounting position and intended demonstration. The code also checks whether sensor data are stale and considers sensor/task faults when selecting the operating mode.

- **Normal mode:** the program may report `OBSTACLE DETECTED` or `ROAD CLEAR` based on the sensor readings.
- **Degraded mode:** the program indicates that the sensor information is not fully trustworthy, logs fault information, and blinks the warning LED.

A disagreement does not necessarily prove that a sensor has failed; different sensing ranges, angles, surfaces, and detection characteristics can also cause disagreement.

## Fault monitoring and logging

The implementation includes several diagnostic mechanisms:

- **Ultrasonic read timeout/error counting** — repeated failed measurements can invalidate the ultrasonic reading.
- **Stale-data detection** — readings older than the configured timeout are treated as stale.
- **Stuck-value suspicion** — an unchanged sensor value for a long interval can be flagged for investigation. This is a suspicion, not proof of failure; a stationary scene can naturally produce a stable reading.
- **Task heartbeats** — the watchdog checks whether key threads are still updating their progress timestamps.
- **Event queue and logger** — threads enqueue timestamped messages; the logger appends them to `/tmp/adas_fault.log`.

The log file is stored under `/tmp`, so its contents may not persist across a reboot. If the event queue fills, new events can be dropped by the queue logic.

## Build and run

1. Open the project in the QNX Momentics workspace configured for your QNX SDP 8 target.
2. Build the C application for the target architecture used by your Raspberry Pi 4 image (typically AArch64 for a 64-bit QNX image).
3. Confirm the GPIO wiring and voltage levels before running the executable.
4. Copy the executable to the Raspberry Pi using your established QNX transfer method.
5. Run it from a shell with the privileges required by the GPIO-access method used in the program:

   ```
   su root
   cd tmp
   ./qnx_adas_sf
   ```

6. Observe the console messages, warning LED

The exact Momentics build settings and deployment commands depend on how your QNX target and project are configured. Direct GPIO mapping may fail even when the C code compiles, because runtime privileges and target support are separate requirements.

## Important implementation checks

- The pasted source in the project discussion was cut off partway through `ultrasonic_thread()`. Check the complete local source before assuming every described behavior is implemented exactly as intended.
- Check every `pthread_create()` return value and report failures clearly.
- Ensure watchdog fault flags cannot be overwritten by the fusion thread before the corresponding task has recovered.
- Check the return values from GPIO setup and sensor reads; do not silently treat an invalid GPIO read as a valid LOW level.
- Validate the HC-SR04 timing and voltage interface on the actual hardware.
- Use measured timing and tested thresholds rather than assuming the configured intervals are guaranteed execution periods.
- Do not use this prototype to control a real vehicle or as the sole basis for a safety decision.

## Possible future improvements

- Use a supported QNX GPIO interface/driver instead of direct physical-register access, if available for the target image.
- Use hardware-assisted timing or a suitable driver for more reliable ultrasonic pulse measurement.
- Add explicit startup/self-test states and a clear recovery policy for degraded mode.
- Record test cases for clear path, obstacle at different distances, sensor unplugged, blocked sensor, and sensor disagreement.
- Add a structured test report with observed results and timing measurements.

## Summary

This project demonstrates **multithreaded sensor acquisition, mutex-protected shared data, basic sensor fusion, watchdog monitoring, fault logging, and a degraded-mode indicator** on QNX. The three core threads are ultrasonic acquisition, IR acquisition, and sensor fusion; the logger, watchdog, and LED threads provide supporting functions.
