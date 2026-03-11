// "MotorManager" class handles the configuration of physical motor connectors (M-0 through M-3)
// These motor connectors are represented and controlled by instances of the "MotorDriver" class

// "Step & Direction" mode needs to be set on MSP software
// Set HLFB to "In Range - Position"
// Set the Input Resolution 

#include "ClearCore.h"
#define motor1 ConnectorM0
#define motor2 ConnectorM1
#define motor3 ConnectorM2
#define motor4 ConnectorM3
#define SerialPort ConnectorUsb

// Group connectors into an array for the [i] loops
MotorDriver *motors[] = {&motor1, &motor2, &motor3, &motor4};

// Hardware Constants
const int32_t encoderResolution = 8000; // Counts per revolution (set to match MSP) , 8000 RPM CAP for step/dir mode
const int32_t velocityRPM = 2000; // Target RPM 
const int32_t velocityMAX = 2300; // max RPM
const uint32_t accelerationLimit = 500; // pulses / sec^2
const uint32_t decelerationLimit = 500; // pulses / sec^2
const int32_t homingVelocity = -500; // move negative toward hard stop
const int32_t exitHomingMove = 200; // Distance to move away to exit homing mode
const int32_t baudRate = 115200; // Communication speed for USB, baudRate needs to match with the host computer, monitor

bool isRotating = false; // Global flag to see if motors are at the desired speed (not ramping)

// Define phase offsets in degree, corresponding to motor 0, 1, 2, 3
const int32_t offsets[] = {0, 90, 180, 270};

// Define specific motor location to move to (such as when to close the valve for emergency)
const int32_t targets[] = {0, 45, 90, 135}; // in degree with respect to the motor 0, initial location


// Function: Synchronize each motor individually with user confirmation
void ManualSyncZero() {
    SerialPort.SendLine("\n--- SEQUENTIAL MANUAL SYNC START ---");

    for (int i = 0; i < 4; i++) {
        // 1. Prompt User for the specific motor
        SerialPort.Send("ACTION REQUIRED: Prepare Motor ");
        SerialPort.Send(i);
        // Set the homing position to 0 in MSP
        SerialPort.SendLine(" (Lock rotor and click Ctrl+0 in MSP).");
        SerialPort.SendLine("Press 'Y' when this motor is ready to sync...");

        // 2. Wait for user confirmation ('Y') for this specific motor
        char response = 0;
        while (response != 'Y' && response != 'y') {
            if (SerialPort.Available() > 0) {
                response = SerialPort.Read(); // Capture individual response 
            }
        }

        // 3. Enable the motor and wait for HLFB (All Systems Go / In-Range)
        // Enabling energizes the coils to hold the physical position 
        motors[i]->EnableRequest(true);
        
        // 4: Wait for motor to finish enabling sequence
        while (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            // Safety: Abort if an alert (like a tracking error) occurs 
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Alert on Motor "); SerialPort.Send(i);
                SerialPort.SendLine("! Homing aborted.");
                return;
            }
        }

        // 5. Set the Logical Zero in ClearCore code
        // This ensures the current shaft orientation is treated as "0" for absolute moves
        motors[i]->PositionRefSet(0);

        // 6: Disble the motor
        motors[i]->EnableRequest(false);

        SerialPort.Send("Motor ");
        SerialPort.Send(i);
        SerialPort.SendLine(" SUCCESS: Enabled and Zeroed.\n");
    }

    SerialPort.SendLine("--- ALL MOTORS SYNCED ---");
    SerialPort.SendLine("CRITICAL: Remove all locking rods before commanding 'R' (Run).");
}

// function to set phase/angle differences
void SetPhaseAngles(int32_t angles[]){
    SerialPort.SendLine("Applying phase offsets...");
    for (int i = 0; i<4; i++){
        // Convert degrees to counts 
        int32_t distance = (angles[i] * encoderResolution) / 360;
        // Move(): move to the absolute position (0 ~ 8000)
        motors[i]->Move(distance, StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
    // Wait for all to finish
    for (int i = 0; i<4; i++){
        while(!motors[i]->StepsComplete()){
            continue;
        }
        motors[i]->PositionRefSet(0); // Make the offset the new "Logical 0"
    }
    SerialPort.SendLine("Phase lock established");
}

// function: Start rotation at RPM
void StartRotation(int32_t rpm){
    // Convert RPM to steps/sec: (RPM/60)*8000
    int32_t velocityStepsPerSec = (rpm * encoderResolution) / 60;

    SerialPort.Send("Starting synchronized rotation at RPM: ");
    SerialPort.SendLine(rpm);

    // Command the velocity move to all four motors synchronously
    for (int i = 0; i < 4; i++){
         motors[i]-> MoveVelocity(velocityStepsPerSec);
    }

    // Wait for the step command to ramp up to the commanded velocity
    SerialPort.SendLine("Ramping to speed...");
    uint32_t startTime = Milliseconds();

    bool allAtSpeed = false;
    while (!allAtSpeed) {
        allAtSpeed = true;

        for (int i = 0; i < 4; i++) {

            // Check for motor alerts
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor Alert During Ramping: Motor ");
                SerialPort.SendLine(i);
                StopAll();
                return;
            }

            // Check if motor reached cruise velocity
            if (!motors[i]->CruiseVelocityReached()) {
                allAtSpeed = false;
            }
        }

        // Time Out Protection: 15s
        if (Milliseconds() - startTime > 15000) {
            SerialPort.SendLine("Error: Motors failed to reach target speed!");
            StopAll();
            return;
        }

        Delay_ms(1); // reduce CPU usage
    }
    SerialPort.Send("All motors have hit the desired RPM:");
    SerialPort.SendLine(rpm);
}

// function: Stop all motors smoothly
void StopAll() {
    SerialPort.SendLine("Initiating controlled stop for all motors...");

    for (int i = 0; i < 4; i++) {
        // "MoveStopDecel": Interrupts any current move and commands the motor to stop. 
        motors[i]->MoveStopDecel(decelerationLimit);
    }

    // Wait for all motors to ramp down and reach zero velocity
    bool allStopped = false;

    while (!allStopped) {
        allStopped = true; 
        for (int i = 0; i < 4; i++) {
            // Check alerts during stopping
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor Alert While Stopping: Motor ");
                SerialPort.SendLine(i);
                return;
            }
            // Check if motor stops
            if (!motors[i]->StepsComplete()) {
                allStopped = false;
            }
        }
        Delay_ms(1);
    }

    SerialPort.SendLine("All motors have come to a stop.");
}

// Function: Smoothly decelerate from 2000 RPM to a specific absolute count
void MoveMotorsToTargets(int32_t targets[]) {
    SerialPort.SendLine("Moving motors to individual absolute targets...");

    for (int i = 0; i < 4; i++) {
        // Always check for alerts before commanding a move 
        if (motors[i]->StatusReg().bit.AlertsPresent) {
            SerialPort.Send("Motor "); 
            SerialPort.Send(i); 
            SerialPort.SendLine(" in alert. Skipping.");
            continue;
        }
        int32_t target_angle = (targets[i] * encoderResolution) / 360;
        // Use MOVE_TARGET_ABSOLUTE to move to the exact count relative to zero
        motors[i]->Move(target_angle, StepGenerator::MOVE_TARGET_ABSOLUTE);     // Need to convert targets degree to counts 
    }

    // Wait for all motors to physically arrive at their targets
    for (int i = 0; i < 4; i++) {
        while (!motors[i]->StepsComplete()) {
            continue;
        }
    }

    SerialPort.SendLine("All motors arrived at target counts.");
}

void EnableAllMotors(){
    SerialPort.SendLine("Enabling all motors...");
    // "EnableRequest": Request all motors to enable
    for (int i = 0; i < 4; i++){
        motors[i]->EnableRequest(true);
    }

    // Wait for all motors to physically energize and clear alerts
    bool allReady = false;
    while (!allReady) {
        allReady = true; 
        for (int i = 0; i < 4; i++) {
            // Check if HLFB is asserted (energized) and no alerts are present
            if (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED || 
                motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor Alert: ");
                SerialPort.SendLine(i);
                return;
            }
        }
    }

    // "ValidateMove": verify that the motor is in a good state before sending a move command 
    for (int i =0; i < 4; i++){
        if (!motors[i]->ValidateMove(false)){
            SerialPort.SendLine("Error: A motor failed validation after enabling");
            return;
        }
    }

    SerialPort.SendLine("All motors are enabled, and ready to go");
}

void DisableAllMotors() {
    SerialPort.SendLine("Disabling all motors...");

    // Check if all motors are stationary
    for (int i = 0; i < 4; i++) {
        // StepsComplete() returns true if the trajectory generator is idle 
        // VelocityRefCommanded() == 0 ensures the target speed is zero
        if (!motors[i]->StepsComplete() || motors[i]->VelocityRefCommanded() != 0) {
            SerialPort.SendLine("ERROR: Motion detected! Cannot disable motors while moving.");
            SerialPort.SendLine("Stop the moving motors first before disabling...");
            return; // Exit the function immediately
        }
    } 

    // Send the disable request to all motors
    for (int i = 0; i < 4; i++) {
        // Passing false de-asserts the Enable signal and removes coil power 
        motors[i]->EnableRequest(false);
    }

    // Wait for the hardware to confirm the motors are disabled
    bool allDisabled = false;
    while (!allDisabled) {
        allDisabled = true; 
        for (int i = 0; i < 4; i++) {
            // Check if internal hardware state is MOTOR_DISABLED 
            if (motors[i]->StatusReg().bit.MotorReadyState != MotorDriver::MOTOR_DISABLED) {
                allDisabled = false;
            }
        }
    }

    SerialPort.SendLine("All motors are disabled and de-energized.");
}

// HLFB mode needs to be set for "In Range-Velocity" on MSP for each motor 
bool allMotorsInRange(){
    for (int i = 0; i < 4; i++){
        // HlfbState() returns HLFB_ASSERTED when the motor is physically
        // within the MSP-defined speed window 
        if (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED){
            SerialPort.Send("Motor out of range: ");
            SerialPort.SendLine(i);
            return false; // At least one motor has drifted or is still ramping
        }
    }
    return true; // All motors are physically at 2000 RPM (within tolerance)
}

// HLFB mode needs to be set for "Pulses Per Revolution (PPR)" on MSP
// Set "4" pulses per revolution 
// void updatePPR() {
//     uint32_t currentTime = Microseconds(); // High-resolution timestamp

//     for (int i = 0; i < 4; i++) {
//         // HlfbHasRisen is "clear-on-read"; it returns true if a pulse started
//         if (motors[i]->HlfbHasRisen()) {
//             pulseCounts[i]++;

//             // Start of a new revolution measurement cycle
//             if (pulseCounts[i] == 1) {
//                 startTimes[i] = currentTime;
//                 // Record this for the global phase difference check
//                 firstPulseTimestamps[i] = currentTime; 
//             }

//             // After 4 pulses, one full revolution has been completed 
//             // We check for pulse #5 to get the exact duration of the 4-pulse gap
//             if (pulseCounts[i] >= 5) {
//                 uint32_t revolutionTime = currentTime - startTimes[i];
                
//                 // RPM formula: (60 seconds * 1,000,000 microseconds) / time per turn
//                 if (revolutionTime > 0) {
//                     motorRPMs[i] = 60000000.0 / revolutionTime;
//                 }

//                 // Reset for the next revolution
//                 pulseCounts[i] = 1; 
//                 startTimes[i] = currentTime;
//             }
//         }
//     }
// }

// void checkPhaseDifference(){
//     for (int i = 0; i < 4; i++){
//         // Calculate the raw timing offset in microseconds
//         int32_t diff = (int32_t)(firstPulseTimestamp[i] - firstPulseTimestamps);

//         SerialPort.Send()
//     }
// }

void setup() {
    // Set the input clocking rate for the MotorDriver connectors as a group, they cannot be individually set. 
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

    // Put all motors into Step and Direction mode. They cannot be individually set. 
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    for (int i = 0; i < 4; i++){
        motors[i]->AccelMax(accelerationLimit);
        motors[i]->VelMax(velocityMAX);

        // HLFB_Mode_Static is required for In-Range, ASG, and Servo On Modes
        motors[i]->HlfbMode(MotorDriver::HLFB_MODE_STATIC);
    }

    // Set the mode to USB Serial
    SerialPort.Mode(Connector::USB_CDC);
    // Set the communication speed (Baud Rate)
    SerialPort.Speed(baudRate); // 9600 or 115200 / 
    SerialPort.PortOpen();
    while(!SerialPort){
        continue;
    }
    SerialPort.SendLine("Commands: 'H'=Homing, 'P'=Phase, 'R'=Run, 'S'=Stop, 'G'= move to target location");
}

void loop() {
    // 1: Listen to Serial Commands to trigger functions
    if (SerialPort.Available() > 0){
        char cmd = SerialPort.Read();
        switch(cmd){
            case 'M': case 'm': 
                ManualSyncZero(); 
                break;
            case 'D': case 'd': 
                DisableAllMotors(); 
                break;
            case 'E': case 'e': 
                EnableAllMotors(); 
                break;
            case 'P': case 'p': 
                SetPhaseAngles(offsets); 
                break;
            case 'R': case 'r': 
                StartRotation(velocityRPM); 
                isRotating = true; // ramping is done
                break;
            case 'S': case 's': 
                StopAll(); 
                isRotating = false; // stationary
                break;
            case 'G': case 'g': 
                MoveMotorsToTargets(targets); 
                break;
        }
    }

    // 2: Continuous Monitoring (Only when motors are at the desired speed)
    if (isRotating){
        if (!allMotorsInRange()){
            SerialPort.SendLine("CRITICAL: Synchronization lost or motor stalled!");
            StopAll(); // safety action, stop the motors when they are not synchronizing 
            isRotating = false;
        }
    }
}

