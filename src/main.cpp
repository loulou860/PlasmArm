/**
 * @file main.cpp
 * @brief ESP32 SCARA Robot Controller - Main Entry Point
 * 
 * This file sets up FreeRTOS tasks, queues, and hardware initialization
 * for the 2-DOF SCARA robotic arm controller.
 * 
 * Core Assignment:
 * - Core 0: Web Server, Trajectory Planner
 * - Core 1: Real-Time Motion Control
 */

 //utiliser deux platformIO, communication serial entre les deux
 //openRB project: lit commandes d'angles et les vitesses (driver servo 12V)
 //esp32 project: calcule les angles et les vitesses des moteurs pour atteindre une position donnée
 

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <queue>  // For std::queue in planner task
#include "Config.h"
#include "core/Types.h"
#include "core/Kinematics.h"
#include "core/Planner.h"
#include "hardware/IMotor.h"
#include "hardware/StepperMotor.h"
#include "hardware/ServoMotor.h"
#include "web/WebServer.h"

// Test mode includes
#if RUN_UNIT_TESTS || RUN_VISUAL_TESTS || RUN_INTERACTIVE_TEST
#include "test/RunTests.h"
#endif

// ============================================================================
// Global Objects
// ============================================================================
Kinematics kinematics(ARM_LENGTH_1, ARM_LENGTH_2);
Planner planner(DEFAULT_SPEED, ACCELERATION);
RobotState robotState;

// Motor instances (using StepperMotor by default, can be changed to ServoMotor)
IMotor* motor1 = nullptr;
IMotor* motor2 = nullptr;

WebServer webServer;

// ============================================================================
// FreeRTOS Queues
// ============================================================================
QueueHandle_t commandQueue;   // Commands from web interface
QueueHandle_t motionQueue;    // Interpolated motion points

// ============================================================================
// FreeRTOS Task Handles
// ============================================================================
TaskHandle_t taskWebHandlerHandle = nullptr;
TaskHandle_t taskPlannerHandle = nullptr;
TaskHandle_t taskMotionControlHandle = nullptr;

// ============================================================================
// Task Function Prototypes
// ============================================================================
void taskWebHandler(void* parameter);
void taskTrajectoryPlanner(void* parameter);
void taskMotionControl(void* parameter);

// ============================================================================
// Setup Function
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    
    #if RUN_INTERACTIVE_TEST
    // Run interactive integration test with real motors
    runInteractiveTest();
    return;  // Exit setup - don't initialize robot
    #endif
    
    #if RUN_VISUAL_TESTS
    // Run visual tests only (detailed output)
    runVisualTestsOnly();
    return;  // Exit setup - don't initialize robot
    #endif
    
    #if RUN_UNIT_TESTS
    // Run unit tests instead of normal operation
    runAllUnitTests();
    return;  // Exit setup - don't initialize robot
    #endif

    Serial.println("\n\n========================================");
    Serial.println("ESP32 SCARA Robot Controller");
    Serial.println("========================================\n");
    
    // Initialize hardware
    Serial.println("Initializing hardware...");
    
    // Create motor instances (StepperMotor implementation)
    motor1 = new StepperMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, MOTOR1_ENABLE_PIN);
    motor2 = new StepperMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, MOTOR2_ENABLE_PIN);
    
    // Initialize motors
    motor1->init();
    motor2->init();
    motor1->enable();
    motor2->enable();
    
    Serial.println("Motors initialized");
    
    // Create FreeRTOS queues
    commandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(Command));
    motionQueue = xQueueCreate(MOTION_QUEUE_SIZE, sizeof(Point2D));
    
    
    Serial.println("Queues created");
    
    // Connect to WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        Serial.print(".");
        retry++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        
        // Initialize web server
        webServer.init(commandQueue);
        webServer.begin();
    } else {
        Serial.println("\nWiFi connection failed!");
        Serial.println("Continuing without web interface...");
    }
    
    // Create FreeRTOS tasks
    
    // Task 1: Web Handler (Core 0, Low Priority)
    xTaskCreatePinnedToCore(
        taskWebHandler,
        "WebHandler",
        TASK_WEB_HANDLER_STACK_SIZE,
        nullptr,
        TASK_WEB_HANDLER_PRIORITY,
        &taskWebHandlerHandle,
        0  // Core 0
    );
    
    // Task 2: Trajectory Planner (Core 0, Medium Priority)
    xTaskCreatePinnedToCore(
        taskTrajectoryPlanner,
        "Planner",
        TASK_PLANNER_STACK_SIZE,
        nullptr,
        TASK_PLANNER_PRIORITY,
        &taskPlannerHandle,
        0  // Core 0
    );
    
    // Task 3: Motion Control (Core 1, High Priority)
    xTaskCreatePinnedToCore(
        taskMotionControl,
        "MotionControl",
        TASK_MOTION_CONTROL_STACK_SIZE,
        nullptr,
        TASK_MOTION_CONTROL_PRIORITY,
        &taskMotionControlHandle,
        1  // Core 1
    );
    
    Serial.println("\nFreeRTOS tasks created:");
    Serial.println("  - WebHandler (Core 0, Priority 1)");
    Serial.println("  - Planner (Core 0, Priority 2)");
    Serial.println("  - MotionControl (Core 1, Priority 3)");
    Serial.println("\nSystem ready!\n");
}

// ============================================================================
// Loop Function (not used in FreeRTOS, but kept for compatibility)
// ============================================================================
void loop() {
    // FreeRTOS tasks handle everything
    // This loop can be used for low-priority maintenance tasks
    webServer.cleanup();
    delay(1000);
}

// ============================================================================
// Task Implementations
// ============================================================================

/**
 * Task A: Web Handler
 * Core: 0
 * Priority: Low
 * 
 * Handles HTTP requests and WebSockets.
 * Parses incoming commands and pushes them to commandQueue.
 */
void taskWebHandler(void* parameter) {
    Serial.println("Task WebHandler started on Core 0");
    
    while (true) {
        // Web server handles requests asynchronously
        // This task mainly serves as a placeholder for future synchronous operations
        
        // Cleanup WebSocket clients periodically
        webServer.cleanup();
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms delay
    }
}

/**
 * Task B: Trajectory Planner
 * Core: 0
 * Priority: Medium
 * 
 * Blocks waiting for commands in commandQueue.
 * Calculates linear interpolation and pushes Point2D to motionQueue.
 */
void taskTrajectoryPlanner(void* parameter) {
    Serial.println("Task Planner started on Core 0");
    
    Command cmd;
    Point2D currentPos = robotState.currentPosition;
    
    while (true) {
        // Wait for command from queue (blocking)
        if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            Serial.printf("Planner: Received command type %d\n", cmd.type);
            
            switch (cmd.type) {
                case Command::MOVE_TO: {
                    // Plan path from current position to target
                    Point2D target = cmd.target;
                    
                    // Check if target is reachable
                    if (!kinematics.isReachable(target)) {
                        Serial.printf("Planner: Target (%.2f, %.2f) is unreachable!\n",
                                     target.x, target.y);
                        break;
                    }
                    
                    // Set planner speed if provided
                    if (cmd.speed > 0) {
                        planner.setSpeed(cmd.speed);
                    }
                    
                    // Generate interpolated points
                    std::queue<Point2D> localQueue;
                    int numPoints = planner.planPath(currentPos, target, localQueue);
                    
                    Serial.printf("Planner: Generated %d points\n", numPoints);
                    
                    // Push all points to motion queue
                    while (!localQueue.empty()) {
                        Point2D point = localQueue.front();
                        localQueue.pop();
                        
                        if (xQueueSend(motionQueue, &point, pdMS_TO_TICKS(100)) != pdTRUE) {
                            Serial.println("Planner: WARNING - Motion queue full!");
                        }
                    }
                    
                    // Update current position
                    currentPos = target;
                    robotState.currentPosition = target;
                    break;
                }
                
                case Command::HOME: {
                    // Home sequence: move to (0, 0) or predefined home position
                    Point2D homePos(0, 0);
                    std::queue<Point2D> localQueue;
                    planner.planPath(currentPos, homePos, localQueue);
                    
                    while (!localQueue.empty()) {
                        Point2D point = localQueue.front();
                        localQueue.pop();
                        xQueueSend(motionQueue, &point, portMAX_DELAY);
                    }
                    
                    currentPos = homePos;
                    robotState.currentPosition = homePos;
                    robotState.isHomed = true;
                    Serial.println("Planner: Homing sequence completed");
                    break;
                }
                
                case Command::STOP: {
                    // Clear motion queue
                    xQueueReset(motionQueue);
                    robotState.isMoving = false;
                    Serial.println("Planner: Emergency stop!");
                    break;
                }
                
                case Command::SET_SPEED: {
                    planner.setSpeed(cmd.speed);
                    Serial.printf("Planner: Speed set to %.2f mm/s\n", cmd.speed);
                    break;
                }
                
                default:
                    Serial.printf("Planner: Unknown command type %d\n", cmd.type);
                    break;
            }
        }
    }
}

/**
 * Task C: Motion Control (Critical Loop)
 * Core: 1
 * Priority: High (Real-time)
 * Frequency: 100 Hz (10ms loop)
 * 
 * Pops points from motionQueue, calculates inverse kinematics,
 * and commands motors to move.
 */
void taskMotionControl(void* parameter) {
    Serial.println("Task MotionControl started on Core 1");
    
    const TickType_t loopDelay = pdMS_TO_TICKS(1000 / MOTION_CONTROL_FREQUENCY);
    Point2D targetPoint;
    JointAngles targetAngles;
    
    while (true) {
        TickType_t lastWakeTime = xTaskGetTickCount();
        
        // Check motion queue
        if (xQueueReceive(motionQueue, &targetPoint, 0) == pdTRUE) {
            // New target point received
            robotState.isMoving = true;
            
            // Calculate inverse kinematics
            if (kinematics.inverse(targetPoint, targetAngles)) {
                // Command motors to move to target angles
                motor1->moveToAngle(targetAngles.theta1);
                motor2->moveToAngle(targetAngles.theta2);
                
                robotState.currentAngles = targetAngles;
                robotState.currentPosition = targetPoint;
                
                #if DEBUG_MOTOR
                Serial.printf("Motion: Target (%.2f, %.2f) -> θ1=%.2f°, θ2=%.2f°\n",
                             targetPoint.x, targetPoint.y,
                             targetAngles.theta1, targetAngles.theta2);
                #endif
            } else {
                Serial.printf("Motion: IK failed for (%.2f, %.2f)\n",
                             targetPoint.x, targetPoint.y);
            }
        } else {
            // No new target, maintain current position
            robotState.isMoving = (motor1->isMoving() || motor2->isMoving());
        }
        
        // Update motors (generate steps for steppers, update PWM for servos)
        motor1->update();
        motor2->update();
        
        // Broadcast status periodically (every 10 loops = 1 second at 100Hz)
        static int statusCounter = 0;
        if (++statusCounter >= 10) {
            statusCounter = 0;
            webServer.broadcastStatus(robotState);
        }
        
        // Fixed frequency loop
        vTaskDelayUntil(&lastWakeTime, loopDelay);
    }
}

// ============================================================================
// Notes on Stack Size Tuning
// ============================================================================
/*
 * Stack Size Tuning Guidelines:
 * 
 * 1. Start with the default values in Config.h
 * 2. Monitor stack usage using FreeRTOS functions:
 *    - uxTaskGetStackHighWaterMark() - returns minimum free stack
 *    - Add this to each task to monitor:
 *      UBaseType_t stack = uxTaskGetStackHighWaterMark(nullptr);
 *      Serial.printf("Task stack free: %d bytes\n", stack * sizeof(StackType_t));
 * 
 * 3. If stack overflow occurs (watchdog reset or crash):
 *    - Increase stack size in Config.h
 *    - Common causes: large local arrays, deep recursion, large strings
 * 
 * 4. Typical stack sizes:
 *    - WebHandler: 8-16 KB (handles HTTP/WebSocket buffers)
 *    - Planner: 4-8 KB (queue operations, math calculations)
 *    - MotionControl: 4-8 KB (motor updates, kinematics)
 * 
 * 5. ESP32 has ~520 KB total RAM, so be mindful of total stack usage
 */
