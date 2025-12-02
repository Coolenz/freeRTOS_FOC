// FreeRTOS双路FOC驱动优化
// 双核并行计算版本

#include "DengFOC.h"
#include <Arduino.h>

// 电机配置
const int M0_PP = 7;
const int M0_DIR = -1;
const int M1_PP = 7;
const int M1_DIR = -1;

extern MotorFOC motor0;
extern MotorFOC motor1;

// FreeRTOS任务句柄
TaskHandle_t motor0_task;
TaskHandle_t motor1_task;
TaskHandle_t serial_task;

// 共享数据保护
SemaphoreHandle_t target_mutex;
volatile float motor0_target = 0.0;
volatile float motor1_target = 0.0;

// Motor0控制任务 (Core0)
void motor0_control_task(void *parameter) {
    Serial.print("Motor0 task on core: ");
    Serial.println(xPortGetCoreID());
    
    while(1) {
        float target;
        xSemaphoreTake(target_mutex, portMAX_DELAY);
        target = motor0_target;
        xSemaphoreGive(target_mutex);
        
        motor0.loopFOC(target);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Motor1控制任务 (Core1)
void motor1_control_task(void *parameter) {
    Serial.print("Motor1 task on core: ");
    Serial.println(xPortGetCoreID());
    
    while(1) {
        float target;
        xSemaphoreTake(target_mutex, portMAX_DELAY);
        target = motor1_target;
        xSemaphoreGive(target_mutex);
        
        motor1.loopFOC(target);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 串口通信任务 (Core1)
void serial_communication_task(void *parameter) {
    Serial.print("Serial task on core: ");
    Serial.println(xPortGetCoreID());
    
    while(1) {
        serialReceiveUserCommand();
        
        xSemaphoreTake(target_mutex, portMAX_DELAY);
        motor0_target = serial_motor0_target();
        motor1_target = serial_motor1_target();
        xSemaphoreGive(target_mutex);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    Serial.println("=== FreeRTOS双路FOC启动 ===");
    Serial.print("主程序核心: ");
    Serial.println(xPortGetCoreID());
    
    // 初始化硬件
    DFOC_Init(12.6);
    motor0.init();
    motor1.init();
    
    // 传感器对齐
    motor0.alignSensor(M0_PP, M0_DIR);
    motor1.alignSensor(M1_PP, M1_DIR);
    
    // 创建互斥锁
    target_mutex = xSemaphoreCreateMutex();
    
    // 创建任务
    xTaskCreatePinnedToCore(motor0_control_task, "Motor0", 8192, NULL, 3, &motor0_task, 0);
    xTaskCreatePinnedToCore(motor1_control_task, "Motor1", 8192, NULL, 3, &motor1_task, 1);
    xTaskCreatePinnedToCore(serial_communication_task, "Serial", 4096, NULL, 2, &serial_task, 1);
    
    Serial.println("FreeRTOS任务创建完成");
}

void loop() {
    // 系统监控和绘图
    static unsigned long last_plot = 0;
    if(millis() - last_plot >= 100) {
        last_plot = millis();
        
        Serial.print("M0_A:");
        Serial.print(motor0.getAngle());
        Serial.print(", M0_T:");
        Serial.print(motor0_target);
        Serial.print(" | M1_A:");
        Serial.print(motor1.getAngle());
        Serial.print(", M1_T:");
        Serial.println(motor1_target);
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
}