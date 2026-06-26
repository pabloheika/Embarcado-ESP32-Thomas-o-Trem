#include "test_modules.h"
#include "unity.h"
#include "pid_controller.h"
#include "line_follower.h"
#include "esp_log.h"

static const char* TAG = "test_unit";

// ==========================================
// TESTES DO PID CONTROLLER
// ==========================================
static void test_pid_initialization(void) {
    pid_state_t pid;
    pid_init(&pid, 0.5f, 0.1f, 0.2f);
    
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, pid.kd);
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, pid.setpoint);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(-255.0f, pid.output_min);
    TEST_ASSERT_EQUAL_FLOAT(255.0f, pid.output_max);
}

static void test_pid_compute_positive_error(void) {
    pid_state_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f); // Apenas Proporcional
    
    // Setpoint 1000, Leitura 0 -> Erro 1000
    // Kp * Erro = 1.0 * 1000 = 1000 -> Clamped para 255 (max)
    float out = pid_compute(&pid, 0.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(255.0f, out);
    
    // Leitura 900 -> Erro 100 -> Saída 100
    out = pid_compute(&pid, 900.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out);
}

static void test_pid_compute_negative_error(void) {
    pid_state_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f);
    
    // Setpoint 1000, Leitura 2000 -> Erro -1000 -> Clamped para -255 (min)
    float out = pid_compute(&pid, 2000.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(-255.0f, out);
}

static void test_pid_anti_windup(void) {
    pid_state_t pid;
    pid_init(&pid, 0.0f, 10.0f, 0.0f); // Apenas Integral
    pid.integral_limit = 20.0f;
    
    // Erro 1000, dt 1s -> integral aumentaria em 1000
    // Mas o limite é 20, então integral fica preso em 20
    // Saída = Ki * integral = 10.0 * 20.0 = 200.0
    float out = pid_compute(&pid, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, out);
}

// ==========================================
// TESTES DO LINE FOLLOWER (3 sensores E18-D80NK)
// ==========================================
static void test_line_follower_center(void) {
    line_follower_init();
    
    ir_sensor_reading_t reading = {0};
    // Simula apenas sensor central detectando a linha
    reading.detected[IR_SENSOR_CENTER] = true;
    reading.black_mask = (1 << IR_SENSOR_CENTER);
    reading.line_detected = true;
    
    line_status_t status = line_follower_compute(&reading);
    
    TEST_ASSERT_TRUE(status.line_detected);
    TEST_ASSERT_FALSE(status.all_white);
    TEST_ASSERT_FALSE(status.all_black);
    TEST_ASSERT_EQUAL_INT(1, status.black_count);
    
    // Posição: sensor central → 1000
    TEST_ASSERT_EQUAL_INT16(1000, status.position);
}

static void test_line_follower_extreme_left(void) {
    line_follower_init();
    
    ir_sensor_reading_t reading = {0};
    // Apenas sensor esquerdo detecta a linha
    reading.detected[IR_SENSOR_LEFT] = true;
    reading.black_mask = (1 << IR_SENSOR_LEFT);
    reading.line_detected = true;
    
    line_status_t status = line_follower_compute(&reading);
    
    TEST_ASSERT_TRUE(status.line_detected);
    TEST_ASSERT_EQUAL_INT16(0, status.position);
}

static void test_line_follower_all_white(void) {
    line_follower_init();
    
    ir_sensor_reading_t reading = {0};
    // Nenhum sensor detecta — tudo branco
    reading.line_detected = false;
    
    line_status_t status = line_follower_compute(&reading);
    
    TEST_ASSERT_FALSE(status.line_detected);
    TEST_ASSERT_TRUE(status.all_white);
    TEST_ASSERT_EQUAL_INT(0, status.black_count);
    // Deve manter a última posição válida (1000 do init)
    TEST_ASSERT_EQUAL_INT16(1000, status.position);
}

void run_all_unit_tests(void) {
    ESP_LOGI(TAG, "Iniciando Testes Unitários via Unity");
    
    UNITY_BEGIN();
    
    RUN_TEST(test_pid_initialization);
    RUN_TEST(test_pid_compute_positive_error);
    RUN_TEST(test_pid_compute_negative_error);
    RUN_TEST(test_pid_anti_windup);
    
    RUN_TEST(test_line_follower_center);
    RUN_TEST(test_line_follower_extreme_left);
    RUN_TEST(test_line_follower_all_white);
    
    UNITY_END();
}
