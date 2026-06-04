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
    TEST_ASSERT_EQUAL_FLOAT(3500.0f, pid.setpoint);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(-255.0f, pid.output_min);
    TEST_ASSERT_EQUAL_FLOAT(255.0f, pid.output_max);
}

static void test_pid_compute_positive_error(void) {
    pid_state_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f); // Apenas Proporcional
    
    // Setpoint 3500, Leitura 3000 -> Erro 500
    // Kp * Erro = 1.0 * 500 = 500 -> Clamped para 255 (max)
    float out = pid_compute(&pid, 3000.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(255.0f, out);
    
    // Leitura 3400 -> Erro 100 -> Saída 100
    out = pid_compute(&pid, 3400.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out);
}

static void test_pid_compute_negative_error(void) {
    pid_state_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f);
    
    // Setpoint 3500, Leitura 4000 -> Erro -500 -> Clamped para -255 (min)
    float out = pid_compute(&pid, 4000.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(-255.0f, out);
}

static void test_pid_anti_windup(void) {
    pid_state_t pid;
    pid_init(&pid, 0.0f, 10.0f, 0.0f); // Apenas Integral
    pid.integral_limit = 20.0f;
    
    // Erro 1000, dt 1s -> integral aumentaria em 1000
    // Mas o limite é 20, então integral fica preso em 20
    // Saída = Ki * integral = 10.0 * 20.0 = 200.0
    float out = pid_compute(&pid, 2500.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, out);
}

// ==========================================
// TESTES DO LINE FOLLOWER
// ==========================================
static void test_line_follower_center(void) {
    line_follower_init();
    
    qtr8rc_reading_t reading = {0};
    // Simula sensores 3 e 4 (centro) vendo preto perfeitamente
    reading.norm[3] = 1000;
    reading.norm[4] = 1000;
    
    line_status_t status = line_follower_compute(&reading);
    
    TEST_ASSERT_TRUE(status.line_detected);
    TEST_ASSERT_FALSE(status.all_white);
    TEST_ASSERT_FALSE(status.all_black);
    TEST_ASSERT_EQUAL_INT(2, status.black_count);
    
    // Posição ponderada: (1000*3000 + 1000*4000) / 2000 = 3500
    TEST_ASSERT_EQUAL_INT16(3500, status.position);
}

static void test_line_follower_extreme_left(void) {
    line_follower_init();
    
    qtr8rc_reading_t reading = {0};
    // Apenas sensor 0 (extrema esquerda) vê preto
    reading.norm[0] = 1000;
    
    line_status_t status = line_follower_compute(&reading);
    
    TEST_ASSERT_TRUE(status.line_detected);
    TEST_ASSERT_EQUAL_INT16(0, status.position);
}

static void test_line_follower_all_white(void) {
    line_follower_init();
    
    qtr8rc_reading_t reading = {0};
    // Todos abaixo do threshold de 600
    for(int i=0; i<8; i++) reading.norm[i] = 100;
    
    line_status_t status = line_follower_compute(&reading);
    
    TEST_ASSERT_FALSE(status.line_detected);
    TEST_ASSERT_TRUE(status.all_white);
    TEST_ASSERT_EQUAL_INT(0, status.black_count);
    // Deve manter a ultima posição válida (3500 do init)
    TEST_ASSERT_EQUAL_INT16(3500, status.position);
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
