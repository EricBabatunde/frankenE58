/**
 * ============================================================================
 *  Franken-E58 Micro-Quadcopter — Phase 1 Flight Core Bootstrap
 * ============================================================================
 *
 *  Target MCU  : ESP32-S3-DevKitC-1
 *  Framework   : ESP-IDF (native, C++ compilation unit)
 *  Sensor      : MPU-6050 (I2C @ 0x68)
 *  Bus Config  : 100 kHz — derated for ~20 cm jumper wire signal integrity
 *
 *  This file establishes:
 *    1. I2C master bus initialisation with defensive pull-up configuration.
 *    2. MPU-6050 power management wake sequence (register-level).
 *    3. A 400 Hz flight control loop pinned to Core 1 with a software
 *       logic analyser (SLA) timing harness for transaction profiling.
 *
 *  All operations are non-blocking; the main app_main thread yields
 *  immediately after task creation.
 *
 *  Author : Eric Babatunde
 *  Date   : 2026-07-11
 * ============================================================================
 */

// ── Compilation Guard ────────────────────────────────────────────────────────
#ifndef __cplusplus
#error "This translation unit requires a C++ compiler. Check framework config."
#endif

// ── ESP-IDF System Headers ───────────────────────────────────────────────────
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ── Standard Math Library ────────────────────────────────────────────────────
#include <math.h>

// ── Diagnostic Log Tag ───────────────────────────────────────────────────────
static const char *TAG = "FLIGHT_CORE";

// ============================================================================
//  I2C Bus Configuration Constants
// ============================================================================
//  CLK derated to 100 kHz: the E58 breadboard prototype uses ~20 cm dupont
//  jumper wires which introduce parasitic capacitance. 400 kHz fast-mode
//  will be re-evaluated once the PCB revision lands.
// ============================================================================
static constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_0;
static constexpr gpio_num_t SDA_IO = GPIO_NUM_8;
static constexpr gpio_num_t SCL_IO = GPIO_NUM_9;
static constexpr uint32_t CLK_SPEED_HZ = 100000; // 100 kHz

// ============================================================================
//  MPU-6050 Register Map (subset for Phase 1)
// ============================================================================
static constexpr uint8_t MPU6050_ADDR = 0x68;
static constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t MPU6050_REG_ACCEL_XOUT = 0x3B;

//  PWR_MGMT_1 payload: clear SLEEP bit, select X-axis gyroscope PLL as
//  clock source (CLKSEL = 0x01) for improved stability over the internal
//  RC oscillator.
static constexpr uint8_t MPU6050_CLK_SEL_X_GYRO = 0x01;

//  IMU burst-read length: 3×accel + temp + 3×gyro = 14 bytes
static constexpr size_t IMU_BURST_READ_LEN = 14;

// ============================================================================
//  IMU Scaling & Complementary Filter Constants
// ============================================================================
//  Default MPU-6050 full-scale ranges after reset:
//    Accelerometer: ±2g  → 16384 LSB/g
//    Gyroscope:     ±250°/s → 131 LSB/(°/s)
// ============================================================================
static constexpr float ACCEL_SCALE = 16384.0f;
static constexpr float GYRO_SCALE  = 131.0f;
static constexpr float RAD_TO_DEG  = 57.29577951f;  // 180.0 / π
static constexpr float ALPHA       = 0.98f;          // Gyro trust weight
static constexpr float DT          = 0.01f;          // 100 Hz → 10 ms step

// ============================================================================
//  I2C Master Bus Initialisation
// ============================================================================
/**
 * @brief Configure and install the I2C master driver on I2C_NUM_0.
 *
 * Internal pull-ups are enabled as a defensive fallback. The hardware
 * prototype carries external 4.7 kΩ pull-ups on SDA/SCL; the internal
 * weak pull-ups (~45 kΩ) act in parallel and have negligible effect on
 * rise-time when the external resistors are present.
 *
 * @return ESP_OK on success, or an esp_err_t error code on failure.
 */
static esp_err_t i2c_master_init(void)
{
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_IO,
        .scl_io_num = SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Internal pull-up backup
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Internal pull-up backup
        .master = {
            .clk_speed = CLK_SPEED_HZ,
        },
        .clk_flags = 0, // Default clock source
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "✗ I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "✓ I2C bus parameters configured  [SDA=%d SCL=%d @ %lu Hz]",
             SDA_IO, SCL_IO, (unsigned long)CLK_SPEED_HZ);

    err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "✗ I2C driver install failed (bus allocation): %s",
                 esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "✓ I2C master driver installed on port %d", I2C_MASTER_NUM);

    return ESP_OK;
}

// ============================================================================
//  MPU-6050 Wake Sequence
// ============================================================================
/**
 * @brief Write to PWR_MGMT_1 to bring the MPU-6050 out of default sleep mode.
 *
 * On power-on-reset the MPU-6050 enters SLEEP mode (bit 6 of 0x6B is set).
 * Writing 0x01 clears SLEEP and selects the X-axis gyroscope PLL as the
 * clock reference — more accurate than the 8 MHz internal oscillator.
 *
 * @return ESP_OK on success, or the I2C transaction error code.
 */
static esp_err_t mpu6050_wake(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, MPU6050_CLK_SEL_X_GYRO, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return err;
}

// ============================================================================
//  IMU Burst-Read (Mock Phase 1)
// ============================================================================
/**
 * @brief Perform a 14-byte burst read starting at register 0x3B.
 *
 * Reads accelerometer (6B), temperature (2B), and gyroscope (6B) data
 * in a single I2C transaction — the MPU-6050 auto-increments the register
 * pointer. In Phase 1 this data is captured but not yet fused; the primary
 * objective is to validate bus timing and transaction integrity.
 *
 * @param[out] buffer  Pre-allocated buffer of at least IMU_BURST_READ_LEN.
 * @return ESP_OK on success, or the I2C transaction error code.
 */
static esp_err_t mpu6050_burst_read(uint8_t *buffer)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Phase 1 — Write the starting register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_ACCEL_XOUT, true);

    // Phase 2 — Repeated-start and read 14 bytes
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, IMU_BURST_READ_LEN, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return err;
}

// ============================================================================
//  Flight Control Loop — Core 1 Task
// ============================================================================
/**
 * @brief Primary flight control loop executing at ~400 Hz on Core 1.
 *
 * Lifecycle:
 *   1. Wake the MPU-6050 — hard-fault on failure (safe infinite loop).
 *   2. Enter deterministic control loop with SLA timing harness.
 *   3. Each iteration: burst-read IMU → scale → trig → complementary filter.
 *
 * Sensor fusion pipeline (per iteration):
 *   raw int16 → float scaling → accel trig angles → complementary filter
 *   → fused pitch/roll angles logged at ESP_LOGI level.
 *
 * @param pvParameters  Unused; reserved for future config struct passthrough.
 */
static void flight_control_loop_task(void *pvParameters)
{
    (void)pvParameters; // Suppress unused-parameter warning

    ESP_LOGI(TAG, "━━━ Flight Control Task started on Core %d ━━━",
             xPortGetCoreID());

    // ── Step 1: Wake MPU-6050 ────────────────────────────────────────────
    ESP_LOGI(TAG, "⏳ Waking MPU-6050 (PWR_MGMT_1 ← 0x%02X)...",
             MPU6050_CLK_SEL_X_GYRO);

    esp_err_t wake_err = mpu6050_wake();
    if (wake_err != ESP_OK)
    {
        ESP_LOGE(TAG, "✗ CRITICAL: MPU-6050 wake FAILED: %s",
                 esp_err_to_name(wake_err));
        ESP_LOGE(TAG, "  → Entering safe infinite loop. Check I2C wiring.");
        ESP_LOGE(TAG, "  → Verify: SDA=%d, SCL=%d, VCC=3.3V, ADDR=0x%02X",
                 SDA_IO, SCL_IO, MPU6050_ADDR);

        // ── Safe halt: prevent downstream instability ────────────────────
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "✓ MPU-6050 awake — clock source: X-axis gyro PLL");
    ESP_LOGI(TAG, "✓ Entering 100 Hz flight control loop");

    // ── Step 2: Deterministic control loop ────────────────────────────────
    uint8_t imu_raw[IMU_BURST_READ_LEN] = {};
    uint32_t loop_count = 0;

    // ── Complementary Filter state ───────────────────────────────────────
    // Persistent across iterations; zeroed on boot — the filter self-
    // corrects within ~1 s via the accelerometer gravity reference.
    static float pitch_fused = 0.0f;
    static float roll_fused  = 0.0f;

    for (;;)
    {
        // ── SLA Timing Harness: capture pre-transaction timestamp ────────
        int64_t t_start = esp_timer_get_time(); // Microsecond precision

        // ── Burst-read 14 bytes of IMU telemetry from 0x3B ──────────────
        esp_err_t read_err = mpu6050_burst_read(imu_raw);

        // ── SLA Timing Harness: compute transaction delta ────────────────
        int64_t t_elapsed = esp_timer_get_time() - t_start;

        if (read_err != ESP_OK)
        {
            ESP_LOGW(TAG, "⚠ IMU read error @ loop %lu: %s",
                     (unsigned long)loop_count, esp_err_to_name(read_err));
        }
        else
        {
            // ── Parse 14-byte big-endian IMU payload into signed 16-bit ──
            // Register map from 0x3B: AX_H AX_L AY_H AY_L AZ_H AZ_L
            //                         TH   TL   GX_H GX_L GY_H GY_L GZ_H GZ_L
            int16_t accel_x  = (int16_t)((imu_raw[0]  << 8) | imu_raw[1]);
            int16_t accel_y  = (int16_t)((imu_raw[2]  << 8) | imu_raw[3]);
            int16_t accel_z  = (int16_t)((imu_raw[4]  << 8) | imu_raw[5]);
            int16_t temp_raw = (int16_t)((imu_raw[6]  << 8) | imu_raw[7]);
            int16_t gyro_x   = (int16_t)((imu_raw[8]  << 8) | imu_raw[9]);
            int16_t gyro_y   = (int16_t)((imu_raw[10] << 8) | imu_raw[11]);
            int16_t gyro_z   = (int16_t)((imu_raw[12] << 8) | imu_raw[13]);

            // Suppress unused-variable warnings — reserved for Phase 2
            (void)temp_raw;
            (void)gyro_z;

            // ── Scale raw integers to physical units ─────────────────────
            //  Accel: LSB → g-force    Gyro: LSB → °/s
            float accel_x_scaled = (float)accel_x / ACCEL_SCALE;
            float accel_y_scaled = (float)accel_y / ACCEL_SCALE;
            float accel_z_scaled = (float)accel_z / ACCEL_SCALE;
            float gyro_x_scaled  = (float)gyro_x  / GYRO_SCALE;
            float gyro_y_scaled  = (float)gyro_y  / GYRO_SCALE;

            // ── Accelerometer-only angle estimation (trig) ──────────────
            //  These angles are stable under static/slow motion but noisy
            //  under vibration — hence the complementary filter below.
            float pitch_acc = atan2f(accel_y_scaled,
                                     sqrtf(accel_x_scaled * accel_x_scaled +
                                           accel_z_scaled * accel_z_scaled))
                              * RAD_TO_DEG;

            float roll_acc  = atan2f(-accel_x_scaled, accel_z_scaled)
                              * RAD_TO_DEG;

            // ── Complementary Filter fusion ──────────────────────────────
            //  High-pass on gyro (fast, driftless short-term) blended with
            //  low-pass on accel (noisy but drift-free long-term).
            //  ALPHA = 0.98 → 98% gyro trust, 2% accel correction per step.
            pitch_fused = ALPHA * (pitch_fused + gyro_x_scaled * DT)
                        + (1.0f - ALPHA) * pitch_acc;

            roll_fused  = ALPHA * (roll_fused  + gyro_y_scaled * DT)
                        + (1.0f - ALPHA) * roll_acc;

            // ── Log fused angles + SLA timing ───────────────────────────
            ESP_LOGI(TAG,
                     "µs: %lld | Pitch: %.2f° | Roll: %.2f°",
                     (long long)t_elapsed,
                     pitch_fused, roll_fused);
        }

        loop_count++;

        // Temporary 100Hz delay to bypass FreeRTOS integer division crash during Phase 1 timing validation.
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Unreachable — flight loop never exits. Defensive cleanup.
    vTaskDelete(NULL);
}

// ============================================================================
//  Application Entry Point
// ============================================================================
/**
 * @brief ESP-IDF application entry point.
 *
 * Initialises the I2C master bus and spawns the flight control task on Core 1.
 * After task creation, app_main returns and its stack is reclaimed by the
 * idle task — all real-time work lives in the pinned FreeRTOS task.
 */
extern "C" void app_main(void)
{
    // Add a 2-second delay to allow the host PC to enumerate the USB port
    // and attach the PlatformIO serial monitor before we start logging.
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   Franken-E58  •  Phase 1 Flight Core Bootstrap ║");
    ESP_LOGI(TAG, "║   ESP32-S3  •  MPU-6050  •  ESP-IDF / FreeRTOS  ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════╝");

    // ── I2C Bus Initialisation ───────────────────────────────────────────
    ESP_LOGI(TAG, "⏳ Initialising I2C master bus...");
    esp_err_t i2c_err = i2c_master_init();
    if (i2c_err != ESP_OK)
    {
        ESP_LOGE(TAG, "✗ FATAL: I2C init failed — aborting task creation.");
        ESP_LOGE(TAG, "  → System halted. Diagnose bus hardware and reboot.");
        return; // app_main returns; system remains alive but inert.
    }
    ESP_LOGI(TAG, "✓ I2C master bus ready");

    // ── Spawn Flight Control Task on Core 1 ──────────────────────────────
    //  Core 0: WiFi/BT stack + system tasks (reserved for Phase 4 telemetry)
    //  Core 1: Real-time flight control (latency-critical)
    ESP_LOGI(TAG, "⏳ Spawning flight_control_loop_task → Core 1...");

    BaseType_t task_err = xTaskCreatePinnedToCore(
        flight_control_loop_task, // Task function
        "flight_ctrl",            // Human-readable name (max 16 chars)
        4096,                     // Stack depth in bytes
        NULL,                     // Parameters (unused in Phase 1)
        10,                       // Priority: high (above default 1)
        NULL,                     // Task handle (not needed yet)
        1                         // Pin to Core 1
    );

    if (task_err != pdPASS)
    {
        ESP_LOGE(TAG, "✗ FATAL: Failed to create flight control task!");
        return;
    }

    ESP_LOGI(TAG, "✓ Flight control task created [core=1 prio=10 stack=4096B]");
    ESP_LOGI(TAG, "━━━ app_main complete — yielding to scheduler ━━━");
}
