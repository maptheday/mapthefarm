#include "FlightControllerProgram.hpp"
#include "platform/ThreadLauncher.hpp"
#include "platform/ThreadMutex.hpp"
#include "config/DroneConfig.hpp"
#include "drivers/SimulatedBarometer.hpp"  // Fallback for simulation

#ifdef ON_REAL_HARDWARE
#include "platform/EspLauncher.hpp"
#include "platform/EspMutex.hpp"
#include "drivers/EspESC.hpp"
#include "drivers/EspIMU.hpp"
#include "drivers/EspBarometer.hpp"  // Real BMP280 driver
#endif

// ── Global state for dual-core execution on hardware ─────────────────
// These need to be global so Arduino setup()/loop() can access them
static FlightControllerProgram* g_program = nullptr;
static IPlatformLauncher*       g_launcher = nullptr;

#ifdef ON_REAL_HARDWARE
// ── Arduino setup() function (called once on ESP32 startup) ──────────
void setup() {
    // Configure drone physical properties
    static const DroneConfig config(
        0.650,   // 650g all-up weight
        0.120,   // 120mm centre-to-motor (5-inch quad)
        9.0      // ~900g thrust per motor in Newtons
    );

    // Initialize hardware drivers
    static EspMutex    physics_mx;
    static EspMutex    ipc_mx;
    static EspLauncher launcher;
    static EspESC      esc;           // DShot600 on GPIO 4/5/6/7 via RMT
    static EspIMU      imu;           // MPU6050 on I2C (SDA=GPIO8, SCL=GPIO9)
    static EspBarometer baro;         // BMP280 on I2C (SDA=GPIO8, SCL=GPIO9)

    // Create flight controller with real hardware
    static FlightControllerProgram program(config, physics_mx, ipc_mx, esc, imu, baro);

    // Store pointers for use in loop()
    g_program = &program;
    g_launcher = &launcher;

    // Run initial setup
    program.setup();

    // Launch dual-core execution
    program.runDualCore(launcher);
}

// ── Arduino loop() function (called repeatedly on ESP32) ──────────────
// On real hardware with dual-core, the flight logic runs on cores 0 & 1
// via FreeRTOS. This loop() becomes essentially a no-op.
void loop() {
    // The actual flight control runs on cores 0 & 1 via FreeRTOS tasks.
    // This loop just yields to let the OS scheduler run other tasks.
    vTaskDelay(pdMS_TO_TICKS(100));  // Yield 100ms
}

#else
// ── Native simulation main() function ──────────────────────────────
int main() {
    // Configure drone physical properties
    DroneConfig config(
        0.650,   // 650g all-up weight
        0.120,   // 120mm centre-to-motor (5-inch quad)
        9.0      // ~900g thrust per motor in Newtons
    );

    // ── Simulation: Fake Hardware ──────────────────────────────
    ThreadMutex    physics_mx;
    ThreadMutex    ipc_mx;
    ThreadLauncher launcher;
    // SimulatedESC/SimulatedIMU/SimulatedBarometer created inside 
    // FlightControllerProgram for the native build

    FlightControllerProgram program(config, physics_mx, ipc_mx);

    program.setup();
    program.runDualCore(launcher);

    std::cout << "\n[SYSTEM] Simulation complete.\n";
    return 0;
}
#endif
