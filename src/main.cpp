#include "FlightControllerProgram.hpp"
#include "platform/ThreadLauncher.hpp"
#include "platform/ThreadMutex.hpp"
#include "config/DroneConfig.hpp"

#ifdef ESP_BUILD
#include "platform/EspLauncher.hpp"
#include "platform/EspMutex.hpp"
#include "drivers/EspESC.hpp"
#endif

int main() {

    // ── Configure your drone's physical properties ────────────
    DroneConfig config(
        0.650,   // 650g all-up weight
        0.120,   // 120mm centre-to-motor (5-inch quad)
        9.0      // ~900g thrust per motor in Newtons
    );

#ifdef ESP_BUILD
    EspMutex    physics_mx;
    EspMutex    ipc_mx;
    EspLauncher launcher;
    EspESC      esc;           // real PWM on GPIO 4/5/6/7
#else
    ThreadMutex    physics_mx;
    ThreadMutex    ipc_mx;
    ThreadLauncher launcher;
    // SimulatedESC is created inside FlightControllerProgram
    // for the native build — no esc object needed here
#endif

#ifdef ESP_BUILD
    FlightControllerProgram program(config, physics_mx, ipc_mx, esc);
#else
    FlightControllerProgram program(config, physics_mx, ipc_mx);
#endif

    program.setup();
    program.runDualCore(launcher);

    std::cout << "\n[SYSTEM] Simulation complete.\n";
    return 0;
}