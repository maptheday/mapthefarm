#include "FlightControllerProgram.hpp"
#include "platform/ThreadLauncher.hpp"

// EspLauncher is only compiled in when targeting the ESP32-S3.
// Add -DESP_BUILD to build_flags in your platformio.ini esp32s3 env.
#ifdef ESP_BUILD
#include "platform/EspLauncher.hpp"
#endif

// ============================================================
//  main()
//
//  The only job of main() is to:
//    1. Pick the right launcher for the current platform.
//    2. Hand it to FlightControllerProgram.
//
//  FlightControllerProgram itself has no platform knowledge.
// ============================================================
int main() {
    FlightControllerProgram program;
    program.setup();

#ifdef ESP_BUILD
    EspLauncher launcher;
#else
    ThreadLauncher launcher;
#endif

    program.runDualCore(launcher);

    std::cout << "\n[SYSTEM] Simulation complete.\n";
    return 0;
}