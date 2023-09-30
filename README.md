# motorcontroller-01
UART peripheral dual PWM PID controller.

Expects a 4-byte command over UART every 500ms or goes into standby mode.

Controls two encoded brushed-DC motors, setting their speed and direction mapped to the range [-1000,1000] as 16-bit integer.

Uses a PI pid to seek the target with a 100ms udpate window. 100ms update was chosen because at most ~240 quadrature pulses can be counted per update. 10ms update would be at most 24, so I wanted better fidelity over responsiveness.

Designed for STM32L432KC, L298 H-bridge, and goBILDA yellowjacket DC motors (223 RPM variant).
