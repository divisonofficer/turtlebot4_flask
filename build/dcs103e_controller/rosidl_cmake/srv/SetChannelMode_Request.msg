# Service to set channel mode
int32 channel         # Channel number (0, 1, 2)
string mode          # "continuous" or "pulsed"
float64 current      # Current value
float64 pulse_width  # Pulse width (for pulsed mode only)
float64 pulse_delay  # Pulse delay (for pulsed mode only)
