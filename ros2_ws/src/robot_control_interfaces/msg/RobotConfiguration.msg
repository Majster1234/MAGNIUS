string revision
JointConfiguration[6] axes
bool[6] changed
bool[6] calibration_invalid
float64 tcp_jog_max_linear_velocity
float64 tcp_jog_max_angular_velocity
float64 tcp_jog_linear_acceleration
float64 tcp_jog_angular_acceleration
bool disable_enable_on_critical_fault true
string ptp_lin_dispatch_mode "streamed"
# Człon całkujący celu F5 w PTP/LIN i JOG TCP (korekcja stałego uchybu napędów).
bool position_integral false
# Minimalna prędkość F5 / dolna granica ditheringu [RPM wału] w PTP/LIN i JOG TCP; efektywnie >= 1 jednostka MKS.
float64 min_dither_rpm 0.125
# Kod przyspieszenia MKS (acc) w ramkach F5 PTP/LIN i JOG TCP, 1–255 dla wszystkich osi;
# 0 = bez nadpisania (execution_acceleration_codes z parametrów wykonania).
uint8 f5_acceleration_code 0
