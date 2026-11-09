# Jeden ruch ciągu (ComputeMotionChain). SI: rad, rad/s, rad/s², m, m/s, m/s².
uint8 PTP=0
uint8 LIN=1
uint8 motion_type
float64[6] target_positions
# Limity osi tego ruchu (> 0, nie większe niż context.max_velocity / max_acceleration).
float64[6] max_joint_velocity
float64[6] max_joint_acceleration
# LIN: dodatnie limity TCP; PTP: trzy zera.
float64 tcp_velocity
float64 tcp_acceleration
float64 tcp_angular_velocity
# Promień pominięcia punktu docelowego na torze TCP [m]: 0 = dojazd i postój (v=0), > 0 = płynne przejście
# do następnego ruchu w tej odległości od punktu (ostatni ruch ciągu zawsze kończy się postojem).
float64 blend_radius
# Nieprzezroczysty identyfikator (id bloku programu).
string tag
