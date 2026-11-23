# Jeden ruch ciągu (ComputeMotionChain). SI: rad, rad/s, rad/s², m, m/s, m/s².
uint8 PTP=0
uint8 LIN=1
# CIRC: łuk okręgu TCP przez punkt pośredni (via_positions) do celu; orientacja interpolowana start→cel.
uint8 CIRC=2
# Orientacja CIRC: interpolacja start→cel (slerp wg kąta łuku) albo stała względem środka okręgu — orientacja
# obraca się z łukiem wokół osi okręgu (różnica do orientacji celu interpolowana w obracającym się układzie).
uint8 CIRC_INTERPOLATED=0
uint8 CIRC_PATH=1
uint8 motion_type
float64[6] target_positions
# CIRC: punkt pośredni łuku w przegubach (liczy się tylko jego położenie TCP); PTP/LIN: ignorowane.
float64[6] via_positions
uint8 circ_orientation
# Limity osi tego ruchu (> 0, nie większe niż context.max_velocity / max_acceleration).
float64[6] max_joint_velocity
float64[6] max_joint_acceleration
# LIN/CIRC: dodatnie limity TCP; PTP: trzy zera.
float64 tcp_velocity
float64 tcp_acceleration
float64 tcp_angular_velocity
# LIN/CIRC: limit przyspieszenia kątowego TCP [rad/s²] (konfiguracja tcp.angular_acceleration); 0 = bez limitu. PTP: 0.
float64 tcp_angular_acceleration
# Promień pominięcia punktu docelowego na torze TCP [m]: 0 = dojazd i postój (v=0), > 0 = płynne przejście
# do następnego ruchu w tej odległości od punktu (ostatni ruch ciągu zawsze kończy się postojem).
float64 blend_radius
# Nieprzezroczysty identyfikator (id bloku programu).
string tag
