# Czasy ruchu w ciągu [s od startu planu]. Odcinki [start_time, end_time] i [end_time, transition_end]
# kolejnych ruchów pokrywają oś czasu planu bez luk.
uint32 segment
string tag
# Wyjście ze strefy przejścia poprzedniego ruchu albo start z postoju.
float64 start_time
# Wejście w strefę przejścia do następnego ruchu albo dojazd do punktu postoju.
float64 end_time
# Koniec strefy przejścia / koniec postoju (= start_time następnego ruchu).
float64 transition_end
bool stop
float64 requested_blend_radius
# Po ograniczeniu (połowa długości odcinków itp.); 0, gdy przejście zastąpiono postojem.
float64 applied_blend_radius
