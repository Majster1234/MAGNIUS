# Czas SYNC od początku; przygotowanie rozpoczyna się jeden cykl wcześniej.
float64 time_from_start
int32[6] position_ticks
uint16[6] speed_rpm
uint8[6] acceleration_code
# 0 = utrzymanie celu, 1 = aktualizacja celu. Nie kodujemy STOP przez speed=0.
uint8[6] operation
uint8[48] payloads
