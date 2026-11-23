# robot_control_interfaces

Typowane wiadomości i usługi ręcznego sterowania wałami silników CAN 1–6.
`ManualState` zawiera sześć rekordów i wiek każdego pomiaru.
`SetMotorEnable`, `ZeroMotor`, `JogMotor` odrzucają przeterminowane żądania;
czas monotoniczny pochodzi z tego samego komputera, a nie z przeglądarki.
Szczegóły: [sterowanie ręczne](../../../docs/manual_control.md).

`JointConfiguration`/`RobotConfiguration` opisują konfigurację w SI.
`ConfigureRobot` obsługuje przygotowanie, commit, rollback, finish, status i odtworzenie
przy starcie. Żądania mają identyfikator i czas monotoniczny. `ConfigurationCommand`
przekazuje wersjonowany JSON do jedynego właściciela plików, `robot_configuration`.
`JogJoint` ma prędkość rad/s i rewizję; `JogMotor.rpm` nadal oznacza RPM wału.
`ManualState` udostępnia rewizję i obie przestrzenie pomiarów.
Opis: [konfiguracja](../../../docs/configuration.md).

`ConfigureRobot.confirm_calibration[6]` jawnie potwierdza odniesienie wskazanych
osi; domyślnie wszystkie wartości false. Odpowiedź `resolved_configuration`
zawiera profil po automatycznym przeliczeniu offsetu dla zmiany przełożenia
oraz flagi nieważnej kalibracji. Koordynator zapisuje ten profil zamiast żądania.
Wymagany spójny build klientów i serwera.
