# robot_control_interfaces

Typowane wiadomości i usługi ręcznego sterowania wałami silników CAN 1–6.
`ManualState` zawiera sześć rekordów i wiek każdego pomiaru.
`SetMotorEnable`, `ZeroMotor`, `JogMotor` odrzucają przeterminowane żądania;
czas monotoniczny pochodzi z tego samego komputera, a nie z przeglądarki.
Szczegóły: [sterowanie ręczne](../../../docs/manual_control.md).
