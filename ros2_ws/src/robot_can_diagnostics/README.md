# robot_can_diagnostics

Węzeł C++ udostępnia diagnostykę SocketCAN, skan identyfikacji i statusy MKS.
Ten sam runtime udostępnia usługi ręcznego ENABLE, zerowania i JOG; sekwencje
realizuje osobny `ManualMotorController` w bibliotece CAN. Po starcie nie są
wysyłane polecenia ruchu ani konfiguracji. Opisy: `docs/can_diagnostics.md`
i `docs/manual_control.md`. Nie uruchamiaj drugiego właściciela CAN przy pluginie.
