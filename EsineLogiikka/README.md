This sketch reads several environmental sensors and (e.g. humidity+thermometer or light meter) interrupts
and controls a short digital RGB LED strip based on readings.

Optionally data is sent to an MQTT broker to be saved or shown in other visualizations.

Note: you must create settings.h with correct credentials:

`cp settings-example.h settings.h  # then edit settings.h`

