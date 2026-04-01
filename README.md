# GFS_2026_Physik

Das ziell ist ein BEschleunigungs senor zu bauen welcher aus zwei mess strategien besteht:
1x DIY -> Messen der Beschleunigung mithilfe eines HX711 und einer Wägezelle 
1x IMU -> Messen der Beschleunigung mithilfe der internen IMU des Arduino Rp2040 Connnect

verwendete Elektronik:
Arduino RP2040 Connect (als Sensor der Beschleunigung)
Raspberry pi Pico (als zentraler Controller)
HX711 (angeschlossen an Pico (GPIO 4/5))
Arduino und Pico sind per 2 Kabel verbindung (D18/D19 -> GPIO2/GPIO3) verbunden
Ich will keine I2C verbindung nutzen.
Es funktioniert alles auser die verbindung zwischen Arduino zu Pico verbindung.
Die verbindung soll dazu dienen Die Beschleunigungsdaten an den Pico zu übertragen. 
Der Pico ließt also quasi die Imu über den arduino aus.

Meine Idee :
selbst erstellte Communikation mithilfe von den 2 digitalen leitungen.
Diese soll wie ein Morse code fungieren und sich an einer seriellen verbindung orientieren (möglichst einfach.)
es sollen Zahlen von -100 bis +100 auf 4 nachkommastellen genau übertragen werden ansonsten keine daten.
Die daten sollen dauerhaft gesendet werden und der Raspberry versucht sich aufzu syncronisieren. Die 2te leitung soll als calibration flag dienen. sobald hier ein pulls kommt soll sich der Raspberry calibrieren.
achte auf eine Schnelle und sichere daten übertragung ( unter 4ms pro nachricht ) die daten sollen mindestens alle 20 ms gesendet werden.

Erstlle eine solche kommunikation als librarie für mein projekt.
