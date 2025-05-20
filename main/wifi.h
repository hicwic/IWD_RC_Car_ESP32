#pragma once

#include <string>

namespace Wifi {

/// Initialise l'ESP32 en mode Point d'Accès (SoftAP)
void startSoftAP(const std::string& ssid, const std::string& password = "");

/// Initialise l'ESP32 en mode Station (client)
void startStation(const std::string& ssid, const std::string& password);

}