#ifndef MODEM_MANAGER_H
#define MODEM_MANAGER_H

#include <TinyGsmClient.h>
#include "Logger.h"

// Reset states for the modem
enum ResetState {
  RESET_NONE,
  RESET_INIT,
  RESET_POWER_OFF,
  RESET_POWER_ON,
  RESET_INIT_AT,
  RESET_COMPLETE
};

class ModemManager {
public:
  ModemManager(TinyGsm& modem, Stream& serial);
  
  // Setup and initialization
  void begin();
  bool setup();
  
  // Connection management
  bool ensureConnection();
  bool isGprsConnected();
  
  // Reset handling
  void startReset();
  bool continueReset();
  
  // Status information
  int getSignalQuality();
  
private:
  TinyGsm& _modem;
  Stream& _serial;
  
  // Reset state tracking
  ResetState _resetState;
  unsigned long _resetStartTime;
  
  // Connection parameters
  const char* _apn;
  const char* _user;
  const char* _pass;
  
  // Private methods
  bool initModem();
  bool connectGprs();
};

#endif // MODEM_MANAGER_H