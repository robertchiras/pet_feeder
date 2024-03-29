#include "Logger.h"

Logger* Logger::instance;

void Logger::doLog(u8 lvl, char *text, u16 len) {
    Serial.print(text);
#ifdef RAM_LOG    
    u16 remaining = LOG_BUF_SIZE - (logP - logBuf);
    if (remaining < len) {
      logP--;
      strncpy(logP, text, remaining);
      logP += remaining;
      *logP = '\0';
      logP = logBuf;
      strncpy(logP, text + remaining, len - remaining);
      logP += len - remaining;
      *logP = '\0';
      logP++;
    } else {
      if (logP != logBuf)
        logP--;
      strncpy(logP, text, len);
      logP += len;
      *logP = '\0';
      logP++;
    }
#endif
}

void Logger::log(u8 lvl, const char *format, ...) {
  if (lvl > LOG_LEVEL)
    return;
    
  char tmp[128];
  va_list args;

  va_start(args, format);
  u16 len = vsprintf(tmp, format, args);
  va_end(args);
  Logger *logger = getInstance();
  logger->doLog(lvl, tmp, len);
}

String Logger::getLog() {
  Logger *logger = getInstance();
  return String(logger->logP) + String(logger->logBuf);
}
