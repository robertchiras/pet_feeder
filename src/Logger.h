#ifndef _LOGGING_H_
#define _LOGGING_H_

#include "Arduino.h"

#pragma once

#define LOG_LEVEL 2
#define RAM_LOG
#define LOG_BUF_SIZE 4096
#define LOG(lvl, ...) Logger::log(lvl, __VA_ARGS__)

class Logger {
  private:
    char logBuf[LOG_BUF_SIZE];
    char *logP = logBuf;
    
  protected:
    static Logger *instance;
    Logger() { memset(logBuf, 0, sizeof(logBuf)); }
    void doLog(u8 lvl, char *text, u16 len);
   
  public:
    Logger(Logger &other) = delete;
    void operator=(const Logger &) = delete;
    
    static Logger *getInstance() {
      if(instance == NULL){
        instance = new Logger();
      }
      return instance;
    }
    
    static void log(u8 lvl, const char *format, ...);
    static String getLog();
};

#endif
