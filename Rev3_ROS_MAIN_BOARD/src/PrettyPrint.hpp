#pragma once

#define SOUT(str) Serial.println((str))
#define CONSOLE(str) SOUT(("CONSOLE : " + String(str)))
#define DEBUG(str) SOUT(("DEBUG : " + String(str)))
#define ERROR(str) SOUT(("ERROR : " + String(str)))
#define SUCCESS(str) SOUT(("SUCCESS : " + String(str)))
#define INIT(str) SOUT(("INIT : " + String(str)))
#define INITED(str) SOUT(("INITED : " + String(str)))
