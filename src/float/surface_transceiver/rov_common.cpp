#include <stdarg.h>
#include <Arduino.h>

void printfln(Stream *serialPointer, const char* input...) {
  va_list args;
  va_start(args, input);
  for (const char* c = input; *c != 0; c++) {
    if (*c != '%') {
        serialPointer->print(*c);
        continue;
    }
    c++;
    switch (*c) {
      case '%': serialPointer->print('%'); break;
      case 's': serialPointer->print(va_arg(args, char*)); break;
      case 'y': serialPointer->print(va_arg(args, uint8_t), DEC); break;
      case 'd': serialPointer->print(va_arg(args, int), DEC); break;
      case 'b': serialPointer->print(va_arg(args, int), BIN); break;
      case 'o': serialPointer->print(va_arg(args, int), OCT); break;
      case 'x': serialPointer->print(va_arg(args, int), HEX); break;
      case 'f': serialPointer->print(va_arg(args, double), 2); break;
    }
  }
  serialPointer->println();
  va_end(args);
}
