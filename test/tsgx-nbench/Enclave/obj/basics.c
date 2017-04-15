#include <stdio.h>
#include <stdarg.h>

#include "Enclave_t.h"
#include "basics.h"

void puts(const char *str) {
  ocall_puts(str);
}

void printf(const char *fmt,...) {
  char buf[BUFSIZ];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, BUFSIZ, fmt, ap);
  va_end(ap);
  ocall_print_string(buf);
}

unsigned long clock() {
  unsigned long ticks;
  ocall_get_clock(&ticks);

  return ticks;
}
