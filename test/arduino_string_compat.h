#ifndef ARDUINO_STRING_COMPAT_H
#define ARDUINO_STRING_COMPAT_H

// Minimal Arduino String shim for host-side unit testing.
// Implements only what buildUdpPayload() needs.

#include <string>
#include <cstdio>

class String {
public:
  std::string buf;

  String() : buf() {}
  String(const char* s) : buf(s) {}
  String(const String& s) : buf(s.buf) {}

  String(int val) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d", val);
    buf = tmp;
  }

  String(float val) {
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%.2f", val);
    buf = tmp;
  }

  String operator+(const String& rhs) const {
    String result;
    result.buf = buf + rhs.buf;
    return result;
  }

  String operator+(const char* rhs) const {
    String result;
    result.buf = buf + rhs;
    return result;
  }

  const char* c_str() const { return buf.c_str(); }

  int indexOf(const char* s) const {
    size_t pos = buf.find(s);
    return (pos == std::string::npos) ? -1 : (int)pos;
  }
};

#endif // ARDUINO_STRING_COMPAT_H
