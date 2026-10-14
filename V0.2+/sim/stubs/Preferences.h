// Заглушка NVS: хранение в памяти процесса.
#pragma once
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>
extern std::map<std::string, std::vector<uint8_t>> simNvs;
extern int simNvsWrites;
class Preferences {
 public:
  bool begin(const char *, bool) { return true; }
  void end() {}
  size_t getBytes(const char *k, void *buf, size_t n) {
    auto it = simNvs.find(k);
    if (it == simNvs.end() || it->second.size() > n) return 0;
    memcpy(buf, it->second.data(), it->second.size());
    return it->second.size();
  }
  size_t putBytes(const char *k, const void *buf, size_t n) {
    simNvs[k].assign((const uint8_t *)buf, (const uint8_t *)buf + n);
    simNvsWrites++;
    return n;
  }
  uint8_t getUChar(const char *k, uint8_t d) { uint8_t v = d; getBytes(k, &v, 1); return v; }
  size_t putUChar(const char *k, uint8_t v) { return putBytes(k, &v, 1); }
  int32_t getInt(const char *k, int32_t d) { int32_t v = d; getBytes(k, &v, 4); return v; }
  size_t putInt(const char *k, int32_t v) { return putBytes(k, &v, 4); }
};
