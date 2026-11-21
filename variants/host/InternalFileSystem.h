// A host filesystem with the Adafruit_LittleFS surface MeshCore's nRF52 build
// expects.
//
// The repeater keeps its identity, its preferences and its client ACL in flash,
// and the CLI writes to them. Stubbing that out would give a node that forgets
// who it is on every restart and a CLI whose settings do nothing — so the files
// are real files, under a directory the simulator owns.
//
// nRF52 rather than the ESP32 branch because the board we also emulate is an
// nRF52840, and having both backends take the same path through MeshCore is
// the point of ADR-0010's cross-check.
#pragma once

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <string>

#include <Stream.h>

#include "lfs_shim.h"

#define FILE_O_READ "rb"
#define FILE_O_WRITE "wb"

namespace Adafruit_LittleFS_Namespace {

// Derived from Stream, as Arduino's File is. CommonCLI hands one to anything
// taking a Stream&, and Identity::readFrom/writeTo are declared on Stream — so
// this is not a convenience, it is the type relationship MeshCore is written
// against.
class File : public Stream {
 public:
  File() = default;
  File(FILE* f, const std::string& path, bool isDir = false)
      : f_(f), path_(path), isDir_(isDir) {}

  explicit operator bool() const { return f_ != nullptr || isDir_; }
  bool operator!() const { return f_ == nullptr && !isDir_; }

  int read() override {
    if (!f_) return -1;
    int c = fgetc(f_);
    return c == EOF ? -1 : c;
  }
  int peek() override { return -1; }
  void flush() override {
    if (f_) fflush(f_);
  }
  int available() override { return f_ ? (int)(size() - (uint32_t)ftell(f_)) : 0; }

  size_t readBytes(uint8_t* buf, size_t len) {
    if (!f_) return 0;
    return fread(buf, 1, len, f_);
  }
  int read(uint8_t* buf, size_t len) { return (int)readBytes(buf, len); }

  size_t write(const uint8_t* buf, size_t len) override {
    if (!f_) return 0;
    return fwrite(buf, 1, len, f_);
  }
  size_t write(uint8_t b) override { return write(&b, 1); }

  // The repeater writes its preferences and its ACL as text, so print and
  // printf are not conveniences here — they are how those files are produced.
  size_t print(const char* s) { return s ? write((const uint8_t*)s, strlen(s)) : 0; }
  size_t print(int v) { return printf("%d", v); }
  size_t println(const char* s) { return print(s) + print("\n"); }
  size_t printf(const char* fmt, ...) {
    if (!f_) return 0;
    va_list ap;
    va_start(ap, fmt);
    int n = vfprintf(f_, fmt, ap);
    va_end(ap);
    return n < 0 ? 0 : (size_t)n;
  }

  bool seek(uint32_t pos) { return f_ && fseek(f_, (long)pos, SEEK_SET) == 0; }
  uint32_t size() {
    if (!f_) return 0;
    long here = ftell(f_);
    fseek(f_, 0, SEEK_END);
    long end = ftell(f_);
    fseek(f_, here, SEEK_SET);
    return (uint32_t)end;
  }
  void close() {
    if (f_) {
      fclose(f_);
      f_ = nullptr;
    }
  }
  const char* name() const { return path_.c_str(); }

  bool isDirectory() {
    struct stat st {};
    return stat(path_.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
  }

  // Iterating a directory, one entry per call, as Adafruit_LittleFS does.
  // Hidden entries are skipped: "." and ".." would make a walk recurse forever,
  // and nothing MeshCore stores begins with a dot.
  File openNextFile(const char* mode = FILE_O_READ) {
    if (!dir_) dir_ = opendir(path_.c_str());
    if (!dir_) return File();
    while (struct dirent* e = readdir(dir_)) {
      if (e->d_name[0] == '.') continue;
      std::string child = path_ + "/" + e->d_name;
      struct stat st {};
      if (stat(child.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) return File(nullptr, child, true);
      return File(fopen(child.c_str(), mode), child);
    }
    closedir(dir_);
    dir_ = nullptr;
    return File();
  }

  void rewindDirectory() {
    if (dir_) {
      closedir(dir_);
      dir_ = nullptr;
    }
  }

 private:
  FILE* f_ = nullptr;
  std::string path_;
  bool isDir_ = false;
  DIR* dir_ = nullptr;
};

}  // namespace Adafruit_LittleFS_Namespace

using Adafruit_LittleFS_Namespace::File;

class Adafruit_LittleFS {
 public:
  // Root is set by the simulator so each node gets its own storage. Sharing one
  // directory between nodes would give every repeater the same identity, which
  // is not a mesh.
  void setRoot(const std::string& dir) { root_ = dir; ensureDir(root_); }

  bool begin() {
    ensureDir(root_);
    fs_.cfg = lfs_host_config();
    fs_.root = root_;
    return true;
  }

  // The companion reads block geometry off this to report free space.
  lfs* _getFS() {
    fs_.cfg = lfs_host_config();
    fs_.root = root_;
    return &fs_;
  }

  bool exists(const char* path) {
    struct stat st {};
    return stat(full(path).c_str(), &st) == 0;
  }
  bool mkdir(const char* path) { return ensureDir(full(path)); }
  bool remove(const char* path) { return ::remove(full(path).c_str()) == 0; }

  File open(const char* path, const char* mode = FILE_O_READ) {
    std::string p = full(path);
    // A write to a path whose directory does not exist is the commonest way
    // this fails, and it fails silently as a null file.
    auto slash = p.find_last_of('/');
    if (slash != std::string::npos) ensureDir(p.substr(0, slash));
    // FILE_O_WRITE as Adafruit_LittleFS opens it on nRF52: read-write,
    // created if missing, not truncated, positioned at the end. fopen's "wb"
    // emptied the file, and putBlobByKey and openAppend rely on keeping it.
    FILE* f;
    if (strcmp(mode, FILE_O_WRITE) == 0) {
      f = fopen(p.c_str(), "r+b");
      if (!f) f = fopen(p.c_str(), "w+b");
      if (f) fseek(f, 0, SEEK_END);
    } else {
      f = fopen(p.c_str(), mode);
    }
    return File(f, p);
  }

  bool format() {
    // Deliberately not a recursive delete of a caller-supplied path. Erasing a
    // filesystem is a real command in the CLI, and a bug here would remove
    // something outside the node's own directory.
    if (root_.empty()) return false;
    return true;
  }

 private:
  std::string full(const char* path) const {
    std::string p = path ? path : "";
    if (!p.empty() && p[0] == '/') p.erase(0, 1);
    return root_.empty() ? p : root_ + "/" + p;
  }
  // Windows' mkdir takes a path and nothing else - no mode - so calling the
  // POSIX form fails to compile there rather than at runtime. One wrapper
  // rather than an #ifdef at the call site, because there will be more calls.
  static int hostMkdir(const char* path) {
#ifdef _WIN32
    return ::mkdir(path);
#else
    return ::mkdir(path, 0755);
#endif
  }

  static bool ensureDir(const std::string& dir) {
    if (dir.empty()) return true;
    std::string acc;
    for (size_t i = 0; i <= dir.size(); i++) {
      if (i == dir.size() || dir[i] == '/') {
        if (!acc.empty()) hostMkdir(acc.c_str());
      }
      if (i < dir.size()) acc += dir[i];
    }
    return true;
  }
  std::string root_;
  lfs fs_{};
};

extern Adafruit_LittleFS InternalFS;
