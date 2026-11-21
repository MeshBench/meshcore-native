// FILE_O_WRITE must open a file as Adafruit_LittleFS does on nRF52.
//
// There it is LFS_O_RDWR | LFS_O_CREAT, then a seek to the end: the file keeps
// what it holds, can be read back, and a write with no seek lands at the end.
// MeshCore leans on all three. DataStore::putBlobByKey reads the advert store
// through a FILE_O_WRITE handle and overwrites one record in place, and the
// repeater's and room server's openAppend adds a line to the packet log. A host
// open that truncated would keep one advert and one log line, and nothing would
// say so.
//
//   c++ -std=c++17 -I variants/host variants/host/filesystem_test.cpp -o /tmp/filesystem_test
//   /tmp/filesystem_test

#include "InternalFileSystem.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

static int failures = 0;

static void expect(bool ok, const char* what) {
  if (!ok) {
    printf("FAIL %s\n", what);
    failures++;
  }
}

static std::string contents(Adafruit_LittleFS& fs, const char* path) {
  File f = fs.open(path, FILE_O_READ);
  std::string out;
  for (int c = f.read(); c >= 0; c = f.read()) out += (char)c;
  f.close();
  return out;
}

int main() {
  char dir[] = "/tmp/filesystem_test.XXXXXX";
  if (!mkdtemp(dir)) {
    perror("mkdtemp");
    return 2;
  }
  Adafruit_LittleFS fs;
  fs.setRoot(dir);

  // A missing file is created.
  File f = fs.open("/blobs", FILE_O_WRITE);
  expect((bool)f, "a missing file opens for writing");
  f.write((const uint8_t*)"AAAABBBBCCCC", 12);
  f.close();
  expect(contents(fs, "/blobs") == "AAAABBBBCCCC", "a new file holds what was written");

  // An existing file keeps its contents, and opens at its end.
  f = fs.open("/blobs", FILE_O_WRITE);
  expect(f.size() == 12, "an existing file keeps its contents");
  f.write((const uint8_t*)"DDDD", 4);
  f.close();
  expect(contents(fs, "/blobs") == "AAAABBBBCCCCDDDD", "a write with no seek lands at the end");

  // It can be read, then overwritten in place, as putBlobByKey does.
  f = fs.open("/blobs", FILE_O_WRITE);
  uint8_t rec[4] = {0};
  f.seek(4);
  expect(f.read(rec, 4) == 4 && memcmp(rec, "BBBB", 4) == 0, "a FILE_O_WRITE handle reads");
  f.seek(4);
  f.write((const uint8_t*)"XXXX", 4);
  f.close();
  expect(contents(fs, "/blobs") == "AAAAXXXXCCCCDDDD", "a record is overwritten in place");

  // A file removed first is written whole, as every other writer does.
  fs.remove("/blobs");
  f = fs.open("/blobs", FILE_O_WRITE);
  f.write((const uint8_t*)"EE", 2);
  f.close();
  expect(contents(fs, "/blobs") == "EE", "remove then open writes a file afresh");

  fs.remove("/blobs");
  rmdir(dir);
  if (failures) return 1;
  printf("ok\n");
  return 0;
}
