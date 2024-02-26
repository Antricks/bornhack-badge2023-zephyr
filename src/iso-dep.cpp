#include "iso-dep.hpp"
#include "dep.hpp"

IsoDep::IsoDep(const Nci &nci) : Dep(nci) {}
IsoDep::~IsoDep() {}

int IsoDep::handle_package(const uint8_t *buf, size_t buf_len) {
    //TODO implement
    return 0;
}

