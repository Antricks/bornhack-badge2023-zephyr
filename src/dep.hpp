#pragma once

#include "nci.hpp"

// Theoretically it could be nice to also abstract away transport here
// but honestly I don't know if that's maybe a bit too much...
class Nci;
class Dep {
  public:
    Dep(Nci *nci);
    virtual ~Dep();

    virtual int handle_apdu(const uint8_t *buf, size_t buf_len) = 0;
  protected:
    Nci *nci;
};
