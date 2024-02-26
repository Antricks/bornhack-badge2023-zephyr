#pragma once
#include "nci.hpp"

class Dep {
  public:
    Dep(const Nci &nci);
    virtual ~Dep();

    virtual int handle_package(const uint8_t *buf, size_t buf_len);
  protected:
    const Nci &nci;
};
