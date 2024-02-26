#include "iso-dep.h"
#include "dep.h"
#include "nci.h"

IsoDep::IsoDep(const Nci &nci) : Dep(&nci) {}
IsoDep::~IsoDep() {}
