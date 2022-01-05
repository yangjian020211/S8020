#include "inter_core.h"
#include "debuglog.h"
#include "interrupt.h"
#include "string.h"
#include "ar8020.h"
#include "cpu_info.h"


void InterCore_CopyConfigureFormFlashToSRAM(void)
{
    ENUM_CPU_ID cpuid = CPUINFO_GetLocalCpuId();
    DLOG_Warning("cpuid=%d", cpuid);
}

