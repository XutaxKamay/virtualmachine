#include "vm_template.h"

using namespace vm;

auto main() -> int
{
   VirtualMachine<default_ram_size> vm;

   vm.setIP(vm.m_pUsableMemory);
   vm.run();

    return 0;
}
