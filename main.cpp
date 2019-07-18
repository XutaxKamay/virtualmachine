#include "vm_template.h"

using namespace vm;

auto main() -> int
{
    VirtualMachine<default_ram_size> vm;

    std::vector<byte_t> instructions = {inst_add,
                                        register_storage_temp1,
                                        slot_1_byte,
                                        op_value,
                                        cast_8,
                                        0x45,
                                        inst_set_ret,
                                        register_storage_temp1,
                                        cast_8,
                                        0x1,
                                        inst_exit};

    std::memcpy(reinterpret_cast<ptr_t>(vm.m_pUsableMemory),
                instructions.data(),
                instructions.size());

    vm.setIP(vm.m_pUsableMemory);
    vm.run();

    return 0;
}
