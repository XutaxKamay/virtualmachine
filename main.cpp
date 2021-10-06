#include "vm_template.h"

using namespace vm;

auto main() -> int
{
    VirtualMachine<default_ram_size> vm;

    std::vector<byte_t> instructions;
    // Add test.
    instructions.push_back(inst_add);
    instructions.push_back(register_storage_temp1);
    instructions.push_back(slot_1_int);
    instructions.push_back(op_value);
    instructions.push_back(cast_32);

    std::vector<byte_t> int_value;
    int_value.resize(sizeof(int));
    *reinterpret_cast<int*>(int_value.data()) = 1337;
    instructions.insert(instructions.end(), int_value.begin(), int_value.end());

    instructions.push_back(inst_push);
    instructions.push_back(op_value);
    instructions.push_back(cast_8);
    instructions.push_back(0x13);

    instructions.push_back(inst_pop);
    instructions.push_back(register_storage_temp2);
    instructions.push_back(slot_1_byte);

    // Set return value.
    instructions.push_back(inst_set_ret);
    instructions.push_back(op_register);
    instructions.push_back(register_storage_temp1);
    instructions.push_back(slot_1_int);
    instructions.push_back(inst_exit);

    std::memcpy(reinterpret_cast<ptr_t>(vm.m_pUsableMemory),
                instructions.data(),
                instructions.size());

    vm.setIP(vm.m_pUsableMemory);
    vm.run();

    if (vm.m_CPU.regs[register_ret].i[0] == 1337)
    {
        std::cout << "Very good mister Freeman" << std::endl;
        std::cout << static_cast<int>(
                         vm.m_CPU.regs[register_storage_temp2].b[0])
                  << std::endl;
    }

    return 0;
}
