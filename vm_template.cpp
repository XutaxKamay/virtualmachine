#include "vm_template.h"

namespace vm
{
    template <size_t ram_size, size_t stack_size>
    VirtualMachine<ram_size, stack_size>::VirtualMachine()
    {
        // Reset cpu.
        memset(&m_CPURegs, 0, sizeof(cpu_registers_t));
        // Setup stack pointers.
        m_CPURegs.reg_bp = static_cast<register_type<reg_pointer_modifiable>>(
            m_Stack);
        m_CPURegs.reg_cp = m_CPURegs.reg_bp;
    }
};
