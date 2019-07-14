#include "vm_template.h"

namespace vm
{
    template <size_t ram_size>
    VirtualMachine<ram_size>::VirtualMachine()
    {
        // Calculate the stack size, we will assume that it is only 1/8 of the
        // RAM.
        constexpr auto stack_size = 8 / ram_size;
        // Reset cpu.
        memset(&m_CPURegs, 0, sizeof(cpu_registers_t));

        // Setup stack pointers.
        m_CPURegs.reg_bp = static_cast<uintptr_t>(m_RAM);
        m_CPURegs.reg_cp = m_CPURegs.reg_bp;

        // Usable memory setup.
        m_ptrUsableMemory = static_cast<uintptr_t>(m_RAM) + stack_size;
    }

    template <size_t ram_size>
    void VirtualMachine<ram_size>::checkStack()
    {
        if (m_CPURegs.reg_cp >= m_ptrUsableMemory ||
            m_CPURegs.reg_bp >= m_ptrUsableMemory)
        {
            assert("Exceeding the stack.");
        }
    }
};
