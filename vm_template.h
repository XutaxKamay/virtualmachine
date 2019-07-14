#ifndef VM_TEMPLATE_H
#define VM_TEMPLATE_H

#include <cstdint>
#include <cstddef>
#include <cassert>
#include <iostream>
#include <type_traits>
#include <cstring>

namespace vm
{
    constexpr auto default_ram_size = 1 << 24;
    constexpr auto default_stack_size = 1 << 20;

    typedef void* ptr_t;
    typedef uint8_t byte_t;
    typedef byte_t* array_t;

    // Type wrapper
    template <typename type_t>
    struct type_wrapper_t
    {
        using type = type_t;
    };

    // Get inline variable from type wrapper.
    template <typename type_t>
    inline constexpr type_wrapper_t<type_t> get_type {};

    typedef enum
    {
        // Mathematical expressions.
        reg_add,
        reg_remove,
        reg_divide,
        reg_multiply,
        // Conditions.
        reg_condition_lower,
        reg_condition_higher,
        reg_condition_equal,
        reg_condition_or,
        reg_condition_and,
        // Bitwise operators
        reg_shift_left,
        reg_shift_right,
        // Stock & erase registers values to the stack
        reg_push_8,
        reg_push_16,
        reg_push_32,
        reg_push_64,
        reg_push_pointer,
        reg_pop_8,
        reg_pop_16,
        reg_pop_32,
        reg_pop_64,
        reg_pop_pointer,
        // Call instruction.
        reg_call,
        // Read & write function to the "usable memory"
        reg_read,
        reg_write
    } instructions_t;

    // Let's create some registers type for each data types.
    typedef enum
    {
        reg_float,
        reg_double,
        reg_pointer
    } registers_type_t;

    // Get register size from an enum.
    template <registers_type_t reg>
    inline constexpr auto _rt()
    {
        if constexpr (reg == reg_float)
        {
            return get_type<float>;
        }
        else if constexpr (reg == reg_double)
        {
            return get_type<double>;
        }
        else if constexpr (reg == reg_pointer)
        {
            return get_type<uintptr_t>;
        }

        static_assert("Unknown register size");
    };

    template <registers_type_t reg>
    using rt = typename decltype(_rt<reg>())::type;
    template <registers_type_t reg>
    using rt = rt<reg>;

    template <typename T>
    inline constexpr auto _register_nb()
    {
        if constexpr (std::is_same<T, rt<reg_float>>::value)
        {
            return reg_float;
        }
        else if constexpr (std::is_same<T, rt<reg_double>>::value)
        {
            return reg_double;
        }
        else if constexpr (std::is_same<T, rt<reg_pointer>>::value)
        {
            return reg_pointer;
        }

        static_assert("Unknown register type");
    }

    template <typename T>
    inline constexpr auto register_nb = _register_nb<T>();

    // Program sections.
    typedef struct
    {
        uintptr_t m_pBegin;
        size_t m_size;
    } section_t;

    typedef enum
    {
        section_code,
        section_data,
        section_max
    } sections_nb_t;

    // Program header.
    class ProgramHeader
    {
     public:
        ProgramHeader();

        template <sections_nb_t section_nb>
        section_t* getSection();

        // Entry point of the program.
        ptr_t m_entryPoint;
        // Sections.
        section_t* m_sections[section_max];
    };

    // Program execution.
    class Program
    {
     public:
        Program(array_t* binaryStub);
        ptr_t getEntryPoint();

        // Header.
        ProgramHeader* m_header;
    };

    // Write program template to write programs...
    template <typename ret_t, typename... args_t>
    class ProgramWrite : public Program
    {
     public:
        using entrypoint_t = ret_t (*)(args_t...);

        ProgramWrite(size_t maxSize) : m_writeSize(0), m_maxSize(maxSize)
        {
            for (auto section = 0; section < section_max; section++)
            {
                m_bSectionComplete[section] = false;
            }
        }

        void processSectionEP()
        {}

        size_t m_writeSize;
        size_t m_maxSize;
        bool m_bSectionComplete[section_max];
    };

    template <auto ram_size = default_ram_size>
    class VirtualMachine
    {
     public:
        VirtualMachine() : m_pUsableMemory(0), m_bPaused(true)
        {
            init();
        }

        auto init()
        {
            // Calculate the stack size, we will assume that it is only 1/8 of
            // the RAM.
            constexpr auto stack_size = 8 / ram_size;
            // Reset cpu.
            memset(&m_CPU, 0, sizeof(cpu_registers_t));

            // Setup stack pointers.
            m_CPU.reg_bp = reinterpret_cast<rt<reg_pointer>>(m_RAM);
            m_CPU.reg_cp = m_CPU.reg_bp;

            // Usable memory setup.
            m_pUsableMemory = reinterpret_cast<uintptr_t>(m_RAM) + stack_size;
        }

        auto runProgram(Program* program)
        {
            init();

            m_bPaused = false;

            m_CPU.reg_ip = m_pUsableMemory + reinterpret_cast<uintptr_t>(
                                                 program->getEntryPoint());

            auto programCode = program->m_header->getSection<section_code>();

            auto imageBase = reinterpret_cast<uintptr_t>(program->m_header);

            m_sectionCode.m_pBegin = m_pUsableMemory + programCode->m_pBegin;
            m_sectionCode.m_size = programCode->m_size;

            auto programData = program->m_header->getSection<section_data>();

            m_sectionData.m_pBegin = m_pUsableMemory + programData->m_pBegin;
            m_sectionData.m_size = programData->m_size;

            // Copy sections to RAM.
            memcpy(reinterpret_cast<ptr_t>(imageBase + programCode->m_pBegin),
                   reinterpret_cast<ptr_t>(m_sectionCode.m_pBegin),
                   m_sectionCode.m_size);

            memcpy(reinterpret_cast<ptr_t>(imageBase + programCode->m_pBegin),
                   reinterpret_cast<ptr_t>(m_sectionData.m_pBegin),
                   m_sectionData.m_size);

            run();
        }

        auto run()
        {}

        auto checkStack()
        {
            if (m_CPU.reg_cp >= m_pUsableMemory ||
                m_CPU.reg_bp >= m_pUsableMemory)
            {
                assert("Exceeding the stack.");
            }
        }

        auto checkCode()
        {}

        // Number of storage registers.
        typedef enum
        {
            // Temporary registers.
            register_storage_temp1,
            register_storage_temp2,
            register_storage_temp3,
            register_storage_temp4,
            register_storage_temp5,
            register_storage_temp6,
            register_storage_temp7,
            register_storage_temp8,
            // Registers for reading to usable memory.
            register_storage_read,
            // Registers for writing to usable memory.
            register_storage_write,
            num_of_storage_registers

        } register_storage_t;

        // The virtual machine registers.
        typedef struct
        {
            // Register of return value.
            rt<reg_pointer> reg_ret;
            // Base stack pointer.
            rt<reg_pointer> reg_bp;
            // Current stack pointer.
            rt<reg_pointer> reg_cp;
            // Current instruction pointer.
            rt<reg_pointer> reg_ip;
            // Random registers for storage.
            rt<reg_pointer> reg_strg[num_of_storage_registers];
            // Registers for mathematical expression.
            rt<reg_double> reg_dbl[num_of_storage_registers];
            rt<reg_float> reg_flt[num_of_storage_registers];
            // Flag is used to know if previous condition was true or false.
            bool m_bFlag;

        } cpu_registers_t;

        // CPU Registers.
        cpu_registers_t m_CPU;
        // RAM.
        byte_t m_RAM[ram_size];
        // Usable memory is after the stack,
        // so we won't write to the stack directly.
        // Usable memory = RAM + stack_size.
        uintptr_t m_pUsableMemory;
        // Code in RAM.
        section_t m_sectionCode;
        // Data in RAM.
        section_t m_sectionData;
        bool m_bPaused;
    };

};

#endif // VM_TEMPLATE_H
