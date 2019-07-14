#ifndef VM_TEMPLATE_H
#define VM_TEMPLATE_H

#include <stdint.h>
#include <stddef.h>

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
        // Stock & remove variables to the stack
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
        // Read & write function
        reg_read,
        reg_write
    } instructions_t;

    // Let's create some registers type for each data types.
    typedef enum
    {
        reg_8,
        reg_16,
        reg_32,
        reg_64,
        reg_float,
        reg_double,
        reg_pointer
    } registers_type_t;

    // Get register size from an enum.
    template <registers_type_t reg>
    inline constexpr auto _register_type()
    {
        if constexpr (reg == reg_8)
        {
            return get_type<uint8_t>;
        }
        else if constexpr (reg == reg_16)
        {
            return get_type<uint16_t>;
        }
        else if constexpr (reg == reg_32)
        {
            return get_type<uint32_t>;
        }
        else if constexpr (reg == reg_64)
        {
            return get_type<uint64_t>;
        }
        else if constexpr (reg == reg_float)
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

        static_assert("Unknown type of register size");
    };

    template <registers_type_t reg>
    using register_type = typename decltype(_register_type<reg>())::type;

    // Program sections.
    typedef struct
    {
        uintptr_t m_ptrBegin;
        uintptr_t m_ptrEnd;
    } program_section_t;

    // Program header.
    template <typename ret_t, typename... args_t>
    class ProgramHeader
    {
     public:
        typedef ret_t (*entrypoint_t)(args_t...);

        ProgramHeader(uintptr_t entryPoint);

        enum
        {
            section_code,
            section_data,
            section_max
        };

        // Entry point of the program.
        entrypoint_t m_entryPoint;
        program_section_t m_sections[section_max];
    };

    template <typename ret_t, typename... args_t>
    class Program
    {
     public:
        Program(array_t* bin, uintptr_t entryPoint);

        void parseHeader();
        void callEntryPoint();

        ProgramHeader<ret_t, args_t...> m_header;
        array_t* instructions;
    };

    template <size_t ram_size = default_ram_size,
              size_t stack_size = default_stack_size>
    class VirtualMachine
    {
     public:
        VirtualMachine();

        // Number of storage registers.
        typedef enum
        {
            register_storage_first,
            register_storage_second,
            num_of_storage_registers

        } register_storage_t;

        // The virtual machine registers.
        typedef struct
        {
            // Register of return value.
            register_type<reg_pointer> reg_ret;
            // Base stack pointer.
            register_type<reg_pointer> reg_bp;
            // Current stack pointer.
            register_type<reg_pointer> reg_cp;
            // Current instruction pointer.
            register_type<reg_pointer> reg_ip;
            // Random registers for storage.
            register_type<reg_8> reg_8[num_of_storage_registers];
            register_type<reg_16> reg_16[num_of_storage_registers];
            register_type<reg_32> reg_32[num_of_storage_registers];
            register_type<reg_64> reg_64[num_of_storage_registers];
            register_type<reg_pointer> reg_pointer[num_of_storage_registers];
            // Registers for mathematical expression.
            register_type<reg_double> reg_double[num_of_storage_registers];
            register_type<reg_float> reg_float[num_of_storage_registers];

        } cpu_registers_t;

        // CPU Registers.
        cpu_registers_t m_CPURegs;
        static constexpr auto cpu_regs_size = sizeof(cpu_registers_t);

        byte_t m_RAM[ram_size];
    };

};

#endif // VM_TEMPLATE_H
