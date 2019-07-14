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
        // Call function register
        reg_call,
    } instructions_t;

    // Let's create some registers type for each data types.
    typedef enum
    {
        reg_8_signed,
        reg_16_signed,
        reg_32_signed,
        reg_64_signed,
        reg_8_unsigned,
        reg_16_unsigned,
        reg_32_unsigned,
        reg_64_unsigned,
        reg_float,
        reg_double,
        reg_pointer,
        reg_pointer_modifiable
    } registers_type_t;

    // Get register size from an enum.
    template <registers_type_t reg>
    inline constexpr auto _register_type()
    {
        if constexpr (reg == reg_8_signed)
        {
            return get_type<int8_t>;
        }
        else if constexpr (reg == reg_16_signed)
        {
            return get_type<int16_t>;
        }
        else if constexpr (reg == reg_32_signed)
        {
            return get_type<int32_t>;
        }
        else if constexpr (reg == reg_64_signed)
        {
            return get_type<int64_t>;
        }
        else if constexpr (reg == reg_8_unsigned)
        {
            return get_type<uint8_t>;
        }
        else if constexpr (reg == reg_16_unsigned)
        {
            return get_type<uint16_t>;
        }
        else if constexpr (reg == reg_32_unsigned)
        {
            return get_type<uint32_t>;
        }
        else if constexpr (reg == reg_64_unsigned)
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
            return get_type<ptr_t>;
        }
        else if constexpr (reg == reg_pointer_modifiable)
        {
            return get_type<uintptr_t>;
        }

        static_assert("Unknown type of register size");
    };

    template <registers_type_t reg>
    using register_type = typename decltype(_register_type<reg>())::type;

    class ProgramSections
    {
     public:
    };

    // Program class.
    class ProgramHeader
    {
     public:
        ProgramHeader();

        // Entry point of the program.
        uintptr_t m_entryPoint;
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
            register_type<reg_pointer_modifiable> reg_ret;
            // Base stack pointer.
            register_type<reg_pointer_modifiable> reg_bp;
            // Current stack pointer.
            register_type<reg_pointer_modifiable> reg_cp;
            // Current instruction pointer.
            register_type<reg_pointer> reg_ip;
            // Random registers for storage.
            register_type<reg_8_signed> reg_8_signed[num_of_storage_registers];
            register_type<reg_16_signed> reg_16_signed[num_of_storage_registers];
            register_type<reg_32_signed> reg_32_signed[num_of_storage_registers];
            register_type<reg_64_signed> reg_64_signed[num_of_storage_registers];
            register_type<reg_8_unsigned>
                reg_8_unsigned[num_of_storage_registers];
            register_type<reg_16_unsigned>
                reg_16_unsigned[num_of_storage_registers];
            register_type<reg_32_unsigned>
                reg_32_unsigned[num_of_storage_registers];
            register_type<reg_64_unsigned>
                reg_64_unsigned[num_of_storage_registers];
            register_type<reg_pointer_modifiable>
                reg_pointer[num_of_storage_registers];
            // Registers for mathematical expression.
            register_type<reg_double> reg_double[num_of_storage_registers];
            register_type<reg_float> reg_float[num_of_storage_registers];

        } cpu_registers_t;

        // CPU Registers.
        cpu_registers_t m_CPURegs;
        static constexpr auto cpu_regs_size = sizeof(cpu_registers_t);

        byte_t m_RAM[ram_size];
        byte_t m_Stack[stack_size];
    };

};

#endif // VM_TEMPLATE_H
