#ifndef VM_TEMPLATE_H
#define VM_TEMPLATE_H

#include <cstdint>
#include <cstddef>
#include <cassert>
#include <iostream>
#include <type_traits>

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
    } registers_size_t;

    // Get register size from an enum.
    template <registers_size_t reg>
    inline constexpr auto _register_size()
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

    template <registers_size_t reg>
    using register_size = typename decltype(_register_size<reg>())::type;

    template <typename T>
    inline constexpr auto _register_type()
    {
        if constexpr (std::is_same<T, register_size<reg_float>>::value)
        {
            return reg_float;
        }
        else if constexpr (std::is_same<T, register_size<reg_double>>::value)
        {
            return reg_double;
        }
        else if constexpr (std::is_same<T, register_size<reg_pointer>>::value)
        {
            return reg_pointer;
        }

        static_assert("Unknown register type");
    }

    template <typename T>
    constexpr auto register_type = _register_type<T>();

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
        Program(array_t* binaryFile);

        void parseHeader();
        void callEntryPoint();

        ProgramHeader<ret_t, args_t...> m_header;
        array_t* instructions;
    };

    template <size_t ram_size = default_ram_size>
    class VirtualMachine
    {
     public:
        VirtualMachine();

        void checkStack();

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
            register_size<reg_pointer> reg_ret;
            // Base stack pointer.
            register_size<reg_pointer> reg_bp;
            // Current stack pointer.
            register_size<reg_pointer> reg_cp;
            // Current instruction pointer.
            register_size<reg_pointer> reg_ip;
            // Random registers for storage.
            register_size<reg_pointer> reg_store[num_of_storage_registers];
            // Registers for mathematical expression.
            register_size<reg_double> reg_double[num_of_storage_registers];
            register_size<reg_float> reg_float[num_of_storage_registers];
            // Flag is used to know if previous condition was true or false.
            bool m_bFlag;

        } cpu_registers_t;

        // CPU Registers.
        cpu_registers_t m_CPURegs;
        // RAM.
        byte_t m_RAM[ram_size];
        // Usable memory is after the stack,
        // so we won't write to the stack directly.
        // Usable memory = RAM + stack_size.
        uintptr_t m_ptrUsableMemory;
    };

};

#endif // VM_TEMPLATE_H
