#ifndef VM_TEMPLATE_H
#define VM_TEMPLATE_H
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <regex>
#include <type_traits>
#include <vector>

// Check GCC
#if __GNUC__
    #if __x86_64__ || __ppc64__
        #define ENVIRONMENT64
    #else
        #define ENVIRONMENT32
    #endif
#else
    #if _WIN32 || _WIN64
        #if _WIN64
            #define ENVIRONMENT64
        #else
            #define ENVIRONMENT32
        #endif
    #endif
#endif

namespace vm
{
    constexpr auto default_ram_size = 0x100000;

    using ptr_t = void*;
    using byte_t = uint8_t;
    using array_t = byte_t*;

    // Type wrapper
    template <typename type_t>
    struct type_wrapper_t
    {
        using type = type_t;
    };

    // Get inline variable from type wrapper.
    template <typename type_t>
    inline constexpr type_wrapper_t<type_t> get_type {};

    enum instructions_t : byte_t
    {
        // Mathematical expressions.
        inst_add,
        inst_minus,
        inst_divide,
        inst_multiply,
        inst_equal,
        inst_mod,
        // Others operations.
        inst_xor,
        inst_and,
        inst_or,
        inst_comp,
        inst_not,
        inst_sleft,
        inst_sright,
        // Conditions.
        inst_condition_lower,
        inst_condition_lowerequal,
        inst_condition_higher,
        inst_condition_higherequal,
        inst_condition_equal,
        inst_condition_or,
        inst_condition_and,
        inst_condition_not,
        // Stock & erase instisters values to the stack
        inst_push_8,
        inst_push_16,
        inst_push_32,
        inst_push_64,
        inst_push_pointer,
        inst_pop_8,
        inst_pop_16,
        inst_pop_32,
        inst_pop_64,
        inst_pop_pointer,
        // Jump instruction.
        inst_jmp,
        // Call instruction.
        inst_call,
        // Return instruction.
        inst_ret,
        // Read & write function to the "usable memory"
        inst_read,
        inst_write,
        // Set return value address.
        inst_set_ret,
        // Set value of instister.
        inst_set_value,
        // Exit machine,
        inst_exit
    };

    enum register_cast_type_t : byte_t
    {
        cast_float,
        cast_double,
        cast_8,
        cast_16,
        cast_32,
        cast_64
    };

    // What kind of operation is applied on, on value or on register.
    enum operation_type_t : byte_t
    {
        op_value,
        op_register

    };

    // Let's create some registers type for each data types.
    enum register_type_t : byte_t
    {
        reg_8,
        reg_16,
        reg_32,
        reg_64,
        reg_8_s,
        reg_16_s,
        reg_32_s,
        reg_64_s,
        reg_float,
        reg_double,
        reg_pointer
    };

    // Number of storage registers.
    enum register_storage_t : byte_t
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
        register_storage_float1,
        register_storage_float2,
        register_storage_float3,
        register_storage_float4,
        register_storage_double1,
        register_storage_double2,
        register_storage_double3,
        register_storage_double4,
        // Registers for reading to usable memory.
        register_storage_read,
        // Registers for writing to usable memory.
        register_storage_write,
        num_of_storage_registers

    };

    enum register_storage_slot_t : byte_t
    {
        slot_1_byte,
        slot_2_byte,
        slot_3_byte,
        slot_4_byte,
        slot_5_byte,
        slot_6_byte,
        slot_7_byte,
        slot_8_byte,
        slot_1_short,
        slot_2_short,
        slot_3_short,
        slot_4_short,
        slot_1_int,
        slot_2_int,
        slot_1_long,
        slot_1_byte_s,
        slot_2_byte_s,
        slot_3_byte_s,
        slot_4_byte_s,
        slot_5_byte_s,
        slot_6_byte_s,
        slot_7_byte_s,
        slot_8_byte_s,
        slot_1_short_s,
        slot_2_short_s,
        slot_3_short_s,
        slot_4_short_s,
        slot_1_int_s,
        slot_2_int_s,
        slot_1_long_s,
        slot_1_float,
        slot_2_float,
        slot_1_double
    };

    // Get register size from an enum.
    template <register_type_t reg>
    inline constexpr auto _rt()
    {
        if constexpr (reg == reg_8)
            return get_type<uint8_t>;

        if constexpr (reg == reg_16)
            return get_type<uint16_t>;

        if constexpr (reg == reg_32)
            return get_type<uint32_t>;

        if constexpr (reg == reg_64)
            return get_type<uint64_t>;

        if constexpr (reg == reg_8_s)
            return get_type<int8_t>;

        if constexpr (reg == reg_16_s)
            return get_type<int16_t>;

        if constexpr (reg == reg_32_s)
            return get_type<int32_t>;

        if constexpr (reg == reg_64_s)
            return get_type<int64_t>;

        if constexpr (reg == reg_float)
            return get_type<float>;

        if constexpr (reg == reg_double)
            return get_type<double>;

        if constexpr (reg == reg_pointer)
            return get_type<uintptr_t>;

        static_assert("Unknown register size");
    };

    template <register_type_t reg>
    using rt = typename decltype(_rt<reg>())::type;
    template <register_type_t reg>
    using rt = rt<reg>;

    template <typename T>
    inline constexpr auto _register_nb()
    {
        if constexpr (std::is_same<T, rt<reg_float>>::value)
        {
            return reg_float;
        }

        if constexpr (std::is_same<T, rt<reg_double>>::value)
        {
            return reg_double;
        }

        if constexpr (std::is_same<T, rt<reg_pointer>>::value)
        {
            return reg_pointer;
        }

        static_assert("Unknown register type");
    }

    template <typename T>
    inline constexpr auto register_nb = _register_nb<T>();

    // Program sections.
    class section_t
    {
     public:
        // Relative offset of where the section begin.
        uintptr_t m_pRBegin;
        // Size of the section.
        size_t m_size;
    };

    enum sections_nb_t : byte_t
    {
        section_code,
        section_data,
        section_relocs,
        section_max
    };

    struct program_header_t
    {
        // Entry point of the program.
        uintptr_t m_entryPoint;
        // Sections.
        section_t m_sections[section_max];
    };

    // Program header.
    class ProgramHeader : public program_header_t
    {
     public:
        template <sections_nb_t section_nb>
        section_t* getSection();
    };

    // Program execution.
    class Program
    {
     public:
        Program();
        explicit Program(ProgramHeader* header);
        uintptr_t getEntryPoint();

        // Header.
        ProgramHeader* m_header;
    };

    struct program_arguments_t
    {
        ptr_t data;
        size_t size;
    };

    // WriteProgram to write programs...
    class ProgramWrite : public Program
    {
     public:
        auto create();
        auto insertInstructions(std::vector<byte_t> instructions);
        auto insertReloc(uintptr_t reloc);
        auto insertData(std::vector<byte_t> data);
        auto setEntryPoint(uintptr_t entryPoint);

        std::vector<byte_t> m_instructions;
        std::vector<byte_t> m_data;
        std::vector<uintptr_t> m_relocs;
    };

    // Convert our own programming language to own CPU instructions.
    class Compiler
    {
     public:
    };

    template <size_t ram_size = default_ram_size>
    class VirtualMachine
    {
     public:
        union cpu_storage_register_t
        {
            // Unsigned
            rt<reg_8> b[8];
            rt<reg_16> s[4];
            rt<reg_32> i[2];
            rt<reg_64> l;
            // Signed
            rt<reg_8_s> b_s[8];
            rt<reg_16_s> s_s[4];
            rt<reg_32_s> i_s[2];
            rt<reg_64_s> l_s;
            rt<reg_pointer> p;
            rt<reg_float> f[2];
            rt<reg_double> d;
        };

        // The virtual machine registers.
        struct cpu_registers_t
        {
            // Register of return value.
            rt<reg_64> reg_ret;
            // Base stack pointer.
            rt<reg_pointer> reg_bp;
            // Current stack pointer.
            rt<reg_pointer> reg_sp;
            // Current call stack pointer.
            rt<reg_pointer> reg_cp;
            // Current instruction pointer.
            rt<reg_pointer> reg_ip;
            // Random registers for storage.
            cpu_storage_register_t reg_strg[num_of_storage_registers];
            // Flag is used to know if previous condition was true or false.
            bool m_bFlag;
        };

        // CPU Registers.
        cpu_registers_t m_CPU;
        // RAM.
        byte_t m_RAM[ram_size] {};
        // Usable memory is after the stack,
        // so we won't write to the stack directly.
        // Usable memory = RAM + stack_size.
        uintptr_t m_pUsableMemory {};
        // Code in RAM.
        section_t m_sectionCode {};
        // Data in RAM.
        section_t m_sectionData {};
        bool m_bPaused {};

        VirtualMachine()
        {
            init();
        }

        auto init()
        {
            // Init the return value.
            m_CPU.reg_ret = -1;

            // Calculate the stack size, we will assume that it is only 1/8 of
            // the RAM.
            constexpr auto stack_size = ram_size / 8;
            // Reset cpu.
            memset(&m_CPU, 0, sizeof(cpu_registers_t));

            // Usable memory setup.
            m_pUsableMemory = reinterpret_cast<uintptr_t>(m_RAM);

            // Setup stack pointer.
            // This time we will go to the lower address to the highest.
            m_CPU.reg_bp = m_pUsableMemory;
            m_CPU.reg_sp = m_CPU.reg_bp;
            m_pUsableMemory += stack_size;

            // Setup call stack pointer.
            // Same here.
            m_CPU.reg_cp = m_pUsableMemory;
            m_pUsableMemory += stack_size;
        }

        auto runProgram(Program* program, program_arguments_t* args = nullptr)
        {
            init();

            m_bPaused = false;

            m_CPU.reg_ip = m_pUsableMemory +
                           static_cast<uintptr_t>(program->getEntryPoint());

            if (args != nullptr)
            {
                // Increment the stack.
                m_CPU.reg_sp += args->size;

                // Setup the stack for the arguments.
                memcpy(static_cast<ptr_t>(m_CPU.reg_sp), args->data, args->size);
            }

            run();
        }

        inline auto incrementIP(size_t size)
        {
            // Increment instruction pointer.
            m_CPU.reg_ip += size;
        }

        inline auto setIP(uintptr_t p)
        {
            // Set instruction pointer.
            m_CPU.reg_ip = p;
        }

        inline auto readInstruction()
        {
            auto instruction = *reinterpret_cast<instructions_t*>(m_CPU.reg_ip);

            incrementIP(sizeof(instruction));

            return instruction;
        }

        inline auto readRegStorage()
        {
            auto regStrg = *reinterpret_cast<register_storage_t*>(m_CPU.reg_ip);
            incrementIP(sizeof(regStrg));

            if (regStrg >= num_of_storage_registers)
                static_assert("Not a storage register.");

            auto reg = &m_CPU.reg_strg[regStrg];

            auto slot = *reinterpret_cast<register_storage_slot_t*>(
                m_CPU.reg_ip);
            incrementIP(sizeof(slot));

            ptr_t slot_addr = nullptr;

            switch (slot)
            {
                // Unsigned.
                case slot_1_byte:
                    slot_addr = &reg->b[0];
                    break;
                case slot_2_byte:
                    slot_addr = &reg->b[1];
                    break;
                case slot_3_byte:
                    slot_addr = &reg->b[2];
                    break;
                case slot_4_byte:
                    slot_addr = &reg->b[3];
                    break;
                case slot_5_byte:
                    slot_addr = &reg->b[4];
                    break;
                case slot_6_byte:
                    slot_addr = &reg->b[5];
                    break;
                case slot_7_byte:
                    slot_addr = &reg->b[6];
                    break;
                case slot_8_byte:
                    slot_addr = &reg->b[7];
                    break;
                case slot_1_short:
                    slot_addr = &reg->s[0];
                    break;
                case slot_2_short:
                    slot_addr = &reg->s[1];
                    break;
                case slot_3_short:
                    slot_addr = &reg->s[2];
                    break;
                case slot_4_short:
                    slot_addr = &reg->s[3];
                    break;
                case slot_1_int:
                    slot_addr = &reg->i[0];
                    break;
                case slot_2_int:
                    slot_addr = &reg->i[1];
                    break;
                case slot_1_long:
                    slot_addr = &reg->l;
                    break;
                // Signed
                case slot_1_byte_s:
                    slot_addr = &reg->b_s[0];
                    break;
                case slot_2_byte_s:
                    slot_addr = &reg->b_s[1];
                    break;
                case slot_3_byte_s:
                    slot_addr = &reg->b_s[2];
                    break;
                case slot_4_byte_s:
                    slot_addr = &reg->b_s[3];
                    break;
                case slot_5_byte_s:
                    slot_addr = &reg->b_s[4];
                    break;
                case slot_6_byte_s:
                    slot_addr = &reg->b_s[5];
                    break;
                case slot_7_byte_s:
                    slot_addr = &reg->b_s[6];
                    break;
                case slot_8_byte_s:
                    slot_addr = &reg->b_s[7];
                    break;
                case slot_1_short_s:
                    slot_addr = &reg->s_s[0];
                    break;
                case slot_2_short_s:
                    slot_addr = &reg->s_s[1];
                    break;
                case slot_3_short_s:
                    slot_addr = &reg->s_s[2];
                    break;
                case slot_4_short_s:
                    slot_addr = &reg->s_s[3];
                    break;
                case slot_1_int_s:
                    slot_addr = &reg->i_s[0];
                    break;
                case slot_2_int_s:
                    slot_addr = &reg->i_s[1];
                    break;
                case slot_1_long_s:
                    slot_addr = &reg->l_s;
                    break;
                // Floats & doubles.
                case slot_1_float:
                    slot_addr = &reg->f[0];
                    break;
                case slot_2_float:
                    slot_addr = &reg->f[1];
                    break;
                case slot_1_double:
                    slot_addr = &reg->d;
                    break;
            }

            if (slot_addr == nullptr)
            {
                static_assert("Unknown register slot.");
            }

            return slot_addr;
        }

        inline operation_type_t readOperationType()
        {
            auto op = *reinterpret_cast<operation_type_t*>(m_CPU.reg_ip);
            incrementIP(sizeof(op));
            return op;
        }

        inline register_cast_type_t readCastType(size_t* sizeType = nullptr)
        {
            auto castType = *reinterpret_cast<register_cast_type_t*>(
                m_CPU.reg_ip);
            incrementIP(sizeof(castType));

            if (sizeType != nullptr)
            {
                switch (castType)
                {
                    case cast_8:
                    {
                        *sizeType = sizeof(uint8_t);
                        break;
                    }

                    case cast_16:
                    {
                        *sizeType = sizeof(uint16_t);
                        break;
                    }

                    case cast_32:
                    {
                        *sizeType = sizeof(uint32_t);
                        break;
                    }

                    case cast_64:
                    {
                        *sizeType = sizeof(uint64_t);
                        break;
                    }
                    case cast_double:
                    {
                        *sizeType = sizeof(double);
                        break;
                    }

                    case cast_float:
                    {
                        *sizeType = sizeof(float);
                        break;
                    }
                }
            }

            return castType;
        }

        inline auto addInstruction()
        {
            auto regResult = readRegStorage();
            // It is a value or register
            auto operationType = readOperationType();

            // Cast type. Float or double.
            size_t sizeType = 0;
            auto cast = readCastType(&sizeType);

            uint64_t val;

            switch (operationType)
            {
                case op_value:
                {
                    std::memcpy(&val,
                                reinterpret_cast<ptr_t>(m_CPU.reg_ip),
                                sizeType);

                    incrementIP(sizeType);
                    break;
                }

                case op_register:
                {
                    auto regSecond = readRegStorage();
                    std::memcpy(&val, regSecond, sizeType);
                    break;
                }
            }

            switch (cast)
            {
                case cast_8:
                {
                    *reinterpret_cast<uint8_t*>(
                        regResult) += *reinterpret_cast<uint8_t*>(&val);
                    break;
                }

                case cast_16:
                {
                    *reinterpret_cast<uint16_t*>(
                        regResult) += *reinterpret_cast<uint16_t*>(&val);
                    break;
                }

                case cast_32:
                {
                    *reinterpret_cast<uint32_t*>(
                        regResult) += *reinterpret_cast<uint32_t*>(&val);
                    break;
                }

                case cast_64:
                {
                    *reinterpret_cast<uint64_t*>(
                        regResult) += *reinterpret_cast<uint64_t*>(&val);
                    break;
                }

                case cast_double:
                {
                    *reinterpret_cast<double*>(
                        regResult) += *reinterpret_cast<double*>(&val);
                    break;
                }

                case cast_float:
                {
                    *reinterpret_cast<float*>(
                        regResult) += *reinterpret_cast<float*>(&val);
                    break;
                }
            }
        }

        auto run()
        {
            bool bExit = false;

            while (!bExit)
            {
                instructions_t instruction = readInstruction();

                // Check for errors.
                if (!checkCode() || !checkStacks())
                {
                    bExit = true;
                    break;
                }

                switch (instruction)
                {
                    case inst_add:
                    {
                        addInstruction();
                        break;
                    }
                    case inst_set_ret:
                    {
                        auto operationType = readOperationType();

                        size_t sizeType = 0;
                        readCastType(&sizeType);

                        uint64_t val;

                        switch (operationType)
                        {
                            case op_value:
                            {
                                std::memcpy(&val,
                                            reinterpret_cast<ptr_t>(
                                                m_CPU.reg_ip),
                                            sizeType);

                                incrementIP(sizeType);
                                break;
                            }

                            case op_register:
                            {
                                auto regSecond = readRegStorage();
                                std::memcpy(&val, regSecond, sizeType);
                                break;
                            }
                        }

                        m_CPU.reg_ret = val;
                        break;
                    }
                    case inst_exit:
                    {
                        bExit = true;
                        break;
                    }
                    default:
                        static_assert("Unknown instruction.");
                        break;
                }
            }

            return m_CPU.reg_ret;
        }

        // Check if exceeding the stacks.
        auto checkStacks()
        {
            return true;
        }

        // Check if exceeding the code section.
        auto checkCode()
        {
            return true;
        }
    };
}; // namespace vm

#endif // VM_TEMPLATE_H
