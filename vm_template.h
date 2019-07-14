#ifndef VM_TEMPLATE_H
#define VM_TEMPLATE_H

#include <cstdint>
#include <cstddef>
#include <cassert>
#include <iostream>
#include <type_traits>
#include <cstring>

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

#pragma pack(1)
    typedef enum
    {
        // Mathematical expressions.
        reg_add,
        reg_minus,
        reg_divide,
        reg_multiply,
        reg_equal,
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
        // jmp instruction.
        reg_jmp,
        // Return instruction.
        reg_ret,
        // Read & write function to the "usable memory"
        reg_read,
        reg_write,
        // Set return value address.
        reg_set_ret,
        // Exit machine,
        reg_exit
    } instructions_t;

    // Let's create some registers type for each data types.
    typedef enum
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
    } register_type_t;

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

    } register_storage_t;

    typedef enum
    {
        reg_1_byte,
        reg_2_byte,
        reg_3_byte,
        reg_4_byte,
        reg_5_byte,
        reg_6_byte,
        reg_7_byte,
        reg_8_byte,
        reg_1_short,
        reg_2_short,
        reg_3_short,
        reg_4_short,
        reg_1_int,
        reg_2_int,
        reg_1_long,
        reg_1_byte_s,
        reg_2_byte_s,
        reg_3_byte_s,
        reg_4_byte_s,
        reg_5_byte_s,
        reg_6_byte_s,
        reg_7_byte_s,
        reg_8_byte_s,
        reg_1_short_s,
        reg_2_short_s,
        reg_3_short_s,
        reg_4_short_s,
        reg_1_int_s,
        reg_2_int_s,
        reg_1_long_s,
        reg_1_float,
        reg_2_float,
        reg_1_double,
    } cpu_register_slot_t;

#pragma pack()

    // Get register size from an enum.
    template <register_type_t reg>
    inline constexpr auto _rt()
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
        if constexpr (reg == reg_8_s)
        {
            return get_type<int8_t>;
        }
        else if constexpr (reg == reg_16_s)
        {
            return get_type<int16_t>;
        }
        else if constexpr (reg == reg_32_s)
        {
            return get_type<int32_t>;
        }
        else if constexpr (reg == reg_64_s)
        {
            return get_type<int64_t>;
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

    typedef struct
    {
        ptr_t data;
        size_t size;
    } program_arguments_t;

    // Write program template to write programs...
    class ProgramWrite : public Program
    {
     public:
        size_t m_writeSize;
        size_t m_maxSize;
        bool m_bSectionComplete[section_max];
    };

    template <auto ram_size = default_ram_size>
    class VirtualMachine
    {
     public:
        typedef union
        {
            rt<reg_8> b[8];
            rt<reg_16> s[4];
            rt<reg_32> i[2];
            rt<reg_64> l;
            rt<reg_8_s> b_s[8];
            rt<reg_16_s> s_s[4];
            rt<reg_32_s> i_s[2];
            rt<reg_64_s> l_s;
            rt<reg_pointer> p;
            rt<reg_float> f[2];
            rt<reg_double> d;
        } cpu_storage_register_t;

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
            cpu_storage_register_t reg_strg[num_of_storage_registers];
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

        auto runProgram(Program* program, program_arguments_t* args)
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

            // Setup the stack for the arguments.
            memcpy(reinterpret_cast<ptr_t>(m_CPU.reg_cp),
                   args->data,
                   args->size);

            // Increment the stack.
            m_CPU.reg_cp += args->size;

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

        inline auto determinateRegStrg()
        {
            auto storageType = *reinterpret_cast<register_storage_t*>(
                m_CPU.reg_ip);

            incrementIP(sizeof(storageType));

            return storageType;
        }

        inline auto determinateRegStrgSlot(cpu_storage_register_t* reg,
                                           register_type_t* type)
        {
            auto slot = *reinterpret_cast<cpu_register_slot_t*>(m_CPU.reg_ip);

            incrementIP(sizeof(slot));

            uintptr_t reg_addr = 0;

            switch (slot)
            {
                // Unsigned.
                case reg_1_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[0];
                    break;
                case reg_2_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[1];
                    break;
                case reg_3_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[2];
                    break;
                case reg_4_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[3];
                    break;
                case reg_5_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[4];
                    break;
                case reg_6_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[5];
                    break;
                case reg_7_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[6];
                    break;
                case reg_8_byte:
                    *type = reg_8;
                    reg_addr = &reg->b[7];
                    break;
                case reg_1_short:
                    *type = reg_16;
                    reg_addr = &reg->s[0];
                    break;
                case reg_2_short:
                    *type = reg_16;
                    reg_addr = &reg->s[1];
                    break;
                case reg_3_short:
                    *type = reg_16;
                    reg_addr = &reg->s[2];
                    break;
                case reg_4_short:
                    *type = reg_16;
                    reg_addr = &reg->s[3];
                    break;
                case reg_1_int:
                    *type = reg_32;
                    reg_addr = &reg->i[0];
                    break;
                case reg_2_int:
                    *type = reg_32;
                    reg_addr = &reg->i[1];
                    break;
                case reg_1_long:
                    *type = reg_64;
                    reg_addr = &reg->l;
                    break;
                // Signed
                case reg_1_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[0];
                    break;
                case reg_2_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[1];
                    break;
                case reg_3_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[2];
                    break;
                case reg_4_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[3];
                    break;
                case reg_5_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[4];
                    break;
                case reg_6_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[5];
                    break;
                case reg_7_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[6];
                    break;
                case reg_8_byte_s:
                    *type = reg_8_s;
                    reg_addr = &reg->b_s[7];
                    break;
                case reg_1_short_s:
                    *type = reg_16_s;
                    reg_addr = &reg->s_s[0];
                    break;
                case reg_2_short_s:
                    *type = reg_16_s;
                    reg_addr = &reg->s_s[1];
                    break;
                case reg_3_short_s:
                    *type = reg_16_s;
                    reg_addr = &reg->s_s[2];
                    break;
                case reg_4_short_s:
                    *type = reg_16_s;
                    reg_addr = &reg->s_s[3];
                    break;
                case reg_1_int_s:
                    *type = reg_32_s;
                    reg_addr = &reg->i_s[0];
                    break;
                case reg_2_int_s:
                    *type = reg_32_s;
                    reg_addr = &reg->i_s[1];
                    break;
                case reg_1_long_s:
                    *type = reg_64_s;
                    reg_addr = &reg->l_s;
                    break;
                // Floats & doubles.
                case reg_1_float:
                    *type = reg_float;
                    reg_addr = &reg->f[0];
                    break;
                case reg_2_float:
                    *type = reg_float;
                    reg_addr = &reg->f[1];
                    break;
                case reg_1_double:
                    *type = reg_double;
                    reg_addr = &reg->d;
                    break;
            }

            if (reg_addr == 0)
            {
                assert("Unknown register slot.");
            }

            return reg_addr;
        }

        inline auto addInstruction()
        {
            // Read first on wich register we will store the result.
            auto resultReg1 = &m_CPU.reg_strg[determinateRegStrg()];
            auto resultReg2 = &m_CPU.reg_strg[determinateRegStrg()];

            // Now we need to know the type of add.
            register_type_t type1, type2;
            auto reg1 = determinateRegStrgSlot(resultReg1, &type1);
            auto reg2 = determinateRegStrgSlot(resultReg2, &type2);

            if (type1 != type2)
            {
                assert("Adding variable of not the same type");
            }

            bool bDone = false;

            switch (type1)
            {
                // Unsigned addition.
                case reg_8:
                {
                    *reinterpret_cast<rt<reg_8>*>(
                        reg1) += *reinterpret_cast<rt<reg_8>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16:
                {
                    *reinterpret_cast<rt<reg_16>*>(
                        reg1) += *reinterpret_cast<rt<reg_16>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32:
                {
                    *reinterpret_cast<rt<reg_32>*>(
                        reg1) += *reinterpret_cast<rt<reg_32>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64:
                {
                    *reinterpret_cast<rt<reg_64>*>(
                        reg1) += *reinterpret_cast<rt<reg_64>*>(reg2);
                    bDone = true;
                    break;
                }
                // Signed addition
                case reg_8_s:
                {
                    *reinterpret_cast<rt<reg_8_s>*>(
                        reg1) += *reinterpret_cast<rt<reg_8_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16_s:
                {
                    *reinterpret_cast<rt<reg_16_s>*>(
                        reg1) += *reinterpret_cast<rt<reg_16_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32_s:
                {
                    *reinterpret_cast<rt<reg_32_s>*>(
                        reg1) += *reinterpret_cast<rt<reg_32_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64_s:
                {
                    *reinterpret_cast<rt<reg_64_s>*>(
                        reg1) += *reinterpret_cast<rt<reg_64_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_float:
                {
                    *reinterpret_cast<rt<reg_float>*>(
                        reg1) += *reinterpret_cast<rt<reg_float>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_double:
                {
                    *reinterpret_cast<rt<reg_double>*>(
                        reg1) += *reinterpret_cast<rt<reg_double>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_pointer:
                {
                    *reinterpret_cast<rt<reg_pointer>*>(
                        reg1) += *reinterpret_cast<rt<reg_pointer>*>(reg2);
                    bDone = true;
                    break;
                }
            }

            return bDone;
        }

        inline auto minusInstruction()
        {
            // Read first on wich register we will store the result.
            auto resultReg1 = &m_CPU.reg_strg[determinateRegStrg()];
            auto resultReg2 = &m_CPU.reg_strg[determinateRegStrg()];

            // Now we need to know the type of minus.
            register_type_t type1, type2;
            auto reg1 = determinateRegStrgSlot(resultReg1, &type1);
            auto reg2 = determinateRegStrgSlot(resultReg2, &type2);

            if (type1 != type2)
            {
                assert("Adding variable of not the same type");
            }

            bool bDone = false;

            switch (type1)
            {
                // Unsigned minus.
                case reg_8:
                {
                    *reinterpret_cast<rt<reg_8>*>(
                        reg1) -= *reinterpret_cast<rt<reg_8>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16:
                {
                    *reinterpret_cast<rt<reg_16>*>(
                        reg1) -= *reinterpret_cast<rt<reg_16>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32:
                {
                    *reinterpret_cast<rt<reg_32>*>(
                        reg1) -= *reinterpret_cast<rt<reg_32>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64:
                {
                    *reinterpret_cast<rt<reg_64>*>(
                        reg1) -= *reinterpret_cast<rt<reg_64>*>(reg2);
                    bDone = true;
                    break;
                }
                // Signed minus
                case reg_8_s:
                {
                    *reinterpret_cast<rt<reg_8_s>*>(
                        reg1) -= *reinterpret_cast<rt<reg_8_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16_s:
                {
                    *reinterpret_cast<rt<reg_16_s>*>(
                        reg1) -= *reinterpret_cast<rt<reg_16_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32_s:
                {
                    *reinterpret_cast<rt<reg_32_s>*>(
                        reg1) -= *reinterpret_cast<rt<reg_32_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64_s:
                {
                    *reinterpret_cast<rt<reg_64_s>*>(
                        reg1) -= *reinterpret_cast<rt<reg_64_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_float:
                {
                    *reinterpret_cast<rt<reg_float>*>(
                        reg1) -= *reinterpret_cast<rt<reg_float>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_double:
                {
                    *reinterpret_cast<rt<reg_double>*>(
                        reg1) -= *reinterpret_cast<rt<reg_double>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_pointer:
                {
                    *reinterpret_cast<rt<reg_pointer>*>(
                        reg1) -= *reinterpret_cast<rt<reg_pointer>*>(reg2);
                    bDone = true;
                    break;
                }
            }

            return bDone;
        }

        inline auto divideInstruction()
        {
            // Read first on wich register we will store the result.
            auto resultReg1 = &m_CPU.reg_strg[determinateRegStrg()];
            auto resultReg2 = &m_CPU.reg_strg[determinateRegStrg()];

            // Now we need to know the type of division.
            register_type_t type1, type2;
            auto reg1 = determinateRegStrgSlot(resultReg1, &type1);
            auto reg2 = determinateRegStrgSlot(resultReg2, &type2);

            if (type1 != type2)
            {
                assert("Adding variable of not the same type");
            }

            bool bDone = false;

            switch (type1)
            {
                // Unsigned division.
                case reg_8:
                {
                    *reinterpret_cast<rt<reg_8>*>(
                        reg1) /= *reinterpret_cast<rt<reg_8>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16:
                {
                    *reinterpret_cast<rt<reg_16>*>(
                        reg1) /= *reinterpret_cast<rt<reg_16>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32:
                {
                    *reinterpret_cast<rt<reg_32>*>(
                        reg1) /= *reinterpret_cast<rt<reg_32>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64:
                {
                    *reinterpret_cast<rt<reg_64>*>(
                        reg1) /= *reinterpret_cast<rt<reg_64>*>(reg2);
                    bDone = true;
                    break;
                }
                // Signed division
                case reg_8_s:
                {
                    *reinterpret_cast<rt<reg_8_s>*>(
                        reg1) /= *reinterpret_cast<rt<reg_8_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16_s:
                {
                    *reinterpret_cast<rt<reg_16_s>*>(
                        reg1) /= *reinterpret_cast<rt<reg_16_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32_s:
                {
                    *reinterpret_cast<rt<reg_32_s>*>(
                        reg1) /= *reinterpret_cast<rt<reg_32_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64_s:
                {
                    *reinterpret_cast<rt<reg_64_s>*>(
                        reg1) /= *reinterpret_cast<rt<reg_64_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_float:
                {
                    *reinterpret_cast<rt<reg_float>*>(
                        reg1) /= *reinterpret_cast<rt<reg_float>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_double:
                {
                    *reinterpret_cast<rt<reg_double>*>(
                        reg1) /= *reinterpret_cast<rt<reg_double>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_pointer:
                {
                    *reinterpret_cast<rt<reg_pointer>*>(
                        reg1) /= *reinterpret_cast<rt<reg_pointer>*>(reg2);
                    bDone = true;
                    break;
                }
            }

            return bDone;
        }

        inline auto multiplyInstruction()
        {
            // Read first on wich register we will store the result.
            auto resultReg1 = &m_CPU.reg_strg[determinateRegStrg()];
            auto resultReg2 = &m_CPU.reg_strg[determinateRegStrg()];

            // Now we need to know the type of division.
            register_type_t type1, type2;
            auto reg1 = determinateRegStrgSlot(resultReg1, &type1);
            auto reg2 = determinateRegStrgSlot(resultReg2, &type2);

            if (type1 != type2)
            {
                assert("Adding variable of not the same type");
            }

            bool bDone = false;

            switch (type1)
            {
                // Unsigned multiply.
                case reg_8:
                {
                    *reinterpret_cast<rt<reg_8>*>(
                        reg1) *= *reinterpret_cast<rt<reg_8>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16:
                {
                    *reinterpret_cast<rt<reg_16>*>(
                        reg1) *= *reinterpret_cast<rt<reg_16>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32:
                {
                    *reinterpret_cast<rt<reg_32>*>(
                        reg1) *= *reinterpret_cast<rt<reg_32>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64:
                {
                    *reinterpret_cast<rt<reg_64>*>(
                        reg1) *= *reinterpret_cast<rt<reg_64>*>(reg2);
                    bDone = true;
                    break;
                }
                // Signed multiply.
                case reg_8_s:
                {
                    *reinterpret_cast<rt<reg_8_s>*>(
                        reg1) *= *reinterpret_cast<rt<reg_8_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_16_s:
                {
                    *reinterpret_cast<rt<reg_16_s>*>(
                        reg1) *= *reinterpret_cast<rt<reg_16_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_32_s:
                {
                    *reinterpret_cast<rt<reg_32_s>*>(
                        reg1) *= *reinterpret_cast<rt<reg_32_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_64_s:
                {
                    *reinterpret_cast<rt<reg_64_s>*>(
                        reg1) *= *reinterpret_cast<rt<reg_64_s>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_float:
                {
                    *reinterpret_cast<rt<reg_float>*>(
                        reg1) *= *reinterpret_cast<rt<reg_float>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_double:
                {
                    *reinterpret_cast<rt<reg_double>*>(
                        reg1) *= *reinterpret_cast<rt<reg_double>*>(reg2);
                    bDone = true;
                    break;
                }
                case reg_pointer:
                {
                    *reinterpret_cast<rt<reg_pointer>*>(
                        reg1) *= *reinterpret_cast<rt<reg_pointer>*>(reg2);
                    bDone = true;
                    break;
                }
            }

            return bDone;
        }

        auto run()
        {
            bool bExit = false;

            while (!bExit)
            {
                instructions_t instruction = readInstruction();

                switch (instruction)
                {
                    case reg_add:
                    {
                        if (!addInstruction())
                        {
                            assert("Unknown addition");
                        }
                        break;
                    }

                    case reg_minus:
                    {
                        if (!minusInstruction())
                        {
                            assert("Unknown minus");
                        }
                        break;
                    }

                    case reg_divide:
                    {
                        if (!divideInstruction())
                        {
                            assert("Unknown divide");
                        }
                        break;
                    }

                    case reg_multiply:
                    {
                        if (!multiplyInstruction())
                        {
                            assert("Unknown mulitply");
                        }
                        break;
                    }

                    case reg_exit:
                        bExit = true;
                        break;
                }
            }
        }

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
    };

};

#endif // VM_TEMPLATE_H
