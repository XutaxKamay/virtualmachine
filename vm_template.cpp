#include "vm_template.h"

namespace vm
{
    template <sections_nb_t section_nb>
    section_t* ProgramHeader::getSection()
    {
        return &m_sections[section_nb];
    }

    Program::Program() : m_header(nullptr)
    {}

    Program::Program(ProgramHeader* header) :
        m_header(static_cast<ProgramHeader*>(header))
    {}

    uintptr_t Program::getEntryPoint()
    {
        if (m_header != nullptr)
        {
            return m_header->m_entryPoint;
        }

        return 0;
    }

    auto ProgramWrite::insertInstructions(std::vector<byte_t> instructions)
    {
        m_instructions.insert(m_instructions.end(),
                              instructions.begin(),
                              instructions.end());
    }

    auto ProgramWrite::insertReloc(uintptr_t reloc)
    {
        m_relocs.push_back(reloc);
    }

    auto ProgramWrite::insertData(std::vector<byte_t> data)
    {
        m_data.insert(m_data.end(), data.begin(), data.end());
    }

    auto ProgramWrite::create()
    {
        std::vector<byte_t> binaryProgram;

        size_t writeSize = sizeof(program_header_t);

        // Write instructions only if it's necessary
        if (!m_instructions.empty())
        {
            // Create section code first.
            auto sectionCode = m_header->getSection<section_code>();

            sectionCode->m_pRBegin = sizeof(program_header_t);
            sectionCode->m_size = m_instructions.size();

            writeSize += sectionCode->m_size;
        }

        // Write data if it's only necessary.
        if (!m_data.empty())
        {
            // Create section data.
            auto sectionData = m_header->getSection<section_data>();

            sectionData->m_pRBegin = writeSize;
            sectionData->m_size = m_data.size();

            writeSize += sectionData->m_size;
        }

        if (!m_relocs.empty())
        {
            // Create section relocation.
            auto sectionRelocs = m_header->getSection<section_relocs>();
            sectionRelocs->m_pRBegin = writeSize;
            sectionRelocs->m_size = m_relocs.size() * sizeof(uintptr_t);

            writeSize += sectionRelocs->m_size;
        }

        // Check if we even need to write something.
        if (writeSize > sizeof(program_header_t))
        {
            // Insert header informations inside the binary.
            std::vector<byte_t> headerBytes;
            headerBytes.resize(sizeof(program_header_t));

            std::memcpy(headerBytes.data(), m_header, sizeof(program_header_t));
            binaryProgram.insert(binaryProgram.end(),
                                 headerBytes.begin(),
                                 headerBytes.end());

            // Copy instructions.
            binaryProgram.insert(binaryProgram.end(),
                                 m_instructions.begin(),
                                 m_instructions.end());

            // Copy data.
            binaryProgram.insert(binaryProgram.end(),
                                 m_data.begin(),
                                 m_data.end());

            // Copy relocations.
            binaryProgram.insert(binaryProgram.end(),
                                 m_relocs.begin(),
                                 m_relocs.end());
        }
        else
        {
            static_assert("Creating empty program");
        }

        return binaryProgram;
    }

    auto ProgramWrite::setEntryPoint(uintptr_t entryPoint)
    {
        m_header->m_entryPoint = entryPoint;
    }
}; // namespace vm
