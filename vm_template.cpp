#include "vm_template.h"

namespace vm
{
    ProgramHeader::ProgramHeader() : m_entryPoint(nullptr)
    {
        memset(m_sections, 0, sizeof(m_sections));
    }

    template <sections_nb_t section_nb>
    section_t* ProgramHeader::getSection()
    {
        return &m_sections[section_nb];
    }

    Program::Program() : m_header(nullptr)
    {}

    Program::Program(array_t* binaryStub) :
        m_header(reinterpret_cast<ProgramHeader*>(binaryStub))
    {}

    ptr_t Program::getEntryPoint()
    {
        if (m_header != nullptr)
        {
            return m_header->m_entryPoint;
        }

        return nullptr;
    }

    ProgramWrite::ProgramWrite()
    {
        m_header = new ProgramHeader();
    }

    ProgramWrite::~ProgramWrite()
    {
        delete m_header;
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

        size_t writeSize = sizeof(ProgramHeader);

        // Write instructions only if it's necessary
        if (m_instructions.size() != 0)
        {
            // Create section code first.
            auto sectionCode = m_header->getSection<section_code>();

            sectionCode->m_pRBegin = sizeof(ProgramHeader);
            sectionCode->m_size = m_instructions.size();

            writeSize += sectionCode->m_size;
        }

        // Write data if it's only necessary.
        if (m_data.size() != 0)
        {
            // Create section data.
            auto sectionData = m_header->getSection<section_data>();

            sectionData->m_pRBegin = writeSize;
            sectionData->m_size = m_data.size();

            writeSize += sectionData->m_size;
        }

        if (m_relocs.size() != 0)
        {
            // Create section relocation.
            auto sectionRelocs = m_header->getSection<section_relocs>();
            sectionRelocs->m_pRBegin = writeSize;
            sectionRelocs->m_size = m_relocs.size() * sizeof(uintptr_t);

            writeSize += sectionRelocs->m_size;
        }

        // Check if we even need to write something.
        if (writeSize > sizeof(ProgramHeader))
        {
            // Resize binary.
            binaryProgram.resize(writeSize);

            auto binaryBytes = reinterpret_cast<array_t>(binaryProgram.data());
            // Insert header informations inside the binary.
            memcpy(binaryProgram.data(), m_header, sizeof(ProgramHeader));

            binaryBytes += sizeof(ProgramHeader);

            // Copy instructions.
            memcpy(binaryBytes, m_instructions.data(), m_instructions.size());
            binaryBytes += m_instructions.size();

            // Copy data.
            memcpy(binaryBytes, m_data.data(), m_data.size());
            binaryBytes += m_data.size();

            // Copy relocations.
            memcpy(binaryBytes, m_relocs.data(), m_relocs.size());
            binaryBytes += m_relocs.size();
        }
        else
        {
            assert("Creating empty program");
        }

        return binaryProgram;
    }

    auto ProgramWrite::setEntryPoint(ptr_t entryPoint)
    {
        m_header->m_entryPoint = entryPoint;
    }
};
