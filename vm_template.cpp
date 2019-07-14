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
        return m_sections[section_nb];
    }

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
};
