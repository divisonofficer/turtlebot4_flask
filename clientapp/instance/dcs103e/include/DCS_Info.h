#ifndef __DCS_INFO_H__
#define __DCS_INFO_H__

#if defined(_WIN32) && defined(BUILD_SHARED)
    #ifdef DCS_SDK_EXPORT
        #define DCS_EXPORT __declspec(dllexport)
    #else
        #define DCS_EXPORT __declspec(dllexport)
    #endif
#else
    #define DCS_EXPORT
#endif

#include <vector>
#include <string>
#include "DCS100.h"

namespace AdvancedIllumination
{

    /** @brief   Values that represents the type of DCS connected. */
    enum class DCS_Type
    {
        ///An enum constant representing the DCS-100E (single output, three channel) device
        DCS_100E,
        ///An enum constant representing the DCS-103E (three output, single channel per output) device
        DCS_103E
    };


    /** @brief   Class containing information about a DCS-100 device. */
    class DCS_EXPORT DCS_Info
    {
    public:
        DCS_Info();
        ~DCS_Info();

        static DCS_Info parse(const std::string& idn, std::string host);

        static std::vector<DCS_Info> findAllInNetwork(bool global = false);

        const std::string& name() const;
        const std::string& lighthead() const;
        const std::string& firmware() const;
        const std::string& host() const;

        DCS_Type type() const;

    private:

        std::string _name;
        std::string _firmware;
        std::string _lighthead;
        std::string _host;
        DCS_Type _type;

        DCS_Info(const std::string& name, const std::string& firm, const DCS_Type type);
    };
}

#endif
