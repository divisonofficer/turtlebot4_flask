#ifndef __DCS_100_H__
#define __DCS_100_H__

#if defined(_WIN32) && defined(BUILD_SHARED)
    #ifdef DCS_SDK_EXPORT
        #define DCS_EXPORT __declspec(dllexport)
    #else
        #define DCS_EXPORT __declspec(dllimport)
    #endif
#else
    #define DCS_EXPORT
#endif

#include <string>
#include <cinttypes>
#include <vector>
#include <iostream>
#include <exception>

namespace AdvancedIllumination
{
    /**
     * @brief   An exception representing a warning message from the device.   
     */
    class DCS_EXPORT DeviceWarning : std::invalid_argument
    {
    public:
        virtual const char* what() const noexcept;
        DeviceWarning(const std::string msg);
        virtual ~DeviceWarning() noexcept;
    private:
        std::string _what;
    };

    /**
     * @brief   An exception representing an error message from the device.  
     */
    class DCS_EXPORT DeviceError : std::runtime_error
    {
    public:
        virtual const char* what() const noexcept;
        DeviceError(const std::string msg);
        virtual ~DeviceError() noexcept;
    private:
        std::string _what;
    };


    /** @brief   Values that represent modes. */
    enum class Mode
    {

        ///An enum constant representing the "Off" mode
        Off = 0,

        ///An enum constant representing the Continuous current mode
        Continuous = 1,

        ///An enum constant representing the Pulsed mode
        Pulsed = 2,

        ///An enum constant representing the Gated mode
        Gated = 3
    };


    /** @brief   Values that represent trigger edges (e.g. how each channel's triggering behaves). */
    enum class Trigger
    {
        /** @brief   A rising-edge trigger. */
        Falling = 0,

        /** @brief   A falling-edge trigger. */
        Rising = 1
    };


    /** @brief   Values that represent channels. */
    enum class Channel
    {
        /** @brief   The first channel. */
        One = 1,

        /** @brief   The second channel. */
        Two = 2,

        /** @brief   The third channel. */
        Three = 3
    };

    /**
     * @brief   Contains information and methods for working with DCS devices.   
     */
    class DCS_EXPORT DCS_100
    {

    public:

        /**
         * @brief   Encaspsulates the data related to a single DCS channel. Allows getting and setting the device current, pulse width, etc.    
         */
        class DCS_EXPORT DCS_Channel
        {
        public:

            double current() const;
            void current(const double);

            double pulseWidth() const;
            void pulseWidth(const double);

            double pulseDelay() const;
            void pulseDelay(const double);

            double maxContinuous() const;
            const double maxStrobe() const;

            Mode mode() const;
            void mode(const Mode);

            Trigger triggerMode() const;
            void triggerMode(const Trigger);

            Channel triggerInput() const;
            void triggerInput(const Channel);

            double maxFrequency() const;

            void trigger() const;

        private:
            friend class DCS_100;

            DCS_Channel();
            DCS_100* parent;

            double _minPulseDelay;
            double _current;
            double _pulseWidth;
            double _pulseDelay;

            double _maxCont;
            double _maxStrobe;
            int _number;
            Mode _mode;
            Trigger _trigger;
            Channel _input;
            double _maxFreq;

            DCS_Channel(DCS_100*);
        };

        friend class DCS_Channel;

        DCS_Channel& Channel(size_t);
        DCS_Channel& operator[](size_t);

        const std::string& name() const;
        void name(const std::string& name);

        const std::string& firmwareVersion() const;

        const std::string& ipAddress() const;
        void ipAddress(const std::string& ip);

        size_t ChannelCount() const;


        const std::string& profileName() const;
        int profileNumber() const noexcept;

        void profileName(const std::string&);
        void profileNumber(int);

        void webConfigEnabled(const bool);
        bool webConfigEnabled() const;

        void saveProfile();

        DCS_100();

        DCS_100(const char* ip);

        void connect(const std::string& ip);
        void disconnect();

        void refreshConfig();

        const std::vector<std::string>& profileNames() const;

        ~DCS_100();

        void refreshProfiles();

    private:
        void parseProfiles(const std::string&);
        void parseConfigJSON(const std::string&);
        void parseConfigXML(const std::string&);

        size_t numChannels;
        DCS_Channel* Channels = nullptr;

        std::string sendCommand(const std::string&);

        std::string _name;
        std::string _firmare;
        std::string _lighthead;
        std::string _address;
        std::string _profileName;
        bool _webConfigEnabled;
        int _activeProfile;

        std::vector<std::string> _profileNames;

        void checkError(const std::string&) const;
        std::string getResponse(const char* prefix);
        class Comms* _comms;
    };
}

#endif
