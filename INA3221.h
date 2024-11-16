#include <cstdint>
#include <cstring>

#ifdef __EXCEPTIONS
#define NOEXCEPT noexcept
#include <exception>
#else
#define NOEXCEPT
#endif

namespace ExternalDevice
{

class IAbstractI2CBus
{
public:
    virtual ~IAbstractI2CBus( ) = default;

    /**
     * @brief Attempt to write specified number of bytes to address, blocking
     *
     * @param aDeviceAddress 7-bit address of device to write to
     * @param aSrc Pointer to data to send
     * @param aLen Length of data in bytes to send
     * @param aNoStop If true, master retains control of the bus at the end of the transfer (no Stop
     * is issued), and the next transfer will begin with a Restart rather than a Start.
     * @return int Number of bytes written, or IAbstractI2CBus::KGenericError if address not
     * acknowledged, no device present.
     */
    virtual int Write( std::uint8_t aDeviceAddress,
                       const std::uint8_t* aSrc,
                       size_t aLen,
                       bool aNoStop ) NOEXCEPT = 0;

    /**
     * @brief Attempt to read specified number of bytes from address, blocking
     *
     * @param aDeviceAddress 7-bit address of device to read from
     * @param aDst Pointer to buffer to receive data
     * @param aLen Length of data in bytes to receive
     * @param aNoStop If true, master retains control of the bus at the end of the transfer (no Stop
     * is issued), and the next transfer will begin with a Restart rather than a Start.
     * @return Number of bytes read, or IAbstractI2CBus::KGenericError if address not acknowledged
     * or no device present.
     */
    virtual int Read( std::uint8_t aDeviceAddress,
                      std::uint8_t* aDst,
                      size_t aLen,
                      bool aNoStop ) NOEXCEPT = 0;

    template < typename taTRegister >
    inline bool
    ReadLastRegisterRaw( std::uint8_t aDeviceAddress, taTRegister& aRegisterValue ) NOEXCEPT
    {
        return Read( aDeviceAddress, reinterpret_cast< std::uint8_t* >( &aRegisterValue ),
                     sizeof( aRegisterValue ), false )
               != sizeof( aRegisterValue );
    }

    template < typename taTRegisterAddress, typename taTRegister >
    bool
    ReadRegisterRaw( std::uint8_t aDeviceAddress,
                     taTRegisterAddress aRegisterAddress,
                     taTRegister& aRegisterValue ) NOEXCEPT
    {
        return ( Write( aDeviceAddress, &aRegisterAddress, sizeof( aRegisterAddress ), true )
                 == sizeof( aRegisterAddress ) )
               && ( ReadLastRegisterRaw( aDeviceAddress, aRegisterValue )
                    != sizeof( aRegisterValue ) );
    }

    template < typename taTRegisterAddress, typename taTRegister >
    bool
    WriteRegisterRaw( std::uint8_t aDeviceAddress,
                      taTRegisterAddress aRegisterAddress,
                      taTRegister aRegisterValue ) NOEXCEPT
    {
        std::uint8_t dataPack[ sizeof( aRegisterAddress ) + sizeof( aRegisterValue ) ];
        std::memcpy( dataPack, &aRegisterAddress, sizeof( aRegisterAddress ) );
        std::memcpy( dataPack + sizeof( aRegisterAddress ), &aRegisterValue,
                     sizeof( aRegisterValue ) );

        return Write( aDeviceAddress, dataPack, sizeof( dataPack ), false ) != sizeof( dataPack );
    }
};

class CIina3221
{
public:
    static constexpr int KOk = 0;
    static constexpr int KGenericError = -1;
    static constexpr int KInvalidArgumentError = -2;
    static constexpr int KInvalidVendor = -3;

#ifdef __EXCEPTIONS
    template < int taError, const char* const taDescription >
    class EBase : public std::exception
    {
    public:
        constexpr int
        error( ) const NOEXCEPT
        {
            return taError;
        }

        const char*
        what( ) const NOEXCEPT override
        {
            return taDescription;
        }
    };
    static constexpr char KGenericErrorDescription[] = "Internal error";
    static constexpr char KInvalidArgumentDescription[] = "Invalid argument";
    static constexpr char KInvalidVendorDescription[] = "Invalid vendor";
    using EGenericError = EBase< KGenericError, KGenericErrorDescription >;
    using EInvalidArgumentError = EBase< KInvalidArgumentError, KInvalidArgumentDescription >;
    using EInvalidVendorError = EBase< KInvalidVendor, KInvalidVendorDescription >;

#endif

    static constexpr std::uint8_t KDefaultAddress = 0x40;  // A0 pulled to GND
    static constexpr std::uint8_t KVSAddress = 0x41;       // A0 pulled to VS
    static constexpr std::uint8_t KSDAAddress = 0x42;      // A0 pulled to SDA
    static constexpr std::uint8_t KSCLAddress = 0x43;      // A0 pulled to SCL

    static constexpr std::uint8_t KChannel1 = 0x01;
    static constexpr std::uint8_t KChannel2 = 0x02;
    static constexpr std::uint8_t KChannel3 = 0x03;

    enum class OperationMode : std::uint8_t
    {
        PowerDown = 0x0,                     // Power-down
        ShuntVoltageSingleShot = 0x1,        // Shunt voltage, single-shot (triggered)
        BusVoltageSingleShot = 0x2,          // Bus voltage, single-shot (triggered)
        ShuntAndBusVoltageSingleShot = 0x3,  // Shunt and bus voltage, single-shot (triggered)
        PowerDown2 = 0x4,                    // Power-down ?
        ShuntVoltageContinuous = 0x5,        // Shunt voltage, continuous
        BusVoltageContinuous = 0x6,          // Bus voltage, continuous
        ShuntAndBusVoltageContinuous = 0x7,  // Shunt and bus voltage, continuous (default)
    };

    enum class ConversionTime : std::uint8_t
    {
        t140us = 0x0,   // 140µs
        t204us = 0x1,   // 204µs
        t332us = 0x2,   // 322µs
        t588us = 0x3,   // 588µs
        t1100us = 0x4,  // 1100µs (default)
        t2116us = 0x5,  // 2116µs
        t4156us = 0x6,  // 4156µs
        t8244us = 0x7,  // 8244µs
    };

    enum class AveragingMode : std::uint8_t
    {
        avg1 = 0x0,     // Average 1 sample (default)
        avg4 = 0x1,     // Average 4 samples
        avg16 = 0x2,    // Average 16 samples
        avg64 = 0x3,    // Average 64 samples
        avg128 = 0x4,   // Average 128 samples
        avg256 = 0x5,   // Average 256 samples
        avg512 = 0x6,   // Average 512 samples
        avg1024 = 0x7,  // Average 1024 samples
    };

    struct CConfig
    {
        CConfig( ){ };
        OperationMode iOperationMode = OperationMode::ShuntAndBusVoltageContinuous;
        ConversionTime iShuntVoltageConversionTime = ConversionTime::t1100us;
        ConversionTime iBusVoltageConversionTime = ConversionTime::t1100us;
        AveragingMode iAveragingMode = AveragingMode::avg1;

        bool iChannel3Enable = true;
        bool iChannel2Enable = true;
        bool iChannel1Enable = true;

        bool iRstart = false;
    };

    CIina3221( IAbstractI2CBus& aI2CBus, std::uint8_t aDeviceAddress = KDefaultAddress ) NOEXCEPT;
    ~CIina3221( ) = default;

    int Init( const CConfig& aConfig = { } ) NOEXCEPT;

    int SetConfig( const CConfig& aConfig ) NOEXCEPT;
    int GetConfig( CConfig& aConfig ) NOEXCEPT;

    int BusVoltageV( float& aVoltage, std::uint8_t aChannel = KChannel1 ) NOEXCEPT;
    int ShuntVoltageV( float& aVoltage, std::uint8_t aChannel = KChannel1 ) NOEXCEPT;

    int GetCriticalAlertLimit( std::uint16_t& aLimit, std::uint8_t aChannel = KChannel1 ) NOEXCEPT;
    int SetCriticalAlertLimit( std::uint16_t aLimit, std::uint8_t aChannel = KChannel1 ) NOEXCEPT;

#ifdef __EXCEPTIONS
    inline float
    BusVoltageV( std::uint8_t aChannel = KChannel1 )
    {
        float voltage = 0.0;
        ThrowOnError( BusVoltageV( voltage, aChannel ) );
        return voltage;
    }

    inline float
    ShuntVoltageV( std::uint8_t aChannel = KChannel1 )
    {
        float voltage = 0.0;
        ThrowOnError( BusVoltageV( voltage, aChannel ) );
        return voltage;
    }
#endif

private:
    /* data */
    IAbstractI2CBus& iI2CBus;
    const std::uint8_t iDeviceAddress;
    std::uint8_t iLastRegisterAddress = 0x00;

    float BusRegisterToVoltage( std::uint16_t aVoltageRegister ) NOEXCEPT;
    std::uint16_t VoltageToBusRegister( float aVoltageRegister ) NOEXCEPT;
    float ShuntRegisterToVoltage( std::uint16_t aShuntVoltageRegister ) NOEXCEPT;
    std::uint16_t ShuntVoltageToRegister( float aShuntVoltage ) NOEXCEPT;

    template < typename taRegisterType >
    int ReadRegister( std::uint8_t aReg, taRegisterType& aRegisterValue ) NOEXCEPT;
    template < typename taRegisterType >
    int WriteRegister( std::uint8_t aReg, taRegisterType aRegisterValue ) NOEXCEPT;

    template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
    inline int GetVoltageRegister( std::uint16_t& aVoltageRegister,
                                   std::uint8_t aChannel ) NOEXCEPT;
    template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
    inline int SetVoltageRegister( std::uint16_t aVoltageRegister, std::uint8_t aChannel ) NOEXCEPT;

    template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
    inline int GetVoltageRegister( float& aVoltage, std::uint8_t aChannel ) NOEXCEPT;
    template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
    inline int SetVoltageRegister( float aVoltage, std::uint8_t aChannel ) NOEXCEPT;

#ifdef __EXCEPTIONS
    static inline int
    ThrowOnError( int aErrorCode )
    {
        switch ( aErrorCode )
        {
        case KOk:
            return aErrorCode;
            break;
        case KInvalidArgumentError:
            throw EInvalidArgumentError{ };
            break;
        case KInvalidVendor:
            throw EInvalidVendorError{ };
            break;
        case KGenericError:
        default:
            throw EGenericError{ };
            break;
        }
        return aErrorCode;
    }
#endif
};

}  // namespace ExternalDevice