#include <algorithm>
#include <cstdint>
#include <cstring>

namespace ExternalDevice
{
constexpr int KGenericError = -1;
constexpr int KOk = 0;

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
                       bool aNoStop )
        = 0;

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
    virtual int Read( std::uint8_t aDeviceAddress, std::uint8_t* aDst, size_t aLen, bool aNoStop )
        = 0;

    template < typename TRegister >
    inline bool
    ReadLastRegisterRaw( std::uint8_t aDeviceAddress, TRegister& aRegisterValue )
    {
        return Read( aDeviceAddress, reinterpret_cast< std::uint8_t* >( &aRegisterValue ),
                     sizeof( aRegisterValue ), false )
               != sizeof( aRegisterValue );
    }

    template < typename TRegisterAddress, typename TRegister >
    bool
    ReadRegisterRaw( std::uint8_t aDeviceAddress,
                     TRegisterAddress aRegisterAddress,
                     TRegister& aRegisterValue )
    {
        return ( Write( aDeviceAddress, &aRegisterAddress, sizeof( aRegisterAddress ), true )
                 == sizeof( aRegisterAddress ) )
               && ( ReadLastRegisterRaw( aDeviceAddress, aRegisterValue )
                    != sizeof( aRegisterValue ) );
    }

    template < typename TRegisterAddress, typename RegisterType >
    bool
    WriteRegisterRaw( std::uint8_t aDeviceAddress,
                      TRegisterAddress aRegisterAddress,
                      RegisterType aRegisterValue )
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
    static constexpr std::uint8_t KDefaultAddress = 0x40;  // A0 pulled to GND
    static constexpr std::uint8_t KVSAddress = 0x41;       // A0 pulled to VS
    static constexpr std::uint8_t KSDAAddress = 0x42;      // A0 pulled to SDA
    static constexpr std::uint8_t KSCLAddress = 0x43;      // A0 pulled to SCL

    static constexpr std::uint8_t KChannel1 = 0x01;
    static constexpr std::uint8_t KHannel2 = 0x02;
    static constexpr std::uint8_t KHannel3 = 0x03;

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

    CIina3221( IAbstractI2CBus& aI2CBus, std::uint8_t aDeviceAddress = KDefaultAddress );
    ~CIina3221( ) = default;

    bool Init( const CConfig& aConfig = { } );

    bool SetConfig( const CConfig& aConfig );
    bool GetConfig( CConfig& aConfig );

    bool SetCriticalAlertLimit( std::uint16_t aLimit, std::uint8_t aChannel = KChannel1 );
    bool GetCriticalAlertLimit( std::uint16_t& aLimit, std::uint8_t aChannel = KChannel1 );

    float BusVoltageV( std::uint8_t aChannel = KChannel1 );
    float ShuntVoltageV( std::uint8_t aChannel = KChannel1 );

    int
    ErrorCode( ) const
    {
        return iErrorCode;
    }

    void
    ResetErrorCode( )
    {
        iErrorCode = KOk;
    }

private:
    /* data */
    IAbstractI2CBus& iI2CBus;
    const std::uint8_t iDeviceAddress;
    int iErrorCode = KOk;
    std::uint8_t iLastRegisterAddress = 0x00;

    float BusRegisterToVoltage( std::uint16_t aVoltageRegister );
    float ShuntRegisterToVoltage( std::uint16_t aShuntVoltageRegister );

    template < typename RegisterType >
    bool ReadRegister( std::uint8_t aReg, RegisterType& registerValue );
    template < typename RegisterType >
    bool WriteRegister( std::uint8_t aReg, RegisterType registerValue );
};

}  // namespace ExternalDevice