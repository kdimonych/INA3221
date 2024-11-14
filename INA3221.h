#include <algorithm>
#include <cstdint>

namespace ExternalDevice
{
constexpr int KGenericError = -1;
constexpr int KOk = 0;

class IAbstarctI2CBus
{
public:
    virtual ~IAbstarctI2CBus( ) = default;

    /**
     * @brief Attempt to write specified number of bytes to address, blocking
     *
     * @param aDeviceAddress 7-bit address of device to write to
     * @param aSrc Pointer to data to send
     * @param aLen Length of data in bytes to send
     * @param aNoStop If true, master retains control of the bus at the end of the transfer (no Stop
     * is issued), and the next transfer will begin with a Restart rather than a Start.
     * @return int Number of bytes written, or IAbstarctI2CBus::KGenericError if address not
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
     * @return Number of bytes read, or IAbstarctI2CBus::KGenericError if address not acknowledged
     * or no device present.
     */
    virtual int Read( std::uint8_t aDeviceAddress, std::uint8_t* aDst, size_t aLen, bool aNoStop )
        = 0;

    template < typename TRegister >
    inline int
    ReadLastRegisterRaw( std::uint8_t aDeviceAddress, TRegister& aRegisterValue )
    {
        return Read( aDeviceAddress, reinterpret_cast< std::uint8_t* >( &aRegisterValue ),
                     sizeof( aRegisterValue ), false );
    }

    template < typename TRegisterAddress, typename TRegister >
    int
    ReadRegisterRaw( std::uint8_t aDeviceAddress,
                     TRegisterAddress aRegisterAddress,
                     TRegister& aRegisterValue )
    {
        int count = Write( aDeviceAddress, &aRegisterAddress, sizeof( aRegisterAddress ), true );
        if ( count < sizeof( aRegisterAddress ) )
        {
            return count;
        }

        return ReadLastRegisterRaw( aDeviceAddress, aRegisterValue );
    }

    template < typename TRegisterAddress, typename RegisterType >
    int
    WriteRegisterRaw( std::uint8_t aDeviceAddress,
                      TRegisterAddress aRegisterAddress,
                      RegisterType aRegisterValue )
    {
        struct alignas( TRegisterAddress ) RawDataPack
        {
            TRegisterAddress iRegisterAddress;
            RegisterType iRegisterValue;
        };

        RawDataPack dataPack{ aRegisterAddress, aRegisterValue };
        return Write( aDeviceAddress, reinterpret_cast< const std::uint8_t* >( &dataPack ),
                      sizeof( dataPack ), false );
    }
};

class CIina3221
{
public:
    static constexpr std::uint8_t KDefaultAddress = 0x40;  // A0 pulled to GND
    static constexpr std::uint8_t KVSAddress = 0x41;       // A0 pulled to VS
    static constexpr std::uint8_t KSDAAddress = 0x42;      // A0 pulled to SDA
    static constexpr std::uint8_t KSCLAddress = 0x43;      // A0 pulled to SCL

    static constexpr std::uint8_t KHannel1 = 0x01;
    static constexpr std::uint8_t KHannel2 = 0x02;
    static constexpr std::uint8_t KHannel3 = 0x03;

    struct Configuration
    {
    };

    CIina3221( IAbstarctI2CBus& aI2CBus, std::uint8_t aDeviceAddress = KDefaultAddress );
    ~CIina3221( ) = default;

    bool Init( );

    float BusVoltageV( std::uint8_t aChannel = KHannel1 );
    float ShuntVoltageV( std::uint8_t aChannel = KHannel1 );

    int
    ErrorCode( ) const
    {
        return iErrorCode;
    }

private:
    /* data */
    IAbstarctI2CBus& iI2CBus;
    const std::uint8_t iDeviceAddress;
    int iErrorCode = KOk;

    float BusRegisterToVoltage( std::uint16_t aVoltageRegister );
    float ShuntRegisterToVoltage( std::uint16_t aShuntVoltageRegister );

    template < typename RegisterType >
    int ReadRegister( std::uint8_t aReg, RegisterType& registerValue );
    template < typename RegisterType >
    int WriteRegister( std::uint8_t aReg, RegisterType registerValue );
};

}  // namespace ExternalDevice