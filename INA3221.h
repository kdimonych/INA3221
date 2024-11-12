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
     * @param aAddr 7-bit address of device to write to
     * @param aSrc Pointer to data to send
     * @param aLen Length of data in bytes to send
     * @param aNoStop If true, master retains control of the bus at the end of the transfer (no Stop
     * is issued), and the next transfer will begin with a Restart rather than a Start.
     * @return int Number of bytes written, or IAbstarctI2CBus::KGenericError if address not
     * acknowledged, no device present.
     */
    virtual int write( std::uint8_t aAddr, const std::uint8_t* aSrc, size_t aLen, bool aNoStop )
        = 0;

    /**
     * @brief Attempt to read specified number of bytes from address, blocking
     *
     * @param aAddr 7-bit address of device to read from
     * @param aDst Pointer to buffer to receive data
     * @param aLen Length of data in bytes to receive
     * @param aNoStop If true, master retains control of the bus at the end of the transfer (no Stop
     * is issued), and the next transfer will begin with a Restart rather than a Start.
     * @return Number of bytes read, or IAbstarctI2CBus::KGenericError if address not acknowledged
     * or no device present.
     */
    virtual int read( std::uint8_t aAddr, std::uint8_t* aDst, size_t aLen, bool aNoStop ) = 0;
};

class CIina3221
{
public:
    static constexpr std::uint8_t KDefaultAddress = 0x40;  // A0 pulled to GND
    static constexpr std::uint8_t KVSAddress = 0x41;       // A0 pulled to VS
    static constexpr std::uint8_t KSDAAddress = 0x42;      // A0 pulled to SDA
    static constexpr std::uint8_t KSCLAddress = 0x43;      // A0 pulled to SCL

    static constexpr std::uint8_t Khannel1 = 0x01;
    static constexpr std::uint8_t Khannel2 = 0x02;
    static constexpr std::uint8_t Khannel3 = 0x03;

    CIina3221( IAbstarctI2CBus& aI2CBus, std::uint8_t aAddr = KDefaultAddress );
    ~CIina3221( ) = default;

    bool init( );

    float voltageV( std::uint8_t aChannel = Khannel1 );
    float currentA( std::uint8_t aChannel = Khannel1 );

    int
    errorCode( ) const
    {
        return iErrorCode;
    }

private:
    /* data */
    IAbstarctI2CBus& iI2CBus;
    const std::uint8_t iAddr;
    int iErrorCode = KOk;

    float ShuntToAmp( int shunt );

    template < typename RegisterType >
    int readRegister( std::uint8_t aReg, RegisterType& registerValue );
    template < typename RegisterType >
    int writeRegister( std::uint8_t aReg, RegisterType registerValue );
};

}  // namespace ExternalDevice