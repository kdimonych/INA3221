#include <INA3221.h>

#define DEVICE_ID 0x40
#define SIGNATURE 0x3220
#define REG_RESET 0x00
#define REG_DATA_ch1 0x01  // ch 1 shunt
#define REG_DATA_ch2 0x03  // ch 2 shunt
#define REG_DATA_ch3 0x05  // ch 3 shunt

namespace ExternalDevice
{
namespace
{

inline std::uint16_t
toTwosComplent( std::uint16_t aValue )
{
    if ( aValue >= 0x8000 )
    {
        return 0x8000 | ( ( -aValue ) * 8 );
    }

    return 0x7FF8 & aValue * 8;
}

inline std::uint16_t
fromTwosComplement( std::uint16_t aTwosComplementValue )
{
    if ( aTwosComplementValue >= 0x8000 )
    {
        aTwosComplementValue = -( 0x7FF8 & aTwosComplementValue );
    }
    return aTwosComplementValue / 8;
}

std::uint32_t
changeEndian( std::uint32_t x )
{
    auto ptr = reinterpret_cast< unsigned char* >( &x );
    std::swap( ptr[ 0 ], ptr[ 3 ] );
    std::swap( ptr[ 1 ], ptr[ 2 ] );
    return x;
}

std::uint16_t
changeEndian( std::uint16_t x )
{
    auto ptr = reinterpret_cast< unsigned char* >( &x );
    std::swap( ptr[ 0 ], ptr[ 1 ] );
    return x;
}

}  // namespace

CIina3221::CIina3221( IAbstarctI2CBus& aI2CBus, std::uint8_t aDeviceAddress )
    : iI2CBus{ aI2CBus }
    , iDeviceAddress{ aDeviceAddress }
{
}

float
CIina3221::busRegisterToVoltage( std::uint16_t aVoltageRegister )
{
    aVoltageRegister = fromTwosComplement( aVoltageRegister );

    // 0xFFF = 32.76V
    const float voltage = ( 32.76 / 0x0FFF ) * static_cast< std::int16_t >( aVoltageRegister );
    return voltage;
}

float
CIina3221::shuntRegisterToVoltage( std::uint16_t aShuntVoltageRegister )
{
    aShuntVoltageRegister = fromTwosComplement( aShuntVoltageRegister );

    // 0xFFF = 0.1638V
    const float shuntVoltage
        = ( 0.1638 / 0x0FFF ) * static_cast< std::int16_t >( aShuntVoltageRegister );
    return shuntVoltage;
}

template < typename RegisterType >
int
CIina3221::readRegister( std::uint8_t aRegisterAddress, RegisterType& aRegisterValue )
{
    int count = iI2CBus.readRegisterRaw( iDeviceAddress, aRegisterAddress, aRegisterValue );
    if ( count < sizeof( aRegisterValue ) )
    {
        return count;
    }

    aRegisterValue = changeEndian( aRegisterValue );
    return count;
}

template < typename RegisterType >
int
CIina3221::writeRegister( std::uint8_t aRegisterAddress, RegisterType aRegisterValue )
{
    aRegisterValue = changeEndian( aRegisterValue );

    return iI2CBus.writeRegisterRaw( iDeviceAddress, &aRegisterAddress, aRegisterValue );
}

bool
CIina3221::init( )
{
    constexpr std::uint16_t KResetCmd = 0b1111111111111111;

    std::uint16_t chaeckVendorId = 0;
    if ( readRegister( 0xFF, chaeckVendorId ) < sizeof( chaeckVendorId ) )
    {
        // Unable to comunicate
        iErrorCode = KGenericError;
        return false;
    }

    if ( chaeckVendorId != SIGNATURE )
    {
        // Invalid device vendor
        iErrorCode = KGenericError;
        return false;
    }

    // Switch device to measurement mode (reset when connect, continous mode, max average) to modify
    // this in next version
    if ( writeRegister( REG_RESET, KResetCmd ) != sizeof( KResetCmd ) )
    {
        // Unable to reset the device
        iErrorCode = KGenericError;
        return false;
    }

    iErrorCode = KOk;
    return true;
}

float
CIina3221::busVoltageV( std::uint8_t aChannel )
{
    constexpr std::uint8_t KShuntBusRegOffset = 0x02;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t voltage = 0;
    readRegister( KShuntBusRegOffset + ( 2 * ( aChannel - 1 ) ), voltage );
    return busRegisterToVoltage( voltage );
}

float
CIina3221::shuntVoltageV( std::uint8_t aChannel )
{
    constexpr std::uint8_t KShuntRegOffset = 0x01;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t shuntRegister = 0;
    readRegister( KShuntRegOffset + ( 2 * ( aChannel - 1 ) ), shuntRegister );
    return shuntRegisterToVoltage( shuntRegister );
}
}  // namespace ExternalDevice