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
ToTwosComplent( std::uint16_t aValue )
{
    if ( aValue >= 0x8000 )
    {
        return 0x8000 | ( ( -aValue ) * 8 );
    }

    return 0x7FF8 & aValue * 8;
}

inline std::uint16_t
FromTwosComplement( std::uint16_t aTwosComplementValue )
{
    if ( aTwosComplementValue >= 0x8000 )
    {
        aTwosComplementValue = -( 0x7FF8 & aTwosComplementValue );
    }
    return aTwosComplementValue / 8;
}

std::uint32_t
ChangeEndian( std::uint32_t x )
{
    auto ptr = reinterpret_cast< unsigned char* >( &x );
    std::swap( ptr[ 0 ], ptr[ 3 ] );
    std::swap( ptr[ 1 ], ptr[ 2 ] );
    return x;
}

std::uint16_t
ChangeEndian( std::uint16_t x )
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
CIina3221::BusRegisterToVoltage( std::uint16_t aVoltageRegister )
{
    aVoltageRegister = FromTwosComplement( aVoltageRegister );

    // 0xFFF = 32.76V
    const float voltage = ( 32.76 / 0x0FFF ) * static_cast< std::int16_t >( aVoltageRegister );
    return voltage;
}

float
CIina3221::ShuntRegisterToVoltage( std::uint16_t aShuntVoltageRegister )
{
    aShuntVoltageRegister = FromTwosComplement( aShuntVoltageRegister );

    // 0xFFF = 0.1638V
    const float shuntVoltage
        = ( 0.1638 / 0x0FFF ) * static_cast< std::int16_t >( aShuntVoltageRegister );
    return shuntVoltage;
}

template < typename RegisterType >
int
CIina3221::ReadRegister( std::uint8_t aRegisterAddress, RegisterType& aRegisterValue )
{
    int count = iI2CBus.ReadRegisterRaw( iDeviceAddress, aRegisterAddress, aRegisterValue );
    if ( count < sizeof( aRegisterValue ) )
    {
        return count;
    }

    aRegisterValue = ChangeEndian( aRegisterValue );
    return count;
}

template < typename RegisterType >
int
CIina3221::WriteRegister( std::uint8_t aRegisterAddress, RegisterType aRegisterValue )
{
    aRegisterValue = ChangeEndian( aRegisterValue );

    return iI2CBus.WriteRegisterRaw( iDeviceAddress, &aRegisterAddress, aRegisterValue );
}

bool
CIina3221::Init( )
{
    constexpr std::uint16_t KResetCmd = 0b1111111111111111;

    std::uint16_t chaeckVendorId = 0;
    if ( ReadRegister( 0xFF, chaeckVendorId ) < sizeof( chaeckVendorId ) )
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
    if ( WriteRegister( REG_RESET, KResetCmd ) != sizeof( KResetCmd ) )
    {
        // Unable to reset the device
        iErrorCode = KGenericError;
        return false;
    }

    iErrorCode = KOk;
    return true;
}

float
CIina3221::BusVoltageV( std::uint8_t aChannel )
{
    constexpr std::uint8_t KShuntBusRegOffset = 0x02;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t voltage = 0;
    ReadRegister( KShuntBusRegOffset + ( 2 * ( aChannel - 1 ) ), voltage );
    return BusRegisterToVoltage( voltage );
}

float
CIina3221::ShuntVoltageV( std::uint8_t aChannel )
{
    constexpr std::uint8_t KShuntRegOffset = 0x01;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t shuntRegister = 0;
    ReadRegister( KShuntRegOffset + ( 2 * ( aChannel - 1 ) ), shuntRegister );
    return ShuntRegisterToVoltage( shuntRegister );
}
}  // namespace ExternalDevice