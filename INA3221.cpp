#include <INA3221.h>

#define DEVICE_ID 0x40
#define SIGNATURE 8242
#define REG_RESET 0x00
#define REG_DATA_ch1 0x01  // ch 1 shunt
#define REG_DATA_ch2 0x03  // ch 2 shunt
#define REG_DATA_ch3 0x05  // ch 3 shunt

namespace ExternalDevice
{
namespace
{
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

CIina3221::CIina3221( IAbstarctI2CBus& aI2CBus, std::uint8_t aAddr )
    : iI2CBus{ aI2CBus }
    , iAddr{ aAddr }
{
}

float
CIina3221::ShuntToAmp( int shunt )
{
    // sign change for negative value (bit 13 is sign)
    if ( shunt > 4096 )
    {
        shunt = -( 8192 - shunt );
    }
    // shunt raw value to mv (163.8mV LSB (SD0):40μV) datasheet
    float amp1mv = ( 163.8 / 4096 ) * shunt;
    // for my shunt the producer does not recomend to go above 80A, IC is able to
    // measure up to 108 A :) I have a 50A/75mv shunt so A on channel is:
    float Amp = amp1mv * 50 / 75;
    // if you do not plan to use external shunt you are limited to 1.6 A / channel
    // default A without external shunt R on device is 0.1 ohm comment line above
    // and uncomment next line:
    //      float Amp=amp1mv*10
    return Amp;
}

template < typename RegisterType >
int
CIina3221::readRegister( std::uint8_t aReg, RegisterType& registerValue )
{
    int count = iI2CBus.write( iAddr, &aReg, sizeof( aReg ), true );
    if ( count < sizeof( aReg ) )
    {
        return KGenericError;
    }

    std::uint16_t registerValue = 0;
    count = iI2CBus.read( iAddr, reinterpret_cast< std::uint8_t* >( &registerValue ),
                          sizeof( registerValue ), false );
    if ( count < sizeof( registerValue ) )
    {
        return KGenericError;
    }
    return count;
}

template < typename RegisterType >
int
CIina3221::writeRegister( std::uint8_t aReg, RegisterType registerValue )
{
    int count = iI2CBus.write( iAddr, &aReg, sizeof( aReg ), true );
    if ( count < sizeof( aReg ) )
    {
        return KGenericError;
    }

    count = iI2CBus.write( iAddr, reinterpret_cast< const std::uint8_t* >( &registerValue ),
                           sizeof( registerValue ), false );
    if ( count < sizeof( registerValue ) )
    {
        return KGenericError;
    }
    return count;
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
CIina3221::voltageV( std::uint8_t aChannel = Khannel1 )
{
    constexpr std::uint8_t KShuntBusRegOffset = 0x02;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t voltage = 0;
    readRegister( KShuntBusRegOffset + ( 2 * ( aChannel - 1 ) ), voltage );
    voltage = changeEndian( voltage ) / 8;  // ?
    return ShuntToAmp( voltage );           // TODO: redo
}

float
CIina3221::currentA( std::uint8_t aChannel = Khannel1 )
{
    constexpr std::uint8_t KShuntBusRegOffset = 0x01;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t shunt = 0;
    readRegister( KShuntBusRegOffset + ( 2 * ( aChannel - 1 ) ), shunt );
    shunt = changeEndian( shunt ) / 8;  // ?
    return ShuntToAmp( shunt );
}
}  // namespace ExternalDevice