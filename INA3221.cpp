#include <INA3221.h>

#define DEVICE_ID 0x40
#define SIGNATURE 0x3220
#define REG_DATA_ch1 0x01  // ch 1 shunt
#define REG_DATA_ch2 0x03  // ch 2 shunt
#define REG_DATA_ch3 0x05  // ch 3 shunt

namespace ExternalDevice
{
namespace
{

static constexpr std::uint8_t KRegConfig = 0x00;
static constexpr std::uint8_t KDieId = 0xFF;

inline std::uint16_t
ToTwosComplement( std::uint16_t aValue )
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

std::uint16_t
PackConfig( const CIina3221::CConfig& aConfig )
{
    std::uint16_t result = 0x0000;
    result |= static_cast< std::uint16_t >( aConfig.iOperationMode );
    result |= static_cast< std::uint16_t >( aConfig.iShuntVoltageConversionTime ) << 3;
    result |= static_cast< std::uint16_t >( aConfig.iBusVoltageConversionTime ) << 6;
    result |= static_cast< std::uint16_t >( aConfig.iAveragingMode ) << 9;
    result |= static_cast< std::uint16_t >( aConfig.iChannel3Enable ) << 12;
    result |= static_cast< std::uint16_t >( aConfig.iChannel3Enable ) << 13;
    result |= static_cast< std::uint16_t >( aConfig.iChannel3Enable ) << 14;
    result |= static_cast< std::uint16_t >( aConfig.iRstart ) << 15;

    return result;
    // return ChangeEndian( result );
}

void
UnpackConfig( CIina3221::CConfig& aConfig, std::uint16_t aPackedConfig )
{
    // aPackedConfig = ChangeEndian( aPackedConfig );
    aConfig.iOperationMode = static_cast< CIina3221::OperationMode >( aPackedConfig & 0x7 );
    aConfig.iShuntVoltageConversionTime
        = static_cast< CIina3221::ConversionTime >( ( aPackedConfig >> 3 ) & 0x7 );
    aConfig.iBusVoltageConversionTime
        = static_cast< CIina3221::ConversionTime >( ( aPackedConfig >> 6 ) & 0x7 );
    aConfig.iAveragingMode
        = static_cast< CIina3221::AveragingMode >( ( aPackedConfig >> 9 ) & 0x7 );
    aConfig.iChannel3Enable = static_cast< bool >( ( aPackedConfig >> 12 ) & 0x1 );
    aConfig.iChannel2Enable = static_cast< bool >( ( aPackedConfig >> 13 ) & 0x1 );
    aConfig.iChannel1Enable = static_cast< bool >( ( aPackedConfig >> 14 ) & 0x1 );
    aConfig.iRstart = static_cast< bool >( ( aPackedConfig >> 15 ) & 0x1 );
}

inline CIina3221::CConfig
UnpackConfig( std::uint16_t aPackedConfig )
{
    CIina3221::CConfig config;
    UnpackConfig( config, aPackedConfig );
    return config;
}

}  // namespace

CIina3221::CIina3221( IAbstractI2CBus& aI2CBus, std::uint8_t aDeviceAddress )
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
bool
CIina3221::ReadRegister( std::uint8_t aRegisterAddress, RegisterType& aRegisterValue )
{
    const auto result
        = aRegisterAddress == iLastRegisterAddress
              ? iI2CBus.ReadLastRegisterRaw( iDeviceAddress, aRegisterValue )
              : iI2CBus.ReadRegisterRaw( iDeviceAddress, aRegisterAddress, aRegisterValue );
    if ( result )
    {
        iLastRegisterAddress = aRegisterAddress;
        aRegisterValue = ChangeEndian( aRegisterValue );
        return true;
    }
    return false;
}

template < typename RegisterType >
bool
CIina3221::WriteRegister( std::uint8_t aRegisterAddress, RegisterType aRegisterValue )
{
    aRegisterValue = ChangeEndian( aRegisterValue );
    const auto result
        = iI2CBus.WriteRegisterRaw( iDeviceAddress, aRegisterAddress, aRegisterValue );
    if ( result )
    {
        iLastRegisterAddress = aRegisterAddress;
    }
    return result;
}

bool
CIina3221::Init( const CConfig& aConfig )
{
    std::uint16_t checkVendorId = 0;
    if ( !ReadRegister( KDieId, checkVendorId ) )
    {
        // Unable to communicate
        iErrorCode = KGenericError;
        return false;
    }

    if ( checkVendorId != SIGNATURE )
    {
        // Invalid device vendor
        iErrorCode = KGenericError;
        return false;
    }

    const std::uint16_t packedConfig
        = PackConfig( aConfig );  // | 0x8000;  // 0x80 just set reset bit
    if ( !WriteRegister( KRegConfig, packedConfig ) )
    {
        // Unable to reset the device
        iErrorCode = KGenericError;
        return false;
    }

    iErrorCode = KOk;
    return true;
}

bool
CIina3221::SetConfig( const CConfig& aConfig )
{
    const std::uint16_t packedConfig = PackConfig( aConfig );
    auto result = WriteRegister( KRegConfig, packedConfig );
    iErrorCode = result ? KGenericError : KOk;
    return result;
}
bool
CIina3221::GetConfig( CConfig& aConfig )
{
    std::uint16_t packedConfig = 0x0000;
    if ( ReadRegister( KRegConfig, packedConfig ) )
    {
        UnpackConfig( aConfig, packedConfig );
        return true;
    }

    // Unable to reset the device
    iErrorCode = KGenericError;
    return false;
}

float
CIina3221::BusVoltageV( std::uint8_t aChannel )
{
    constexpr std::uint8_t KOffset = 0x02;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t voltage = 0;
    ReadRegister( KOffset + ( 2 * ( aChannel - 1 ) ), voltage );
    return BusRegisterToVoltage( voltage );
}

float
CIina3221::ShuntVoltageV( std::uint8_t aChannel )
{
    constexpr std::uint8_t KOffset = 0x01;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return 0.0;
    }

    std::uint16_t shuntRegister = 0;
    ReadRegister( KOffset + ( 2 * ( aChannel - 1 ) ), shuntRegister );
    return ShuntRegisterToVoltage( shuntRegister );
}

bool
CIina3221::SetCriticalAlertLimit( std::uint16_t aLimit, std::uint8_t aChannel )
{
    constexpr std::uint8_t KOffset = 0x07;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return false;
    }
    return WriteRegister( KOffset + ( 2 * ( aChannel - 1 ) ), aLimit );
}
bool
CIina3221::GetCriticalAlertLimit( std::uint16_t& aLimit, std::uint8_t aChannel )
{
    constexpr std::uint8_t KOffset = 0x07;
    if ( aChannel > 3 )
    {
        iErrorCode = KGenericError;
        return false;
    }
    return ReadRegister( KOffset + ( 2 * ( aChannel - 1 ) ), aLimit );
}
}  // namespace ExternalDevice