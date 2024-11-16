#include <INA3221.h>

#include <functional>
#include <algorithm>

#define DEVICE_ID 0x40
#define SIGNATURE 0x3220
#define REG_DATA_ch1 0x01  // ch 1 shunt
#define REG_DATA_ch2 0x03  // ch 2 shunt
#define REG_DATA_ch3 0x05  // ch 3 shunt

namespace ExternalDevice
{
namespace
{

constexpr std::uint32_t KFullScaleRegisterValue = 0x0FFF;
constexpr float KMaxVoltage = 32.76f;        // 0xFFF = 32.76V
constexpr float KMaxShuntVoltage = 0.1638f;  // 0xFFF = 0.1638V

static constexpr std::uint8_t KRegConfig = 0x00;
static constexpr std::uint8_t KDieId = 0xFF;

template < class taValue, class taCompare = std::less< taValue > >
constexpr const taValue&
Clamp( const taValue& aValue, const taValue& aLo, const taValue& aHi, taCompare aComp = { } )
{
    return aComp( aValue, aLo ) ? aLo : aComp( aHi, aValue ) ? aHi : aValue;
}

inline std::uint16_t
ToTwosComplement( std::uint16_t aValue )
{
    if ( aValue >= 0x8000 )
    {
        return 0x8000 | ( ( -aValue ) * 8 );
    }

    return 0x7FF8 & aValue * 8;  // shift right for 3 bit
}

inline std::uint16_t
FromTwosComplement( std::uint16_t aTwosComplementValue )
{
    if ( aTwosComplementValue >= 0x8000 )
    {
        aTwosComplementValue = -( 0x7FF8 & aTwosComplementValue );
    }
    return aTwosComplementValue / 8;  // shift left for 3 bit
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

inline constexpr std::uint8_t
MultiRegisterAddress( std::uint8_t aOffset, std::uint8_t aPeriod, std::uint8_t aRegisterNumber )
{
    return aOffset + ( aPeriod * ( aRegisterNumber - 1 ) );
}

}  // namespace

CIina3221::CIina3221( IAbstractI2CBus& aI2CBus, std::uint8_t aDeviceAddress ) NOEXCEPT
    : iI2CBus{ aI2CBus },
      iDeviceAddress{ aDeviceAddress }
{
}

float
CIina3221::BusRegisterToVoltage( std::uint16_t aVoltageRegister ) NOEXCEPT
{
    aVoltageRegister = FromTwosComplement( aVoltageRegister );
    // 0xFFF = 32.76V
    const float voltage = KMaxVoltage * aVoltageRegister / KFullScaleRegisterValue;
    return voltage;
}

std::uint16_t
CIina3221::VoltageToBusRegister( float aVoltage ) NOEXCEPT
{
    aVoltage = Clamp( aVoltage, 0.0f, KMaxVoltage ) * KFullScaleRegisterValue / KMaxVoltage;
    const auto voltageRegister = static_cast< std::uint16_t >( aVoltage );
    return ToTwosComplement( voltageRegister );
}

float
CIina3221::ShuntRegisterToVoltage( std::uint16_t aShuntVoltageRegister ) NOEXCEPT
{
    aShuntVoltageRegister = FromTwosComplement( aShuntVoltageRegister );
    const float shuntVoltage = KMaxShuntVoltage * aShuntVoltageRegister / KFullScaleRegisterValue;
    return shuntVoltage;
}

std::uint16_t
CIina3221::ShuntVoltageToRegister( float aShuntVoltage ) NOEXCEPT
{
    aShuntVoltage = Clamp( aShuntVoltage, 0.0f, KMaxShuntVoltage ) * KFullScaleRegisterValue
                    / KMaxShuntVoltage;
    const auto shuntVoltageRegister = static_cast< std::uint16_t >( aShuntVoltage );
    return ToTwosComplement( shuntVoltageRegister );
}

template < typename taRegisterType >
int
CIina3221::ReadRegister( std::uint8_t aRegisterAddress, taRegisterType& aRegisterValue ) NOEXCEPT
{
    const auto result
        = aRegisterAddress == iLastRegisterAddress
              ? iI2CBus.ReadLastRegisterRaw( iDeviceAddress, aRegisterValue )
              : iI2CBus.ReadRegisterRaw( iDeviceAddress, aRegisterAddress, aRegisterValue );
    if ( result )
    {
        iLastRegisterAddress = aRegisterAddress;
        aRegisterValue = ChangeEndian( aRegisterValue );
        return KOk;
    }
    return KGenericError;
}

template < typename taRegisterType >
int
CIina3221::WriteRegister( std::uint8_t aRegisterAddress, taRegisterType aRegisterValue ) NOEXCEPT
{
    aRegisterValue = ChangeEndian( aRegisterValue );
    const auto result
        = iI2CBus.WriteRegisterRaw( iDeviceAddress, aRegisterAddress, aRegisterValue );
    if ( result )
    {
        iLastRegisterAddress = aRegisterAddress;
        return KOk;
    }
    return KGenericError;
}

int
CIina3221::Init( const CConfig& aConfig ) NOEXCEPT
{
    std::uint16_t checkVendorId = 0;
    {
        const auto operationResult = ReadRegister( KDieId, checkVendorId );
        if ( operationResult != KOk )
        {
            // Unable to communicate
            return operationResult;
        }
    }

    if ( checkVendorId != SIGNATURE )
    {
        // Invalid device vendor
        return KInvalidVendor;
    }

    {
        const std::uint16_t packedConfig
            = PackConfig( aConfig );  // | 0x8000;  // 0x80 just set reset bit
        const auto operationResult = WriteRegister( KRegConfig, packedConfig );
        if ( operationResult != KOk )
        {
            // Unable to reset the device
            return operationResult;
        }
    }

    return KOk;
}

int
CIina3221::SetConfig( const CConfig& aConfig ) NOEXCEPT
{
    const std::uint16_t packedConfig = PackConfig( aConfig );
    return WriteRegister( KRegConfig, packedConfig );
}

int
CIina3221::GetConfig( CConfig& aConfig ) NOEXCEPT
{
    std::uint16_t packedConfig = 0x0000;
    const auto result = ReadRegister( KRegConfig, packedConfig );
    if ( result == KOk )
    {
        UnpackConfig( aConfig, packedConfig );
    }
    return result;
}
int
CIina3221::BusVoltageV( float& aVoltage, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KOffset = 0x02;
    constexpr std::uint8_t KPeriod = 2;

    if ( aChannel > 3 )
    {
        return KInvalidArgumentError;
    }

    std::uint16_t voltageRegister = 0;
    const auto result
        = ReadRegister( MultiRegisterAddress( KOffset, KPeriod, aChannel ), voltageRegister );
    if ( result == KOk )
    {
        aVoltage = BusRegisterToVoltage( voltageRegister );
    }
    return result;
}

int
CIina3221::ShuntVoltageV( float& aVoltage, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KOffset = 0x02;
    constexpr std::uint8_t KPeriod = 2;

    if ( aChannel > 3 )
    {
        return KInvalidArgumentError;
    }

    std::uint16_t shuntRegister = 0;
    const auto result
        = ReadRegister( MultiRegisterAddress( KOffset, KPeriod, aChannel ), shuntRegister );
    if ( result == KOk )
    {
        aVoltage = ShuntRegisterToVoltage( shuntRegister );
    }
    return result;
}

int
CIina3221::GetCriticalAlertLimit( std::uint16_t& aLimit, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KOffset = 0x07;
    constexpr std::uint8_t KPeriod = 2;

    if ( aChannel > 3 )
    {
        return KInvalidArgumentError;
    }
    return ReadRegister( MultiRegisterAddress( KOffset, KPeriod, aChannel ), aLimit );
}

int
CIina3221::SetCriticalAlertLimit( std::uint16_t aLimit, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KOffset = 0x07;
    constexpr std::uint8_t KPeriod = 2;

    if ( aChannel > 3 )
    {
        return KInvalidArgumentError;
    }
    return WriteRegister( MultiRegisterAddress( KOffset, KPeriod, aChannel ), aLimit );
}

template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
int
CIina3221::GetVoltageRegister( std::uint16_t& aVoltageRegister, std::uint8_t aChannel ) NOEXCEPT
{
    if ( aChannel > 3 )
    {
        return KInvalidArgumentError;
    }
    return ReadRegister(
        MultiRegisterAddress( taMultiRegisterOffset, taMultiRegisterPeriod, aChannel ),
        aVoltageRegister );
}

template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
int
CIina3221::SetVoltageRegister( std::uint16_t aVoltageRegister, std::uint8_t aChannel ) NOEXCEPT
{
    if ( aChannel > 3 )
    {
        return KInvalidArgumentError;
    }

    return WriteRegister(
        MultiRegisterAddress( taMultiRegisterOffset, taMultiRegisterPeriod, aChannel ),
        aVoltageRegister );
}

template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
int
CIina3221::GetVoltageRegister( float& aVoltage, std::uint8_t aChannel ) NOEXCEPT
{
    std::uint16_t voltageRegister = 0;
    const auto result = GetVoltageRegister< taMultiRegisterOffset, taMultiRegisterPeriod >(
        voltageRegister, aChannel );

    if ( result == KOk )
    {
        aVoltage = BusRegisterToVoltage( voltageRegister );
    }
    return result;
}

template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
int
CIina3221::SetVoltageRegister( float aVoltage, std::uint8_t aChannel ) NOEXCEPT
{
    const std::uint16_t voltageRegister = VoltageToBusRegister( aVoltage );
    return SetVoltageRegister< taMultiRegisterOffset, taMultiRegisterPeriod >( voltageRegister,
                                                                               aChannel );
}

}  // namespace ExternalDevice