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

constexpr std::int16_t KFullScaleRegisterValue = 0x0FFF * 8;
constexpr std::int16_t KFullScalePowerUperLimitRegisterValue = 0x2710;   // Per 10.00 V
constexpr std::int16_t KFullScalePowerLowerLimitRegisterValue = 0x2328;  // Per 9.00 V
constexpr float KMaxBusVoltage = 32.76f;                                 // 0x0FFF * 8 = 32.76V
constexpr float KMaxShuntVoltage = 0.1638f;                              // 0x0FFF * 8 = 0.1638V
constexpr float KFullScalePowerUperLimitVoltage = 10.0f;                 // 10.00 V
constexpr float KFullScalePowerLowerLimitVoltage = 9.0f;                 // 9.00 V

static constexpr std::uint8_t KRegConfig = 0x00;
static constexpr std::uint8_t KDieId = 0xFF;

constexpr std::uint8_t KChannelNumber = 3;

template < class taValue, class taCompare = std::less< taValue > >
constexpr const taValue&
Clamp( const taValue& aValue, const taValue& aLo, const taValue& aHi, taCompare aComp = { } )
{
    return aComp( aValue, aLo ) ? aLo : aComp( aHi, aValue ) ? aHi : aValue;
}

inline std::uint16_t
ToTwosComplement( std::int16_t aValue ) NOEXCEPT
{
    if ( aValue < 0 )
    {
        return 0x8000 | static_cast< std::uint16_t >( -aValue );
    }

    return static_cast< std::uint16_t >( aValue );
}

inline std::int16_t
FromTwosComplement( std::uint16_t aTwosComplementValue ) NOEXCEPT
{
    if ( aTwosComplementValue >= 0x8000 )
    {
        aTwosComplementValue = -static_cast< std::int16_t >( 0x7FFF & aTwosComplementValue );
    }
    return aTwosComplementValue;
}

std::uint32_t
ChangeEndian( std::uint32_t x ) NOEXCEPT
{
    auto ptr = reinterpret_cast< unsigned char* >( &x );
    std::swap( ptr[ 0 ], ptr[ 3 ] );
    std::swap( ptr[ 1 ], ptr[ 2 ] );
    return x;
}

std::uint16_t
ChangeEndian( std::uint16_t x ) NOEXCEPT
{
    auto ptr = reinterpret_cast< unsigned char* >( &x );
    std::swap( ptr[ 0 ], ptr[ 1 ] );
    return x;
}

template < std::uint8_t taDataLShift = 3 >
float
BusRegisterToVoltage( std::uint16_t aVoltageRegister,
                      float aFullScaleAbsoluteVoltage,
                      std::int16_t aFullScaleRegisterValue = KFullScaleRegisterValue ) NOEXCEPT
{
    constexpr uint16_t mask = 0xFFFF << taDataLShift;

    const auto rawVoltage = FromTwosComplement( aVoltageRegister & mask );
    const float voltage = aFullScaleAbsoluteVoltage * rawVoltage / aFullScaleRegisterValue;
    return voltage;
}

template < std::uint8_t taDataLShift = 3 >
std::uint16_t
VoltageToBusRegister( float aVoltage,
                      float aFullScaleAbsoluteVoltage,
                      std::int16_t aFullScaleRegisterValue = KFullScaleRegisterValue ) NOEXCEPT
{
    constexpr uint16_t mask = 0xFFFF << taDataLShift;

    aVoltage = Clamp( aVoltage, -aFullScaleAbsoluteVoltage, aFullScaleAbsoluteVoltage )
               * aFullScaleRegisterValue / aFullScaleAbsoluteVoltage;
    const auto rawVoltage = static_cast< std::int16_t >( aVoltage );
    return ToTwosComplement( rawVoltage ) & mask;
}

std::uint16_t
PackConfig( const CIina3221::CConfig& aConfig ) NOEXCEPT
{
    std::uint16_t result = 0x0000;
    result |= static_cast< std::uint16_t >( aConfig.iOperationMode );
    result |= static_cast< std::uint16_t >( aConfig.iShuntVoltageConversionTime ) << 3;
    result |= static_cast< std::uint16_t >( aConfig.iBusVoltageConversionTime ) << 6;
    result |= static_cast< std::uint16_t >( aConfig.iAveragingMode ) << 9;
    result |= static_cast< std::uint16_t >( aConfig.iChannel3Enable ) << 12;
    result |= static_cast< std::uint16_t >( aConfig.iChannel2Enable ) << 13;
    result |= static_cast< std::uint16_t >( aConfig.iChannel1Enable ) << 14;
    result |= static_cast< std::uint16_t >( aConfig.iRstart ) << 15;

    return result;
}

void
UnpackConfig( CIina3221::CConfig& aConfig, std::uint16_t aPackedConfig ) NOEXCEPT
{
    aConfig.iOperationMode = static_cast< CIina3221::OperationMode >( aPackedConfig & 0x7 );
    aConfig.iShuntVoltageConversionTime
        = static_cast< CIina3221::ConversionTime >( ( aPackedConfig >> KChannelNumber ) & 0x7 );
    aConfig.iBusVoltageConversionTime
        = static_cast< CIina3221::ConversionTime >( ( aPackedConfig >> 6 ) & 0x7 );
    aConfig.iAveragingMode
        = static_cast< CIina3221::AveragingMode >( ( aPackedConfig >> 9 ) & 0x7 );
    aConfig.iChannel3Enable = static_cast< bool >( ( aPackedConfig >> 12 ) & 0x1 );
    aConfig.iChannel2Enable = static_cast< bool >( ( aPackedConfig >> 13 ) & 0x1 );
    aConfig.iChannel1Enable = static_cast< bool >( ( aPackedConfig >> 14 ) & 0x1 );
    aConfig.iRstart = static_cast< bool >( ( aPackedConfig >> 15 ) & 0x1 );
}

std::uint16_t
PackMaskEnable( const CIina3221::CMaskEnable& aMaskEnable ) NOEXCEPT
{
    std::uint16_t result = 0x0000;
    result |= static_cast< std::uint16_t >( aMaskEnable.iCVRF );
    result |= static_cast< std::uint16_t >( aMaskEnable.iTCF ) << 1;
    result |= static_cast< std::uint16_t >( aMaskEnable.iPVF ) << 2;
    result |= static_cast< std::uint16_t >( aMaskEnable.iWF3 ) << 3;
    result |= static_cast< std::uint16_t >( aMaskEnable.iWF2 ) << 4;
    result |= static_cast< std::uint16_t >( aMaskEnable.iWF1 ) << 5;
    result |= static_cast< std::uint16_t >( aMaskEnable.iSF ) << 6;
    result |= static_cast< std::uint16_t >( aMaskEnable.iCF3 ) << 7;
    result |= static_cast< std::uint16_t >( aMaskEnable.iCF2 ) << 8;
    result |= static_cast< std::uint16_t >( aMaskEnable.iCF1 ) << 9;
    result |= static_cast< std::uint16_t >( aMaskEnable.iCEN ) << 10;
    result |= static_cast< std::uint16_t >( aMaskEnable.iWEN ) << 11;
    result |= static_cast< std::uint16_t >( aMaskEnable.iSSC3 ) << 12;
    result |= static_cast< std::uint16_t >( aMaskEnable.iSSC2 ) << 13;
    result |= static_cast< std::uint16_t >( aMaskEnable.iSSC1 ) << 14;

    return result;
}

void
UnpackMaskEnable( CIina3221::CMaskEnable& aMaskEnable, std::uint16_t aPackedMaskEnable ) NOEXCEPT
{
    aMaskEnable.iCVRF = static_cast< bool >( aPackedMaskEnable & 0x1 );
    aMaskEnable.iTCF = static_cast< bool >( aPackedMaskEnable >> 1 & 0x1 );
    aMaskEnable.iPVF = static_cast< bool >( aPackedMaskEnable >> 2 & 0x1 );
    aMaskEnable.iWF3 = static_cast< bool >( aPackedMaskEnable >> 3 & 0x1 );
    aMaskEnable.iWF2 = static_cast< bool >( aPackedMaskEnable >> 4 & 0x1 );
    aMaskEnable.iWF1 = static_cast< bool >( aPackedMaskEnable >> 5 & 0x1 );
    aMaskEnable.iSF = static_cast< bool >( aPackedMaskEnable >> 6 & 0x1 );
    aMaskEnable.iCF3 = static_cast< bool >( aPackedMaskEnable >> 7 & 0x1 );
    aMaskEnable.iCF2 = static_cast< bool >( aPackedMaskEnable >> 8 & 0x1 );
    aMaskEnable.iCF1 = static_cast< bool >( aPackedMaskEnable >> 9 & 0x1 );
    aMaskEnable.iCEN = static_cast< bool >( aPackedMaskEnable >> 10 & 0x1 );
    aMaskEnable.iWEN = static_cast< bool >( aPackedMaskEnable >> 11 & 0x1 );
    aMaskEnable.iSSC3 = static_cast< bool >( aPackedMaskEnable >> 12 & 0x1 );
    aMaskEnable.iSSC2 = static_cast< bool >( aPackedMaskEnable >> 13 & 0x1 );
    aMaskEnable.iSSC1 = static_cast< bool >( aPackedMaskEnable >> 14 & 0x1 );
}

inline CIina3221::CConfig
UnpackConfig( std::uint16_t aPackedConfig ) NOEXCEPT
{
    CIina3221::CConfig config;
    UnpackConfig( config, aPackedConfig );
    return config;
}

inline constexpr std::uint8_t
MultiRegisterAddress( std::uint8_t aOffset,
                      std::uint8_t aPeriod,
                      std::uint8_t aRegisterNumber ) NOEXCEPT
{
    return aOffset + ( aPeriod * ( aRegisterNumber - 1 ) );
}

}  // namespace

CIina3221::CIina3221( IAbstractI2CBus& aI2CBus, std::uint8_t aDeviceAddress ) NOEXCEPT
    : iI2CBus{ aI2CBus },
      iDeviceAddress{ aDeviceAddress }
{
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
        const auto operationResult = Reset( aConfig );
        if ( operationResult != KOk )
        {
            // Unable to reset the device
            return operationResult;
        }
    }

    return KOk;
}

int
CIina3221::GetConfig( CConfig& aConfig ) NOEXCEPT
{
    std::uint16_t packedConfigRegister = 0x0000;
    const auto result = ReadRegister( KRegConfig, packedConfigRegister );
    if ( result == KOk )
    {
        UnpackConfig( aConfig, packedConfigRegister );
    }
    return result;
}

int
CIina3221::SetConfig( const CConfig& aConfig ) NOEXCEPT
{
    const std::uint16_t packedConfigRegister = PackConfig( aConfig );
    return WriteRegister( KRegConfig, packedConfigRegister );
}

int
CIina3221::ShuntVoltageV( float& aVoltage, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddressOffset = 0x01;
    constexpr std::uint8_t KRegisterPeriod = 2;

    return GetVoltageRegister< KRegisterAddressOffset, KRegisterPeriod >(
        aVoltage, KMaxShuntVoltage, aChannel );
}

int
CIina3221::BusVoltageV( float& aVoltage, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddressOffset = 0x02;
    constexpr std::uint8_t KRegisterPeriod = 2;

    return GetVoltageRegister< KRegisterAddressOffset, KRegisterPeriod >( aVoltage, KMaxBusVoltage,
                                                                          aChannel );
}

int
CIina3221::GetShuntCriticalAlertLimit( float& aShuntLimit, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddressOffset = 0x07;
    constexpr std::uint8_t KRegisterPeriod = 2;

    return GetVoltageRegister< KRegisterAddressOffset, KRegisterPeriod >(
        aShuntLimit, KMaxShuntVoltage, aChannel );
}

int
CIina3221::SetShuntCriticalAlertLimit( float aShuntLimit, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddressOffset = 0x07;
    constexpr std::uint8_t KRegisterPeriod = 2;

    return SetVoltageRegister< KRegisterAddressOffset, KRegisterPeriod >(
        aShuntLimit, KMaxShuntVoltage, aChannel );
}

int
CIina3221::GetShuntWarningAlertLimit( float& aShuntLimit, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddressOffset = 0x08;
    constexpr std::uint8_t KRegisterPeriod = 2;

    return GetVoltageRegister< KRegisterAddressOffset, KRegisterPeriod >(
        aShuntLimit, KMaxShuntVoltage, aChannel );
}

int
CIina3221::SetShuntWarningAlertLimit( float aShuntLimit, std::uint8_t aChannel ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddressOffset = 0x08;
    constexpr std::uint8_t KRegisterPeriod = 2;

    return SetVoltageRegister< KRegisterAddressOffset, KRegisterPeriod >(
        aShuntLimit, KMaxShuntVoltage, aChannel );
}

int
CIina3221::GetShuntVoltageSum( float& aShuntSum ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x0D;
    std::uint16_t voltageRegister;
    const auto result = ReadRegister( KRegisterAddress, voltageRegister );
    if ( result != KOk )
    {
        aShuntSum = BusRegisterToVoltage< 2 >( voltageRegister, KMaxShuntVoltage );
    }
    return result;
}

int
CIina3221::GetShuntVoltageSumLimit( float& aShuntSumLimit ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x0E;
    std::uint16_t voltageRegister = 0;
    const auto result = ReadRegister( KRegisterAddress, voltageRegister );
    if ( result == KOk )
    {
        aShuntSumLimit = BusRegisterToVoltage< 2 >( voltageRegister, KMaxShuntVoltage );
    }
    return result;
}

int
CIina3221::SetShuntVoltageSumLimit( float aShuntSumLimit ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x0E;
    const std::uint16_t voltageRegister
        = VoltageToBusRegister< 2 >( aShuntSumLimit, KMaxShuntVoltage );
    return WriteRegister( KRegisterAddress, voltageRegister );
}

int
CIina3221::GetMaskEnable( CMaskEnable& aMaskEnable ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x0F;
    std::uint16_t maskEnableRegister = 0x0000;
    const auto result = ReadRegister( KRegisterAddress, maskEnableRegister );
    if ( result == KOk )
    {
        UnpackMaskEnable( aMaskEnable, maskEnableRegister );
    }
    return result;
}

int
CIina3221::SetMaskEnable( const CMaskEnable& aMaskEnable ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x0F;
    const std::uint16_t maskEnableRegister = PackMaskEnable( aMaskEnable );
    return WriteRegister( KRegisterAddress, maskEnableRegister );
}

int
CIina3221::GetPowerValidUpperLimit( float& aPowerValidUpperLimit ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x10;
    std::uint16_t voltageRegister = 0;
    const auto result = ReadRegister( KRegisterAddress, voltageRegister );
    if ( result == KOk )
    {
        aPowerValidUpperLimit
            = BusRegisterToVoltage( voltageRegister, KFullScalePowerUperLimitVoltage,
                                    KFullScalePowerUperLimitRegisterValue );
    }
    return result;
}
int
CIina3221::SetPowerValidUpperLimit( float aPowerValidUpperLimit ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x10;
    const std::uint16_t voltageRegister
        = VoltageToBusRegister( aPowerValidUpperLimit, KFullScalePowerUperLimitVoltage,
                                KFullScalePowerUperLimitRegisterValue );
    return WriteRegister( KRegisterAddress, voltageRegister );
}

int
CIina3221::GetPowerValidLowerLimit( float& aPowerValidLowerLimit ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x11;
    std::uint16_t voltageRegister = 0;
    const auto result = ReadRegister( KRegisterAddress, voltageRegister );
    if ( result == KOk )
    {
        aPowerValidLowerLimit
            = BusRegisterToVoltage( voltageRegister, KFullScalePowerLowerLimitVoltage,
                                    KFullScalePowerLowerLimitRegisterValue );
    }
    return result;
}

int
CIina3221::SetPowerValidLowerLimit( float aPowerValidLowerLimit ) NOEXCEPT
{
    constexpr std::uint8_t KRegisterAddress = 0x11;
    const std::uint16_t voltageRegister
        = VoltageToBusRegister( aPowerValidLowerLimit, KFullScalePowerLowerLimitVoltage,
                                KFullScalePowerLowerLimitRegisterValue );
    return WriteRegister( KRegisterAddress, voltageRegister );
}

/************************ Private part ************************/
template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
int
CIina3221::GetVoltageRegister( std::uint16_t& aVoltageRegister, std::uint8_t aChannel ) NOEXCEPT
{
    if ( aChannel > KChannelNumber )
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
    if ( aChannel > KChannelNumber )
    {
        return KInvalidArgumentError;
    }

    return WriteRegister(
        MultiRegisterAddress( taMultiRegisterOffset, taMultiRegisterPeriod, aChannel ),
        aVoltageRegister );
}

template < std::uint8_t taMultiRegisterOffset,
           std::uint8_t taMultiRegisterPeriod,
           std::uint8_t taDataLShift >
int
CIina3221::GetVoltageRegister( float& aVoltage,
                               float aMaxAbsoluteVoltage,
                               std::uint8_t aChannel ) NOEXCEPT
{
    std::uint16_t voltageRegister = 0;
    const auto result = GetVoltageRegister< taMultiRegisterOffset, taMultiRegisterPeriod >(
        voltageRegister, aChannel );

    if ( result == KOk )
    {
        aVoltage = BusRegisterToVoltage< taDataLShift >( voltageRegister, aMaxAbsoluteVoltage );
    }
    return result;
}

template < std::uint8_t taMultiRegisterOffset,
           std::uint8_t taMultiRegisterPeriod,
           std::uint8_t taDataLShift >
int
CIina3221::SetVoltageRegister( float aVoltage,
                               float aMaxAbsoluteVoltage,
                               std::uint8_t aChannel ) NOEXCEPT
{
    const std::uint16_t voltageRegister
        = VoltageToBusRegister< taDataLShift >( aVoltage, aMaxAbsoluteVoltage );
    return SetVoltageRegister< taMultiRegisterOffset, taMultiRegisterPeriod >( voltageRegister,
                                                                               aChannel );
}

}  // namespace ExternalDevice