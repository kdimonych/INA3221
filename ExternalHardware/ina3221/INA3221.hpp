#pragma once

#include <cstdint>
#include <cstring>
#include <AbstractPlatform/common/Platform.hpp>
#include <AbstractPlatform/common/ErrorCode.hpp>
#include <AbstractPlatform/common/PlatformLiteral.hpp>
#include <AbstractPlatform/i2c/AbstractI2C.hpp>

namespace ExternalHardware
{
class CIina3221
{
public:
    static constexpr float KMaxBusVoltage = 32.76f;     // 0x0FFF = 32.76V
    static constexpr float KMaxShuntVoltage = 0.1638f;  // 0x0FFF = 0.1638V

    float iMaxBusVoltage = KMaxBusVoltage;
    float iMaxShuntVoltage = KMaxShuntVoltage;

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

    struct CMaskEnable
    {
        CMaskEnable( ){ };

        bool iCVRF = false;  // Bit 0
        bool iTCF = true;    // Bit 1
        bool iPVF = false;   // Bit 2

        bool iWF3 = false;  // Bit 3
        bool iWF2 = false;  // Bit 4
        bool iWF1 = false;  // Bit 5

        bool iSF = false;  // Bit 6

        bool iCF3 = false;  // Bit 7
        bool iCF2 = false;  // Bit 8
        bool iCF1 = false;  // Bit 9
        bool iCEN = false;  // Bit 10
        bool iWEN = false;  // Bit 11

        bool iSSC3 = false;  // Bit 12
        bool iSSC2 = false;  // Bit 13
        bool iSSC1 = false;  // Bit 14
    };

    CIina3221( AbstractPlatform::IAbstractI2CBus& aI2CBus,
               std::uint8_t aDeviceAddress = KDefaultAddress ) NOEXCEPT;
    ~CIina3221( ) = default;

    AbstractPlatform::TErrorCode Init( const CConfig& aConfig = { } ) NOEXCEPT;

    inline AbstractPlatform::TErrorCode
    Reset( ) NOEXCEPT
    {
        CConfig config;
        config.iRstart = true;
        return SetConfig( config ) == AbstractPlatform::KOk;
    }

    inline AbstractPlatform::TErrorCode
    Reset( CConfig aConfig ) NOEXCEPT
    {
        aConfig.iRstart = true;
        auto result = SetConfig( aConfig ) == AbstractPlatform::KOk;
        if ( result == AbstractPlatform::KOk )
        {
            aConfig.iRstart = false;
            return SetConfig( aConfig );
        }
        return result;
    }

    AbstractPlatform::TErrorCode GetConfig( CConfig& aConfig ) NOEXCEPT;
    AbstractPlatform::TErrorCode SetConfig( const CConfig& aConfig ) NOEXCEPT;

    AbstractPlatform::TErrorCode ShuntVoltageV( float& aVoltage,
                                                std::uint8_t aChannel = KChannel1 ) NOEXCEPT;
    AbstractPlatform::TErrorCode BusVoltageV( float& aVoltage,
                                              std::uint8_t aChannel = KChannel1 ) NOEXCEPT;

    AbstractPlatform::TErrorCode GetShuntCriticalAlertLimit( float& aShuntLimit,
                                                             std::uint8_t aChannel
                                                             = KChannel1 ) NOEXCEPT;
    AbstractPlatform::TErrorCode SetShuntCriticalAlertLimit( float aShuntLimit,
                                                             std::uint8_t aChannel
                                                             = KChannel1 ) NOEXCEPT;

    AbstractPlatform::TErrorCode GetShuntWarningAlertLimit( float& aShuntLimit,
                                                            std::uint8_t aChannel
                                                            = KChannel1 ) NOEXCEPT;
    AbstractPlatform::TErrorCode SetShuntWarningAlertLimit( float aShuntLimit,
                                                            std::uint8_t aChannel
                                                            = KChannel1 ) NOEXCEPT;

    AbstractPlatform::TErrorCode GetShuntVoltageSum( float& aShuntSum ) NOEXCEPT;

    AbstractPlatform::TErrorCode GetShuntVoltageSumLimit( float& aShuntSumLimit ) NOEXCEPT;
    AbstractPlatform::TErrorCode SetShuntVoltageSumLimit( float aShuntSumLimit ) NOEXCEPT;

    AbstractPlatform::TErrorCode GetMaskEnable( CMaskEnable& aMaskEnable ) NOEXCEPT;
    AbstractPlatform::TErrorCode SetMaskEnable( const CMaskEnable& aMaskEnable ) NOEXCEPT;

    AbstractPlatform::TErrorCode GetPowerValidUpperLimit( float& aPowerValidUpperLimit ) NOEXCEPT;
    AbstractPlatform::TErrorCode SetPowerValidUpperLimit( float aPowerValidUpperLimit ) NOEXCEPT;

    AbstractPlatform::TErrorCode GetPowerValidLowerLimit( float& aPowerValidLowerLimit ) NOEXCEPT;
    AbstractPlatform::TErrorCode SetPowerValidLowerLimit( float aPowerValidLowerLimit ) NOEXCEPT;

#ifdef __EXCEPTIONS
    inline float
    ShuntVoltageV( std::uint8_t aChannel = KChannel1 )
    {
        float voltage = 0.0;
        AbstractPlatform::ThrowOnError( ShuntVoltageV( voltage, aChannel ) );
        return voltage;
    }

    inline float
    BusVoltageV( std::uint8_t aChannel = KChannel1 )
    {
        float voltage = 0.0;
        AbstractPlatform::ThrowOnError( BusVoltageV( voltage, aChannel ) );
        return voltage;
    }

    inline float
    GetPowerValidUpperLimit( )
    {
        float voltage = 0.0;
        AbstractPlatform::ThrowOnError( GetPowerValidUpperLimit( voltage ) );
        return voltage;
    }

    inline float
    GetPowerValidLowerLimit( )
    {
        float voltage = 0.0;
        AbstractPlatform::ThrowOnError( GetPowerValidLowerLimit( voltage ) );
        return voltage;
    }
#endif

private:
    /* data */
    AbstractPlatform::IAbstractI2CBus& iI2CBus;
    const std::uint8_t iDeviceAddress;
    std::uint8_t iLastRegisterAddress = 0x00;

    template < typename taRegisterType >
    AbstractPlatform::TErrorCode ReadRegister( std::uint8_t aReg,
                                               taRegisterType& aRegisterValue ) NOEXCEPT;
    template < typename taRegisterType >
    AbstractPlatform::TErrorCode WriteRegister( std::uint8_t aReg,
                                                taRegisterType aRegisterValue ) NOEXCEPT;

    template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
    inline AbstractPlatform::TErrorCode GetVoltageRegister( std::uint16_t& aVoltageRegister,
                                                            std::uint8_t aChannel ) NOEXCEPT;
    template < std::uint8_t taMultiRegisterOffset, std::uint8_t taMultiRegisterPeriod >
    inline AbstractPlatform::TErrorCode SetVoltageRegister( std::uint16_t aVoltageRegister,
                                                            std::uint8_t aChannel ) NOEXCEPT;

    template < std::uint8_t taMultiRegisterOffset,
               std::uint8_t taMultiRegisterPeriod,
               std::uint8_t taDataLShift = 3 >
    inline AbstractPlatform::TErrorCode GetVoltageRegister( float& aVoltage,
                                                            float aMaxAbsoluteVoltage,
                                                            std::uint8_t aChannel ) NOEXCEPT;
    template < std::uint8_t taMultiRegisterOffset,
               std::uint8_t taMultiRegisterPeriod,
               std::uint8_t taDataLShift = 3 >
    inline AbstractPlatform::TErrorCode SetVoltageRegister( float aVoltage,
                                                            float aMaxAbsoluteVoltage,
                                                            std::uint8_t aChannel ) NOEXCEPT;
};

}  // namespace ExternalHardware