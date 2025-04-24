#ifndef CAN_XL_ANALYZER_SETTINGS
#define CAN_XL_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

class CAN_XLAnalyzerSettings : public AnalyzerSettings
{
public:
	CAN_XLAnalyzerSettings();
	virtual ~CAN_XLAnalyzerSettings();

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings( const char* settings );
	virtual const char* SaveSettings();

	
	Channel mInputChannel;
	U32 mBitRate;

  public:
    U32 arbitrationBitRate( void ) const
    {
        return mArbitrationBitRate*1000;
    }

  public:
    U32 FDBitRate( void ) const
    {
        return mDataBitRate*1000;
    }

  public:
    U32 XLBitRate( void ) const
    {
        return mXLBitRate*1000;
    }

  public:
    U32 arbitrationSamplePoint( void ) const
    {
        return mArbitrationSamplePoint;
    }

  public:
    U32 FDSamplePoint( void ) const
    {
        return mDataSamplePoint;
    }

  public:
    U32 XLSamplePoint( void ) const
    {
        return mXLSamplePoint;
    }

  public:
    bool inverted( void ) const
    {
        return mInverted;
    }

  public:
      bool compactList(void) const
      {
          return !mCompactListing;
    }

protected:
	std::auto_ptr< AnalyzerSettingInterfaceChannel > mInputChannelInterface;
    std::shared_ptr<AnalyzerSettingInterfaceInteger> mArbitrationBitRateInterface;
    std::shared_ptr<AnalyzerSettingInterfaceInteger> mDataBitRateInterface;
    std::shared_ptr<AnalyzerSettingInterfaceInteger> mArbitrationSamplePointInterface;
    std::shared_ptr<AnalyzerSettingInterfaceInteger> mDataSamplePointInterface;
    std::shared_ptr<AnalyzerSettingInterfaceInteger> mXLBitRateInterface;
    std::shared_ptr<AnalyzerSettingInterfaceInteger> mXLSamplePointInterface;
    std::shared_ptr<AnalyzerSettingInterfaceNumberList> mCanChannelInvertedInterface;
    std::shared_ptr<AnalyzerSettingInterfaceBool> mCompactListingInterface;

  protected:
  
    U32 mArbitrationBitRate = 500;  
    U32 mDataBitRate = 2000;
    U32 mSimulatorRandomSeed;
    U32 mArbitrationSamplePoint = 80;  
    U32 mDataSamplePoint = 50;
    U32 mXLBitRate = 1000;
    U32 mXLSamplePoint = 50;
    bool mInverted = false;
    bool mCompactListing = false;


};

#endif //CAN_XL_ANALYZER_SETTINGS
