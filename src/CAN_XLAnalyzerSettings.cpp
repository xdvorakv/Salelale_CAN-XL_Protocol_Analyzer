#include "CAN_XLAnalyzerSettings.h"
#include <AnalyzerHelpers.h>


CAN_XLAnalyzerSettings::CAN_XLAnalyzerSettings()
:	mInputChannel( UNDEFINED_CHANNEL ), 
	mArbitrationBitRate( 500 ),
	mBitRate( 2000 )
{
	mInputChannelInterface.reset( new AnalyzerSettingInterfaceChannel() );
	mInputChannelInterface->SetTitleAndTooltip( "CAN XL", "My first experiment for CAN XL" );
	mInputChannelInterface->SetChannel( mInputChannel );
    //--- Arbitaration  br
	mArbitrationBitRateInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mArbitrationBitRateInterface->SetTitleAndTooltip( "Arbitration Bit Rate (kBits/s)", "Specify the bit rate in kbits per second." );
    mArbitrationBitRateInterface->SetMax( 1000 );
    mArbitrationBitRateInterface->SetMin( 10 );	
	mArbitrationBitRateInterface->SetInteger( mArbitrationBitRate );

    //--- FD data br
	mDataBitRateInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mDataBitRateInterface->SetTitleAndTooltip( "FD Data Bit Rate (kBits/s)", "Specify the FD bit rate in kbits per second." );
    mDataBitRateInterface->SetMax( 20000 );
    mDataBitRateInterface->SetMin( 125 );
    mDataBitRateInterface->SetInteger( mDataBitRate );

    //--- XL bit rate
    mXLBitRateInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mXLBitRateInterface->SetTitleAndTooltip( "XL Data Bit Rate (kBits/s)", "Specify the XL bit rate in kbits per seconds." );
    mXLBitRateInterface->SetMax( 20000 );
    mXLBitRateInterface->SetMin( 125 );
    mXLBitRateInterface->SetInteger( mXLBitRate );

    //--- Arbitration Sample Point
	mArbitrationSamplePointInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mArbitrationSamplePointInterface->SetTitleAndTooltip( "Arbitration Sample Point (%)", "Sample Point location in arbitration bit." );
    mArbitrationSamplePointInterface->SetMax( 90 );
    mArbitrationSamplePointInterface->SetMin( 40 );
    mArbitrationSamplePointInterface->SetInteger( mArbitrationSamplePoint );

    //--- Data Sample Point
	mDataSamplePointInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mDataSamplePointInterface->SetTitleAndTooltip( "Data Sample Point (%)", "Sample Point location in data bit." );

    mDataSamplePointInterface->SetMax( 90 );
    mDataSamplePointInterface->SetMin( 40 );
    mDataSamplePointInterface->SetInteger( mDataSamplePoint );

    //--- XL Sample Point
    mXLSamplePointInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mXLSamplePointInterface->SetTitleAndTooltip( "XL Sample Point (%)", "Sample Point location in XL data phase." );

    mXLSamplePointInterface->SetMax( 90 );
    mXLSamplePointInterface->SetMin( 20 );
    mXLSamplePointInterface->SetInteger( mXLSamplePoint );

    
    //--- Add Channel level inversion
    mCanChannelInvertedInterface.reset( new AnalyzerSettingInterfaceNumberList() );
    mCanChannelInvertedInterface->SetTitleAndTooltip( "Dominant Logic Level", "" );
    mCanChannelInvertedInterface->AddNumber( 0.0, "Low", "Low is the usual dominant level" );
    mCanChannelInvertedInterface->AddNumber( 1.0, "High", "High is the inverted dominant level" );

    //--- Add Channel level inversion
    mCompactListingInterface.reset( new AnalyzerSettingInterfaceBool() );
    mCompactListingInterface->SetTitleAndTooltip( "Enable detail listing to data table", "" );
    //mCompactListingInterface->AddNumber( 0.0, "Low", "Low is the usual dominant level" );
    //mCompactListingInterface->AddNumber( 1.0, "High", "High is the inverted dominant level" );

	AddInterface( mInputChannelInterface.get() );
    AddInterface( mArbitrationBitRateInterface.get() );
    AddInterface( mDataBitRateInterface.get() );    
    AddInterface( mXLBitRateInterface.get() );
    AddInterface( mArbitrationSamplePointInterface.get() );
    AddInterface( mDataSamplePointInterface.get() );    
    AddInterface( mXLSamplePointInterface.get() );
    AddInterface( mCanChannelInvertedInterface.get() );
    AddInterface( mCompactListingInterface.get() );

	AddExportOption( 0, "Export as text/csv file" );
	AddExportExtension( 0, "text", "txt" );
	AddExportExtension( 0, "csv", "csv" );

	ClearChannels();
	AddChannel( mInputChannel, "Serial", false );
}

CAN_XLAnalyzerSettings::~CAN_XLAnalyzerSettings()
{
}

bool CAN_XLAnalyzerSettings::SetSettingsFromInterfaces()
{
	mInputChannel = mInputChannelInterface->GetChannel();
    mArbitrationBitRate = mArbitrationBitRateInterface->GetInteger();
    mArbitrationSamplePoint = mArbitrationSamplePointInterface->GetInteger();
    mDataBitRate = mDataBitRateInterface->GetInteger();    
    mDataSamplePoint = mDataSamplePointInterface->GetInteger();
    mXLBitRate = mXLBitRateInterface->GetInteger();
    mXLSamplePoint = mXLSamplePointInterface->GetInteger();
    mInverted = U32( mCanChannelInvertedInterface->GetNumber() ) != 0;
    mCompactListing = mCompactListingInterface->GetValue();



	ClearChannels();
	AddChannel( mInputChannel, "CAN XL", true );

	return true;
}

void CAN_XLAnalyzerSettings::UpdateInterfacesFromSettings()
{
	mInputChannelInterface->SetChannel( mInputChannel );
    mArbitrationBitRateInterface->SetInteger( mArbitrationBitRate );
    mDataBitRateInterface->SetInteger( mDataBitRate );
    mArbitrationSamplePointInterface->SetInteger( mArbitrationSamplePoint );
    mDataSamplePointInterface->SetInteger( mDataSamplePoint );
    mCanChannelInvertedInterface->SetNumber( double( mInverted ) );
    mXLBitRateInterface->SetInteger( mXLBitRate );
    mXLSamplePointInterface->SetInteger( mXLSamplePoint );
    mCompactListingInterface->SetValue(mCompactListing );
}

void CAN_XLAnalyzerSettings::LoadSettings( const char* settings )
{
	SimpleArchive text_archive;
	text_archive.SetString( settings );

	text_archive >> mInputChannel;
    text_archive >> mArbitrationBitRate;
    text_archive >> mDataBitRate;
    text_archive >> mXLBitRate;
    text_archive >> mInverted;
    text_archive >> mArbitrationSamplePoint;
    text_archive >> mDataSamplePoint;
    text_archive >> mXLSamplePoint;
    text_archive >> mCompactListing;

	ClearChannels();
	AddChannel( mInputChannel, "CAN-XL by CAN_XL", true );

	UpdateInterfacesFromSettings();
}

const char* CAN_XLAnalyzerSettings::SaveSettings()
{
	SimpleArchive text_archive;

	text_archive << mInputChannel;
    text_archive << mArbitrationBitRate;
    text_archive << mDataBitRate;
    text_archive << mXLBitRate;
    text_archive << mInverted;
    text_archive << mArbitrationSamplePoint;
    text_archive << mDataSamplePoint;
    text_archive << mXLSamplePoint;
    text_archive << mCompactListing;

	return SetReturnString( text_archive.GetString() );
}
