#include "CAN_XLAnalyzer.h"
#include "CAN_XLAnalyzerSettings.h"
#include <AnalyzerChannelData.h>
#include <sstream> // Include the <sstream> header

CAN_XLAnalyzer::CAN_XLAnalyzer() : Analyzer2(), mSettings( new CAN_XLAnalyzerSettings() ), mSimulationInitilized( false )
{
    SetAnalyzerSettings( mSettings.get() );
    UseFrameV2();
}

CAN_XLAnalyzer::~CAN_XLAnalyzer()
{
    KillThread();
}

void CAN_XLAnalyzer::SetupResults()
{
    mResults.reset( new CAN_XLAnalyzerResults( this, mSettings.get() ) );
    SetAnalyzerResults( mResults.get() );
    mResults->AddChannelBubblesWillAppearOn( mSettings->mInputChannel );
}

void CAN_XLAnalyzer::WorkerThread()
{
    const bool inverted = mSettings->inverted();
    mSampleRateHz = GetSampleRate();
    AnalyzerChannelData* serial = GetAnalyzerChannelData( mSettings->mInputChannel );


    //--- Sample settings
    mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate();
    mCurrentSamplePoint = mSettings->arbitrationSamplePoint() * mCurrentSamplesPerBit / 100; // sample to SP from bit start
    //--- initial  bus integration
    bool integrated = false;

    do
    {
        bool tmp1 = serial->GetBitState();
        if( serial->GetBitState() == BIT_HIGH )
        {
            const U64 start = serial->GetSampleNumber();
            serial->AdvanceToNextEdge();
            const U64 edge = serial->GetSampleNumber();
            if( ( edge - start ) > 7 * mCurrentSamplesPerBit )
            {
                addMark( edge, AnalyzerResults::Start );
                serial->Advance( mCurrentSamplePoint ); // align to sample point

                if( serial->GetBitState() == BIT_LOW )
                {
                    U64 sof_sp = serial->GetSampleNumber();
                    integrated = true;
                    // mPreviousBit = BIT_HIGH;
                    mUnstuffingActive = false;
                    mFrameFieldEngineState = FrameFieldEngineState::IDLE;
                    mResults->CancelPacketAndStartNewPacket();
                    // enterBit( BIT_LOW, sof_sp );
                }
            }
            else
            {
                addMark( edge, AnalyzerResults::ErrorX );
                serial->AdvanceToNextEdge();
            }
        }
        else
        {
            serial->AdvanceToNextEdge();
            addMark( serial->GetSampleNumber(), AnalyzerResults::ErrorX );
        }
        mResults->CommitResults();
    } while( !integrated );

    //---  bit decoding ( here should be on SP of SOF)
    U64 SP = serial->GetSampleNumber();


    // do frame processing and also next SOF integration
    do
    {
        const U64 nextEdge = serial->GetSampleOfNextEdge();
        bool currentBitValue = serial->GetBitState();

        while( SP < nextEdge )
        {
            enterBit( currentBitValue, SP ); // replace

            /* if( currentBitValue == BIT_HIGH )
            {
                addMark( SP, AnalyzerResults::One );
            }
            else
            {
                addMark( SP, AnalyzerResults::Zero );
            }*/

            SP += mCurrentSamplesPerBit;
        }

        mResults->CommitResults();

        // do hard sync on dominnat edge
        if( currentBitValue == BIT_HIGH )
        {
            serial->AdvanceToNextEdge();
            addMark( nextEdge, AnalyzerResults::DownArrow );
            serial->Advance( mCurrentSamplePoint ); // move to next SP
            SP = serial->GetSampleNumber();
        }
        else
        {
            serial->AdvanceToAbsPosition( SP );
        }

    } while( 1 );
}

/*


mSampleRateHz = GetSampleRate();

mSerial = GetAnalyzerChannelData( mSettings->mInputChannel );

if( mSerial->GetBitState() == BIT_LOW )
    mSerial->AdvanceToNextEdge();

U32 samples_per_bit = mSampleRateHz / mSettings->mBitRate;
U32 samples_to_first_center_of_first_data_bit = U32( 1.5 * double( mSampleRateHz ) / double( mSettings->mBitRate ) );

for( ; ; )
{
    U8 data = 0;
    U8 mask = 1 << 7;

    mSerial->AdvanceToNextEdge(); //falling edge -- beginning of the start bit

    U64 starting_sample = mSerial->GetSampleNumber();

    mSerial->Advance( samples_to_first_center_of_first_data_bit );

    for( U32 i=0; i<8; i++ )
    {
        //let's put a dot exactly where we sample this bit:
        mResults->AddMarker( mSerial->GetSampleNumber(), AnalyzerResults::Dot, mSettings->mInputChannel );

        if( mSerial->GetBitState() == BIT_HIGH )
            data |= mask;

        mSerial->Advance( samples_per_bit );

        mask = mask >> 1;
    }


    //we have a byte to save.
    Frame frame;
    frame.mData1 = data;
    frame.mFlags = 0;
    frame.mStartingSampleInclusive = starting_sample;
    frame.mEndingSampleInclusive = mSerial->GetSampleNumber();

    mResults->AddFrame( frame );
    mResults->CommitResults();
    ReportProgress( frame.mEndingSampleInclusive );
}*/

bool CAN_XLAnalyzer::NeedsRerun()
{
    return false;
}

U32 CAN_XLAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate,
                                            SimulationChannelDescriptor** simulation_channels )
{
    if( mSimulationInitilized == false )
    {
        mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
        mSimulationInitilized = true;
    }

    return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 CAN_XLAnalyzer::GetMinimumSampleRateHz()
{
    // return mSettings->mBitRate * 4;

    const U32 arbitrationBitRate = mSettings->arbitrationBitRate();
    const U32 XLBitRate = mSettings->XLBitRate();
    const U32 FDBitRate = mSettings->FDBitRate();
    const U32 max = ( FDBitRate > XLBitRate ) ? FDBitRate : XLBitRate;
    return max * 12;
}

const char* CAN_XLAnalyzer::GetAnalyzerName() const
{
    return "CAN XL decoder by CAN_XL";
}

const char* GetAnalyzerName()
{
    return "CAN XL decoder by CAN_XL";
}

Analyzer* CreateAnalyzer()
{
    return new CAN_XLAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
    delete analyzer;
}

//----------------------------------------------------------------------------------------
//  CAN FRAME DECODER
//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::enterBit( const bool inBit, U64& ioBitSamplePointNumber )
{
    if( mXLFixStuffingActive )
    {
        // XAN XL static stuff bit handling
        mXLbitsCount++;
        if( mXLbitsCount == 11 )
        {
            if( inBit != mPreviousBit )
            {
                // OK - fixed Stuff bit - discarded
                addMark( ioBitSamplePointNumber, AnalyzerResults::X );
                mXLbitsCount = 0;
            }
            else
            {
                // Stuff Error
                addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorX );
                // enterInErrorMode( ioBitSamplePointNumber + mCurrentSamplesPerBit / 2 );
                mXLbitsCount = 0;
            }
        }
        else
        {
            decodeFrameBit( inBit, ioBitSamplePointNumber );
        }
        mPreviousBit = inBit;
    }
    else if( !mUnstuffingActive )
    {
        decodeFrameBit( inBit, ioBitSamplePointNumber );
        mPreviousBit = inBit;
    }
    else if( ( mConsecutiveBitCountOfSamePolarity == 5 ) && ( inBit != mPreviousBit ) )
    {
        // Stuff bit - discarded
        addMark( ioBitSamplePointNumber, AnalyzerResults::X );
        mConsecutiveBitCountOfSamePolarity = 1;
        mPreviousBit = inBit;
        mStuffBitCount += 1;
        enterBitInCRC17( inBit );
        enterBitInCRC21( inBit );
    }
    else if( ( mConsecutiveBitCountOfSamePolarity == 5 ) && ( mPreviousBit == inBit ) )
    { // Stuff Error
        addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorX );
        //   enterInErrorMode( ioBitSamplePointNumber + mCurrentSamplesPerBit / 2 );
        mConsecutiveBitCountOfSamePolarity += 1;
    }
    else if( mPreviousBit == inBit )
    {
        mConsecutiveBitCountOfSamePolarity += 1;
        decodeFrameBit( inBit, ioBitSamplePointNumber );
        enterBitInCRC17( inBit );
        enterBitInCRC21( inBit );
    }
    else
    {
        mConsecutiveBitCountOfSamePolarity = 1;
        mPreviousBit = inBit;
        decodeFrameBit( inBit, ioBitSamplePointNumber );
        enterBitInCRC17( inBit );
        enterBitInCRC21( inBit );
    }
}

//----------------------------------------------------------------------------------------

static const uint8_t CANFD_LENGTH[ 16 ] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::decodeFrameBit( const bool inBit, U64& ioBitSamplePointNumber )
{
    switch( mFrameFieldEngineState )
    {
    case FrameFieldEngineState::IDLE:
        handle_IDLE_state( inBit, ioBitSamplePointNumber );
        break;
    /*/ case FrameFieldEngineState::SOF:
        handle_SOF_state( inBit, ioBitSamplePointNumber );
        break;*/
    case FrameFieldEngineState::IDENTIFIER:
        handle_IDENTIFIER_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CONTROL_EXTENDED:
        handle_CONTROL_EXTENDED_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CONTROL_BASE:
        handle_CONTROL_BASE_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CONTROL_CCFF_DLC:
        handle_CONTROL_CCFF_DLC_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CONTROL_FD:
        handle_CONTROL_FD_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CONTROL_XL:
        handle_CONTROL_XL_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::DATA:
        handle_DATA_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::XL_FCRC:
        handle_XL_FCRC_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::SBC:
        handle_SBC_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CRC15:
        handle_CRC15_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CRC17:
        handle_CRC17_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CRC21:
        handle_CRC21_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::XL_DAS:
        handle_XL_DAS_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::CRCDEL:
        handle_CRCDEL_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::ACK:
        handle_ACK_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::ENDOFFRAME:
        handle_ENDOFFRAME_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::INTERMISSION:
        handle_INTERMISSION_state( inBit, ioBitSamplePointNumber );
        break;
    case FrameFieldEngineState::DECODER_ERROR:
        handle_DECODER_ERROR_state( inBit, ioBitSamplePointNumber );
        break;
    default:
        break;
    }
}


//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_IDLE_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    if( inBit )
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::Stop );
    }
    else
    { // SOF
        mUnstuffingActive = true;
        mXLFixStuffingActive = false;
        mCRC15Accumulator = 0;
        mCRC17Accumulator = 1 << 16;
        mCRC21Accumulator = 1 << 20;
        mConsecutiveBitCountOfSamePolarity = 1;
        mPreviousBit = false;
        enterBitInCRC15( inBit );
        enterBitInCRC17( inBit );
        enterBitInCRC21( inBit );
        addMark( inBitSamplePointNumber, AnalyzerResults::Start ); // sample point
        mFieldBitIndex = 0;
        mIdentifier = 0;
        mStuffBitCount = 0;
        mFrameFieldEngineState = FrameFieldEngineState::IDENTIFIER;
        mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate();
        mStartOfFieldSampleNumber = inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
        mStartOfFrameSampleNumber = inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_IDENTIFIER_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    enterBitInCRC15( inBit );
    mFieldBitIndex++;
    if( mFieldBitIndex <= 11 )
    { // Standard identifier
        addMark( inBitSamplePointNumber, AnalyzerResults::Dot );
        mIdentifier <<= 1;
        mIdentifier |= inBit;
    }
    else if( mFieldBitIndex == 12 )
    { // RTR or SRR bit
        addMark( inBitSamplePointNumber, AnalyzerResults::Square );
        mFrameType = inBit ? FrameType::remote : FrameType::canData;
    }
    else if( mFieldBitIndex == 13 )
    { // IDE bit
        mFrameFormat = inBit ? FrameFormat::extended : FrameFormat::base;
        if( !inBit )
        { // IDE dominant -> base frame
            //--- IDE Mark
            addMark( inBitSamplePointNumber, AnalyzerResults::Zero );
            //--- bubble
            addBubble( STANDARD_IDENTIFIER_FIELD_RESULT, mIdentifier,
                       mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
                       inBitSamplePointNumber - mCurrentSamplesPerBit - mCurrentSamplePoint );
            mFieldBitIndex = 0;
            mStartOfFieldSampleNumber = inBitSamplePointNumber + mCurrentSamplesPerBit - mCurrentSamplePoint;
            mFrameFieldEngineState = FrameFieldEngineState::CONTROL_BASE;
        }
        else
        { // IDE recessive -> extended frame
            //--- SRR mark
            //   addMark( inBitSamplePointNumber - mCurrentSamplesPerBit, inBit ? AnalyzerResults::One : AnalyzerResults::ErrorSquare );
            //--- IDE Mark
            addMark( inBitSamplePointNumber, AnalyzerResults::One );
        }
    }
    else if( mFieldBitIndex < 32 )
    { // ID17 ... ID0
        addMark( inBitSamplePointNumber, AnalyzerResults::Dot );
        mIdentifier <<= 1;
        mIdentifier |= inBit;
    }
    else
    { // RTR
        mFrameType = inBit ? FrameType::remote : FrameType::canData;
        addMark( inBitSamplePointNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow );
        //--- Bubble
        addBubble( EXTENDED_IDENTIFIER_FIELD_RESULT, mIdentifier,
                   mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
                   inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
        mFieldBitIndex = 0;
        mFrameFieldEngineState = FrameFieldEngineState::CONTROL_EXTENDED;
    }
}

//----------------------------------------------------------------------------------------


void CAN_XLAnalyzer::handle_CONTROL_BASE_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    enterBitInCRC15( inBit );
    mFieldBitIndex++;
    switch( mFieldBitIndex )
    {
    case 1: // FDF bit
        if( inBit )
        { // FDF recessive -> CAN FD or XL frame - continue
            addMark( inBitSamplePointNumber, AnalyzerResults::One );
        }
        else
        { //  CCFF - switch to CC processing
            addMark( inBitSamplePointNumber, AnalyzerResults::Zero );
            mFieldBitIndex = 0;
            mDataCodeLength = 0;
            mFrameFieldEngineState = FrameFieldEngineState::CONTROL_CCFF_DLC;
        }
        break;
    case 2: // res or  XLF
        if( inBit )
        { // XLF bit recessive -> XL
            addMark( inBitSamplePointNumber, AnalyzerResults::One );
            mFrameType = FrameType::canXL;
            mUnstuffingActive = false;
            mFieldBitIndex = 0;
            mFrameFieldEngineState = FrameFieldEngineState::CONTROL_XL;
            addBubble( CANXL_ARB_FIELD_RESULT, 0, 0, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
        }
        else
        { //  dominant: -> FD - switch to FD processing
            addMark( inBitSamplePointNumber, AnalyzerResults::Zero );
            mFrameType = FrameType::canfdData;
            mFieldBitIndex = 0;
            mDataCodeLength = 0;
            mFrameFieldEngineState = FrameFieldEngineState::CONTROL_FD;
        }
        break;
    default:
        break;
    }
}

//----------------------------------------------------------------------------------------


void CAN_XLAnalyzer::handle_CONTROL_EXTENDED_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    enterBitInCRC15( inBit );
    mFieldBitIndex++;
    switch( mFieldBitIndex )
    {
    case 1: // FDF bit
        if( inBit )
        { // FDF recessive -> CAN FD
            addMark( inBitSamplePointNumber, AnalyzerResults::One );
            mFrameType = FrameType::canfdData;
        }
        else
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::DownArrow );
        }
        break;

    case 2: // res or r0
        if( inBit )
        { // R0 bit recessive -> error
            addMark( inBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( inBitSamplePointNumber );
        }
        else
        { // R0 dominant: ok
            addMark( inBitSamplePointNumber, AnalyzerResults::Zero );
            mDataCodeLength = 0;
            mFieldBitIndex = 0;
            if( mFrameType == FrameType::canfdData )
            {
                mFrameFieldEngineState = FrameFieldEngineState::CONTROL_FD;
            }
            else
            {
                mFrameFieldEngineState = FrameFieldEngineState::CONTROL_CCFF_DLC;
            }
        }
        break;
    }
}

//----------------------------------------------------------------------------------------
// for CCFF
void CAN_XLAnalyzer::handle_CONTROL_CCFF_DLC_state( const bool inBit, U64& ioBitSamplePointNumber )
{
    enterBitInCRC15( inBit );
    mFieldBitIndex++;
    if( ( mFrameType == FrameType::canData ) || ( mFrameType == FrameType::remote ) )
    {
        addMark( ioBitSamplePointNumber, AnalyzerResults::Dot );
        mDataCodeLength <<= 1;
        mDataCodeLength |= inBit;
        if( mFieldBitIndex == 4 )
        {
            addBubble( CAN20B_CONTROL_FIELD_RESULT, mDataCodeLength, 0,
                       ioBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
            mFieldBitIndex = 0;
            if( mDataCodeLength > 8 )
            {
                mDataCodeLength = 8;
            }
            mCRC15 = mCRC15Accumulator;
            if( mFrameType == FrameType::remote )
            {
                mFrameFieldEngineState = FrameFieldEngineState::CRC15;
            }
            else if( mDataCodeLength > 0 )
            {
                mFrameFieldEngineState = FrameFieldEngineState::DATA;
            }
            else if( mFrameType == FrameType::canData )
            {
                mFrameFieldEngineState = FrameFieldEngineState::CRC15;
            }
        }
    }
}

//----------------------------------------------------------------------------------------
// for FD CAN from the  BRS field
void CAN_XLAnalyzer::handle_CONTROL_FD_state( const bool inBit, U64& ioBitSamplePointNumber )
{
    U32 bitEnd = ioBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    enterBitInCRC15( inBit );
    mFieldBitIndex++;
    if( mFrameType == FrameType::canfdData )
    {
        if( mFieldBitIndex == 1 )
        { // BRS
            mBRS = inBit;
            if( inBit )
            { // Switch to data bit rate
                const U64 samplesForDataBitRate = mSampleRateHz / mSettings->FDBitRate();
                addMark( ioBitSamplePointNumber, AnalyzerResults::Dot );
                //--- Switch to Data Bit Rate
                mCurrentSamplesPerBit = samplesForDataBitRate;
                //--- Adjust for SP of next bit
                // shift sample poit to firts high speed bit SP and then back for 1bit, since SP willb eshifter by 1 bit on next loop
                mCurrentSamplePoint = mSettings->FDSamplePoint() * mCurrentSamplesPerBit / 100;
            }
            else
            {
                addMark( ioBitSamplePointNumber, AnalyzerResults::Zero );
            }
        }
        else if( mFieldBitIndex == 2 )
        { // ESI
            addMark( ioBitSamplePointNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow );
            mESI = inBit;
            mDataCodeLength = 0;
        }
        else
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
            mDataCodeLength <<= 1;
            mDataCodeLength |= inBit;
            if( mFieldBitIndex == 6 )
            {
                const U32 data2 = U32( mBRS ) | ( U32( mESI ) << 1 );
                addBubble( CANFD_CONTROL_FIELD_RESULT, mDataCodeLength, data2, bitEnd );
                mFieldBitIndex = 0;
                if( mDataCodeLength != 0 )
                {
                    mFrameFieldEngineState = FrameFieldEngineState::DATA;
                }
                else
                { // No Data, CANFD
                    mUnstuffingActive = false;
                    mFrameFieldEngineState = FrameFieldEngineState::SBC;
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_CONTROL_XL_state( const bool inBit, U64& ioBitSamplePointNumber )
{
    U32 bitEnd = ioBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    mFieldBitIndex++;
    if( mFieldBitIndex == 1 )
    { // resXL
        if( inBit )
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorX );
        }
        else
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::Dot );
        }
        mStartOfFieldSampleNumber = bitEnd; // adjust start of ADS field
    }
    else if( mFieldBitIndex == 2 )
    { // ADH
        if( inBit )
        { // Switch to XL bit rate
            const U64 samplesForXLBitRate = mSampleRateHz / mSettings->XLBitRate();
            const U64 XLTSEG2 = samplesForXLBitRate - ( mSettings->XLSamplePoint() * samplesForXLBitRate / 100 ); // XL SP in samples
            const U64 ARTSEG2 = mCurrentSamplesPerBit - mCurrentSamplePoint;                                      // XL SP in samples

            addMark( ioBitSamplePointNumber, AnalyzerResults::Stop );
            //--- Adjust for SP of next bit
            ioBitSamplePointNumber += ARTSEG2; // Advance to bit end
            ioBitSamplePointNumber -= XLTSEG2; // return back for XL bit TSEG2, to reach correct SP after SP increase for bit time
            mCurrentSamplesPerBit = samplesForXLBitRate;
            mCurrentSamplePoint = mSettings->XLSamplePoint() * mCurrentSamplesPerBit / 100; // sample to SP from bit start
        }
        else
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( ioBitSamplePointNumber );
        }
    }
    else if( mFieldBitIndex <= 4 )
    { // DH1
        // DH2
        if( inBit )
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::Start );
        }
        else
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( ioBitSamplePointNumber );
        }
    }
    else if( mFieldBitIndex == 5 )
    { // DL1
        if( !inBit )
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
            addBubble( CANXL_ADS_FIELD_RESULT, 0, 0, bitEnd );
            mXLFixStuffingActive = true;
            mXLbitsCount = 1;
        }
        else
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( ioBitSamplePointNumber );
        }
        // preset the start of SDT field
        mSDT = 0;
        mStartOfFieldSampleNumber = bitEnd;
    }
    else if( mFieldBitIndex <= 13 ) //   (5+8))
    {                               // SDT fied 8 bits
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        mSDT <<= 1;
        mSDT |= inBit;
        if( mFieldBitIndex == 13 )
        {
            addBubble( CANXL_SDT_FIELD_RESULT, mSDT, 0, bitEnd );
            mStartOfFieldSampleNumber = bitEnd; // adjust start of SEC field
        }
    }
    else if( mFieldBitIndex == 14 ) // SEC
    {
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        addBubble( CANXL_SEC_FIELD_RESULT, inBit, 0, bitEnd );
        mStartOfFieldSampleNumber = bitEnd + mCurrentSamplesPerBit; // adjust start of DLC field (+1 fix stuff bit)
        mDataCodeLength = 0;
    }
    else if( mFieldBitIndex <= 25 ) //   (14+11))
    {                               // DLC field = 10 bits
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        mDataCodeLength <<= 1;
        mDataCodeLength |= inBit;
        if( mFieldBitIndex == 25 )
        {
            addBubble( CANXL_DLC_FIELD_RESULT, mDataCodeLength, 0, bitEnd );
            mStartOfFieldSampleNumber = bitEnd; // adjust start of SBC field
            mXLSBC = 0;
        }
    }
    else if( mFieldBitIndex <= 28 ) //   (25+3))
    {                               // SBC field = 3 bits
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        mXLSBC <<= 1;
        mXLSBC |= inBit;
        if( mFieldBitIndex == 28 )
        {
            addBubble( CANXL_SBC_FIELD_RESULT, mXLSBC, 0, bitEnd );
            mStartOfFieldSampleNumber = bitEnd; // adjust start of PCRC field
            mPCRC = 0;
        }
    }
    else if( mFieldBitIndex <= 41 ) //   (28+13))
    {                               // PCRC field = 13 bits
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        mPCRC <<= 1;
        mPCRC |= inBit;
        if( mFieldBitIndex == 41 )
        {
            addBubble( CANXL_PCRC_FIELD_RESULT, mPCRC, 0, bitEnd );
            mStartOfFieldSampleNumber = bitEnd; // adjust start of VCID field
            mVCID = 0;
        }
    }
    else if( mFieldBitIndex <= 49 ) //   (41+8))
    {                               // VCID field = 8 bits
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        mVCID <<= 1;
        mVCID |= inBit;
        if( mFieldBitIndex == 49 )
        {
            addBubble( CANXL_VCID_FIELD_RESULT, mVCID, 0, bitEnd );
            mStartOfFieldSampleNumber = bitEnd; // adjust start of AF field
            mAF = 0;
        }
    }
    else if( mFieldBitIndex <= 81 ) //   (49+32))
    {                               // AF field = 32 bits
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        mAF <<= 1;
        mAF |= inBit;
        if( mFieldBitIndex == 81 )
        {
            addBubble( CANXL_AF_FIELD_RESULT, mAF, 0, bitEnd );
            mStartOfFieldSampleNumber = bitEnd; // adjust start of DATA field
            mFieldBitIndex = 0;
            mFrameFieldEngineState = FrameFieldEngineState::DATA;
        }
    }

    else
    {
        mResults->CommitPacketAndStartNewPacket();
    }
}


//----------------------------------------------------------------------------------------


void CAN_XLAnalyzer::handle_DATA_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    U32 bitEnd = inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    enterBitInCRC15( inBit );
    addMark( inBitSamplePointNumber, AnalyzerResults::Dot );
    mData[ mFieldBitIndex / 8 ] <<= 1;
    mData[ mFieldBitIndex / 8 ] |= inBit;
    mFieldBitIndex += 1;
    if( ( mFieldBitIndex % 8 ) == 0 )
    {
        const U32 dataIndex = ( mFieldBitIndex - 1 ) / 8;        
        addBubble( DATA_FIELD_RESULT, mData[ dataIndex ], dataIndex, bitEnd );
    }
    if( mFrameType == FrameType::canXL )
    {
        if( mFieldBitIndex == ( 8 * ( mDataCodeLength + 1 ) ) )
        {
            mFieldBitIndex = 0;
            mFCRC = 0;
            mStartOfFieldSampleNumber = bitEnd; // adjust start of FRCR field
            mFrameFieldEngineState = FrameFieldEngineState::XL_FCRC;
        }
    }
    else if( mFieldBitIndex == ( 8 * CANFD_LENGTH[ mDataCodeLength ] ) )
    {
        mFieldBitIndex = 0;
        if( mFrameType != FrameType::canfdData )
        {
            mCRC15 = mCRC15Accumulator;
            mFrameFieldEngineState = FrameFieldEngineState::CRC15;
        }
        else
        {
            mFrameFieldEngineState = FrameFieldEngineState::SBC;
            mUnstuffingActive = false;
        }
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_CRC15_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    enterBitInCRC15( inBit );
    addMark( inBitSamplePointNumber, AnalyzerResults::Dot );
    mFieldBitIndex += 1;
    if( mFieldBitIndex == 15 )
    {
        mFieldBitIndex = 0;
        mFrameFieldEngineState = FrameFieldEngineState::CRCDEL;
        addBubble( CRC15_FIELD_RESULT, mCRC15, mCRC15Accumulator, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
        if( mCRC15Accumulator != 0 )
        {
            mFrameFieldEngineState = DECODER_ERROR;
        }
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_SBC_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    mFieldBitIndex += 1;
    if( mFieldBitIndex == 1 )
    { // Forced Stuff Bit
        mSBCField = 0;
        if( inBit == mPreviousBit )
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::ErrorX );
            enterInErrorMode( inBitSamplePointNumber );
        }
        else
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::X );
        }
    }
    else if( mFieldBitIndex <= 4 )
    {
        enterBitInCRC17( inBit );
        enterBitInCRC21( inBit );
        mSBCField <<= 1;
        mSBCField |= inBit;
        addMark( inBitSamplePointNumber, AnalyzerResults::Dot );
    }
    else
    { // Parity bit
        enterBitInCRC17( inBit );
        enterBitInCRC21( inBit );
        const U8 GRAY_CODE_DECODER[ 8 ] = { 0, 1, 3, 2, 7, 6, 4, 5 };
        const U8 suffBitCountMod8 = GRAY_CODE_DECODER[ mSBCField ];
        mSBCField <<= 1;
        mSBCField |= inBit;
        //--- Check parity
        bool oneBitCountIsEven = true;
        U32 v = mSBCField;
        while( v > 0 )
        {
            oneBitCountIsEven ^= ( v & 1 ) != 0;
            v >>= 1;
        }
        addMark( inBitSamplePointNumber, oneBitCountIsEven ? AnalyzerResults::Dot : AnalyzerResults::ErrorX );
        const U32 data2 = ( ( mStuffBitCount % 8 ) << 1 ) | !oneBitCountIsEven;
        addBubble( SBC_FIELD_RESULT, suffBitCountMod8, data2, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
        mUnstuffingActive = false;
        mFieldBitIndex = 0;
        if( mDataCodeLength <= 10 )
        {
            mCRC17 = mCRC17Accumulator;
            mFrameFieldEngineState = FrameFieldEngineState::CRC17;
        }
        else
        {
            mCRC21 = mCRC21Accumulator;
            mFrameFieldEngineState = FrameFieldEngineState::CRC21;
        }
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_XL_FCRC_state( const bool inBit, const U64& ioBitSamplePointNumber )
{
    U32 bitEnd = ioBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    mFieldBitIndex++;

    if( mFieldBitIndex <= 32 ) //   FPCRC field = 32 bits
    {
        addMark( ioBitSamplePointNumber, AnalyzerResults::Square );
        mFCRC <<= 1;
        mFCRC |= inBit;
        if( mFieldBitIndex == 32 )
        {
            addBubble( CANXL_FCRC_FIELD_RESULT, mFCRC, 0, bitEnd );
            mStartOfFieldSampleNumber = bitEnd; // adjust start of FCP field
            mUnstuffingActive = false;
            mXLFixStuffingActive = false;
        }
    }
    else if( mFieldBitIndex <= 34 ) // FCP field 3-2
    {
        if( inBit )
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::One );
        }
        else
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( ioBitSamplePointNumber );
        }
    }
    else if( mFieldBitIndex <= 36 ) // FCP field 1-0
    {
        if( !inBit )
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::Zero );
        }
        else
        {
            addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( ioBitSamplePointNumber );
        }


        if( mFieldBitIndex == 36 ) // FCP field last bit, switch to ARB bit rate
        {
            const U64 samplesForXLBitRate = mSampleRateHz / mSettings->XLBitRate();
            const U64 samplesForARBBitRate = mSampleRateHz / mSettings->arbitrationBitRate();
            const U64 XLTSEG2 = samplesForXLBitRate - ( mSettings->XLSamplePoint() * samplesForXLBitRate / 100 ); // XL SP in samples
            const U64 ARTSEG2 =
                samplesForARBBitRate - ( mSettings->arbitrationSamplePoint() * samplesForARBBitRate / 100 ); // ARB SP in samples
            mCurrentSamplesPerBit = samplesForARBBitRate;
            mCurrentSamplePoint = mSettings->arbitrationSamplePoint() * mCurrentSamplesPerBit / 100; // sample to SP from bit start

            mFieldBitIndex = 0;
            mStartOfFieldSampleNumber = bitEnd; // adjust start of DAS field
            mFrameFieldEngineState = FrameFieldEngineState::XL_DAS;
        }
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_XL_DAS_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    U32 bitEnd = inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    mFieldBitIndex++;
    switch( mFieldBitIndex )
    {
    case 1:
        // mStartOfFieldSampleNumber = bitEnd;
    case 2:
        if( inBit )
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::One );
        }
        else
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( inBitSamplePointNumber );
        }
        break;
    case 3:
        if( !inBit )
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::Zero );
        }
        else
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( inBitSamplePointNumber );
        }
        break;
    case 4:
        if( inBit )
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::One );
            addBubble( CANXL_DAS_FIELD_RESULT, 1, 0, bitEnd );
            mFieldBitIndex = 0;
            mStartOfFieldSampleNumber = bitEnd; // adjust start of ACK field
            mFrameFieldEngineState = FrameFieldEngineState::ACK;
        }
        else
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( inBitSamplePointNumber );
        }
        break;
    default:
        return;
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_CRC17_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    if( ( mFieldBitIndex % 5 ) != 0 )
    {
        enterBitInCRC17( inBit );
        addMark( inBitSamplePointNumber, AnalyzerResults::Dot );
    }
    else if( inBit == mPreviousBit )
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::ErrorX );
        enterInErrorMode( inBitSamplePointNumber );
    }
    else
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::X );
    }
    mFieldBitIndex += 1;
    if( mFieldBitIndex == 22 )
    {
        mFieldBitIndex = 0;
        mFrameFieldEngineState = FrameFieldEngineState::CRCDEL;
        addBubble( CRC17_FIELD_RESULT, mCRC17, mCRC17Accumulator, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_CRC21_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    if( ( mFieldBitIndex % 5 ) != 0 )
    {
        enterBitInCRC21( inBit );
        addMark( inBitSamplePointNumber, AnalyzerResults::Dot );
    }
    else if( inBit == mPreviousBit )
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::ErrorX );
        enterInErrorMode( inBitSamplePointNumber );
    }
    else
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::X );
    }
    mFieldBitIndex++;
    if( mFieldBitIndex == 27 )
    {
        mFieldBitIndex = 0;
        mFrameFieldEngineState = FrameFieldEngineState::CRCDEL;
        addBubble( CRC21_FIELD_RESULT, mCRC21, mCRC21Accumulator, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_CRCDEL_state( const bool inBit, U64& ioBitSamplePointNumber )
{
    mUnstuffingActive = false;
    if( inBit )
    { // Handle Bit Rate Switch: data bit rate -> arbitration bit rate
        const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate();
        const U64 CRCDELsamplesX100 =
            mSettings->FDSamplePoint() * mCurrentSamplesPerBit + ( 100 - mSettings->arbitrationSamplePoint() ) * samplesPerArbitrationBit;
        const U64 centerCRCDEL = ioBitSamplePointNumber - mCurrentSamplesPerBit / 2 + CRCDELsamplesX100 / 200;
        addMark( centerCRCDEL, AnalyzerResults::One );
        //--- Adjust for center of next bit
        ioBitSamplePointNumber -= mCurrentSamplesPerBit / 2;    // Returns at the beginning of CRCDEL bit
        ioBitSamplePointNumber += CRCDELsamplesX100 / 100;      // Advance at the beginning of next bit
        ioBitSamplePointNumber -= samplesPerArbitrationBit / 2; // Back half of a arbitration bit rate bit
                                                                //--- Switch to Data Bit Rate
        mCurrentSamplesPerBit = samplesPerArbitrationBit;
    }
    else
    {
        addMark( ioBitSamplePointNumber, AnalyzerResults::ErrorX );
        enterInErrorMode( ioBitSamplePointNumber );
    }
    mStartOfFieldSampleNumber = ioBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    mFrameFieldEngineState = FrameFieldEngineState::ACK;
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_ACK_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    U32 bitEnd = inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit;
    mFieldBitIndex++;
    if( mFieldBitIndex == 1 )
    { // ACK SLOT
        addMark( inBitSamplePointNumber, inBit ? AnalyzerResults::ErrorSquare : AnalyzerResults::DownArrow );
        mAcked = inBit;

        if( mSettings->compactList() )
        {
           addFrameBubble( mStartOfFrameSampleNumber, inBitSamplePointNumber - mCurrentSamplePoint + 8*mCurrentSamplesPerBit );
        }


    }
    else
    { // ACK DELIMITER
        addBubble( ACK_FIELD_RESULT, mAcked, 0, bitEnd );       
        mFrameFieldEngineState = FrameFieldEngineState::ENDOFFRAME;        
        if( inBit )
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::One );
        }
        else
        {
            addMark( inBitSamplePointNumber, AnalyzerResults::ErrorDot );
            enterInErrorMode( inBitSamplePointNumber );
        }
        mFieldBitIndex = 0;
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_ENDOFFRAME_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    if( inBit )
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::One );
    }
    else
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::ErrorX );
        enterInErrorMode( inBitSamplePointNumber );
    }
    mFieldBitIndex++;
    if( mFieldBitIndex == 7 )
    {
        addBubble( EOF_FIELD_RESULT, 0, 0, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
        mFieldBitIndex = 0;
        mFrameFieldEngineState = FrameFieldEngineState::INTERMISSION;

     //   if( mSettings->compactList() )
     //   {
     //       addFrameBubble( mStartOfFrameSampleNumber, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );           
     //   }
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_INTERMISSION_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    if( inBit )
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::One );
    }
    else
    {
        addMark( inBitSamplePointNumber, AnalyzerResults::ErrorX );
        enterInErrorMode( inBitSamplePointNumber );
    }
    mFieldBitIndex++;
    if( mFieldBitIndex == 3 )
    {
        addBubble( INTERMISSION_FIELD_RESULT, 0, 0, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
        mFieldBitIndex = 0;
        mFrameFieldEngineState = FrameFieldEngineState::IDLE;
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::handle_DECODER_ERROR_state( const bool inBit, const U64 inBitSamplePointNumber )
{
    mUnstuffingActive = false;
    addMark( inBitSamplePointNumber, AnalyzerResults::ErrorDot );
    if( mPreviousBit != inBit )
    {
        mConsecutiveBitCountOfSamePolarity = 1;
        mPreviousBit = inBit;
    }
    else if( inBit )
    {
        mConsecutiveBitCountOfSamePolarity += 1;
        if( mConsecutiveBitCountOfSamePolarity == 11 )
        {
            addBubble( CAN_ERROR_RESULT, 0, 0, inBitSamplePointNumber - mCurrentSamplePoint + mCurrentSamplesPerBit );
            mFrameFieldEngineState = FrameFieldEngineState::IDLE;
        }
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::enterBitInCRC15( const bool inBit )
{
    const bool bit14 = ( mCRC15Accumulator & ( 1 << 14 ) ) != 0;
    const bool crc_nxt = inBit ^ bit14;
    mCRC15Accumulator <<= 1;
    mCRC15Accumulator &= 0x7FFF;
    if( crc_nxt )
    {
        mCRC15Accumulator ^= 0x4599;
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::enterBitInCRC17( const bool inBit )
{
    const bool bit16 = ( mCRC17Accumulator & ( 1 << 16 ) ) != 0;
    const bool crc_nxt = inBit ^ bit16;
    mCRC17Accumulator <<= 1;
    mCRC17Accumulator &= 0x1FFFF;
    if( crc_nxt )
    {
        mCRC17Accumulator ^= 0x1685B;
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::enterBitInCRC21( const bool inBit )
{
    const bool bit20 = ( mCRC21Accumulator & ( 1 << 20 ) ) != 0;
    const bool crc_nxt = inBit ^ bit20;
    mCRC21Accumulator <<= 1;
    mCRC21Accumulator &= 0x1FFFFF;
    if( crc_nxt )
    {
        mCRC21Accumulator ^= 0x102899;
    }
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::addMark( const U64 inBitSamplePointNumber, const AnalyzerResults::MarkerType inMarker )
{
    mResults->AddMarker( inBitSamplePointNumber, inMarker, mSettings->mInputChannel );
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::addFrameBubble( const U64 start, const U64 stop )
{
    FrameV2 frameV2;

    // const U8 idf[ 2 ] = { U8( inData1 >> 8 ), U8( inData1 ) };
    std::stringstream type;

    if( mFrameFormat == base )
    {
        const U8 idf[ 2 ] = { U8( mIdentifier >> 8 ), U8( mIdentifier ) };
        frameV2.AddByteArray( "ID", idf, 2 );
    }
    else
    {
        const U8 idf[ 4 ] = { U8( mIdentifier >> 24 ), U8( mIdentifier >> 16 ), U8( mIdentifier >> 8 ), U8( mIdentifier ) };
        frameV2.AddByteArray( "ID", idf, 4 );
    }

    frameV2.AddDouble( "DLC", mDataCodeLength );


    if( mFrameType == FrameType::canXL )
    {
     //   if( mDataCodeLength > 63 )
     //   {
     //       frameV2.AddByteArray( "DATA", mData, 63 );
     //   }
     //   else
      //  {
            frameV2.AddByteArray( "DATA", mData, mDataCodeLength + 1 );
      //  }
    }
    else
    {
        frameV2.AddByteArray( "DATA", mData, CANFD_LENGTH[ mDataCodeLength ] );
    }


    switch( mFrameType )
    {
    case canData:
        type << "CAN";
        break;
    case remote:
        type << "remote";
        break;
    case canfdData:
        type << "CAN FD";
        break;
    case canXL:
        type << "CAN XL";
        break;
    default:
        return;
    }


    mResults->AddFrameV2( frameV2, type.str().c_str(), start, stop );
}


//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::addBubble( const U8 inBubbleType, const U64 inData1, const U64 inData2, const U64 inBitSamplePointNumber )
{
    Frame frame;
    frame.mType = inBubbleType;
    frame.mFlags = 0;
    frame.mData1 = inData1;
    frame.mData2 = inData2;
    frame.mStartingSampleInclusive = mStartOfFieldSampleNumber;
    const U64 endSampleNumber = inBitSamplePointNumber;
    //+mCurrentSamplesPerBit - mCurrentSamplePoint;
   // U8 buf[ 8 ];

    std::stringstream str;

    frame.mEndingSampleInclusive = endSampleNumber;
    mResults->AddFrame( frame );


    FrameV2 frameV2;

    if( mSettings->compactList() == false )
    {
        switch( inBubbleType )
        {
        case STANDARD_IDENTIFIER_FIELD_RESULT:
        {
            const U8 idf[ 2 ] = { U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", idf, 2 );
            mResults->AddFrameV2( frameV2, "Std Id", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case EXTENDED_IDENTIFIER_FIELD_RESULT:
        {
            const U8 idf[ 4 ] = { U8( inData1 >> 24 ), U8( inData1 >> 16 ), U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", idf, 4 );
            mResults->AddFrameV2( frameV2, "Ext Id", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CAN20B_CONTROL_FIELD_RESULT:
            frameV2.AddByte( "Value", inData1 );
            mResults->AddFrameV2( frameV2, "Ctrl", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANFD_CONTROL_FIELD_RESULT:
        {
            frameV2.AddByte( "Value", inData1 );

            str << "Ctrl (FDF";
            if( ( inData2 & 1 ) != 0 )
            {
                str << ", BRS";
            }
            if( ( inData2 & 2 ) != 0 )
            {
                str << ", ESI";
            }
            str << ")";
            mResults->AddFrameV2( frameV2, str.str().c_str(), mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case DATA_FIELD_RESULT:
        {
            frameV2.AddByte( "Value", inData1 );
            str << "D" << inData2;
            mResults->AddFrameV2( frameV2, str.str().c_str(), mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CRC15_FIELD_RESULT:
        {
            const U8 crc[ 2 ] = { U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", crc, 2 );
            mResults->AddFrameV2( frameV2, "CRC15", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CRC17_FIELD_RESULT:
        {
            const U8 crc[ 3 ] = { U8( inData1 >> 16 ), U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", crc, 3 );
            mResults->AddFrameV2( frameV2, "CRC17", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CRC21_FIELD_RESULT:
        {
            const U8 crc[ 3 ] = { U8( inData1 >> 16 ), U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", crc, 3 );
            mResults->AddFrameV2( frameV2, "CRC21", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case ACK_FIELD_RESULT:
            mResults->AddFrameV2( frameV2, "ACK", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case EOF_FIELD_RESULT:
            mResults->AddFrameV2( frameV2, "EOF", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case INTERMISSION_FIELD_RESULT:
            mResults->AddFrameV2( frameV2, "IFS", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CAN_ERROR_RESULT:
            mResults->AddFrameV2( frameV2, "Error", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_ARB_FIELD_RESULT:
            mResults->AddFrameV2( frameV2, "FD/XL", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_ADS_FIELD_RESULT:
            //  frameV2.AddByte( "Value", 0 );
            mResults->AddFrameV2( frameV2, "ADS", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_SDT_FIELD_RESULT:
        {
            const U8 sdt[ 2 ] = { U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", sdt, 2 );
            mResults->AddFrameV2( frameV2, "SDT", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CANXL_SEC_FIELD_RESULT:
            frameV2.AddByte( "Value", ( U8 )inData1 );
            mResults->AddFrameV2( frameV2, "SEC", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_DLC_FIELD_RESULT:
            frameV2.AddByte( "Value", inData1 );
            mResults->AddFrameV2( frameV2, "DLC", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_SBC_FIELD_RESULT:
            frameV2.AddByte( "Value", inData1 );
            mResults->AddFrameV2( frameV2, "SBC", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_PCRC_FIELD_RESULT:
        {
            const U8 pcrc[ 2 ] = { U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", pcrc, 2 );
            mResults->AddFrameV2( frameV2, "PCRC", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CANXL_VCID_FIELD_RESULT:
            frameV2.AddByte( "Value", inData1 );
            mResults->AddFrameV2( frameV2, "VCID", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_AF_FIELD_RESULT:
        {
            const U8 af[ 4 ] = { U8( inData1 >> 24 ), U8( inData1 >> 16 ), U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", af, 4 );
            mResults->AddFrameV2( frameV2, "AF", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CANXL_DATA_FIELD_RESULT:
            frameV2.AddByte( "Value", inData1 );
            str << "D" << inData2;
            mResults->AddFrameV2( frameV2, str.str().c_str(), mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case CANXL_FCRC_FIELD_RESULT:
        {
            const U8 fcrc[ 4 ] = { U8( inData1 >> 24 ), U8( inData1 >> 16 ), U8( inData1 >> 8 ), U8( inData1 ) };
            frameV2.AddByteArray( "Value", fcrc, 4 );
            mResults->AddFrameV2( frameV2, "FCRC", mStartOfFieldSampleNumber, endSampleNumber );
        }
        break;
        case CANXL_DAS_FIELD_RESULT:
            frameV2.AddByte( "Value", inData1 );
            mResults->AddFrameV2( frameV2, "DAS", mStartOfFieldSampleNumber, endSampleNumber );
            break;
        case 0xFFF:
            break;
        }
    }

    mResults->CommitResults();
    ReportProgress( frame.mEndingSampleInclusive );
    //--- Prepare for next bubble
    mStartOfFieldSampleNumber = endSampleNumber;
}

//----------------------------------------------------------------------------------------

void CAN_XLAnalyzer::enterInErrorMode( const U64 inBitSamplePointNumber )
{
    mStartOfFieldSampleNumber = inBitSamplePointNumber;
    mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate();
    mFrameFieldEngineState = DECODER_ERROR;
    mUnstuffingActive = false;
}

//----------------------------------------------------------------------------------------
