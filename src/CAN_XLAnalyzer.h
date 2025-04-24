#ifndef CAN_XLANALYZER_H
#define CAN_XLANALYZER_H

#include <Analyzer.h>
#include "CAN_XLAnalyzerResults.h"
#include "CAN_XLSimulationDataGenerator.h"

class CAN_XLAnalyzerSettings;
class ANALYZER_EXPORT CAN_XLAnalyzer : public Analyzer2
{
public:
	CAN_XLAnalyzer();
	virtual ~CAN_XLAnalyzer();

	virtual void SetupResults();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
	virtual U32 GetMinimumSampleRateHz();

	virtual const char* GetAnalyzerName() const;
	virtual bool NeedsRerun();

protected: //vars
	std::auto_ptr< CAN_XLAnalyzerSettings > mSettings;
	std::auto_ptr< CAN_XLAnalyzerResults > mResults;
	AnalyzerChannelData* mSerial;

	CAN_XLSimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitilized;

	//Serial analysis vars:
	U32 mSampleRateHz;
	
//---------------- CAN decoder
  private:
      U64 mStartOfFieldSampleNumber;
      U64 mStartOfFrameSampleNumber;
      U32 mCurrentSamplesPerBit;
      U32 mCurrentSamplePoint;

//--- CAN protocol
  private:
      typedef enum
      {
          IDLE,
    //      SOF,
          IDENTIFIER,
          CONTROL_BASE,
          CONTROL_EXTENDED,
          CONTROL_CCFF_DLC,
          CONTROL_FD,
          CONTROL_XL,
          DATA,
          SBC,
          CRC15,
          CRC17,
          CRC21,
          CRCDEL,
          XL_FCRC,
          XL_DAS,
          ACK,
          ENDOFFRAME,
          INTERMISSION,
          DECODER_ERROR,
          dummy,
      } FrameFieldEngineState;

  private:
    FrameFieldEngineState mFrameFieldEngineState;
    int mFieldBitIndex;
    int mConsecutiveBitCountOfSamePolarity;
    bool mPreviousBit;
    bool mUnstuffingActive;
    bool mXLFixStuffingActive;
    int mXLbitsCount;

    //--- Received frame
    uint32_t mIdentifier;
    U32 mSBCField;
    U32 mStuffBitCount;
    U32 mDataCodeLength;
    U8 mData[ 2048 ];
    U16 mCRC15Accumulator;
    U16 mCRC15;
    U32 mCRC17Accumulator;
    U32 mCRC17;
    U32 mCRC21Accumulator;
    U32 mCRC21;
    // received XL frame fields
    U32 mSDT;
    U32 mSEC;   
    U32 mXLSBC;
    U32 mPCRC;
    U32 mVCID;
    U32 mAF;
   // U8 mXLData[ 2048 ];
    U32 mFCRC;




    typedef enum
    {
        base,
        extended
    } FrameFormat;
    
    FrameFormat mFrameFormat;
    
    typedef enum
    {
        canData,
        remote,
        canfdData,
        canXL
    } FrameType;
    
    FrameType mFrameType;

    bool mBRS;
    bool mESI;
    bool mAcked;


  private:
//    AnalyzerResults::MarkerType mMarkerTypeForDataAndCRC;

    //---------------- CAN decoder methods
  private:
    void enterBit( const bool inBit, U64& ioBitSamplePointNumber );
    void decodeFrameBit( const bool inBit, U64& ioBitSamplePointNumber );
    void enterBitInCRC15( const bool inBit );
    void enterBitInCRC17( const bool inBit );
    void enterBitInCRC21( const bool inBit );
    void addMark( const U64 inBitSamplePointNumber, const AnalyzerResults::MarkerType inMarker );
    void addBubble( const U8 inBubbleType, const U64 inData1, const U64 inData2, const U64 inEndSampleNumber );
    void enterInErrorMode( const U64 inBitSamplePointNumber );
    void handle_IDLE_state( const bool inBit, const U64 inBitSamplePointNumber );
  //  void handle_SOF_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_IDENTIFIER_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_CONTROL_BASE_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_CONTROL_EXTENDED_state( const bool inBit, const U64 inBitSamplePointNumber );
    //void handle_CONTROL_AFTER_R0_state( const bool inBit, U64& ioBitSamplePointNumber );
    void handle_CONTROL_CCFF_DLC_state( const bool inBit, U64& ioBitSamplePointNumber );
    void handle_CONTROL_FD_state( const bool inBit, U64& ioBitSamplePointNumber );
    void handle_CONTROL_XL_state( const bool inBit, U64& ioBitSamplePointNumber );
    void handle_DATA_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_SBC_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_XL_FCRC_state( const bool inBit, const U64& ioBitSamplePointNumber );
    void handle_XL_DAS_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_CRC15_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_CRC17_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_CRC21_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_CRCDEL_state( const bool inBit, U64& ioBitSamplePointNumber );
    void handle_ACK_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_ENDOFFRAME_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_INTERMISSION_state( const bool inBit, const U64 inBitSamplePointNumber );
    void handle_DECODER_ERROR_state( const bool inBit, const U64 inBitSamplePointNumber );    
    void addFrameBubble( const U64 start, const U64 stop );
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //CAN_XL_ANALYZER_H
