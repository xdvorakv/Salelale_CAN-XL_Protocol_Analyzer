#ifndef CAN_XL_ANALYZER_RESULTS
#define CAN_XL_ANALYZER_RESULTS

#include <AnalyzerResults.h>


enum CanFrameType
{
    STANDARD_IDENTIFIER_FIELD_RESULT,
    EXTENDED_IDENTIFIER_FIELD_RESULT,
    CAN20B_CONTROL_FIELD_RESULT,    
    CANFD_CONTROL_FIELD_RESULT,
    DATA_FIELD_RESULT,
    CRC15_FIELD_RESULT,
    CRC17_FIELD_RESULT,
    CRC21_FIELD_RESULT,
    SBC_FIELD_RESULT,
    ACK_FIELD_RESULT,
    EOF_FIELD_RESULT,
    INTERMISSION_FIELD_RESULT,
    CAN_ERROR_RESULT,
    CANXL_ARB_FIELD_RESULT,
    CANXL_ADS_FIELD_RESULT,
    CANXL_SDT_FIELD_RESULT,
    CANXL_SEC_FIELD_RESULT,
    CANXL_DLC_FIELD_RESULT,
    CANXL_SBC_FIELD_RESULT,
    CANXL_PCRC_FIELD_RESULT,
    CANXL_VCID_FIELD_RESULT,
    CANXL_AF_FIELD_RESULT,
    CANXL_DATA_FIELD_RESULT,
    CANXL_FCRC_FIELD_RESULT,
    CANXL_DAS_FIELD_RESULT
};



class CAN_XLAnalyzer;
class CAN_XLAnalyzerSettings;

class CAN_XLAnalyzerResults : public AnalyzerResults
{
public:
	CAN_XLAnalyzerResults( CAN_XLAnalyzer* analyzer, CAN_XLAnalyzerSettings* settings );
	virtual ~CAN_XLAnalyzerResults();

	virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
	virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

	virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
	virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
	virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions
    void GenerateText( const Frame& inFrame, const DisplayBase inDisplayBase, const bool inBubbleText, std::stringstream& ioText );
  protected:  //vars
	CAN_XLAnalyzerSettings* mSettings;
	CAN_XLAnalyzer* mAnalyzer;
};

#endif //CAN_XL_ANALYZER_RESULTS
