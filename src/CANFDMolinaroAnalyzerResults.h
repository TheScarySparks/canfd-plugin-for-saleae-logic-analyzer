#ifndef CANMOLINARO_ANALYZER_RESULTS
#define CANMOLINARO_ANALYZER_RESULTS

//----------------------------------------------------------------------------------------

#include <AnalyzerResults.h>

//----------------------------------------------------------------------------------------

enum CanFrameType {
  STANDARD_IDENTIFIER_FIELD_RESULT,
  EXTENDED_IDENTIFIER_FIELD_RESULT,
  RTR_FIELD_RESULT,
  SRR_FIELD_RESULT,
  IDE_FIELD_RESULT,
  R0_FIELD_RESULT,
  R1_FIELD_RESULT,
  FDF_FIELD_RESULT,
  BRS_FIELD_RESULT,
  ESI_FIELD_RESULT,
  CAN20B_CONTROL_FIELD_RESULT,
  CANFD_CONTROL_FIELD_RESULT,
  DATA_FIELD_RESULT,
  CRC15_FIELD_RESULT,
  CRC17_FIELD_RESULT,
  CRC21_FIELD_RESULT,
  CRC_DEL_FIELD_RESULT,
  SBC_FIELD_RESULT,
  ACK_FIELD_RESULT,
  ACK_DEL_FIELD_RESULT,
  EOF_FIELD_RESULT,
  INTERMISSION_FIELD_RESULT,
  CAN_ERROR_RESULT
} ;

//----------------------------------------------------------------------------------------
//  Which specific check failed when a frame enters the error state. Stored
//  as a small int code (Frame.mData1) since GenerateText is only ever
//  handed the Frame object, not live analyzer state. Mirrors the enum
//  added to the sibling classic CAN 2.0B plugin, extended with the extra
//  checks CAN FD adds (dual CRC widths, the ISO-only SBC field).
//----------------------------------------------------------------------------------------

enum CanErrorReason {
  ERROR_STUFF,
  ERROR_FORM_R0,
  ERROR_FORM_R1,
  ERROR_FORM_CRCDEL,
  ERROR_FORM_ACKDEL,
  ERROR_FORM_EOF,
  ERROR_FORM_INTERMISSION,
  ERROR_CRC15,
  ERROR_CRC17,
  ERROR_CRC21,
  ERROR_SBC_STUFF
} ;

const char* CanErrorReasonText (CanErrorReason inReason) ;

//----------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzer;
class CANFDMolinaroAnalyzerSettings;

//----------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzerResults : public AnalyzerResults {
public:
  CANFDMolinaroAnalyzerResults( CANFDMolinaroAnalyzer* analyzer, CANFDMolinaroAnalyzerSettings* settings );
  virtual ~CANFDMolinaroAnalyzerResults();

  virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
  virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

  virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
  virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
  virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions
  void GenerateText (const Frame & inFrame,
                     const DisplayBase inDisplayBase,
                     const bool inBubbleText,
                     std::stringstream & ioText) ;

protected:  //vars
  CANFDMolinaroAnalyzerSettings* mSettings;
  CANFDMolinaroAnalyzer* mAnalyzer;
};

//----------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_RESULTS
