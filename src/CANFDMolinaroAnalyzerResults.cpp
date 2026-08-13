#include "CANFDMolinaroAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "CANFDMolinaroAnalyzer.h"
#include "CANFDMolinaroAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <sstream>

//----------------------------------------------------------------------------------------

const char* CanErrorReasonText (CanErrorReason inReason) {
  switch (inReason) {
  case ERROR_STUFF : return "Stuff Error" ;
  case ERROR_FORM_R0 : return "Form Error (R0)" ;
  case ERROR_FORM_R1 : return "Form Error (R1)" ;
  case ERROR_FORM_CRCDEL : return "Form Error (CRC Delimiter)" ;
  case ERROR_FORM_ACKDEL : return "Form Error (ACK Delimiter)" ;
  case ERROR_FORM_EOF : return "Form Error (EOF)" ;
  case ERROR_FORM_INTERMISSION : return "Form Error (Intermission)" ;
  case ERROR_CRC15 : return "CRC Error (CRC15)" ;
  case ERROR_CRC17 : return "CRC Error (CRC17)" ;
  case ERROR_CRC21 : return "CRC Error (CRC21)" ;
  case ERROR_SBC_STUFF : return "Stuff Error (SBC)" ;
  default : return "Error" ;
  }
}

//----------------------------------------------------------------------------------------
//  Mirrors CANFD_LENGTH in CANFDMolinaroAnalyzer.cpp (kept as a separate
//  copy since that one has internal linkage) -- DLC nibble (0-15) to
//  actual CAN FD payload byte count.
//----------------------------------------------------------------------------------------

static const uint8_t CANFD_LENGTH [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerResults::CANFDMolinaroAnalyzerResults (CANFDMolinaroAnalyzer* analyzer,
                                                            CANFDMolinaroAnalyzerSettings* settings ) :
AnalyzerResults(),
mSettings (settings),
mAnalyzer (analyzer) {
}

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerResults::~CANFDMolinaroAnalyzerResults () {
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateText (const Frame & inFrame,
                                               const DisplayBase inDisplayBase,
                                               const bool inBubbleText,
                                               std::stringstream & ioText) {
  char numberString [128] = "" ;
  switch (inFrame.mType) {
  case STANDARD_IDENTIFIER_FIELD_RESULT :
    snprintf (numberString, 128, "0x%03llX", inFrame.mData1) ;
    ioText << "STD ID: " << numberString << "\n" ;
    break ;
  case EXTENDED_IDENTIFIER_FIELD_RESULT :
    snprintf (numberString, 128, "0x%08llX", inFrame.mData1) ;
    ioText << "EXT ID: " << numberString << "\n" ;
    break ;
  case RTR_FIELD_RESULT :
    if (inBubbleText) {
      ioText << ((inFrame.mData1 == 0) ? "RTR: True\n" : "RTR: False\n") ;
    }
    break ;
  case SRR_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "SRR\n" ;
    }
    break ;
  case IDE_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "IDE\n" ;
    }
    break ;
  case R0_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "R0\n" ;
    }
    break ;
  case R1_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "R1\n" ;
    }
    break ;
  case FDF_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "FDF\n" ;
    }
    break ;
  case BRS_FIELD_RESULT :
    if (inBubbleText) {
      ioText << ((inFrame.mData1 != 0) ? "BRS: True\n" : "BRS: False\n") ;
    }
    break ;
  case ESI_FIELD_RESULT :
    if (inBubbleText) {
      ioText << ((inFrame.mData1 != 0) ? "ESI: True\n" : "ESI: False\n") ;
    }
    break ;
  case CAN20B_CONTROL_FIELD_RESULT :
    if (!inBubbleText) {
      ioText << "  " ;
    }
    ioText << "DLC: " << inFrame.mData1 << "\n" ;
    break ;
  case CANFD_CONTROL_FIELD_RESULT :
    { if (!inBubbleText) {
        ioText << "  " ;
      }
      const uint8_t dlcIndex = (inFrame.mData1 < 16) ? uint8_t (inFrame.mData1) : uint8_t (15) ;
      ioText << "DLC: " << U32 (CANFD_LENGTH [dlcIndex]) << "\n" ;
    }
    break ;
  case DATA_FIELD_RESULT :
    if (!inBubbleText) {
      ioText << "  " ;
    }
//    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 8, numberString, 128);
    snprintf (numberString, 128, "0x%02llX", inFrame.mData1) ;
    ioText << "D" << inFrame.mData2 << ": " << numberString << "\n" ;
    break ;
  case CRC15_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    if (!inBubbleText) {
      ioText << "  " ;
    }
    snprintf (numberString, 128, "0x%04llX", inFrame.mData1) ;
    ioText << "CRC15: " << numberString ;
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 16, numberString, 128);
    if (inFrame.mData2 != 0) {
      ioText << " (error)" ;
    }
    ioText << "\n" ;
    break ;
  case CRC17_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    if (!inBubbleText) {
      ioText << "  " ;
    }
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 20, numberString, 128);
    snprintf (numberString, 128, "0x%05llX", inFrame.mData1) ;
    ioText << "CRC17: " << numberString ;
    if (inFrame.mData2 != 0) {
      ioText << " (error)" ;
    }
    ioText << "\n" ;
    break ;
  case CRC21_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    if (!inBubbleText) {
      ioText << "  " ;
    }
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 24, numberString, 128);
    snprintf (numberString, 128, "0x%06llX", inFrame.mData1) ;
    ioText << "CRC21: " << numberString ;
    if (inFrame.mData2 != 0) {
      ioText << " (error)" ;
    }
    ioText << "\n" ;
    break ;
  case ACK_FIELD_RESULT :
    if (inBubbleText) {
      if (inFrame.mData1 != 0) {
        ioText << "NAK\n" ;
      }else{
        ioText << "ACK\n" ;
      }
    }
    break ;
  case ACK_DEL_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "ACK DEL\n" ;
    }
    break ;
  case CRC_DEL_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "CRC DEL\n" ;
    }
    break ;
  case SBC_FIELD_RESULT :
    { const bool parityError = (inFrame.mData2 & 1) != 0 ;
      const bool stuffBitCountError = (inFrame.mData2 >> 1) != inFrame.mData1 ;
      if (!inBubbleText) {
        ioText << "  " ;
      }
      ioText << "SBC: " << inFrame.mData1 ;
      if (parityError && stuffBitCountError) {
        ioText << " (error " << (inFrame.mData2 >> 1) << ", P)" ;
      }else if (parityError) {
        ioText << " (error P)" ;
      }else if (stuffBitCountError) {
        ioText << " (error " << (inFrame.mData2 >> 1) << ")" ;
      }
      ioText << "\n" ;
    } break ;
  case EOF_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "End of Frame\n" ;
    }
    break ;
  case INTERMISSION_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "3-bit intermission\n" ;
    }
    break ;
  case CAN_ERROR_RESULT :
    ioText << CanErrorReasonText (CanErrorReason (inFrame.mData1)) << "\n" ;
    break ;
  default :
    if (!inBubbleText) {
      ioText << "  " ;
    }
    ioText << "Error\n" ;
    break ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateBubbleText (const U64 inFrameIndex,
                                                     Channel& channel,
                                                     const DisplayBase inDisplayBase) {
  const Frame frame = GetFrame (inFrameIndex) ;
  std::stringstream text ;
  GenerateText (frame, inDisplayBase, true, text) ;
  ClearResultStrings () ;
  AddResultString (text.str().c_str ()) ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateFrameTabularText (const U64 inFrameIndex,
                                                           const DisplayBase inDisplayBase) {
  // Per-field Frame objects are kept only to position the waveform bubbles
  // (GenerateBubbleText, above, is untouched and still uses them). The
  // Data Table is fully covered by the one consolidated FrameV2 row per
  // message (emitConsolidatedFrameV2 in CANFDMolinaroAnalyzer.cpp), so
  // this used to just be redundant clutter -- a generic "Value" column
  // showing per-field text (raw ID, each individual data byte, etc.)
  // alongside the properly named FrameV2 columns that already cover the
  // same data.
  ClearTabularText () ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateExportFile (const char* file,
                                                       DisplayBase display_base,
                                                       U32 export_type_user_id) {
  std::ofstream file_stream (file, std::ios::out) ;

  const U64 trigger_sample = mAnalyzer->GetTriggerSample();
  const U32 sample_rate = mAnalyzer->GetSampleRate();

  file_stream << "Time [s],Value" << std::endl;

  U64 num_frames = GetNumFrames();
  for(U32 i = 0 ; i < num_frames ; i++) {
    Frame frame = GetFrame( i );

    char time_str[128] ;
    AnalyzerHelpers::GetTimeString (frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128);

    char number_str[128] ;
    AnalyzerHelpers::GetNumberString (frame.mData1, display_base, 8, number_str, 128) ;

    file_stream << time_str << "," << number_str << std::endl;

    if (UpdateExportProgressAndCheckForCancel (i, num_frames) == true) {
      file_stream.close () ;
      return ;
    }
  }

  file_stream.close();
}

//----------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzerResults::GeneratePacketTabularText (U64 packet_id, DisplayBase display_base) {
  //not supported
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateTransactionTabularText (U64 transaction_id,
                                                                   DisplayBase display_base) {
  //not supported
}

//----------------------------------------------------------------------------------------
