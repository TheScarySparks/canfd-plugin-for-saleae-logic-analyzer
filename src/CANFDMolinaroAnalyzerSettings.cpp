#include "CANFDMolinaroAnalyzerSettings.h"
#include <AnalyzerHelpers.h>
#include <cctype>

//----------------------------------------------------------------------------------------
//  Whitespace-tolerant hex-byte-pair parser for the "Fixed Frame Data"
//  setting (e.g. "01 02 03 04" or "01020304"). Rejects anything that
//  isn't hex digits/whitespace, an odd digit count, or more than 64
//  bytes -- reported back through SetErrorText/return false in
//  SetSettingsFromInterfaces, same contract as the DBC folder path.
//----------------------------------------------------------------------------------------

static bool parseHexBytes (const std::string & inText, std::vector <uint8_t> & outBytes, std::string & outError) {
  outBytes.clear () ;
  std::string digits ;
  for (const char c : inText) {
    if (std::isspace (static_cast <unsigned char> (c))) {
      continue ;
    }
    if (! std::isxdigit (static_cast <unsigned char> (c))) {
      outError = "Fixed Frame Data contains a non-hex character" ;
      return false ;
    }
    digits += c ;
  }
  if ((digits.size () % 2) != 0) {
    outError = "Fixed Frame Data must have an even number of hex digits" ;
    return false ;
  }
  if (digits.size () > 128) {   // 64 bytes max
    outError = "Fixed Frame Data exceeds 64 bytes" ;
    return false ;
  }
  for (size_t i = 0 ; i < digits.size () ; i += 2) {
    outBytes.push_back (uint8_t (std::stoul (digits.substr (i, 2), nullptr, 16))) ;
  }
  return true ;
}

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerSettings::CANFDMolinaroAnalyzerSettings() :
mInputChannel (UNDEFINED_CHANNEL),
mArbitrationBitRate (1000 * 1000),
mDataBitRate (1000 * 1000) {
  mInputChannelInterface.reset (new AnalyzerSettingInterfaceChannel ());
  mInputChannelInterface->SetTitleAndTooltip ("Serial", "CAN FD");
  mInputChannelInterface->SetChannel (mInputChannel);

//--- Arbitration Bit Rate
  mArbitrationBitRateInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mArbitrationBitRateInterface->SetTitleAndTooltip ("CAN Arbitration Bit Rate (bit/s)",
                                         "CAN arbitration bit rate in bits per second." );

  mArbitrationBitRateInterface->SetMax (1 * 1000 * 1000) ;
  mArbitrationBitRateInterface->SetMin (1) ;
  mArbitrationBitRateInterface->SetInteger (mArbitrationBitRate) ;

//--- Simulator random Seed
  mSimulatorRandomSeedInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mSimulatorRandomSeedInterface->SetTitleAndTooltip ("Simulator Random Seed", "") ;
  mSimulatorRandomSeedInterface->SetMax (1 * 1000 * 1000) ;
  mSimulatorRandomSeedInterface->SetMin (0) ;
  mSimulatorRandomSeedInterface->SetInteger (mSimulatorRandomSeed) ;

//--- Data Bit Rate
  mDataBitRateInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mDataBitRateInterface->SetTitleAndTooltip ("CAN Data Bit Rate (bit/s)",
                            "CAN data bit rate in bits per second, a multiple of Arbitration Bit Rate." );

  mDataBitRateInterface->SetMax (12 * 1000 * 1000) ;
  mDataBitRateInterface->SetMin (1) ;
  mDataBitRateInterface->SetInteger (mDataBitRate) ;

//--- Arbitration Sample Point
  mArbitrationSamplePointInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mArbitrationSamplePointInterface->SetTitleAndTooltip ("Arbitration Sample Point (%)",
                            "Sample Point location in arbitration bit." );

  mArbitrationSamplePointInterface->SetMax (90) ;
  mArbitrationSamplePointInterface->SetMin (50) ;
  mArbitrationSamplePointInterface->SetInteger (mArbitrationSamplePoint) ;

//--- Data Sample Point
  mDataSamplePointInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mDataSamplePointInterface->SetTitleAndTooltip ("Data Sample Point (%)",
                            "Sample Point location in data bit." );

  mDataSamplePointInterface->SetMax (90) ;
  mDataSamplePointInterface->SetMin (50) ;
  mDataSamplePointInterface->SetInteger (mDataSamplePoint) ;

//--- DBC Folder -- optional, decodes DATA bytes into named signals using
//    every .dbc file found in the folder. Empty = feature off.
  mDbcFolderInterface.reset (new AnalyzerSettingInterfaceText ()) ;
  mDbcFolderInterface->SetTitleAndTooltip ("DBC Folder (optional)",
                                           "Decode data bytes into named signals using every .dbc file found in this folder.") ;
  mDbcFolderInterface->SetTextType (AnalyzerSettingInterfaceText::FolderPath) ;

//--- Fixed Test Frame -- lets one specific ID+data frame be reproduced on
//    demand instead of hunting through random simulator output (e.g. to
//    validate a specific DBC message decodes correctly).
  mUseFixedTestFrameInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mUseFixedTestFrameInterface->SetTitleAndTooltip ("Use Fixed Test Frame", "") ;
  mUseFixedTestFrameInterface->AddNumber (double (FIXED_TEST_FRAME_DISABLED), "Disabled", "") ;
  mUseFixedTestFrameInterface->AddNumber (double (FIXED_TEST_FRAME_ENABLED), "Enabled", "") ;
  mUseFixedTestFrameInterface->SetNumber (double (FIXED_TEST_FRAME_DISABLED)) ;

  mFixedFrameFormatInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mFixedFrameFormatInterface->SetTitleAndTooltip ("Fixed Frame ID Format", "") ;
  mFixedFrameFormatInterface->AddNumber (double (FIXED_FRAME_FORMAT_STANDARD), "Standard", "") ;
  mFixedFrameFormatInterface->AddNumber (double (FIXED_FRAME_FORMAT_EXTENDED), "Extended", "") ;
  mFixedFrameFormatInterface->SetNumber (double (FIXED_FRAME_FORMAT_STANDARD)) ;

  mFixedFrameIdInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
  mFixedFrameIdInterface->SetTitleAndTooltip ("Fixed Frame ID",
                                              "Used only when Use Fixed Test Frame is Enabled. Masked to 11 bits if Format is Standard.") ;
  mFixedFrameIdInterface->SetMax (0x1FFFFFFF) ;
  mFixedFrameIdInterface->SetMin (0) ;
  mFixedFrameIdInterface->SetInteger (mFixedFrameId) ;

  mFixedFrameDataInterface.reset (new AnalyzerSettingInterfaceText ()) ;
  mFixedFrameDataInterface->SetTitleAndTooltip ("Fixed Frame Data (hex bytes)",
                                                "e.g. \"01 02 03 04\". Used only when Use Fixed Test Frame is Enabled. Up to 64 bytes for CAN FD frames (see Simulator Generated Frames Format), 8 for classic frames -- data is truncated or length-rounded-up to fit whichever the Format setting currently selects.") ;
  mFixedFrameDataInterface->SetTextType (AnalyzerSettingInterfaceText::NormalText) ;

//--- Add Channel level inversion
  mCanChannelInvertedInterface.reset (new AnalyzerSettingInterfaceNumberList ( )) ;
  mCanChannelInvertedInterface->SetTitleAndTooltip ("Dominant Logic Level", "" );
  mCanChannelInvertedInterface->AddNumber (0.0,
                                           "Low",
                                           "Low is the usual dominant level") ;
  mCanChannelInvertedInterface->AddNumber (1.0,
                                           "High",
                                           "High is the inverted dominant level") ;
  mCanChannelInvertedInterface->SetNumber (1.0) ;

//--- Add Protocol
  mProtocolInterface.reset (new AnalyzerSettingInterfaceNumberList ( )) ;
  mProtocolInterface->SetTitleAndTooltip ("CANFD Protocol", "" );
  mProtocolInterface->AddNumber (0.0, "ISO", "") ;
  mProtocolInterface->AddNumber (1.0, "Non IS0", "") ;

//--- Simulator ACK level
  mSimulatorAckGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mSimulatorAckGenerationInterface->SetTitleAndTooltip ("Simulator ACK SLOT generated level", "");
  mSimulatorAckGenerationInterface->AddNumber (0.0,
                                               "Dominant",
                                               "Dominant is the valid level for ACK SLOT") ;
  mSimulatorAckGenerationInterface->AddNumber (1.0,
                                               "Recessive",
                                               "Recessive is the invalid level for ACK SLOT") ;
  mSimulatorAckGenerationInterface->AddNumber (2.0,
                                               "Random",
                               "The simulator generates dominant or recessive level randomly") ;

//--- Simulator BSR level
  mSimulatorBSRGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mSimulatorBSRGenerationInterface->SetTitleAndTooltip ("Simulator BSR generated level", "");
  mSimulatorBSRGenerationInterface->AddNumber (0.0,
                                               "Dominant (CANFD data sent with Arbitration Bit Rate)",
                                               "") ;
  mSimulatorBSRGenerationInterface->AddNumber (1.0,
                                               "Recessive (CANFD data sent with Data Bit Rate)",
                                               "") ;
  mSimulatorBSRGenerationInterface->AddNumber (2.0,
                                               "Random",
                               "The simulator generates dominant or recessive level randomly") ;

//--- Simulator ESI level
  mSimulatorESIGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mSimulatorESIGenerationInterface->SetTitleAndTooltip ("Simulator ESI generated level", "");
  mSimulatorESIGenerationInterface->AddNumber (0.0,
                                               "Dominant (means the sender is error active)",
                                               "Dominant means the sender is error active") ;
  mSimulatorESIGenerationInterface->AddNumber (1.0,
                                               "Recessive (means the sender is error passive)",
                                               "Recessive means the sender is error passive") ;
  mSimulatorESIGenerationInterface->AddNumber (2.0,
                                               "Random",
                               "The simulator generates dominant or recessive level randomly") ;

//--- Simulator Generated frames
  mSimulatorFrameTypeGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
  mSimulatorFrameTypeGenerationInterface->SetTitleAndTooltip ("Simulator Generated Frames", "");
  mSimulatorFrameTypeGenerationInterface->AddNumber (0.0, "All Types", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (1.0, "Only CAN2.0B Standard Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (2.0, "Only CAN2.0B Extended Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (3.0, "Only CAN2.0B Standard Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (4.0, "Only CAN2.0B Extended Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (5.0, "Only CANFD Base Data Frames, 0-16 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (6.0, "Only CANFD Extended Data Frames, 0-16 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (7.0, "Only CANFD Base Data Frames, 20-64 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (8.0, "Only CANFD Extended Data Frames, 20-64 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (0.0) ;

//--- Install interfaces
  AddInterface (mInputChannelInterface.get ()) ;
  AddInterface (mArbitrationBitRateInterface.get ());
  AddInterface (mDataBitRateInterface.get ());
  AddInterface (mCanChannelInvertedInterface.get ());
  AddInterface (mArbitrationSamplePointInterface.get ());
  AddInterface (mDataSamplePointInterface.get ());
  AddInterface (mProtocolInterface.get ());
  AddInterface (mSimulatorRandomSeedInterface.get ());
  AddInterface (mDbcFolderInterface.get ());
  AddInterface (mUseFixedTestFrameInterface.get ());
  AddInterface (mFixedFrameFormatInterface.get ());
  AddInterface (mFixedFrameIdInterface.get ());
  AddInterface (mFixedFrameDataInterface.get ());
  AddInterface (mSimulatorAckGenerationInterface.get ());
  AddInterface (mSimulatorFrameTypeGenerationInterface.get ());
  AddInterface (mSimulatorBSRGenerationInterface.get ());
  AddInterface (mSimulatorESIGenerationInterface.get ());

  AddExportOption( 0, "Export as text/csv file" );
  AddExportExtension( 0, "text", "txt" );
  AddExportExtension( 0, "csv", "csv" );

  ClearChannels ();
  AddChannel (mInputChannel, "Serial", false) ;
}

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerSettings::~CANFDMolinaroAnalyzerSettings(){
}

//----------------------------------------------------------------------------------------

bool CANFDMolinaroAnalyzerSettings::SetSettingsFromInterfaces () {
  mInputChannel = mInputChannelInterface->GetChannel();

  mArbitrationSamplePoint = mArbitrationSamplePointInterface->GetInteger();
  mDataSamplePoint = mDataSamplePointInterface->GetInteger();
  mArbitrationBitRate = mArbitrationBitRateInterface->GetInteger();
  mSimulatorRandomSeed = mSimulatorRandomSeedInterface->GetInteger () ;
  mDataBitRate = mDataBitRateInterface->GetInteger();

  mInverted = U32 (mCanChannelInvertedInterface->GetNumber ()) != 0 ;

  mProtocol = ProtocolSetting (mProtocolInterface->GetNumber ()) ;

  mSimulatorGeneratedAckSlot
    = SimulatorGeneratedBit (mSimulatorAckGenerationInterface->GetNumber ()) ;

  mSimulatorGeneratedFrameType
    = SimulatorGeneratedFrameType (mSimulatorFrameTypeGenerationInterface->GetNumber ()) ;

  mSimulatorGeneratedBSRSlot
    = SimulatorGeneratedBit (mSimulatorBSRGenerationInterface->GetNumber ()) ;

  mSimulatorGeneratedESISlot
    = SimulatorGeneratedBit (mSimulatorESIGenerationInterface->GetNumber ()) ;

  { const char * path = mDbcFolderInterface->GetText () ;
    mDbcFolderPath = path ? path : "" ;
    if (! mDbcFolderPath.empty ()) {
      std::string err ;
      if (! mDbcDatabase.LoadFromFolder (mDbcFolderPath, err)) {
        SetErrorText (err.c_str ()) ;
        return false ;
      }
    }else{
      mDbcDatabase.Clear () ;
    }
  }

  mUseFixedTestFrame = U32 (mUseFixedTestFrameInterface->GetNumber ()) == FIXED_TEST_FRAME_ENABLED ;
  mFixedFrameExtended = U32 (mFixedFrameFormatInterface->GetNumber ()) == FIXED_FRAME_FORMAT_EXTENDED ;
  mFixedFrameId = mFixedFrameIdInterface->GetInteger () ;
  { const char * text = mFixedFrameDataInterface->GetText () ;
    mFixedFrameDataText = text ? text : "" ;
    std::string err ;
    if (! parseHexBytes (mFixedFrameDataText, mFixedFrameData, err)) {
      SetErrorText (err.c_str ()) ;
      return false ;
    }
  }

  ClearChannels();
  AddChannel (mInputChannel, "CAN FD", true) ;

  return true;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerSettings::UpdateInterfacesFromSettings () {
  mInputChannelInterface->SetChannel (mInputChannel) ;
  mSimulatorRandomSeedInterface->SetInteger (mSimulatorRandomSeed) ;
  mArbitrationBitRateInterface->SetInteger (mArbitrationBitRate) ;
  mDataBitRateInterface->SetInteger (mDataBitRate) ;
  mArbitrationSamplePointInterface->SetInteger (mArbitrationSamplePoint) ;
  mDataSamplePointInterface->SetInteger (mDataSamplePoint) ;
  mCanChannelInvertedInterface->SetNumber (double (mInverted)) ;
  mProtocolInterface->SetNumber (double (mProtocol)) ;
  mSimulatorAckGenerationInterface->SetNumber (mSimulatorGeneratedAckSlot) ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (mSimulatorGeneratedFrameType) ;
  mSimulatorBSRGenerationInterface->SetNumber (mSimulatorGeneratedBSRSlot) ;
  mSimulatorESIGenerationInterface->SetNumber (mSimulatorGeneratedESISlot) ;
  mDbcFolderInterface->SetText (mDbcFolderPath.c_str ()) ;
  mUseFixedTestFrameInterface->SetNumber (double (mUseFixedTestFrame ? FIXED_TEST_FRAME_ENABLED : FIXED_TEST_FRAME_DISABLED)) ;
  mFixedFrameFormatInterface->SetNumber (double (mFixedFrameExtended ? FIXED_FRAME_FORMAT_EXTENDED : FIXED_FRAME_FORMAT_STANDARD)) ;
  mFixedFrameIdInterface->SetInteger (mFixedFrameId) ;
  mFixedFrameDataInterface->SetText (mFixedFrameDataText.c_str ()) ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerSettings::LoadSettings (const char* settings) {
  U32 value ;
  SimpleArchive text_archive;
  text_archive.SetString (settings) ;

  text_archive >> mInputChannel;
  text_archive >> mArbitrationBitRate;
  text_archive >> mDataBitRate;
  text_archive >> mInverted;
  text_archive >> mArbitrationSamplePoint ;
  text_archive >> mDataSamplePoint ;

  text_archive >> value ;
  mProtocol = ProtocolSetting (value) ;

  text_archive >> value ;
  mSimulatorGeneratedAckSlot = SimulatorGeneratedBit (value) ;

  text_archive >> value ;
  mSimulatorGeneratedFrameType = SimulatorGeneratedFrameType (value) ;

  text_archive >> value ;
  mSimulatorGeneratedESISlot = SimulatorGeneratedBit (value) ;

  text_archive >> value ;
  mSimulatorGeneratedBSRSlot = SimulatorGeneratedBit (value) ;

  { const char * path = nullptr ;
    text_archive >> & path ;   // absent in archives saved before this setting existed -- path stays nullptr, treated as "off"
    mDbcFolderPath = path ? path : "" ;
    if (! mDbcFolderPath.empty ()) {
      std::string err ;
      mDbcDatabase.LoadFromFolder (mDbcFolderPath, err) ;   // best-effort on reload -- e.g. the folder may have moved since the capture was saved; no way to surface SetErrorText from here
    }
  }

  text_archive >> mUseFixedTestFrame ;
  text_archive >> mFixedFrameExtended ;
  text_archive >> mFixedFrameId ;
  { const char * text = nullptr ;
    text_archive >> & text ;
    mFixedFrameDataText = text ? text : "" ;
    std::string err ;
    parseHexBytes (mFixedFrameDataText, mFixedFrameData, err) ;   // best-effort on reload, same reasoning as the DBC folder path
  }

  ClearChannels();
  AddChannel( mInputChannel, "CAN FD", true );

  UpdateInterfacesFromSettings();
}

//----------------------------------------------------------------------------------------

const char* CANFDMolinaroAnalyzerSettings::SaveSettings () {
  SimpleArchive text_archive;

  text_archive << mInputChannel;
  text_archive << mArbitrationBitRate;
  text_archive << mDataBitRate;
  text_archive << mInverted;
  text_archive << U32 (mProtocol) ;
  text_archive << U32 (mSimulatorGeneratedAckSlot) ;
  text_archive << U32 (mSimulatorGeneratedFrameType) ;
  text_archive << U32 (mSimulatorGeneratedBSRSlot) ;
  text_archive << U32 (mSimulatorGeneratedESISlot) ;
  text_archive << mDbcFolderPath.c_str () ;
  text_archive << mUseFixedTestFrame ;
  text_archive << mFixedFrameExtended ;
  text_archive << mFixedFrameId ;
  text_archive << mFixedFrameDataText.c_str () ;

  return SetReturnString (text_archive.GetString ()) ;
}

//----------------------------------------------------------------------------------------
