#include "CANFDMolinaroAnalyzer.h"
#include "CANFDMolinaroAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

#include <string>
#include <sstream>

//----------------------------------------------------------------------------------------
//   CANFDMolinaroAnalyzer
//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::CANFDMolinaroAnalyzer (void) :
Analyzer2 (),
mSettings (new CANFDMolinaroAnalyzerSettings ()),
mSimulationInitialized (false) {
  SetAnalyzerSettings (mSettings.get()) ;
  UseFrameV2 () ;
}

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::~CANFDMolinaroAnalyzer (void) {
  KillThread();
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::SetupResults (void) {
  mResults.reset (new CANFDMolinaroAnalyzerResults (this, mSettings.get())) ;
  SetAnalyzerResults (mResults.get()) ;
  mResults->AddChannelBubblesWillAppearOn (mSettings->mInputChannel) ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::WorkerThread (void) {
  const bool inverted = mSettings->inverted () ;
  mSampleRateHz = GetSampleRate () ;
  AnalyzerChannelData * serial = GetAnalyzerChannelData (mSettings->mInputChannel) ;
//--- Sample settings
  mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
//--- Synchronize to recessive level
  if (serial->GetBitState() == (inverted ? BIT_HIGH : BIT_LOW)) {
    serial->AdvanceToNextEdge () ;
  }
//---
  mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
  mUnstuffingActive = false ;
  mPreviousBit = (serial->GetBitState () == BIT_HIGH) ^ inverted ;
//---
  while (1) {
    const bool currentBitValue = (serial->GetBitState () == BIT_HIGH) ^ inverted ;
    const U64 start = serial->GetSampleNumber () ;
    const U64 nextEdge = serial->GetSampleOfNextEdge () ;

    U64 currentCenter = start + mCurrentSamplesPerBit / 2 ;
    while (currentCenter < nextEdge) {
      enterBit (currentBitValue, currentCenter) ;
      currentCenter += mCurrentSamplesPerBit ;
    }
  //---
    mResults->CommitResults () ;
    serial->AdvanceToNextEdge () ;
  }
}

//----------------------------------------------------------------------------------------

bool CANFDMolinaroAnalyzer::NeedsRerun () {
  return false;
}

//----------------------------------------------------------------------------------------

U32 CANFDMolinaroAnalyzer::GenerateSimulationData (U64 minimum_sample_index,
                                                 U32 device_sample_rate,
                                                 SimulationChannelDescriptor** simulation_channels ) {
  if (mSimulationInitialized == false) {
    mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
    mSimulationInitialized = true;
  }
  return mSimulationDataGenerator.GenerateSimulationData (minimum_sample_index,
                                                          device_sample_rate,
                                                          simulation_channels) ;
}

//----------------------------------------------------------------------------------------

U32 CANFDMolinaroAnalyzer::GetMinimumSampleRateHz () {
  const U32 arbitrationBitRate = mSettings->arbitrationBitRate () ;
  const U32 dataBitRate = mSettings->dataBitRate () ;
  const U32 max = (dataBitRate > arbitrationBitRate) ? dataBitRate : arbitrationBitRate ;
  return max * 12 ;
}

//----------------------------------------------------------------------------------------

const char* CANFDMolinaroAnalyzer::GetAnalyzerName () const {
  return "CAN FD";
}

//----------------------------------------------------------------------------------------

const char* GetAnalyzerName () {
  return "CAN FD";
}

//----------------------------------------------------------------------------------------

Analyzer* CreateAnalyzer () {
  return new CANFDMolinaroAnalyzer();
}

//----------------------------------------------------------------------------------------

void DestroyAnalyzer (Analyzer* analyzer) {
  delete analyzer;
}

//----------------------------------------------------------------------------------------
//  CAN FRAME DECODER
//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBit (const bool inBit, U64 & ioBitCenterSampleNumber) {
  if (!mUnstuffingActive) {
    decodeFrameBit (inBit, ioBitCenterSampleNumber) ;
    mPreviousBit = inBit ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (inBit != mPreviousBit)) {
   // Stuff bit - discarded
    addMark (ioBitCenterSampleNumber, AnalyzerResults::X);
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    mStuffBitCount += 1 ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (mPreviousBit == inBit)) { // Stuff Error
    addMark (ioBitCenterSampleNumber, AnalyzerResults::ErrorX);
    enterInErrorMode (ioBitCenterSampleNumber + mCurrentSamplesPerBit / 2, ERROR_STUFF) ;
    mConsecutiveBitCountOfSamePolarity += 1 ;
  }else if (mPreviousBit == inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    decodeFrameBit (inBit, ioBitCenterSampleNumber) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else{
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    decodeFrameBit (inBit, ioBitCenterSampleNumber) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }
}

//----------------------------------------------------------------------------------------

static const uint8_t CANFD_LENGTH [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::decodeFrameBit (const bool inBit, U64 & ioBitCenterSampleNumber) {
  switch (mFrameFieldEngineState) {
  case FrameFieldEngineState::IDLE :
    handle_IDLE_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::IDENTIFIER :
    handle_IDENTIFIER_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_EXTENDED :
    handle_CONTROL_EXTENDED_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_BASE :
    handle_CONTROL_BASE_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_AFTER_R0 :
    handle_CONTROL_AFTER_R0_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::DATA :
    handle_DATA_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::SBC :
    handle_SBC_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC15 :
    handle_CRC15_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC17 :
    handle_CRC17_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC21 :
    handle_CRC21_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRCDEL :
    handle_CRCDEL_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::ACK :
    handle_ACK_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::ENDOFFRAME :
    handle_ENDOFFRAME_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::INTERMISSION :
    handle_INTERMISSION_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::DECODER_ERROR :
    handle_DECODER_ERROR_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_IDLE_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if (inBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::Stop) ;
  }else{ // SOF
    mUnstuffingActive = true ;
    mCRC15Accumulator = 0 ;
    switch (mSettings->protocol ()) {
    case CANFD_NON_ISO_PROTOCOL :
      mCRC17Accumulator = 0 ;
      mCRC21Accumulator = 0 ;
      break ;
    case CANFD_ISO_PROTOCOL :
      mCRC17Accumulator = 1 << 16 ;
      mCRC21Accumulator = 1 << 20 ;
      break ;
    }
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = false ;
    enterBitInCRC15 (inBit) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
    addMark (inBitCenterSampleNumber, AnalyzerResults::Start);
    mFieldBitIndex = 0 ;
    mIdentifier = 0 ;
    mStuffBitCount = 0 ;
    mHaveIdentifier = false ;
    mHaveCrc = false ;
    mCrcWidth = 0 ;
    mHaveAck = false ;
    mHaveSbc = false ;
    mFrameFieldEngineState = FrameFieldEngineState::IDENTIFIER ;
    mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
    mStartOfFieldSampleNumber = inBitCenterSampleNumber + mCurrentSamplesPerBit / 2 ;
    mStartOfFrameSampleNumber = inBitCenterSampleNumber ;
    mMarkerTypeForDataAndCRC = AnalyzerResults::Dot ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_IDENTIFIER_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex <= 11) { // Standard identifier
    addMark (inBitCenterSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else if (mFieldBitIndex == 12) { // RTR or SRR bit
    mFrameType = inBit ? FrameType::remote : FrameType::canData  ;
    // Remembered rather than marked/closed off immediately: at this point
    // we don't yet know if this is a base or extended frame (that's the
    // *next* bit, IDE), so we can't yet draw the correct RTR-vs-SRR mark
    // or close any bubble -- for an extended frame the identifier isn't
    // even complete yet.
    mRtrCenterSampleNumber = inBitCenterSampleNumber ;
  }else if (mFieldBitIndex == 13) { // IDE bit
    mFrameFormat = inBit ? FrameFormat::extended : FrameFormat::base ;
    if (!inBit) { // IDE dominant -> base frame
    //--- RTR mark: uses the actually-captured RTR value from bit 12, not
    //    this bit's (IDE's) value -- the original code drew this using
    //    `inBit` here, which is always the IDE bit, so the RTR mark never
    //    reflected the real RTR bit at all.
      addMark (mRtrCenterSampleNumber,
               (mFrameType == FrameType::remote) ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    //--- IDE Mark
      addMark (inBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
    //--- Bubbles: identifier (ends right before RTR), RTR, IDE -- each its
    //    own bubble rather than all three bundled together. addBubble's
    //    end argument is the CENTER of the last bit a bubble is to cover
    //    (it adds half a bit internally to land on the boundary right
    //    after that bit).
      mHaveIdentifier = true ;
      addBubble (STANDARD_IDENTIFIER_FIELD_RESULT,
                 mIdentifier,
                 mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
                 mRtrCenterSampleNumber - mCurrentSamplesPerBit) ; // ends right before RTR (after bit 11)
      addBubble (RTR_FIELD_RESULT, mFrameType == FrameType::canData, 0, mRtrCenterSampleNumber) ;
      addBubble (IDE_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CONTROL_BASE ;
    }else{ // IDE recessive -> extended frame
    //--- SRR mark: same fix as the RTR mark above -- uses the actually-
    //    captured bit-12 value, not this bit's (IDE's) value.
      addMark (mRtrCenterSampleNumber,
               (mFrameType == FrameType::remote) ? AnalyzerResults::One : AnalyzerResults::ErrorSquare) ;
    //--- IDE Mark
      addMark (inBitCenterSampleNumber, AnalyzerResults::UpArrow) ;
    //--- SRR/IDE centers remembered for later: the identifier bubble stays
    //    bundled/still-accumulating for now, closed (and split around
    //    SRR/IDE) once the full 29-bit value is known, at the trailing
    //    RTR bit below.
      mSrrCenterSampleNumber = mRtrCenterSampleNumber ;
      mIdeCenterSampleNumber = inBitCenterSampleNumber ;
    }
  }else if (mFieldBitIndex < 32) { // ID17 ... ID0
    addMark (inBitCenterSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else{ // RTR
    mFrameType = inBit ? FrameType::remote : FrameType::canData ;
    addMark (inBitCenterSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
  //--- The full 29-bit value is known by now, so the identifier bubble is
  //    split into pieces around SRR and IDE, all showing the same
  //    complete, correct value -- rather than one monolithic bubble
  //    spanning the whole identifier plus SRR plus IDE plus RTR.
    mHaveIdentifier = true ;
    addBubble (EXTENDED_IDENTIFIER_FIELD_RESULT,
               mIdentifier,
               mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
               mSrrCenterSampleNumber - mCurrentSamplesPerBit) ; // ends right before SRR (after bit 11)
    addBubble (SRR_FIELD_RESULT, 0, 0, mSrrCenterSampleNumber) ;
    addBubble (IDE_FIELD_RESULT, 0, 0, mIdeCenterSampleNumber) ;
    addBubble (EXTENDED_IDENTIFIER_FIELD_RESULT,
               mIdentifier,
               mFrameType == FrameType::canData,
               inBitCenterSampleNumber - mCurrentSamplesPerBit) ; // ends right before the real RTR
    addBubble (RTR_FIELD_RESULT, mFrameType == FrameType::canData, 0, inBitCenterSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_EXTENDED ;
  }
}

//----------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzer::handle_CONTROL_BASE_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // FDF bit -- classic base frame's r0, repurposed by CAN FD
    if (inBit) { // FDF recessive -> CANFD frame; a genuine r0 bit still follows
      addMark (inBitCenterSampleNumber, AnalyzerResults::UpArrow) ;
      mFrameType = FrameType::canfdData ;
      addBubble (FDF_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    }else{ // FDF dominant -> this bit *is* r0 (base frames have only one reserved bit)
      addMark (inBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
      addBubble (R0_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
      mFieldBitIndex = 0 ;
      mDataCodeLength = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
    }
  }else if (inBit) { // R0 bit recessive -> error
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot) ;
    enterInErrorMode (inBitCenterSampleNumber, ERROR_FORM_R0) ;
  }else{ // R0 dominant: ok
    addMark (inBitCenterSampleNumber, AnalyzerResults::Zero) ;
    addBubble (R0_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    mDataCodeLength = 0 ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
  }
}

//----------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzer::handle_CONTROL_EXTENDED_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // FDF bit -- classic extended frame's r1, repurposed by CAN FD
    if (inBit) { // FDF recessive -> CANFD frame
      addMark (inBitCenterSampleNumber, AnalyzerResults::UpArrow) ;
      mFrameType = FrameType::canfdData ;
      addBubble (FDF_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    }else{ // FDF dominant -> this bit *is* r1 (classic extended frame); r0 still follows
      addMark (inBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
      addBubble (R1_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    }
  }else if (inBit) { // R0 bit recessive -> error
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot) ;
    enterInErrorMode (inBitCenterSampleNumber, ERROR_FORM_R0) ;
  }else{ // R0 dominant: ok
    addMark (inBitCenterSampleNumber, AnalyzerResults::Zero) ;
    addBubble (R0_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    mDataCodeLength = 0 ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CONTROL_AFTER_R0_state (const bool inBit,
                                                           U64 & ioBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFrameType == FrameType::canfdData) {
    if (mFieldBitIndex == 1) { // BRS
    //--- BRS is still sampled at the arbitration bit rate -- captured
    //    before mCurrentSamplesPerBit potentially switches below, so the
    //    bubble's own end boundary uses the right bit width.
      const U64 brsOwnCenter = ioBitCenterSampleNumber ;
      mBRS = inBit ;
      if (inBit) { // Switch to data bit rate
        const U64 samplesForDataBitRate = mSampleRateHz / mSettings->dataBitRate () ;
        const U64 BSRsamplesX100 =
          mSettings->arbitrationSamplePoint () * mCurrentSamplesPerBit
        +
          (100 - mSettings->dataSamplePoint ()) * samplesForDataBitRate
        ;
        const U64 centerBSR = ioBitCenterSampleNumber - mCurrentSamplesPerBit / 2 + BSRsamplesX100 / 200 ;
        addMark (centerBSR, AnalyzerResults::UpArrow) ;
      //--- BRS bubble ends at the actual bit-rate-switch resync boundary
      //    ("beginning of next bit", computed the same way the timing
      //    adjustment below does) -- NOT one full arbitration-bit width
      //    after BRS's own center. Those two disagree by a lot once the
      //    data rate is much faster than arbitration, and using the naive
      //    boundary made the BRS bubble stretch out far enough to swallow
      //    ESI's bubble entirely (ESI ends up with a start sample past its
      //    end sample, which Logic 2 silently doesn't draw).
        const U64 resyncBoundary = ioBitCenterSampleNumber - mCurrentSamplesPerBit / 2 + BSRsamplesX100 / 100 ;
        addBubble (BRS_FIELD_RESULT, 1, 0, resyncBoundary - mCurrentSamplesPerBit / 2) ;
      //--- Adjust for center of next bit
        ioBitCenterSampleNumber -= mCurrentSamplesPerBit / 2 ; // Returns at the beginning of BRS bit
        ioBitCenterSampleNumber += BSRsamplesX100 / 100 ; // Advance at the beginning of next bit
        ioBitCenterSampleNumber -= samplesForDataBitRate / 2 ; // Back half of a data bit rate bit
      //--- Switch to Data Bit Rate
        mCurrentSamplesPerBit = samplesForDataBitRate ;
        mMarkerTypeForDataAndCRC = AnalyzerResults::Square ;
      }else{
        addMark (ioBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
        addBubble (BRS_FIELD_RESULT, 0, 0, brsOwnCenter) ;
      }
    }else if (mFieldBitIndex == 2) { // ESI
      addMark (ioBitCenterSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
      mESI = inBit ;
      addBubble (ESI_FIELD_RESULT, inBit, 0, ioBitCenterSampleNumber) ;
    }else{
      addMark (ioBitCenterSampleNumber, mMarkerTypeForDataAndCRC) ;
      mDataCodeLength <<= 1 ;
      mDataCodeLength |= inBit ;
      if (mFieldBitIndex == 6) {
        addBubble (CANFD_CONTROL_FIELD_RESULT, mDataCodeLength, 0, ioBitCenterSampleNumber) ;
        mFieldBitIndex = 0 ;
        if (mDataCodeLength != 0) {
          mFrameFieldEngineState = FrameFieldEngineState::DATA ;
        }else if (mSettings->protocol () == CANFD_NON_ISO_PROTOCOL) { // No Data, CANFD non ISO
          mCRC17 = mCRC17Accumulator ;
          mHaveCrc = true ;
          mCrcWidth = 17 ;
          mUnstuffingActive = false ;
          mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
        }else{  // No Data, CANFD ISO
          mUnstuffingActive = false ;
          mFrameFieldEngineState = FrameFieldEngineState::SBC ;
        }
      }
    }
  }else{ // Base frame
    addMark (ioBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
    mDataCodeLength <<= 1 ;
    mDataCodeLength |= inBit ;
    if (mFieldBitIndex == 4) {
      addBubble (CAN20B_CONTROL_FIELD_RESULT, mDataCodeLength, 0, ioBitCenterSampleNumber) ;
      mFieldBitIndex = 0 ;
      if ((mDataCodeLength > 8) && (mFrameType != FrameType::canfdData)) {
        mDataCodeLength = 8 ;
      }
      mCRC15 = mCRC15Accumulator ;
      mHaveCrc = true ;
      mCrcWidth = 15 ;
      if (mFrameType == FrameType::remote) {
        mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
      }else if (mDataCodeLength > 0) {
        mFrameFieldEngineState = FrameFieldEngineState::DATA ;
      }else if (mFrameType == FrameType::canData) {
        mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
      }
    }
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_DATA_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  mData [mFieldBitIndex / 8] <<= 1 ;
  mData [mFieldBitIndex / 8] |= inBit ;
  mFieldBitIndex += 1 ;
  if ((mFieldBitIndex % 8) == 0) {
    const U32 dataIndex = (mFieldBitIndex - 1) / 8 ;
    addBubble (DATA_FIELD_RESULT, mData [dataIndex], dataIndex, inBitCenterSampleNumber) ;
  }
  if (mFieldBitIndex == (8 * CANFD_LENGTH [mDataCodeLength])) {
    mFieldBitIndex = 0 ;
    if (mFrameType != FrameType::canfdData) {
      mCRC15 = mCRC15Accumulator ;
      mHaveCrc = true ;
      mCrcWidth = 15 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
    }else if (mSettings->protocol () == CANFD_ISO_PROTOCOL) {
      mFrameFieldEngineState = FrameFieldEngineState::SBC ;
      mUnstuffingActive = false ;
    }else if (mDataCodeLength <= 10) {
      mCRC17 = mCRC17Accumulator ;
      mHaveCrc = true ;
      mCrcWidth = 17 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
      mUnstuffingActive = false ;
    }else{
      mCRC21 = mCRC21Accumulator ;
      mHaveCrc = true ;
      mCrcWidth = 21 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC21 ;
      mUnstuffingActive = false ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC15_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 15) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC15_FIELD_RESULT, mCRC15, mCRC15Accumulator, inBitCenterSampleNumber) ;
    if (mCRC15Accumulator != 0) {
      enterInErrorMode (inBitCenterSampleNumber, ERROR_CRC15) ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_SBC_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 1) { // Forced Stuff Bit
    mSBCField = 0 ;
    if (inBit == mPreviousBit) {
      addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
      enterInErrorMode (inBitCenterSampleNumber, ERROR_SBC_STUFF) ;
    }else{
      addMark (inBitCenterSampleNumber, AnalyzerResults::X);
    }
  }else if (mFieldBitIndex <= 4) {
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
    mSBCField <<= 1 ;
    mSBCField |= inBit ;
    addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  }else{ // Parity bit
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
    const U8 GRAY_CODE_DECODER [8] = {0, 1, 3, 2, 7, 6, 4, 5} ;
    const U8 suffBitCountMod8 = GRAY_CODE_DECODER [mSBCField] ;
    mSBCField <<= 1 ;
    mSBCField |= inBit ;
  //--- Check parity
    bool oneBitCountIsEven = true ;
    U32 v = mSBCField ;
    while (v > 0) {
      oneBitCountIsEven ^= (v & 1) != 0 ;
      v >>= 1 ;
    }
    addMark (inBitCenterSampleNumber, oneBitCountIsEven ? AnalyzerResults::Dot : AnalyzerResults::ErrorX) ;
    const U32 data2 = ((mStuffBitCount % 8) << 1) | !oneBitCountIsEven ;
    addBubble (SBC_FIELD_RESULT, suffBitCountMod8, data2, inBitCenterSampleNumber) ;
    mHaveSbc = true ;
    mSbcStuffBitCount = suffBitCountMod8 ;
  //--- SBC-OK requires BOTH checks to pass: parity, and that the
  //    transmitter's declared count (suffBitCountMod8, mod 8 since it's
  //    only a 3-bit field) agrees with what we independently counted
  //    (mStuffBitCount, frozen since the end of the data field). Parity
  //    alone -- what this used to be -- can't catch a genuine stuff-count
  //    disagreement, which is the exact failure SBC exists to catch.
    mSbcOk = oneBitCountIsEven && (suffBitCountMod8 == (mStuffBitCount % 8)) ;
    mUnstuffingActive = false ;
    mFieldBitIndex = 0 ;
    if (mDataCodeLength <= 10) {
      mCRC17 = mCRC17Accumulator ;
      mHaveCrc = true ;
      mCrcWidth = 17 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
    }else{
      mCRC21 = mCRC21Accumulator ;
      mHaveCrc = true ;
      mCrcWidth = 21 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC21 ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC17_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if ((mFieldBitIndex % 5) != 0) {
    enterBitInCRC17 (inBit) ;
    addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  }else if (inBit == mPreviousBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber, ERROR_CRC17) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::X);
  }
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 22) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC17_FIELD_RESULT, mCRC17, mCRC17Accumulator, inBitCenterSampleNumber) ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC21_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if ((mFieldBitIndex % 5) != 0) {
    enterBitInCRC21 (inBit) ;
    addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  }else if (inBit == mPreviousBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber, ERROR_CRC21) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::X);
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 27) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC21_FIELD_RESULT, mCRC21, mCRC21Accumulator, inBitCenterSampleNumber) ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRCDEL_state (const bool inBit, U64 & ioBitCenterSampleNumber) {
  mUnstuffingActive = false ;
//--- CRCDEL's own center, captured before ioBitCenterSampleNumber/
//    mCurrentSamplesPerBit potentially get rewritten below for the
//    bit-rate switch back to arbitration rate -- used as addBubble's end
//    argument so the (previously entirely missing) CRC DEL bubble gets the
//    right width.
  const U64 crcDelOwnCenter = ioBitCenterSampleNumber ;
  if (inBit) { // Handle Bit Rate Switch: data bit rate -> arbitration bit rate
    const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
    const U64 CRCDELsamplesX100 =
      mSettings->dataSamplePoint () * mCurrentSamplesPerBit
    +
      (100 - mSettings->arbitrationSamplePoint ()) * samplesPerArbitrationBit
    ;
    const U64 centerCRCDEL = ioBitCenterSampleNumber - mCurrentSamplesPerBit / 2 + CRCDELsamplesX100 / 200 ;
    addMark (centerCRCDEL, AnalyzerResults::One) ;
  //--- Same fix as the mirror-image BRS bubble in
  //    handle_CONTROL_AFTER_R0_state: end at the actual resync boundary
  //    ("beginning of next bit" below), not one full data-bit width after
  //    CRCDEL's own center -- here that made the bubble too NARROW rather
  //    than too wide (data rate is the faster of the two), leaving an
  //    unlabeled gap before ACK's bubble instead of an overlap.
    const U64 resyncBoundary = ioBitCenterSampleNumber - mCurrentSamplesPerBit / 2 + CRCDELsamplesX100 / 100 ;
    addBubble (CRC_DEL_FIELD_RESULT, 0, 0, resyncBoundary - mCurrentSamplesPerBit / 2) ;
  //--- Adjust for center of next bit
    ioBitCenterSampleNumber -= mCurrentSamplesPerBit / 2 ; // Returns at the beginning of CRCDEL bit
    ioBitCenterSampleNumber += CRCDELsamplesX100 / 100 ; // Advance at the beginning of next bit
    ioBitCenterSampleNumber -= samplesPerArbitrationBit / 2 ; // Back half of a arbitration bit rate bit
  //--- Switch to Data Bit Rate
    mCurrentSamplesPerBit = samplesPerArbitrationBit ;
  }else{
    addMark (ioBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    addBubble (CRC_DEL_FIELD_RESULT, 0, 0, crcDelOwnCenter) ;
    enterInErrorMode (ioBitCenterSampleNumber, ERROR_FORM_CRCDEL) ;
  }
  mStartOfFieldSampleNumber = ioBitCenterSampleNumber + mCurrentSamplesPerBit / 2 ;
  mFrameFieldEngineState = FrameFieldEngineState::ACK ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_ACK_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // ACK SLOT
    addMark (inBitCenterSampleNumber, inBit ? AnalyzerResults::ErrorSquare : AnalyzerResults::DownArrow);
    mAcked = inBit ;
    mHaveAck = true ;
    addBubble (ACK_FIELD_RESULT, mAcked, 0, inBitCenterSampleNumber) ;
  }else{ // ACK DELIMITER
    addBubble (ACK_DEL_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    mFrameFieldEngineState = FrameFieldEngineState::ENDOFFRAME ;
    if (inBit) {
      addMark (inBitCenterSampleNumber, AnalyzerResults::One) ;
    }else{
      addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot) ;
      enterInErrorMode (inBitCenterSampleNumber, ERROR_FORM_ACKDEL) ;
    }
    mFieldBitIndex = 0 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_ENDOFFRAME_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if (inBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::One) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber, ERROR_FORM_EOF) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 7) {
    addBubble (EOF_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::INTERMISSION ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_INTERMISSION_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if (inBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::One) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber, ERROR_FORM_INTERMISSION) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 3) {
    addBubble (INTERMISSION_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    emitConsolidatedFrameV2 (inBitCenterSampleNumber, false) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_DECODER_ERROR_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  mUnstuffingActive = false ;
  addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot);
  if (mPreviousBit != inBit) {
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
  }else if (inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    if (mConsecutiveBitCountOfSamePolarity == 11) {
      addBubble (CAN_ERROR_RESULT, mErrorReason, 0, inBitCenterSampleNumber) ;
      emitConsolidatedFrameV2 (inBitCenterSampleNumber, true) ;
      mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC15 (const bool inBit) {
  const bool bit14 = (mCRC15Accumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBit ^ bit14 ;
  mCRC15Accumulator <<= 1 ;
  mCRC15Accumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRC15Accumulator ^= 0x4599 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC17 (const bool inBit) {
  const bool bit16 = (mCRC17Accumulator & (1 << 16)) != 0 ;
  const bool crc_nxt = inBit ^ bit16 ;
  mCRC17Accumulator <<= 1 ;
  mCRC17Accumulator &= 0x1FFFF ;
  if (crc_nxt) {
    mCRC17Accumulator ^= 0x1685B ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC21 (const bool inBit) {
  const bool bit20 = (mCRC21Accumulator & (1 << 20)) != 0 ;
  const bool crc_nxt = inBit ^ bit20 ;
  mCRC21Accumulator <<= 1 ;
  mCRC21Accumulator &= 0x1FFFFF ;
  if (crc_nxt) {
    mCRC21Accumulator ^= 0x102899 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::addMark (const U64 inBitCenterSampleNumber,
                                     const AnalyzerResults::MarkerType inMarker) {
  mResults->AddMarker (inBitCenterSampleNumber, inMarker, mSettings->mInputChannel);
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::addBubble (const U8 inBubbleType,
                                       const U64 inData1,
                                       const U64 inData2,
                                       const U64 inBitCenterSampleNumber) {
//--- Old-style Frame: drives the per-field waveform bubble/tabular text
//    (GenerateText in CANFDMolinaroAnalyzerResults.cpp switches on
//    frame.mType) -- kept exactly as before, still positioned precisely at
//    each field's own bit location on the trace. The FrameV2 side (Data
//    Table) is handled separately now, once per whole message, by
//    emitConsolidatedFrameV2() -- see its call sites in
//    handle_INTERMISSION_state and handle_DECODER_ERROR_state.
  Frame frame ;
  frame.mType = inBubbleType ;
  frame.mFlags = 0 ;
  frame.mData1 = inData1 ;
  frame.mData2 = inData2 ;
  frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
  const U64 endSampleNumber = inBitCenterSampleNumber + mCurrentSamplesPerBit / 2 ;
  frame.mEndingSampleInclusive = endSampleNumber ;
  mResults->AddFrame (frame) ;

  mResults->CommitResults () ;
  ReportProgress (frame.mEndingSampleInclusive) ;
//--- Prepare for next bubble
  mStartOfFieldSampleNumber = endSampleNumber ;
}

//----------------------------------------------------------------------------------------

static std::string formatHex (const uint32_t inValue) {
  char buffer [16] ;
  snprintf (buffer, sizeof (buffer), "0x%X", inValue) ;
  return std::string (buffer) ;
}

//----------------------------------------------------------------------------------------

static std::string formatCrc (const U32 inValue, const U8 inWidthBits) {
  char buffer [16] ;
  const int hexDigits = (inWidthBits + 3) / 4 ;
  snprintf (buffer, sizeof (buffer), "0x%0*X", hexDigits, inValue) ;
  return std::string (buffer) ;
}

//----------------------------------------------------------------------------------------

static std::string formatData (const uint8_t * inData, const int inLength) {
  std::string result ;
  char buffer [8] ;
  for (int i=0 ; i<inLength ; i++) {
    if (i > 0) {
      result += ' ' ;
    }
    snprintf (buffer, sizeof (buffer), "%02X", inData [i]) ;
    result += buffer ;
  }
  return result ;
}

//----------------------------------------------------------------------------------------
//  One Data Table row per CAN/CAN FD message, mirroring the sibling
//  classic CAN 2.0B plugin's consolidated row (ID, CAN-TYPE, RTR, DLC,
//  DATA, CRC, CRC-OK, ACK, LENGTH, STUFFBITS), extended with the fields
//  unique to CAN FD (FDF, BRS, ESI, SBC/SBC-OK -- ISO frames only). Fields
//  for sub-parts of the message that were never reached (e.g. an error
//  before ACK) are simply omitted, same convention the classic plugin
//  uses -- "ID" falls back to "?" only when even the identifier was never
//  captured.
//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::emitConsolidatedFrameV2 (const U64 inEndSampleNumber, const bool inError) {
  FrameV2 frameV2 ;
//--- ERROR is always present (blank for a clean frame) rather than only
//    added when inError is true -- so it's one reliable column to scan or
//    filter on across every row, instead of only showing up on whichever
//    rows happened to error.
  frameV2.AddString ("ERROR", inError ? CanErrorReasonText (mErrorReason) : "") ;
  if (mHaveIdentifier) {
    frameV2.AddString ("ID", formatHex (mIdentifier).c_str ()) ;
    frameV2.AddString ("CAN-TYPE", (mFrameFormat == FrameFormat::extended) ? "EXT" : "STD") ;
    frameV2.AddBoolean ("RTR", mFrameType == FrameType::remote) ;
    frameV2.AddBoolean ("FDF", mFrameType == FrameType::canfdData) ;
    if (mFrameType == FrameType::canfdData) {
      frameV2.AddBoolean ("BRS", mBRS) ;
      frameV2.AddBoolean ("ESI", mESI) ;
    }
  //--- mDataCodeLength holds the raw DLC nibble (0-15) for CAN FD frames --
  //    an index into CANFD_LENGTH[], not a byte count -- and the already-
  //    clamped (0-8) byte count for classic-format frames.
    const U32 dlcByteCount = (mFrameType == FrameType::canfdData)
      ? CANFD_LENGTH [mDataCodeLength]
      : mDataCodeLength ;
    frameV2.AddString ("DLC", std::to_string (dlcByteCount).c_str ()) ;
    const std::string dataStr = (mFrameType == FrameType::remote)
      ? std::string ()
      : formatData (mData, dlcByteCount) ;
    frameV2.AddString ("DATA", dataStr.c_str ()) ;
    if (mFrameType != FrameType::remote) {
      const DbcMessage * dbcMsg = mSettings->dbcDatabase ().FindMessage (mIdentifier, mFrameFormat == FrameFormat::extended) ;
      if (dbcMsg != nullptr) {
        DecodeDbcSignalsIntoFrame (*dbcMsg, mData, int (dlcByteCount), frameV2) ;
      }
    }
  }else{
    frameV2.AddString ("ID", "?") ;
  }
  if (mHaveCrc) {
    U32 crcValue = 0 ;
    U32 crcAccumulator = 0 ;
    switch (mCrcWidth) {
    case 15 : crcValue = mCRC15 ; crcAccumulator = mCRC15Accumulator ; break ;
    case 17 : crcValue = mCRC17 ; crcAccumulator = mCRC17Accumulator ; break ;
    case 21 : crcValue = mCRC21 ; crcAccumulator = mCRC21Accumulator ; break ;
    }
    std::stringstream crcLabel ;
    crcLabel << "CRC" << U32 (mCrcWidth) ;
    frameV2.AddString (crcLabel.str ().c_str (), formatCrc (crcValue, mCrcWidth).c_str ()) ;
    frameV2.AddBoolean ("CRC-OK", crcAccumulator == 0) ;
  }
  if (mHaveSbc) {
    frameV2.AddString ("SBC", std::to_string (mSbcStuffBitCount).c_str ()) ;
    frameV2.AddBoolean ("SBC-OK", mSbcOk) ;
  }
  if (mHaveAck) {
    frameV2.AddBoolean ("ACK", ! mAcked) ; // mAcked holds the raw (recessive = NOT acked) bit
  }
//--- Frame duration -- reported in microseconds rather than a bit count.
//    The sibling classic CAN 2.0B plugin reports a bit count (sample
//    count / one fixed bit width), which works because classic frames
//    never change bit rate. CAN FD frames can run at two different rates
//    (arbitration and data, switched via BRS), so that same division
//    would give a meaningless number here -- duration stays correct
//    regardless of how many rate switches happened.
  const U64 frameSampleCount = inEndSampleNumber - mStartOfFrameSampleNumber ;
  const U64 frameDurationUs = (frameSampleCount * 1000000) / mSampleRateHz ;
  frameV2.AddString ("LENGTH", (std::to_string (frameDurationUs) + " us").c_str ()) ;
  frameV2.AddString ("STUFFBITS", std::to_string (mStuffBitCount).c_str ()) ;
  mResults->AddFrameV2 (frameV2, "CAN Frame", mStartOfFrameSampleNumber, inEndSampleNumber) ;
  mResults->CommitResults () ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterInErrorMode (const U64 inBitCenterSampleNumber, const CanErrorReason inReason) {
  mStartOfFieldSampleNumber = inBitCenterSampleNumber ;
  mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
  mFrameFieldEngineState = DECODER_ERROR ;
  mUnstuffingActive = false ;
  mErrorReason = inReason ;
}

//----------------------------------------------------------------------------------------
