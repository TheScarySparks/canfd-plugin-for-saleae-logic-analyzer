#ifndef CANFDMOLINARO_ANALYZER_H
#define CANFDMOLINARO_ANALYZER_H

//----------------------------------------------------------------------------------------

#include <Analyzer.h>
#include <AnalyzerResults.h>
#include "CANFDMolinaroAnalyzerResults.h"
#include "CANFDMolinaroSimulationDataGenerator.h"

//----------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzerSettings;

//----------------------------------------------------------------------------------------


class ANALYZER_EXPORT CANFDMolinaroAnalyzer : public Analyzer2 {

  public: CANFDMolinaroAnalyzer();

  public: virtual ~CANFDMolinaroAnalyzer();

  public: virtual void SetupResults();

  public: virtual void WorkerThread();

  public: virtual U32 GenerateSimulationData (U64 newest_sample_requested,
                                              U32 sample_rate,
                                              SimulationChannelDescriptor** simulation_channels);
  public: virtual U32 GetMinimumSampleRateHz () ;

  public: virtual const char* GetAnalyzerName() const ;

  public: virtual bool NeedsRerun () ;

//--- Protected properties
  protected: std::shared_ptr < CANFDMolinaroAnalyzerSettings > mSettings;
  protected: std::shared_ptr < CANFDMolinaroAnalyzerResults > mResults;
  protected: // AnalyzerChannelData* mSerial;

  protected: CANMolinaroSimulationDataGenerator mSimulationDataGenerator;
   protected: bool mSimulationInitialized ;

  protected: U32 mSampleRateHz;


//---------------- CAN decoder
  private: U64 mStartOfFieldSampleNumber ;
  private: U64 mStartOfFrameSampleNumber ;
  private: U32 mCurrentSamplesPerBit ;

//--- CAN protocol
  private: typedef enum  {
    IDLE, IDENTIFIER, CONTROL_BASE, CONTROL_EXTENDED, CONTROL_AFTER_R0, DATA, SBC,
    CRC15, CRC17, CRC21, CRCDEL, ACK, ENDOFFRAME, INTERMISSION, DECODER_ERROR
  } FrameFieldEngineState ;

  private: FrameFieldEngineState mFrameFieldEngineState ;
  private: int mFieldBitIndex ;
  private: int mConsecutiveBitCountOfSamePolarity ;
  private: bool mPreviousBit ;
  private: bool mUnstuffingActive ;

//--- Received frame
  private: uint32_t mIdentifier ;
  private: U32 mSBCField ;
  private: U32 mStuffBitCount ;
  private: U32 mDataCodeLength ;
  private: U8 mData [64] ;
  private: U16 mCRC15Accumulator ;
  private: U16 mCRC15 ;
  private: U32 mCRC17Accumulator ;
  private: U32 mCRC17 ;
  private: U32 mCRC21Accumulator ;
  private: U32 mCRC21 ;
  private: typedef enum {base, extended} FrameFormat ;
  private: FrameFormat mFrameFormat ;
  private: typedef enum {canData, remote, canfdData} FrameType ;
  private: FrameType mFrameType ;
  private: bool mBRS ;
  private: bool mESI ;
  private: bool mAcked ;
  private: AnalyzerResults::MarkerType mMarkerTypeForDataAndCRC ;

//--- RTR/SRR's own center sample, captured at bit 12 (shared slot -- RTR
//    for base-format frames, SRR for extended) so it can be split into its
//    own bubble later, when the identifier bubble it used to be lumped
//    into is finally closed. addBubble's end argument is the CENTER of the
//    last bit a bubble is to cover (it adds half a bit internally to land
//    on the boundary right after that bit) -- so these are plain bit
//    centers, not boundaries, matching every other call site in this file.
//    Reused for the *real* trailing RTR bit on the extended path (bit 32)
//    once the base-path use is done with it.
  private: U64 mRtrCenterSampleNumber ;

//--- SRR's own center specifically (extended frames only), copied out of
//    mRtrCenterSampleNumber before that gets reused for the real trailing
//    RTR bit. Held onto until the identifier value is fully known (at that
//    trailing RTR bit), so the identifier bubble can be split into pieces
//    around SRR and IDE with the same complete, correct value shown in
//    every piece.
  private: U64 mSrrCenterSampleNumber ;

//--- IDE's own center sample (extended frames only).
  private: U64 mIdeCenterSampleNumber ;

//--- Tracks how far the current frame got, and which CRC width was
//    actually used, for the consolidated FrameV2 row (a frame that errors
//    out partway through still gets one row, with whichever fields were
//    actually captured before the error).
  private: bool mHaveIdentifier ;
  private: bool mHaveCrc ;
  private: U8 mCrcWidth ; // 0 = none yet, else 15/17/21
  private: bool mHaveAck ;
  private: bool mHaveSbc ;
  private: U8 mSbcStuffBitCount ;
  private: bool mSbcOk ;
  private: CanErrorReason mErrorReason ;

//---------------- CAN decoder methods
  private: void enterBit (const bool inBit, U64 & ioBitCenterSampleNumber) ;
  private: void decodeFrameBit (const bool inBit, U64 & ioBitCenterSampleNumber) ;
  private: void enterBitInCRC15 (const bool inBit) ;
  private: void enterBitInCRC17 (const bool inBit) ;
  private: void enterBitInCRC21 (const bool inBit) ;
  private: void addMark (const U64 inBitCenterSampleNumber, const AnalyzerResults::MarkerType inMarker) ;
  private: void addBubble (const U8 inBubbleType,
                           const U64 inData1,
                           const U64 inData2,
                           const U64 inEndSampleNumber) ;
  private: void emitConsolidatedFrameV2 (const U64 inEndSampleNumber, const bool inError) ;
  private: void enterInErrorMode (const U64 inBitCenterSampleNumber, const CanErrorReason inReason) ;

  private: void handle_IDLE_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_IDENTIFIER_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_CONTROL_BASE_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_CONTROL_EXTENDED_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_CONTROL_AFTER_R0_state (const bool inBit, U64 & ioBitCenterSampleNumber) ;
  private: void handle_DATA_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_SBC_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_CRC15_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_CRC17_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_CRC21_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_CRCDEL_state (const bool inBit, U64 & ioBitCenterSampleNumber) ;
  private: void handle_ACK_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_ENDOFFRAME_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_INTERMISSION_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
  private: void handle_DECODER_ERROR_state (const bool inBit, const U64 inBitCenterSampleNumber) ;
} ;

//----------------------------------------------------------------------------------------

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

//----------------------------------------------------------------------------------------

#endif //CANFDMOLINARO_ANALYZER_H
