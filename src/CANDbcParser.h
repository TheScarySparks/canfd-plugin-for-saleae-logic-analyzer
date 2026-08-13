#ifndef CAN_DBC_PARSER_H
#define CAN_DBC_PARSER_H

//----------------------------------------------------------------------------------------

#include <AnalyzerTypes.h>
#include <AnalyzerResults.h>   // FrameV2
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

//----------------------------------------------------------------------------------------
//  In-memory representation of a (subset of a) Vector DBC file: plain
//  signals plus *simple* multiplexing only (one 'M' selector per message,
//  'm<n>' signals gated on its value). Extended/multiple multiplexing
//  (SG_MUL_VAL_), VAL_ value tables, comments and attributes are not
//  parsed -- those lines are silently skipped, same as any other
//  unrecognized line, rather than failing the whole load.
//----------------------------------------------------------------------------------------

enum class DbcByteOrder : uint8_t { Little, Big } ;                     // '@1' / '@0' -- verified against cantools' DBC parser source, NOT the '@0'/'@1' pairing several third-party writeups state
enum class DbcMuxRole   : uint8_t { None, Multiplexor, Multiplexed } ;  // '' / 'M' / 'm<n>'

//----------------------------------------------------------------------------------------

struct DbcSignal {
  std::string  mName ;
  int          mStartBit = 0 ;
  int          mLength = 0 ;
  DbcByteOrder mByteOrder = DbcByteOrder::Little ;
  bool         mIsSigned = false ;
  double       mScale = 1.0 ;
  double       mOffset = 0.0 ;
  double       mMin = 0.0 ;   // parsed, unused for decode -- kept for possible future clamping/diagnostics
  double       mMax = 0.0 ;
  std::string  mUnit ;
  DbcMuxRole   mMuxRole = DbcMuxRole::None ;
  int          mMuxValue = -1 ;   // only meaningful when mMuxRole == Multiplexed
} ;

//----------------------------------------------------------------------------------------

struct DbcMessage {
  uint32_t                mId = 0 ;          // actual CAN ID, extended-ID marker bit already stripped
  bool                     mIsExtended = false ;
  std::string              mName ;
  int                      mDlc = 0 ;
  std::vector <DbcSignal>  mSignals ;         // file order preserved
} ;

//----------------------------------------------------------------------------------------
//  Aggregates every message from every .dbc file found in a folder. One
//  DbcDatabase instance lives on the analyzer's Settings object and is
//  (re)built whenever the "DBC Folder" setting is applied.
//----------------------------------------------------------------------------------------

class DbcDatabase {

  public: bool LoadFromFolder (const std::string & inFolderPath, std::string & outError) ;
  public: const DbcMessage * FindMessage (uint32_t inId, bool inIsExtended) const ;
  public: void Clear (void) ;

  private: bool LoadOneFile (const std::string & inFilePath) ;   // merges into mMessagesById, first-loaded-wins on ID collision
  private: static uint64_t Key (uint32_t inId, bool inIsExtended) {
    return (uint64_t (inIsExtended ? 1 : 0) << 32) | uint64_t (inId) ;
  }

  private: std::unordered_map <uint64_t, DbcMessage> mMessagesById ;

} ;

//----------------------------------------------------------------------------------------
//  Extracts and scales every signal in inMsg out of inData (inDataLen
//  valid bytes), resolving simple multiplexing first, and adds each
//  decoded signal to inFrameV2 as a formatted string (never a raw
//  numeric AddX -- the Data Table zero-pads raw integers to 64-bit hex
//  regardless of the field's real size, same reason every other numeric
//  field in this analyzer is formatted as a string before being added).
//----------------------------------------------------------------------------------------

void DecodeDbcSignalsIntoFrame (const DbcMessage & inMsg, const uint8_t * inData, int inDataLen, FrameV2 & ioFrameV2) ;

//----------------------------------------------------------------------------------------

#endif //CAN_DBC_PARSER_H
