#include "CANDbcParser.h"

#include <fstream>
#include <regex>
#include <cstdio>

#ifdef _WIN32
#include <windows.h>
#endif

//----------------------------------------------------------------------------------------
//  Bit extraction -- the highest-risk piece of this whole feature. DBC's
//  Motorola (big-endian) start-bit numbering is the single most common
//  source of bugs in third-party DBC decoders: both byte orders share the
//  same per-byte bit index (0 = LSB .. 7 = MSB), but Motorola signals walk
//  that index *backwards*, and crossing a byte boundary going backwards is
//  a jump of +15 in the flat byte*8+bit index space, not -1.
//
//  Derived from the standard DBC bit-diagram and self-checked (Motorola
//  startBit=7,length=16 -> (data[0]<<8)|data[1], the expected two-byte
//  big-endian read) -- NOT yet validated against an independent reference
//  decoder. Validate against a known-good DBC + a reference tool (e.g.
//  `cantools decode`) before trusting this on real captures.
//----------------------------------------------------------------------------------------

static uint64_t extractSignalRaw (const uint8_t * inData, int inDataLen,
                                  int inStartBit, int inLength, bool inIsBigEndian) {
  uint64_t raw = 0 ;
  int pos = inStartBit ;
  for (int i=0 ; i<inLength ; i++) {
    const int byteIdx = pos / 8 ;
    const int bitIdx = pos % 8 ;   // 0 = LSB, 7 = MSB of that byte
    const uint64_t bit = ((byteIdx >= 0) && (byteIdx < inDataLen)) ? ((inData [byteIdx] >> bitIdx) & 1u) : 0 ;
    if (inIsBigEndian) {
      // Motorola: startBit is the signal's MSB -- first bit read (i=0) is
      // the most significant, so shift the running value left and OR in.
      raw = (raw << 1) | bit ;
      pos = ((pos % 8) == 0) ? (pos + 15) : (pos - 1) ;
    }else{
      // Intel: startBit is the signal's LSB -- first bit read (i=0) is the
      // least significant, so it goes at raw's bit 0, next at bit 1, etc.
      raw |= (bit << i) ;
      pos += 1 ;
    }
  }
  return raw ;
}

//----------------------------------------------------------------------------------------

static int64_t signExtend (uint64_t inRaw, int inLength) {
  if ((inLength <= 0) || (inLength >= 64)) {
    return int64_t (inRaw) ;
  }
  const uint64_t signBit = uint64_t (1) << (inLength - 1) ;
  if ((inRaw & signBit) != 0) {
    const uint64_t signMask = ~ uint64_t (0) << inLength ;
    return int64_t (inRaw | signMask) ;
  }
  return int64_t (inRaw) ;
}

//----------------------------------------------------------------------------------------
//  Fixed high precision then trim, so scale=1 doesn't produce "24.000000"
//  and scale=0.125 doesn't produce spurious trailing digits, without
//  needing to parse the DBC's original scale-literal precision.
//----------------------------------------------------------------------------------------

static std::string formatPhysical (double inValue, const std::string & inUnit) {
  char buffer [64] ;
  snprintf (buffer, sizeof (buffer), "%.6f", inValue) ;
  std::string s (buffer) ;
  const size_t dot = s.find ('.') ;
  if (dot != std::string::npos) {
    const size_t last = s.find_last_not_of ('0') ;
    s.erase (last + 1) ;
    if (!s.empty () && (s.back () == '.')) {
      s.pop_back () ;
    }
  }
  if (!inUnit.empty ()) {
    s += ' ' ;
    s += inUnit ;
  }
  return s ;
}

//----------------------------------------------------------------------------------------

void DecodeDbcSignalsIntoFrame (const DbcMessage & inMsg, const uint8_t * inData, int inDataLen, FrameV2 & ioFrameV2) {
//--- Pass 1: resolve the (at most one) multiplexor signal, if any, before
//    any multiplexed signal can be evaluated against it.
  bool haveMuxValue = false ;
  int64_t muxSelectorValue = 0 ;
  for (const DbcSignal & sig : inMsg.mSignals) {
    if (sig.mMuxRole == DbcMuxRole::Multiplexor) {
      const uint64_t raw = extractSignalRaw (inData, inDataLen, sig.mStartBit, sig.mLength, sig.mByteOrder == DbcByteOrder::Big) ;
      muxSelectorValue = sig.mIsSigned ? signExtend (raw, sig.mLength) : int64_t (raw) ;
      haveMuxValue = true ;
      break ;   // at most one Multiplexor per message -- extras are demoted to None at load time
    }
  }
//--- Pass 2: decode and emit every signal that applies, in file order.
  for (const DbcSignal & sig : inMsg.mSignals) {
    if (sig.mMuxRole == DbcMuxRole::Multiplexed) {
      if ((!haveMuxValue) || (int64_t (sig.mMuxValue) != muxSelectorValue)) {
        continue ;   // not selected by this frame's mux value -- no blank placeholder column
      }
    }
    const uint64_t raw = extractSignalRaw (inData, inDataLen, sig.mStartBit, sig.mLength, sig.mByteOrder == DbcByteOrder::Big) ;
    const double rawAsNumber = sig.mIsSigned ? double (signExtend (raw, sig.mLength)) : double (raw) ;
    const double physical = rawAsNumber * sig.mScale + sig.mOffset ;
    ioFrameV2.AddString (sig.mName.c_str (), formatPhysical (physical, sig.mUnit).c_str ()) ;
  }
}

//----------------------------------------------------------------------------------------

const DbcMessage * DbcDatabase::FindMessage (uint32_t inId, bool inIsExtended) const {
  const auto it = mMessagesById.find (Key (inId, inIsExtended)) ;
  return (it == mMessagesById.end ()) ? nullptr : & (it->second) ;
}

//----------------------------------------------------------------------------------------

void DbcDatabase::Clear (void) {
  mMessagesById.clear () ;
}

//----------------------------------------------------------------------------------------
//  Line-oriented state machine, not a full grammar parser: classify each
//  line by its leading keyword, BO_ starts a new (locally-built) message
//  and becomes "current," SG_ attaches a parsed signal to it, everything
//  else (CM_, BA_, VAL_, BU_, SG_MUL_VAL_, blank lines, ...) is silently
//  skipped -- matches this feature's "skip unrecognized, don't fail"
//  scope. Messages are built into a local vector first and merged into
//  the aggregate map afterward (see LoadFromFolder), so a single file
//  never partially corrupts the aggregate on a mid-file problem.
//----------------------------------------------------------------------------------------

bool DbcDatabase::LoadOneFile (const std::string & inFilePath) {
  std::ifstream file (inFilePath.c_str ()) ;
  if (! file.is_open ()) {
    return false ;
  }

  static const std::regex boRegex (
    R"RX(^\s*BO_\s+(\d+)\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(\d+)\s+(\S+)\s*$)RX"
  ) ;
  static const std::regex sgRegex (
    R"RX(^\s*SG_\s+([A-Za-z_][A-Za-z0-9_]*)\s*(M|m[0-9]+)?\s*:\s*([0-9]+)\|([0-9]+)@([01])([+-])\s*\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)\s*\[\s*([-+0-9.eE]*)\s*\|\s*([-+0-9.eE]*)\s*\]\s*"([^"]*)"\s*(.*)$)RX"
  ) ;

  std::vector <DbcMessage> localMessages ;
  DbcMessage * currentMessage = nullptr ;
  std::string line ;
  bool firstLine = true ;

  while (std::getline (file, line)) {
    if (! line.empty () && (line.back () == '\r')) {
      line.pop_back () ;
    }
    if (firstLine) {
      firstLine = false ;
      if ((line.size () >= 3)
          && (uint8_t (line [0]) == 0xEF) && (uint8_t (line [1]) == 0xBB) && (uint8_t (line [2]) == 0xBF)) {
        line.erase (0, 3) ;   // strip UTF-8 BOM, some DBC exports include one
      }
    }

    std::smatch m ;
    if (std::regex_match (line, m, boRegex)) {
      DbcMessage msg ;
      const uint32_t rawId = uint32_t (std::stoul (m [1].str ())) ;
      msg.mIsExtended = (rawId & 0x80000000u) != 0 ;
      msg.mId = rawId & 0x1FFFFFFFu ;
      msg.mName = m [2].str () ;
      msg.mDlc = std::stoi (m [3].str ()) ;
      localMessages.push_back (msg) ;
      currentMessage = & localMessages.back () ;
    }else if ((currentMessage != nullptr) && std::regex_match (line, m, sgRegex)) {
      DbcSignal sig ;
      sig.mName = m [1].str () ;
      const std::string muxToken = m [2].str () ;
      if (muxToken == "M") {
        sig.mMuxRole = DbcMuxRole::Multiplexor ;
      }else if (! muxToken.empty ()) {   // "m<digits>"
        sig.mMuxRole = DbcMuxRole::Multiplexed ;
        sig.mMuxValue = std::stoi (muxToken.substr (1)) ;
      }
      sig.mStartBit = std::stoi (m [3].str ()) ;
      sig.mLength = std::stoi (m [4].str ()) ;
    //--- DBC byte-order digit convention (confirmed against cantools'
    //    own DBC parser source, dbc.py: byte_order = 'big_endian' if
    //    digit == '0' else 'little_endian') -- '0' = Motorola/big-endian,
    //    '1' = Intel/little-endian. Easy to get backwards (several
    //    otherwise-reputable third-party writeups state it the other way
    //    round); this is the actually-verified mapping.
      sig.mByteOrder = (m [5].str () == "0") ? DbcByteOrder::Big : DbcByteOrder::Little ;
      sig.mIsSigned = (m [6].str () == "-") ;
      sig.mScale = std::stod (m [7].str ()) ;
      sig.mOffset = std::stod (m [8].str ()) ;
      sig.mMin = m [9].str ().empty () ? 0.0 : std::stod (m [9].str ()) ;
      sig.mMax = m [10].str ().empty () ? 0.0 : std::stod (m [10].str ()) ;
      sig.mUnit = m [11].str () ;
      currentMessage->mSignals.push_back (sig) ;
    }
    // else: unrecognized line, skip -- CM_/BA_/VAL_/BU_/SG_MUL_VAL_/blank/etc.
  }

//--- If a message somehow has more than one 'M' signal, keep the first and
//    demote the rest to plain signals rather than failing the load.
  for (DbcMessage & msg : localMessages) {
    bool sawMultiplexor = false ;
    for (DbcSignal & sig : msg.mSignals) {
      if (sig.mMuxRole == DbcMuxRole::Multiplexor) {
        if (sawMultiplexor) {
          sig.mMuxRole = DbcMuxRole::None ;
        }else{
          sawMultiplexor = true ;
        }
      }
    }
  }

//--- Merge into the aggregate map -- first-loaded-wins on ID collision,
//    whether the collision is against an earlier file or an earlier
//    (malformed, duplicate) BO_ within this same file.
  for (DbcMessage & msg : localMessages) {
    const uint64_t key = Key (msg.mId, msg.mIsExtended) ;
    if (mMessagesById.find (key) == mMessagesById.end ()) {
      mMessagesById.emplace (key, std::move (msg)) ;
    }
  }

  return true ;
}

//----------------------------------------------------------------------------------------
//  Enumerates every *.dbc file directly in inFolderPath and merges them
//  all into this database. Only fails (returns false, sets outError) if
//  the folder itself can't be opened -- an empty or all-.dbc-free folder
//  is not an error (the feature just contributes nothing), and a single
//  unreadable/unparseable file inside the folder doesn't block the rest
//  from loading.
//----------------------------------------------------------------------------------------

bool DbcDatabase::LoadFromFolder (const std::string & inFolderPath, std::string & outError) {
  mMessagesById.clear () ;

#ifdef _WIN32
  const DWORD attrs = GetFileAttributesA (inFolderPath.c_str ()) ;
  if ((attrs == INVALID_FILE_ATTRIBUTES) || ((attrs & FILE_ATTRIBUTE_DIRECTORY) == 0)) {
    outError = "DBC folder does not exist or is not a folder: " + inFolderPath ;
    return false ;
  }

  std::string base = inFolderPath ;
  if (! base.empty () && (base.back () != '\\') && (base.back () != '/')) {
    base += '\\' ;
  }

  WIN32_FIND_DATAA findData ;
  const std::string pattern = base + "*.dbc" ;
  HANDLE hFind = FindFirstFileA (pattern.c_str (), & findData) ;
  if (hFind != INVALID_HANDLE_VALUE) {
    do {
      if ((findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0) {
        LoadOneFile (base + findData.cFileName) ;   // best-effort: a bad file is skipped, not fatal
      }
    } while (FindNextFileA (hFind, & findData) != 0) ;
    FindClose (hFind) ;
  }
  // No .dbc files found is not an error -- mMessagesById just stays empty.
  return true ;
#else
  outError = "DBC folder loading is only implemented on Windows." ;
  return false ;
#endif
}
