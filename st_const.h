#ifndef ST_CONST_INCLUDED
#define ST_CONST_INCLUDED

// system settings
#define NCHAN           12  // number of tracking channel
#define NPSE            3   // number of parameter search engine
#define NSIGNAL_MODES   4   // number of signal strength modes
#define NGPS_SVS        32  // number of GPS Satellites
#define NGALILEO_SVS    0   // number of Galileo satellites
#define NGLONASS_SVS    0   // number of GLONASS satellites
#define NSBAS_SVS       6   // number of SBAS satellites (WAAS, EGNOS, MTSAS..)

#define NSFWORDS        10  // number of words per subframe
#define NSFS            5   // number of subframes per frame
#define NFRAME_PAGES    25  // number of pages of GPS broadcast data

#define TCXO_FREQ       (16367598L)  // TCXO frequency in Hz

// Receiver fix mode
enum {
  FIX_NONE,        // no fix
  FIX_PREDICTION,  // fix in prediction
  FIX_2D,          // 2D fix
  FIX_3D,          // 3D fix
  FIX_DIFF,        // fix with differential corrections
  FIX_NMODE };     // number of fix mode

// Channel state
enum {
  CH_IDLE,         // idle
  CH_PULLIN,       // pull-in state
  CH_BITSYNC,      // channel is waiting bit synchronization
  CH_TRACK,        // channel is in-tracking
  CH_FRAMESYNC,    // channel is waiting frame synchronization
  CH_EPHEMERIS,    // channel is receiving ephemeris data
  CH_UNHEALTHY,    // channel is available, but the SV is un-healthy
  CH_AVAILABLE,    // channel is available for position fix
  CH_NSTATE };

// Channel fix mode
enum {
  CH_NOFIX,        // no use in position fix
  CH_FIX,          // has used in position fix
  CH_DIFFERENTIAL, // has used in position fix with differential data
  CH_NFMODE };

// General Mathmetical constants
#define R2D  (57.2957795131)  // deg / rad

// Bool constants
#define FALSE       0
#define TRUE        (!FALSE)

// Bit constants
#define BIT0        (0x01)
#define BIT1        (0x02)
#define BIT2        (0x04)
#define BIT3        (0x08)
#define BIT4        (0x10)
#define BIT5        (0x20)
#define BIT6        (0x40)
#define BIT7        (0x80)
#define BIT8        (0x0100)
#define BIT9        (0x0200)
#define BIT10       (0x0400)
#define BIT11       (0x0800)
#define BIT12       (0x1000)
#define BIT13       (0x2000)
#define BIT14       (0x4000)
#define BIT15       (0x8000)
#define BIT16       (0x00010000L)
#define BIT17       (0x00020000L)
#define BIT18       (0x00040000L)
#define BIT19       (0x00080000L)
#define BIT20       (0x00100000L)
#define BIT21       (0x00200000L)
#define BIT22       (0x00400000L)
#define BIT23       (0x00800000L)
#define BIT24       (0x01000000L)
#define BIT25       (0x02000000L)
#define BIT26       (0x04000000L)
#define BIT27       (0x08000000L)
#define BIT28       (0x10000000L)
#define BIT29       (0x20000000L)
#define BIT30       (0x40000000L)
#define BIT31       (0x80000000L)

#endif  // #ifndef ST_CONST_INCLUDED

