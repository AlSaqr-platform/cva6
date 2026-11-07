// dmi_debug_dpi.cc -- Standalone SimDTM DPI-C driver for CVA6-CFI Issue 3.
//
// Drives the standard RISC-V Debug Transport Module (DMI) interface (`debug_tick`)
// already built into `ariane_testharness.sv` (`SimDTM` -> `i_dm_top`):
//   1. Activates Debug Module (`DMControl.dmactive = 1`) at tick 20.
//   2. Issues two standard DMI halt+resume sequences (Pulse 1 at tick 520 for Subtest 1;
//      Pulse 2 at tick 950 for Subtest 2):
//      - Write `DMControl (0x10) = 0x80000001` (`haltreq=1, dmactive=1`)
//      - Poll `DMStatus (0x11)` until `allhalted` (bit 9) == 1
//      - Write `DMControl (0x10) = 0x40000001` (`haltreq=0, resumereq=1, dmactive=1`)
//      - Poll `DMStatus (0x11)` until `allresumeack` (bit 17) == 1
//      - Write `DMControl (0x10) = 0x00000001` (`haltreq=0, resumereq=0, dmactive=1`)

typedef unsigned char svBit;

extern "C" int debug_tick(svBit* debug_req_valid, svBit debug_req_ready,
                          int* debug_req_bits_addr, int* debug_req_bits_op,
                          int* debug_req_bits_data, svBit debug_resp_valid,
                          svBit* debug_resp_ready, int debug_resp_bits_resp,
                          int debug_resp_bits_data) {
  (void)debug_resp_bits_resp;

  enum State {
    IDLE = 0,
    SEND_HALT,
    WAIT_HALTED,
    SEND_RESUME,
    WAIT_RESUMED,
    CLEAR_RESUME
  };

  static int tick = 0;
  static State state = IDLE;
  static int wait_resp = 0;
  tick++;

  *debug_resp_ready = 1;
  *debug_req_valid = 0;
  *debug_req_bits_addr = 0;
  *debug_req_bits_op = 0;
  *debug_req_bits_data = 0;

  // Step 0: Activate Debug Module (`dmactive = 1`) after reset
  if (tick == 20 && debug_req_ready) {
    *debug_req_valid = 1;
    *debug_req_bits_addr = 0x10;            // dm::DMControl
    *debug_req_bits_op = 2;                 // dm::DTM_WRITE
    *debug_req_bits_data = (int)0x00000001u; // dmactive=1
    return 0;
  }

  // Trigger Pulse 1 at tick 520 (Subtest 1) and Pulse 2 at tick 1074 (Subtest 2, cycle 1085)
  if ((tick == 520 || tick == 1074) && state == IDLE) {
    state = SEND_HALT;
    wait_resp = 0;
  }

  if (debug_resp_valid && wait_resp) {
    wait_resp = 0;
    if (state == WAIT_HALTED && ((debug_resp_bits_data >> 9) & 1)) {
      state = SEND_RESUME;
    } else if (state == WAIT_RESUMED && ((debug_resp_bits_data >> 17) & 1)) {
      state = CLEAR_RESUME;
    }
  }

  if (!debug_req_ready || wait_resp) {
    return 0;
  }

  if (state == SEND_HALT) {
    *debug_req_valid = 1;
    *debug_req_bits_addr = 0x10;             // dm::DMControl
    *debug_req_bits_op = 2;                  // dm::DTM_WRITE
    *debug_req_bits_data = (int)0x80000001u; // haltreq=1, dmactive=1
    state = WAIT_HALTED;
    wait_resp = 1;
  } else if (state == WAIT_HALTED || state == WAIT_RESUMED) {
    *debug_req_valid = 1;
    *debug_req_bits_addr = 0x11;             // dm::DMStatus
    *debug_req_bits_op = 1;                  // dm::DTM_READ
    *debug_req_bits_data = 0;
    wait_resp = 1;
  } else if (state == SEND_RESUME) {
    *debug_req_valid = 1;
    *debug_req_bits_addr = 0x10;             // dm::DMControl
    *debug_req_bits_op = 2;                  // dm::DTM_WRITE
    *debug_req_bits_data = (int)0x40000001u; // haltreq=0, resumereq=1, dmactive=1
    state = WAIT_RESUMED;
    wait_resp = 1;
  } else if (state == CLEAR_RESUME) {
    *debug_req_valid = 1;
    *debug_req_bits_addr = 0x10;             // dm::DMControl
    *debug_req_bits_op = 2;                  // dm::DTM_WRITE
    *debug_req_bits_data = (int)0x00000001u; // haltreq=0, resumereq=0, dmactive=1
    state = IDLE;
    wait_resp = 1;
  }

  return 0;
}

extern "C" int jtag_tick(svBit* jtag_TCK, svBit* jtag_TMS, svBit* jtag_TDI,
                         svBit* jtag_TRSTn, svBit jtag_TDO) {
  (void)jtag_TDO;
  *jtag_TCK = 0;
  *jtag_TMS = 0;
  *jtag_TDI = 0;
  *jtag_TRSTn = 1;
  return 0;
}
