
/* ----- XHCI Capability Register Fields ------- */
use core::time::Duration;

pub const CAP_HCCPARAMS1_MAX_PORT_SHIFT: u32 = 24;
pub const CAP_HCCPARAMS1_MAX_PORT_MASK: u32 = 0xFF00_0000;
pub const CAP_HCCPARAMS1_SLOTS_SHIFT: u32 = 0;
pub const CAP_HCCPARAMS1_SLOTS_MASK: u32 = 0x0000_00FF;

/* ------- XHCI Operation Register Fields -------- */
pub const OP_CMD_RUN_STOP_SHIFT: u32 = 0x0;
pub const OP_CMD_RUN_STOP_MASK: u32 = 0x1 << OP_CMD_RUN_STOP_SHIFT;
pub const OP_CMD_RESET_MASK: u32 = 0x1 << 1;

pub const OP_STS_HLT_SHIFT: u32 = 0;
pub const OP_STS_HLT_MASK: u32 = 0x1 << OP_STS_HLT_SHIFT;
pub const OP_STS_CNR_SHIFT: u32 = 11;
pub const OP_STS_CNR_MASK: u32 = 1 << OP_STS_CNR_SHIFT;


/* --------- TIMEOUT Constants ----------------- */
pub const HALT_TIMEOUT: Duration = Duration::from_millis(16);
pub const RESET_TIMEOUT: Duration = Duration::from_millis(250);
