/* ----- XHCI Capability Register Fields ------- */
use core::time::Duration;

pub const CAP_HCCPARAMS1_MAX_PORT_SHIFT: u32 = 24;
pub const CAP_HCCPARAMS1_MAX_PORT_MASK: u32 = 0xFF00_0000;
pub const CAP_HCCPARAMS1_SLOTS_SHIFT: u32 = 0;
pub const CAP_HCCPARAMS1_SLOTS_MASK: u32 = 0x0000_00FF;

/* ------- XHCI Operation Register Fields ------ */
pub const OP_CMD_RUN_STOP_SHIFT: u32 = 0x0;
pub const OP_CMD_RUN_STOP_MASK: u32 = 0x1 << OP_CMD_RUN_STOP_SHIFT;
pub const OP_CMD_RESET_MASK: u32 = 0x1 << 1;

pub const OP_STS_HLT_SHIFT: u32 = 0;
pub const OP_STS_HLT_MASK: u32 = 0x1 << OP_STS_HLT_SHIFT;
pub const OP_STS_CNR_SHIFT: u32 = 11;
pub const OP_STS_CNR_MASK: u32 = 1 << OP_STS_CNR_SHIFT;

pub const OP_CRCR_RCS_SHIFT: u64 = 0;
pub const OP_CRCR_RCS_MASK: u64 = 1 << OP_CRCR_RCS_SHIFT;
pub const OP_CRCR_CS_SHIFT: u64 = 1;
pub const OP_CRCR_CS_MASK: u64 = 1 << OP_CRCR_CS_SHIFT;
pub const OP_CRCR_CA_SHIFT: u64 = 2;
pub const OP_CRCR_CA_MASK: u64 = 1 << OP_CRCR_CA_SHIFT;
pub const OP_CRCR_CRR_SHIFT: u64 = 2;
pub const OP_CRCR_CRR_MASK: u64 = 1 << OP_CRCR_CRR_SHIFT;
pub const OP_CRCR_RES_MASK: u64 = 0b11 << 4;
pub const OP_CRCR_CRPTR_MASK: u64 = 0xFFFF_FFFF_FFFF_FFC0; // 63:6

/* ---------------- TRB Fields ----------------- */
pub const TRB_TYPE_SHIFT: u16 = 10;
pub const TRB_TYPE_MASK: u16 = 0x3F << TRB_TYPE_SHIFT;

pub const TRB_LINK_TOGGLE_SHIFT: u16 = 1;
pub const TRB_LINK_TOGGLE_MASK: u16 = 0x1 << TRB_LINK_TOGGLE_SHIFT;

/* ---------------- TRB Types ------------------ */
pub const TRB_TYPE_RESERVED: u16 = 0;
pub const TRB_TYPE_NORMAL: u16 = 1;
pub const TRB_TYPE_SETUP: u16 = 2;
pub const TRB_TYPE_DATA: u16 = 3;
pub const TRB_TYPE_STATUS: u16 = 4;
pub const TRB_TYPE_ISOCH: u16 = 5;
pub const TRB_TYPE_LINK: u16 = 6;
pub const TRB_TYPE_EVENT_DATA: u16 = 7;
pub const TRB_TYPE_NOOP: u16 = 8;
pub const TRB_TYPE_ENABLE_SLOT_CMD: u16 = 9;
pub const TRB_TYPE_DISABLE_SLOT_CMD: u16 = 10;
pub const TRB_TYPE_ADDRESS_DEVICE_CMD: u16 = 11;
pub const TRB_TYPE_CONFIG_ENDPOINT_CMD: u16 = 12;

/* --------- TIMEOUT Constants ----------------- */
pub const HALT_TIMEOUT: Duration = Duration::from_millis(16);
pub const RESET_TIMEOUT: Duration = Duration::from_millis(250);
