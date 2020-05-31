use core::time::Duration;

/* ------ XHCI PCI Config Space ---------------- */
pub const USB_INTEL_XUSB2PR: u8 = 0xD0;
pub const USB_INTEL_USB2PRM: u8 = 0xD4;
pub const USB_INTEL_USB3_PSSEN: u8 = 0xD8;
pub const USB_INTEL_USB3PRM: u8 = 0xDC;

/* ----- XHCI Capability Register Fields ------- */
pub const CAP_HC_VERSION_SHIFT: u32 = 16;
pub const CAP_HC_VERSION_MASK: u32 = 0xFFFF_0000;
pub const CAP_HCCPARAMS1_MAX_PORT_SHIFT: u32 = 24;
pub const CAP_HCCPARAMS1_MAX_PORT_MASK: u32 = 0xFF00_0000;
pub const CAP_HCCPARAMS1_SLOTS_SHIFT: u32 = 0;
pub const CAP_HCCPARAMS1_SLOTS_MASK: u32 = 0x0000_00FF;

pub const CAP_DBOFFSET_MASK: u32 = 0xFFFF_FFFC;
// Bit 1:0 reserved
pub const CAP_RTSOFFSET_MASK: u32 = 0xFFFF_FFE0; // Bit 4:0 reserved

/* ------- XHCI Operation Register Fields ------ */
pub const OP_CMD_RUN_STOP_SHIFT: u32 = 0x0;
pub const OP_CMD_RUN_STOP_MASK: u32 = 0x1 << OP_CMD_RUN_STOP_SHIFT;
pub const OP_CMD_INT_EN_MASK: u32 = 1 << 2;
pub const OP_CMD_HSERR_EN_MASK: u32 = 1 << 3;
pub const OP_CMD_RESET_MASK: u32 = 0x1 << 1;

pub const OP_STS_HLT_SHIFT: u32 = 0;
pub const OP_STS_HLT_MASK: u32 = 0x1 << OP_STS_HLT_SHIFT;
pub const OP_STS_CNR_SHIFT: u32 = 11;
pub const OP_STS_CNR_MASK: u32 = 1 << OP_STS_CNR_SHIFT;

pub const OP_CRCR_RCS_SHIFT: u64 = 0;
pub const OP_CRCR_RCS_MASK: u64 = 1 << OP_CRCR_RCS_SHIFT;
pub const OP_CRCR_CS_SHIFT: u64 = 0;
pub const OP_CRCR_CS_MASK: u64 = 1 << OP_CRCR_CS_SHIFT;
pub const OP_CRCR_CA_SHIFT: u64 = 2;
pub const OP_CRCR_CA_MASK: u64 = 1 << OP_CRCR_CA_SHIFT;
pub const OP_CRCR_CRR_SHIFT: u64 = 2;
pub const OP_CRCR_CRR_MASK: u64 = 1 << OP_CRCR_CRR_SHIFT;
pub const OP_CRCR_RES_MASK: u64 = 0b11 << 4;
pub const OP_CRCR_CRPTR_MASK: u64 = 0xFFFF_FFFF_FFFF_FFC0; // 63:6

/* ----------- XHCI Operational Register Per Port ------------- */
pub const OP_PORT_STATUS_CCS_SHIFT: u32 = 0;
pub const OP_PORT_STATUS_CCS_MASK: u32 = 0x1 << OP_PORT_STATUS_CCS_SHIFT;
pub const OP_PORT_STATUS_PED_SHIFT: u32 = 1;
pub const OP_PORT_STATUS_PED_MASK: u32 = 1 << OP_PORT_STATUS_PED_SHIFT;
pub const OP_PORT_STATUS_CSC_SHIFT: u32 = 17;
pub const OP_PORT_STATUS_CSC_MASK: u32 = 1 << OP_PORT_STATUS_CSC_SHIFT;
pub const OP_PORT_STATUS_PEC_SHIFT: u32 = 18;
pub const OP_PORT_STATUS_PEC_MASK: u32 = 1 << OP_PORT_STATUS_PEC_SHIFT;
pub const OP_PORT_STATUS_WRC_SHIFT: u32 = 19;
pub const OP_PORT_STATUS_WRC_MASK: u32 = 1 << OP_PORT_STATUS_WRC_SHIFT;
pub const OP_PORT_STATUS_OCC_SHIFT: u32 = 20;
pub const OP_PORT_STATUS_OCC_MASK: u32 = 1 << OP_PORT_STATUS_OCC_SHIFT;

pub const INT_IRQ_FLAG_INT_PENDING_MASK: u32 = 0x1;
pub const INT_IRQ_FLAG_INT_EN_MASK: u32 = 0x2;
pub const INT_ERDP_BUSY_MASK: u64 = 0x1 << 3;
pub const INT_ERDP_DESI_MASK: u64 = 0b111;
pub const INT_ERDP_DEQUEUE_PTR_MASK: u64 = 0xFFFF_FFFF_FFFF_FFF0;
pub const INT_ERSTSZ_TABLE_SIZE_MASK: u32 = 0x0000_FFFF;

/* ---------------- TRB Fields ----------------- */
pub const TRB_COMMON_TYPE_SHIFT: u16 = 10;
pub const TRB_COMMON_TYPE_MASK: u16 = 0x3F << TRB_COMMON_TYPE_SHIFT;

pub const TRB_COMMON_CYCLE_STATE_MASK: u16 = 1;
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
pub const TRB_TYPE_NOOP_COMMAND: u16 = 23;
pub const TRB_TYPE_EVNT_TRANSFER: u16 = 32;
pub const TRB_TYPE_EVNT_CMD_COMPLETE: u16 = 33;
pub const TRB_TYPE_EVNT_PORT_STATUS_CHG: u16 = 34;
pub const TRB_TYPE_EVNT_BW_REQUEST: u16 = 35;
pub const TRB_TYPE_EVNT_DOORBELL: u16 = 36;
pub const TRB_TYPE_EVNT_HC: u16 = 37;
pub const TRB_TYPE_EVNT_DEV_NOTIFY: u16 = 38;
pub const TRB_TYPE_EVNT_MFINDEX_WRAP: u16 = 39;

pub const EVENT_RING_NUM_SEGMENTS: usize = 1;
pub const TRBS_PER_SEGMENT: usize = 256;

/* --------- TIMEOUT Constants ----------------- */
pub const HALT_TIMEOUT: Duration = Duration::from_millis(16);
pub const RESET_TIMEOUT: Duration = Duration::from_millis(250);

