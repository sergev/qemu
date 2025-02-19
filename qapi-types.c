/* AUTOMATICALLY GENERATED, DO NOT MODIFY */

/*
 * deallocation functions for schema-defined QAPI types
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *  Michael Roth      <mdroth@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#include "qapi/dealloc-visitor.h"
#include "qapi-types.h"
#include "qapi-visit.h"

const char * const ErrorClass_lookup[] = {
    [ERROR_CLASS_GENERIC_ERROR] = "GenericError",
    [ERROR_CLASS_COMMAND_NOT_FOUND] = "CommandNotFound",
    [ERROR_CLASS_DEVICE_ENCRYPTED] = "DeviceEncrypted",
    [ERROR_CLASS_DEVICE_NOT_ACTIVE] = "DeviceNotActive",
    [ERROR_CLASS_DEVICE_NOT_FOUND] = "DeviceNotFound",
    [ERROR_CLASS_KVM_MISSING_CAP] = "KVMMissingCap",
    [ERROR_CLASS_MAX] = NULL,
};

const char * const OnOffAuto_lookup[] = {
    [ON_OFF_AUTO_AUTO] = "auto",
    [ON_OFF_AUTO_ON] = "on",
    [ON_OFF_AUTO_OFF] = "off",
    [ON_OFF_AUTO_MAX] = NULL,
};

const char * const ImageInfoSpecificKind_lookup[] = {
    [IMAGE_INFO_SPECIFIC_KIND_QCOW2] = "qcow2",
    [IMAGE_INFO_SPECIFIC_KIND_VMDK] = "vmdk",
    [IMAGE_INFO_SPECIFIC_KIND_MAX] = NULL,
};

const char * const BlockDeviceIoStatus_lookup[] = {
    [BLOCK_DEVICE_IO_STATUS_OK] = "ok",
    [BLOCK_DEVICE_IO_STATUS_FAILED] = "failed",
    [BLOCK_DEVICE_IO_STATUS_NOSPACE] = "nospace",
    [BLOCK_DEVICE_IO_STATUS_MAX] = NULL,
};

const char * const DirtyBitmapStatus_lookup[] = {
    [DIRTY_BITMAP_STATUS_ACTIVE] = "active",
    [DIRTY_BITMAP_STATUS_DISABLED] = "disabled",
    [DIRTY_BITMAP_STATUS_FROZEN] = "frozen",
    [DIRTY_BITMAP_STATUS_MAX] = NULL,
};

const char * const BlockdevOnError_lookup[] = {
    [BLOCKDEV_ON_ERROR_REPORT] = "report",
    [BLOCKDEV_ON_ERROR_IGNORE] = "ignore",
    [BLOCKDEV_ON_ERROR_ENOSPC] = "enospc",
    [BLOCKDEV_ON_ERROR_STOP] = "stop",
    [BLOCKDEV_ON_ERROR_MAX] = NULL,
};

const char * const MirrorSyncMode_lookup[] = {
    [MIRROR_SYNC_MODE_TOP] = "top",
    [MIRROR_SYNC_MODE_FULL] = "full",
    [MIRROR_SYNC_MODE_NONE] = "none",
    [MIRROR_SYNC_MODE_DIRTY_BITMAP] = "dirty-bitmap",
    [MIRROR_SYNC_MODE_MAX] = NULL,
};

const char * const BlockJobType_lookup[] = {
    [BLOCK_JOB_TYPE_COMMIT] = "commit",
    [BLOCK_JOB_TYPE_STREAM] = "stream",
    [BLOCK_JOB_TYPE_MIRROR] = "mirror",
    [BLOCK_JOB_TYPE_BACKUP] = "backup",
    [BLOCK_JOB_TYPE_MAX] = NULL,
};

const char * const NewImageMode_lookup[] = {
    [NEW_IMAGE_MODE_EXISTING] = "existing",
    [NEW_IMAGE_MODE_ABSOLUTE_PATHS] = "absolute-paths",
    [NEW_IMAGE_MODE_MAX] = NULL,
};

const char * const BlockdevDiscardOptions_lookup[] = {
    [BLOCKDEV_DISCARD_OPTIONS_IGNORE] = "ignore",
    [BLOCKDEV_DISCARD_OPTIONS_UNMAP] = "unmap",
    [BLOCKDEV_DISCARD_OPTIONS_MAX] = NULL,
};

const char * const BlockdevDetectZeroesOptions_lookup[] = {
    [BLOCKDEV_DETECT_ZEROES_OPTIONS_OFF] = "off",
    [BLOCKDEV_DETECT_ZEROES_OPTIONS_ON] = "on",
    [BLOCKDEV_DETECT_ZEROES_OPTIONS_UNMAP] = "unmap",
    [BLOCKDEV_DETECT_ZEROES_OPTIONS_MAX] = NULL,
};

const char * const BlockdevAioOptions_lookup[] = {
    [BLOCKDEV_AIO_OPTIONS_THREADS] = "threads",
    [BLOCKDEV_AIO_OPTIONS_NATIVE] = "native",
    [BLOCKDEV_AIO_OPTIONS_MAX] = NULL,
};

const char * const BlockdevDriver_lookup[] = {
    [BLOCKDEV_DRIVER_ARCHIPELAGO] = "archipelago",
    [BLOCKDEV_DRIVER_BLKDEBUG] = "blkdebug",
    [BLOCKDEV_DRIVER_BLKVERIFY] = "blkverify",
    [BLOCKDEV_DRIVER_BOCHS] = "bochs",
    [BLOCKDEV_DRIVER_CLOOP] = "cloop",
    [BLOCKDEV_DRIVER_DMG] = "dmg",
    [BLOCKDEV_DRIVER_FILE] = "file",
    [BLOCKDEV_DRIVER_FTP] = "ftp",
    [BLOCKDEV_DRIVER_FTPS] = "ftps",
    [BLOCKDEV_DRIVER_HOST_CDROM] = "host_cdrom",
    [BLOCKDEV_DRIVER_HOST_DEVICE] = "host_device",
    [BLOCKDEV_DRIVER_HOST_FLOPPY] = "host_floppy",
    [BLOCKDEV_DRIVER_HTTP] = "http",
    [BLOCKDEV_DRIVER_HTTPS] = "https",
    [BLOCKDEV_DRIVER_NULL_AIO] = "null-aio",
    [BLOCKDEV_DRIVER_NULL_CO] = "null-co",
    [BLOCKDEV_DRIVER_PARALLELS] = "parallels",
    [BLOCKDEV_DRIVER_QCOW] = "qcow",
    [BLOCKDEV_DRIVER_QCOW2] = "qcow2",
    [BLOCKDEV_DRIVER_QED] = "qed",
    [BLOCKDEV_DRIVER_QUORUM] = "quorum",
    [BLOCKDEV_DRIVER_RAW] = "raw",
    [BLOCKDEV_DRIVER_TFTP] = "tftp",
    [BLOCKDEV_DRIVER_VDI] = "vdi",
    [BLOCKDEV_DRIVER_VHDX] = "vhdx",
    [BLOCKDEV_DRIVER_VMDK] = "vmdk",
    [BLOCKDEV_DRIVER_VPC] = "vpc",
    [BLOCKDEV_DRIVER_VVFAT] = "vvfat",
    [BLOCKDEV_DRIVER_MAX] = NULL,
};

const char * const Qcow2OverlapCheckMode_lookup[] = {
    [QCOW2_OVERLAP_CHECK_MODE_NONE] = "none",
    [QCOW2_OVERLAP_CHECK_MODE_CONSTANT] = "constant",
    [QCOW2_OVERLAP_CHECK_MODE_CACHED] = "cached",
    [QCOW2_OVERLAP_CHECK_MODE_ALL] = "all",
    [QCOW2_OVERLAP_CHECK_MODE_MAX] = NULL,
};

const char * const Qcow2OverlapChecksKind_lookup[] = {
    [QCOW2_OVERLAP_CHECKS_KIND_FLAGS] = "flags",
    [QCOW2_OVERLAP_CHECKS_KIND_MODE] = "mode",
    [QCOW2_OVERLAP_CHECKS_KIND_MAX] = NULL,
};

const int Qcow2OverlapChecks_qtypes[QTYPE_MAX] = {
    [QTYPE_QDICT] = QCOW2_OVERLAP_CHECKS_KIND_FLAGS,
    [QTYPE_QSTRING] = QCOW2_OVERLAP_CHECKS_KIND_MODE,
};
const char * const BlkdebugEvent_lookup[] = {
    [BLKDEBUG_EVENT_L1_UPDATE] = "l1_update",
    [BLKDEBUG_EVENT_L1_GROW_ALLOC_TABLE] = "l1_grow.alloc_table",
    [BLKDEBUG_EVENT_L1_GROW_WRITE_TABLE] = "l1_grow.write_table",
    [BLKDEBUG_EVENT_L1_GROW_ACTIVATE_TABLE] = "l1_grow.activate_table",
    [BLKDEBUG_EVENT_L2_LOAD] = "l2_load",
    [BLKDEBUG_EVENT_L2_UPDATE] = "l2_update",
    [BLKDEBUG_EVENT_L2_UPDATE_COMPRESSED] = "l2_update_compressed",
    [BLKDEBUG_EVENT_L2_ALLOC_COW_READ] = "l2_alloc.cow_read",
    [BLKDEBUG_EVENT_L2_ALLOC_WRITE] = "l2_alloc.write",
    [BLKDEBUG_EVENT_READ_AIO] = "read_aio",
    [BLKDEBUG_EVENT_READ_BACKING_AIO] = "read_backing_aio",
    [BLKDEBUG_EVENT_READ_COMPRESSED] = "read_compressed",
    [BLKDEBUG_EVENT_WRITE_AIO] = "write_aio",
    [BLKDEBUG_EVENT_WRITE_COMPRESSED] = "write_compressed",
    [BLKDEBUG_EVENT_VMSTATE_LOAD] = "vmstate_load",
    [BLKDEBUG_EVENT_VMSTATE_SAVE] = "vmstate_save",
    [BLKDEBUG_EVENT_COW_READ] = "cow_read",
    [BLKDEBUG_EVENT_COW_WRITE] = "cow_write",
    [BLKDEBUG_EVENT_REFTABLE_LOAD] = "reftable_load",
    [BLKDEBUG_EVENT_REFTABLE_GROW] = "reftable_grow",
    [BLKDEBUG_EVENT_REFTABLE_UPDATE] = "reftable_update",
    [BLKDEBUG_EVENT_REFBLOCK_LOAD] = "refblock_load",
    [BLKDEBUG_EVENT_REFBLOCK_UPDATE] = "refblock_update",
    [BLKDEBUG_EVENT_REFBLOCK_UPDATE_PART] = "refblock_update_part",
    [BLKDEBUG_EVENT_REFBLOCK_ALLOC] = "refblock_alloc",
    [BLKDEBUG_EVENT_REFBLOCK_ALLOC_HOOKUP] = "refblock_alloc.hookup",
    [BLKDEBUG_EVENT_REFBLOCK_ALLOC_WRITE] = "refblock_alloc.write",
    [BLKDEBUG_EVENT_REFBLOCK_ALLOC_WRITE_BLOCKS] = "refblock_alloc.write_blocks",
    [BLKDEBUG_EVENT_REFBLOCK_ALLOC_WRITE_TABLE] = "refblock_alloc.write_table",
    [BLKDEBUG_EVENT_REFBLOCK_ALLOC_SWITCH_TABLE] = "refblock_alloc.switch_table",
    [BLKDEBUG_EVENT_CLUSTER_ALLOC] = "cluster_alloc",
    [BLKDEBUG_EVENT_CLUSTER_ALLOC_BYTES] = "cluster_alloc_bytes",
    [BLKDEBUG_EVENT_CLUSTER_FREE] = "cluster_free",
    [BLKDEBUG_EVENT_FLUSH_TO_OS] = "flush_to_os",
    [BLKDEBUG_EVENT_FLUSH_TO_DISK] = "flush_to_disk",
    [BLKDEBUG_EVENT_PWRITEV_RMW_HEAD] = "pwritev_rmw.head",
    [BLKDEBUG_EVENT_PWRITEV_RMW_AFTER_HEAD] = "pwritev_rmw.after_head",
    [BLKDEBUG_EVENT_PWRITEV_RMW_TAIL] = "pwritev_rmw.tail",
    [BLKDEBUG_EVENT_PWRITEV_RMW_AFTER_TAIL] = "pwritev_rmw.after_tail",
    [BLKDEBUG_EVENT_PWRITEV] = "pwritev",
    [BLKDEBUG_EVENT_PWRITEV_ZERO] = "pwritev_zero",
    [BLKDEBUG_EVENT_PWRITEV_DONE] = "pwritev_done",
    [BLKDEBUG_EVENT_EMPTY_IMAGE_PREPARE] = "empty_image_prepare",
    [BLKDEBUG_EVENT_MAX] = NULL,
};

const char * const QuorumReadPattern_lookup[] = {
    [QUORUM_READ_PATTERN_QUORUM] = "quorum",
    [QUORUM_READ_PATTERN_FIFO] = "fifo",
    [QUORUM_READ_PATTERN_MAX] = NULL,
};

const char * const BlockdevRefKind_lookup[] = {
    [BLOCKDEV_REF_KIND_DEFINITION] = "definition",
    [BLOCKDEV_REF_KIND_REFERENCE] = "reference",
    [BLOCKDEV_REF_KIND_MAX] = NULL,
};

const int BlockdevRef_qtypes[QTYPE_MAX] = {
    [QTYPE_QDICT] = BLOCKDEV_REF_KIND_DEFINITION,
    [QTYPE_QSTRING] = BLOCKDEV_REF_KIND_REFERENCE,
};
const char * const BlockErrorAction_lookup[] = {
    [BLOCK_ERROR_ACTION_IGNORE] = "ignore",
    [BLOCK_ERROR_ACTION_REPORT] = "report",
    [BLOCK_ERROR_ACTION_STOP] = "stop",
    [BLOCK_ERROR_ACTION_MAX] = NULL,
};

const char * const PreallocMode_lookup[] = {
    [PREALLOC_MODE_OFF] = "off",
    [PREALLOC_MODE_METADATA] = "metadata",
    [PREALLOC_MODE_FALLOC] = "falloc",
    [PREALLOC_MODE_FULL] = "full",
    [PREALLOC_MODE_MAX] = NULL,
};

const char * const BiosAtaTranslation_lookup[] = {
    [BIOS_ATA_TRANSLATION_AUTO] = "auto",
    [BIOS_ATA_TRANSLATION_NONE] = "none",
    [BIOS_ATA_TRANSLATION_LBA] = "lba",
    [BIOS_ATA_TRANSLATION_LARGE] = "large",
    [BIOS_ATA_TRANSLATION_RECHS] = "rechs",
    [BIOS_ATA_TRANSLATION_MAX] = NULL,
};

const char * const TraceEventState_lookup[] = {
    [TRACE_EVENT_STATE_UNAVAILABLE] = "unavailable",
    [TRACE_EVENT_STATE_DISABLED] = "disabled",
    [TRACE_EVENT_STATE_ENABLED] = "enabled",
    [TRACE_EVENT_STATE_MAX] = NULL,
};

const char * const LostTickPolicy_lookup[] = {
    [LOST_TICK_POLICY_DISCARD] = "discard",
    [LOST_TICK_POLICY_DELAY] = "delay",
    [LOST_TICK_POLICY_MERGE] = "merge",
    [LOST_TICK_POLICY_SLEW] = "slew",
    [LOST_TICK_POLICY_MAX] = NULL,
};

const char * const RunState_lookup[] = {
    [RUN_STATE_DEBUG] = "debug",
    [RUN_STATE_INMIGRATE] = "inmigrate",
    [RUN_STATE_INTERNAL_ERROR] = "internal-error",
    [RUN_STATE_IO_ERROR] = "io-error",
    [RUN_STATE_PAUSED] = "paused",
    [RUN_STATE_POSTMIGRATE] = "postmigrate",
    [RUN_STATE_PRELAUNCH] = "prelaunch",
    [RUN_STATE_FINISH_MIGRATE] = "finish-migrate",
    [RUN_STATE_RESTORE_VM] = "restore-vm",
    [RUN_STATE_RUNNING] = "running",
    [RUN_STATE_SAVE_VM] = "save-vm",
    [RUN_STATE_SHUTDOWN] = "shutdown",
    [RUN_STATE_SUSPENDED] = "suspended",
    [RUN_STATE_WATCHDOG] = "watchdog",
    [RUN_STATE_GUEST_PANICKED] = "guest-panicked",
    [RUN_STATE_MAX] = NULL,
};

const char * const DataFormat_lookup[] = {
    [DATA_FORMAT_UTF8] = "utf8",
    [DATA_FORMAT_BASE64] = "base64",
    [DATA_FORMAT_MAX] = NULL,
};

const char * const MigrationStatus_lookup[] = {
    [MIGRATION_STATUS_NONE] = "none",
    [MIGRATION_STATUS_SETUP] = "setup",
    [MIGRATION_STATUS_CANCELLING] = "cancelling",
    [MIGRATION_STATUS_CANCELLED] = "cancelled",
    [MIGRATION_STATUS_ACTIVE] = "active",
    [MIGRATION_STATUS_COMPLETED] = "completed",
    [MIGRATION_STATUS_FAILED] = "failed",
    [MIGRATION_STATUS_MAX] = NULL,
};

const char * const MigrationCapability_lookup[] = {
    [MIGRATION_CAPABILITY_XBZRLE] = "xbzrle",
    [MIGRATION_CAPABILITY_RDMA_PIN_ALL] = "rdma-pin-all",
    [MIGRATION_CAPABILITY_AUTO_CONVERGE] = "auto-converge",
    [MIGRATION_CAPABILITY_ZERO_BLOCKS] = "zero-blocks",
    [MIGRATION_CAPABILITY_COMPRESS] = "compress",
    [MIGRATION_CAPABILITY_MAX] = NULL,
};

const char * const MigrationParameter_lookup[] = {
    [MIGRATION_PARAMETER_COMPRESS_LEVEL] = "compress-level",
    [MIGRATION_PARAMETER_COMPRESS_THREADS] = "compress-threads",
    [MIGRATION_PARAMETER_DECOMPRESS_THREADS] = "decompress-threads",
    [MIGRATION_PARAMETER_MAX] = NULL,
};

const char * const NetworkAddressFamily_lookup[] = {
    [NETWORK_ADDRESS_FAMILY_IPV4] = "ipv4",
    [NETWORK_ADDRESS_FAMILY_IPV6] = "ipv6",
    [NETWORK_ADDRESS_FAMILY_UNIX] = "unix",
    [NETWORK_ADDRESS_FAMILY_UNKNOWN] = "unknown",
    [NETWORK_ADDRESS_FAMILY_MAX] = NULL,
};

const char * const VncPrimaryAuth_lookup[] = {
    [VNC_PRIMARY_AUTH_NONE] = "none",
    [VNC_PRIMARY_AUTH_VNC] = "vnc",
    [VNC_PRIMARY_AUTH_RA2] = "ra2",
    [VNC_PRIMARY_AUTH_RA2NE] = "ra2ne",
    [VNC_PRIMARY_AUTH_TIGHT] = "tight",
    [VNC_PRIMARY_AUTH_ULTRA] = "ultra",
    [VNC_PRIMARY_AUTH_TLS] = "tls",
    [VNC_PRIMARY_AUTH_VENCRYPT] = "vencrypt",
    [VNC_PRIMARY_AUTH_SASL] = "sasl",
    [VNC_PRIMARY_AUTH_MAX] = NULL,
};

const char * const VncVencryptSubAuth_lookup[] = {
    [VNC_VENCRYPT_SUB_AUTH_PLAIN] = "plain",
    [VNC_VENCRYPT_SUB_AUTH_TLS_NONE] = "tls-none",
    [VNC_VENCRYPT_SUB_AUTH_X509_NONE] = "x509-none",
    [VNC_VENCRYPT_SUB_AUTH_TLS_VNC] = "tls-vnc",
    [VNC_VENCRYPT_SUB_AUTH_X509_VNC] = "x509-vnc",
    [VNC_VENCRYPT_SUB_AUTH_TLS_PLAIN] = "tls-plain",
    [VNC_VENCRYPT_SUB_AUTH_X509_PLAIN] = "x509-plain",
    [VNC_VENCRYPT_SUB_AUTH_TLS_SASL] = "tls-sasl",
    [VNC_VENCRYPT_SUB_AUTH_X509_SASL] = "x509-sasl",
    [VNC_VENCRYPT_SUB_AUTH_MAX] = NULL,
};

const char * const SpiceQueryMouseMode_lookup[] = {
    [SPICE_QUERY_MOUSE_MODE_CLIENT] = "client",
    [SPICE_QUERY_MOUSE_MODE_SERVER] = "server",
    [SPICE_QUERY_MOUSE_MODE_UNKNOWN] = "unknown",
    [SPICE_QUERY_MOUSE_MODE_MAX] = NULL,
};

const char * const TransactionActionKind_lookup[] = {
    [TRANSACTION_ACTION_KIND_BLOCKDEV_SNAPSHOT_SYNC] = "blockdev-snapshot-sync",
    [TRANSACTION_ACTION_KIND_DRIVE_BACKUP] = "drive-backup",
    [TRANSACTION_ACTION_KIND_BLOCKDEV_BACKUP] = "blockdev-backup",
    [TRANSACTION_ACTION_KIND_ABORT] = "abort",
    [TRANSACTION_ACTION_KIND_BLOCKDEV_SNAPSHOT_INTERNAL_SYNC] = "blockdev-snapshot-internal-sync",
    [TRANSACTION_ACTION_KIND_MAX] = NULL,
};

const char * const DumpGuestMemoryFormat_lookup[] = {
    [DUMP_GUEST_MEMORY_FORMAT_ELF] = "elf",
    [DUMP_GUEST_MEMORY_FORMAT_KDUMP_ZLIB] = "kdump-zlib",
    [DUMP_GUEST_MEMORY_FORMAT_KDUMP_LZO] = "kdump-lzo",
    [DUMP_GUEST_MEMORY_FORMAT_KDUMP_SNAPPY] = "kdump-snappy",
    [DUMP_GUEST_MEMORY_FORMAT_MAX] = NULL,
};

const char * const NetClientOptionsKind_lookup[] = {
    [NET_CLIENT_OPTIONS_KIND_NONE] = "none",
    [NET_CLIENT_OPTIONS_KIND_NIC] = "nic",
    [NET_CLIENT_OPTIONS_KIND_USER] = "user",
    [NET_CLIENT_OPTIONS_KIND_TAP] = "tap",
    [NET_CLIENT_OPTIONS_KIND_L2TPV3] = "l2tpv3",
    [NET_CLIENT_OPTIONS_KIND_SOCKET] = "socket",
    [NET_CLIENT_OPTIONS_KIND_VDE] = "vde",
    [NET_CLIENT_OPTIONS_KIND_DUMP] = "dump",
    [NET_CLIENT_OPTIONS_KIND_BRIDGE] = "bridge",
    [NET_CLIENT_OPTIONS_KIND_HUBPORT] = "hubport",
    [NET_CLIENT_OPTIONS_KIND_NETMAP] = "netmap",
    [NET_CLIENT_OPTIONS_KIND_VHOST_USER] = "vhost-user",
    [NET_CLIENT_OPTIONS_KIND_MAX] = NULL,
};

const char * const SocketAddressKind_lookup[] = {
    [SOCKET_ADDRESS_KIND_INET] = "inet",
    [SOCKET_ADDRESS_KIND_UNIX] = "unix",
    [SOCKET_ADDRESS_KIND_FD] = "fd",
    [SOCKET_ADDRESS_KIND_MAX] = NULL,
};

const char * const QKeyCode_lookup[] = {
    [Q_KEY_CODE_UNMAPPED] = "unmapped",
    [Q_KEY_CODE_SHIFT] = "shift",
    [Q_KEY_CODE_SHIFT_R] = "shift_r",
    [Q_KEY_CODE_ALT] = "alt",
    [Q_KEY_CODE_ALT_R] = "alt_r",
    [Q_KEY_CODE_ALTGR] = "altgr",
    [Q_KEY_CODE_ALTGR_R] = "altgr_r",
    [Q_KEY_CODE_CTRL] = "ctrl",
    [Q_KEY_CODE_CTRL_R] = "ctrl_r",
    [Q_KEY_CODE_MENU] = "menu",
    [Q_KEY_CODE_ESC] = "esc",
    [Q_KEY_CODE_1] = "1",
    [Q_KEY_CODE_2] = "2",
    [Q_KEY_CODE_3] = "3",
    [Q_KEY_CODE_4] = "4",
    [Q_KEY_CODE_5] = "5",
    [Q_KEY_CODE_6] = "6",
    [Q_KEY_CODE_7] = "7",
    [Q_KEY_CODE_8] = "8",
    [Q_KEY_CODE_9] = "9",
    [Q_KEY_CODE_0] = "0",
    [Q_KEY_CODE_MINUS] = "minus",
    [Q_KEY_CODE_EQUAL] = "equal",
    [Q_KEY_CODE_BACKSPACE] = "backspace",
    [Q_KEY_CODE_TAB] = "tab",
    [Q_KEY_CODE_Q] = "q",
    [Q_KEY_CODE_W] = "w",
    [Q_KEY_CODE_E] = "e",
    [Q_KEY_CODE_R] = "r",
    [Q_KEY_CODE_T] = "t",
    [Q_KEY_CODE_Y] = "y",
    [Q_KEY_CODE_U] = "u",
    [Q_KEY_CODE_I] = "i",
    [Q_KEY_CODE_O] = "o",
    [Q_KEY_CODE_P] = "p",
    [Q_KEY_CODE_BRACKET_LEFT] = "bracket_left",
    [Q_KEY_CODE_BRACKET_RIGHT] = "bracket_right",
    [Q_KEY_CODE_RET] = "ret",
    [Q_KEY_CODE_A] = "a",
    [Q_KEY_CODE_S] = "s",
    [Q_KEY_CODE_D] = "d",
    [Q_KEY_CODE_F] = "f",
    [Q_KEY_CODE_G] = "g",
    [Q_KEY_CODE_H] = "h",
    [Q_KEY_CODE_J] = "j",
    [Q_KEY_CODE_K] = "k",
    [Q_KEY_CODE_L] = "l",
    [Q_KEY_CODE_SEMICOLON] = "semicolon",
    [Q_KEY_CODE_APOSTROPHE] = "apostrophe",
    [Q_KEY_CODE_GRAVE_ACCENT] = "grave_accent",
    [Q_KEY_CODE_BACKSLASH] = "backslash",
    [Q_KEY_CODE_Z] = "z",
    [Q_KEY_CODE_X] = "x",
    [Q_KEY_CODE_C] = "c",
    [Q_KEY_CODE_V] = "v",
    [Q_KEY_CODE_B] = "b",
    [Q_KEY_CODE_N] = "n",
    [Q_KEY_CODE_M] = "m",
    [Q_KEY_CODE_COMMA] = "comma",
    [Q_KEY_CODE_DOT] = "dot",
    [Q_KEY_CODE_SLASH] = "slash",
    [Q_KEY_CODE_ASTERISK] = "asterisk",
    [Q_KEY_CODE_SPC] = "spc",
    [Q_KEY_CODE_CAPS_LOCK] = "caps_lock",
    [Q_KEY_CODE_F1] = "f1",
    [Q_KEY_CODE_F2] = "f2",
    [Q_KEY_CODE_F3] = "f3",
    [Q_KEY_CODE_F4] = "f4",
    [Q_KEY_CODE_F5] = "f5",
    [Q_KEY_CODE_F6] = "f6",
    [Q_KEY_CODE_F7] = "f7",
    [Q_KEY_CODE_F8] = "f8",
    [Q_KEY_CODE_F9] = "f9",
    [Q_KEY_CODE_F10] = "f10",
    [Q_KEY_CODE_NUM_LOCK] = "num_lock",
    [Q_KEY_CODE_SCROLL_LOCK] = "scroll_lock",
    [Q_KEY_CODE_KP_DIVIDE] = "kp_divide",
    [Q_KEY_CODE_KP_MULTIPLY] = "kp_multiply",
    [Q_KEY_CODE_KP_SUBTRACT] = "kp_subtract",
    [Q_KEY_CODE_KP_ADD] = "kp_add",
    [Q_KEY_CODE_KP_ENTER] = "kp_enter",
    [Q_KEY_CODE_KP_DECIMAL] = "kp_decimal",
    [Q_KEY_CODE_SYSRQ] = "sysrq",
    [Q_KEY_CODE_KP_0] = "kp_0",
    [Q_KEY_CODE_KP_1] = "kp_1",
    [Q_KEY_CODE_KP_2] = "kp_2",
    [Q_KEY_CODE_KP_3] = "kp_3",
    [Q_KEY_CODE_KP_4] = "kp_4",
    [Q_KEY_CODE_KP_5] = "kp_5",
    [Q_KEY_CODE_KP_6] = "kp_6",
    [Q_KEY_CODE_KP_7] = "kp_7",
    [Q_KEY_CODE_KP_8] = "kp_8",
    [Q_KEY_CODE_KP_9] = "kp_9",
    [Q_KEY_CODE_LESS] = "less",
    [Q_KEY_CODE_F11] = "f11",
    [Q_KEY_CODE_F12] = "f12",
    [Q_KEY_CODE_PRINT] = "print",
    [Q_KEY_CODE_HOME] = "home",
    [Q_KEY_CODE_PGUP] = "pgup",
    [Q_KEY_CODE_PGDN] = "pgdn",
    [Q_KEY_CODE_END] = "end",
    [Q_KEY_CODE_LEFT] = "left",
    [Q_KEY_CODE_UP] = "up",
    [Q_KEY_CODE_DOWN] = "down",
    [Q_KEY_CODE_RIGHT] = "right",
    [Q_KEY_CODE_INSERT] = "insert",
    [Q_KEY_CODE_DELETE] = "delete",
    [Q_KEY_CODE_STOP] = "stop",
    [Q_KEY_CODE_AGAIN] = "again",
    [Q_KEY_CODE_PROPS] = "props",
    [Q_KEY_CODE_UNDO] = "undo",
    [Q_KEY_CODE_FRONT] = "front",
    [Q_KEY_CODE_COPY] = "copy",
    [Q_KEY_CODE_OPEN] = "open",
    [Q_KEY_CODE_PASTE] = "paste",
    [Q_KEY_CODE_FIND] = "find",
    [Q_KEY_CODE_CUT] = "cut",
    [Q_KEY_CODE_LF] = "lf",
    [Q_KEY_CODE_HELP] = "help",
    [Q_KEY_CODE_META_L] = "meta_l",
    [Q_KEY_CODE_META_R] = "meta_r",
    [Q_KEY_CODE_COMPOSE] = "compose",
    [Q_KEY_CODE_PAUSE] = "pause",
    [Q_KEY_CODE_RO] = "ro",
    [Q_KEY_CODE_KP_COMMA] = "kp_comma",
    [Q_KEY_CODE_MAX] = NULL,
};

const char * const KeyValueKind_lookup[] = {
    [KEY_VALUE_KIND_NUMBER] = "number",
    [KEY_VALUE_KIND_QCODE] = "qcode",
    [KEY_VALUE_KIND_MAX] = NULL,
};

const char * const ChardevBackendKind_lookup[] = {
    [CHARDEV_BACKEND_KIND_FILE] = "file",
    [CHARDEV_BACKEND_KIND_SERIAL] = "serial",
    [CHARDEV_BACKEND_KIND_PARALLEL] = "parallel",
    [CHARDEV_BACKEND_KIND_PIPE] = "pipe",
    [CHARDEV_BACKEND_KIND_SOCKET] = "socket",
    [CHARDEV_BACKEND_KIND_UDP] = "udp",
    [CHARDEV_BACKEND_KIND_PTY] = "pty",
    [CHARDEV_BACKEND_KIND_NULL] = "null",
    [CHARDEV_BACKEND_KIND_MUX] = "mux",
    [CHARDEV_BACKEND_KIND_MSMOUSE] = "msmouse",
    [CHARDEV_BACKEND_KIND_BRAILLE] = "braille",
    [CHARDEV_BACKEND_KIND_TESTDEV] = "testdev",
    [CHARDEV_BACKEND_KIND_STDIO] = "stdio",
    [CHARDEV_BACKEND_KIND_CONSOLE] = "console",
    [CHARDEV_BACKEND_KIND_SPICEVMC] = "spicevmc",
    [CHARDEV_BACKEND_KIND_SPICEPORT] = "spiceport",
    [CHARDEV_BACKEND_KIND_VC] = "vc",
    [CHARDEV_BACKEND_KIND_RINGBUF] = "ringbuf",
    [CHARDEV_BACKEND_KIND_MEMORY] = "memory",
    [CHARDEV_BACKEND_KIND_MAX] = NULL,
};

const char * const TpmModel_lookup[] = {
    [TPM_MODEL_TPM_TIS] = "tpm-tis",
    [TPM_MODEL_MAX] = NULL,
};

const char * const TpmType_lookup[] = {
    [TPM_TYPE_PASSTHROUGH] = "passthrough",
    [TPM_TYPE_MAX] = NULL,
};

const char * const TpmTypeOptionsKind_lookup[] = {
    [TPM_TYPE_OPTIONS_KIND_PASSTHROUGH] = "passthrough",
    [TPM_TYPE_OPTIONS_KIND_MAX] = NULL,
};

const char * const CommandLineParameterType_lookup[] = {
    [COMMAND_LINE_PARAMETER_TYPE_STRING] = "string",
    [COMMAND_LINE_PARAMETER_TYPE_BOOLEAN] = "boolean",
    [COMMAND_LINE_PARAMETER_TYPE_NUMBER] = "number",
    [COMMAND_LINE_PARAMETER_TYPE_SIZE] = "size",
    [COMMAND_LINE_PARAMETER_TYPE_MAX] = NULL,
};

const char * const X86CPURegister32_lookup[] = {
    [X86_CPU_REGISTER32_EAX] = "EAX",
    [X86_CPU_REGISTER32_EBX] = "EBX",
    [X86_CPU_REGISTER32_ECX] = "ECX",
    [X86_CPU_REGISTER32_EDX] = "EDX",
    [X86_CPU_REGISTER32_ESP] = "ESP",
    [X86_CPU_REGISTER32_EBP] = "EBP",
    [X86_CPU_REGISTER32_ESI] = "ESI",
    [X86_CPU_REGISTER32_EDI] = "EDI",
    [X86_CPU_REGISTER32_MAX] = NULL,
};

const char * const RxState_lookup[] = {
    [RX_STATE_NORMAL] = "normal",
    [RX_STATE_NONE] = "none",
    [RX_STATE_ALL] = "all",
    [RX_STATE_MAX] = NULL,
};

const char * const InputButton_lookup[] = {
    [INPUT_BUTTON_LEFT] = "Left",
    [INPUT_BUTTON_MIDDLE] = "Middle",
    [INPUT_BUTTON_RIGHT] = "Right",
    [INPUT_BUTTON_WHEEL_UP] = "WheelUp",
    [INPUT_BUTTON_WHEEL_DOWN] = "WheelDown",
    [INPUT_BUTTON_MAX] = NULL,
};

const char * const InputAxis_lookup[] = {
    [INPUT_AXIS_X] = "X",
    [INPUT_AXIS_Y] = "Y",
    [INPUT_AXIS_MAX] = NULL,
};

const char * const InputEventKind_lookup[] = {
    [INPUT_EVENT_KIND_KEY] = "key",
    [INPUT_EVENT_KIND_BTN] = "btn",
    [INPUT_EVENT_KIND_REL] = "rel",
    [INPUT_EVENT_KIND_ABS] = "abs",
    [INPUT_EVENT_KIND_MAX] = NULL,
};

const char * const NumaOptionsKind_lookup[] = {
    [NUMA_OPTIONS_KIND_NODE] = "node",
    [NUMA_OPTIONS_KIND_MAX] = NULL,
};

const char * const HostMemPolicy_lookup[] = {
    [HOST_MEM_POLICY_DEFAULT] = "default",
    [HOST_MEM_POLICY_PREFERRED] = "preferred",
    [HOST_MEM_POLICY_BIND] = "bind",
    [HOST_MEM_POLICY_INTERLEAVE] = "interleave",
    [HOST_MEM_POLICY_MAX] = NULL,
};

const char * const MemoryDeviceInfoKind_lookup[] = {
    [MEMORY_DEVICE_INFO_KIND_DIMM] = "dimm",
    [MEMORY_DEVICE_INFO_KIND_MAX] = NULL,
};

const char * const ACPISlotType_lookup[] = {
    [ACPI_SLOT_TYPE_DIMM] = "DIMM",
    [ACPI_SLOT_TYPE_MAX] = NULL,
};

const char * const WatchdogExpirationAction_lookup[] = {
    [WATCHDOG_EXPIRATION_ACTION_RESET] = "reset",
    [WATCHDOG_EXPIRATION_ACTION_SHUTDOWN] = "shutdown",
    [WATCHDOG_EXPIRATION_ACTION_POWEROFF] = "poweroff",
    [WATCHDOG_EXPIRATION_ACTION_PAUSE] = "pause",
    [WATCHDOG_EXPIRATION_ACTION_DEBUG] = "debug",
    [WATCHDOG_EXPIRATION_ACTION_NONE] = "none",
    [WATCHDOG_EXPIRATION_ACTION_INJECT_NMI] = "inject-nmi",
    [WATCHDOG_EXPIRATION_ACTION_MAX] = NULL,
};

const char * const IoOperationType_lookup[] = {
    [IO_OPERATION_TYPE_READ] = "read",
    [IO_OPERATION_TYPE_WRITE] = "write",
    [IO_OPERATION_TYPE_MAX] = NULL,
};

const char * const GuestPanicAction_lookup[] = {
    [GUEST_PANIC_ACTION_PAUSE] = "pause",
    [GUEST_PANIC_ACTION_MAX] = NULL,
};

const char * const RockerPortDuplex_lookup[] = {
    [ROCKER_PORT_DUPLEX_HALF] = "half",
    [ROCKER_PORT_DUPLEX_FULL] = "full",
    [ROCKER_PORT_DUPLEX_MAX] = NULL,
};

const char * const RockerPortAutoneg_lookup[] = {
    [ROCKER_PORT_AUTONEG_OFF] = "off",
    [ROCKER_PORT_AUTONEG_ON] = "on",
    [ROCKER_PORT_AUTONEG_MAX] = NULL,
};


#ifndef QAPI_TYPES_BUILTIN_CLEANUP_DEF_H
#define QAPI_TYPES_BUILTIN_CLEANUP_DEF_H


void qapi_free_int32List(int32List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int32List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_int16List(int16List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int16List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_intList(intList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_intList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_uint8List(uint8List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_uint8List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_numberList(numberList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_numberList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_int8List(int8List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int8List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_sizeList(sizeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_sizeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_uint64List(uint64List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_uint64List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_uint16List(uint16List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_uint16List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_boolList(boolList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_boolList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_strList(strList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_strList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_int64List(int64List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int64List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qapi_free_uint32List(uint32List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_uint32List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

#endif /* QAPI_TYPES_BUILTIN_CLEANUP_DEF_H */


void qapi_free_ErrorClassList(ErrorClassList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ErrorClassList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VersionTripleList(VersionTripleList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VersionTripleList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VersionTriple(VersionTriple *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VersionTriple(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VersionInfoList(VersionInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VersionInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VersionInfo(VersionInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VersionInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CommandInfoList(CommandInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CommandInfo(CommandInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_OnOffAutoList(OnOffAutoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_OnOffAutoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SnapshotInfoList(SnapshotInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SnapshotInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SnapshotInfo(SnapshotInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SnapshotInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfoSpecificQCow2List(ImageInfoSpecificQCow2List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfoSpecificQCow2List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfoSpecificQCow2(ImageInfoSpecificQCow2 *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfoSpecificQCow2(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfoSpecificVmdkList(ImageInfoSpecificVmdkList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfoSpecificVmdkList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfoSpecificVmdk(ImageInfoSpecificVmdk *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfoSpecificVmdk(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfoSpecificList(ImageInfoSpecificList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfoSpecificList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfoSpecific(ImageInfoSpecific *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfoSpecific(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfoList(ImageInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageInfo(ImageInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageCheckList(ImageCheckList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageCheckList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ImageCheck(ImageCheck *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ImageCheck(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevCacheInfoList(BlockdevCacheInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevCacheInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevCacheInfo(BlockdevCacheInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevCacheInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDeviceInfoList(BlockDeviceInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDeviceInfo(BlockDeviceInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDeviceIoStatusList(BlockDeviceIoStatusList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceIoStatusList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDeviceMapEntryList(BlockDeviceMapEntryList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceMapEntryList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDeviceMapEntry(BlockDeviceMapEntry *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceMapEntry(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DirtyBitmapStatusList(DirtyBitmapStatusList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DirtyBitmapStatusList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDirtyInfoList(BlockDirtyInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDirtyInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDirtyInfo(BlockDirtyInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDirtyInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockInfoList(BlockInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockInfo(BlockInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDeviceStatsList(BlockDeviceStatsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceStatsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDeviceStats(BlockDeviceStats *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceStats(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockStatsList(BlockStatsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockStatsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockStats(BlockStats *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockStats(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOnErrorList(BlockdevOnErrorList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOnErrorList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MirrorSyncModeList(MirrorSyncModeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MirrorSyncModeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockJobTypeList(BlockJobTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockJobTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockJobInfoList(BlockJobInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockJobInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockJobInfo(BlockJobInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockJobInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NewImageModeList(NewImageModeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NewImageModeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevSnapshotList(BlockdevSnapshotList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevSnapshotList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevSnapshot(BlockdevSnapshot *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevSnapshot(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DriveBackupList(DriveBackupList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DriveBackupList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DriveBackup(DriveBackup *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DriveBackup(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevBackupList(BlockdevBackupList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevBackupList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevBackup(BlockdevBackup *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevBackup(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDirtyBitmapList(BlockDirtyBitmapList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDirtyBitmapList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDirtyBitmap(BlockDirtyBitmap *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDirtyBitmap(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDirtyBitmapAddList(BlockDirtyBitmapAddList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDirtyBitmapAddList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockDirtyBitmapAdd(BlockDirtyBitmapAdd *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDirtyBitmapAdd(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevDiscardOptionsList(BlockdevDiscardOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevDiscardOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevDetectZeroesOptionsList(BlockdevDetectZeroesOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevDetectZeroesOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevAioOptionsList(BlockdevAioOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevAioOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevCacheOptionsList(BlockdevCacheOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevCacheOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevCacheOptions(BlockdevCacheOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevCacheOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevDriverList(BlockdevDriverList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevDriverList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsBaseList(BlockdevOptionsBaseList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsBaseList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsBase(BlockdevOptionsBase *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsBase(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsFileList(BlockdevOptionsFileList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsFileList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsFile(BlockdevOptionsFile *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsFile(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsNullList(BlockdevOptionsNullList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsNullList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsNull(BlockdevOptionsNull *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsNull(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsVVFATList(BlockdevOptionsVVFATList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsVVFATList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsVVFAT(BlockdevOptionsVVFAT *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsVVFAT(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsGenericFormatList(BlockdevOptionsGenericFormatList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsGenericFormatList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsGenericFormat(BlockdevOptionsGenericFormat *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsGenericFormat(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsGenericCOWFormatList(BlockdevOptionsGenericCOWFormatList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsGenericCOWFormatList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsGenericCOWFormat(BlockdevOptionsGenericCOWFormat *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsGenericCOWFormat(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Qcow2OverlapCheckModeList(Qcow2OverlapCheckModeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Qcow2OverlapCheckModeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Qcow2OverlapCheckFlagsList(Qcow2OverlapCheckFlagsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Qcow2OverlapCheckFlagsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Qcow2OverlapCheckFlags(Qcow2OverlapCheckFlags *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Qcow2OverlapCheckFlags(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Qcow2OverlapChecksList(Qcow2OverlapChecksList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Qcow2OverlapChecksList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Qcow2OverlapChecks(Qcow2OverlapChecks *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Qcow2OverlapChecks(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsQcow2List(BlockdevOptionsQcow2List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsQcow2List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsQcow2(BlockdevOptionsQcow2 *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsQcow2(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsArchipelagoList(BlockdevOptionsArchipelagoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsArchipelagoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsArchipelago(BlockdevOptionsArchipelago *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsArchipelago(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlkdebugEventList(BlkdebugEventList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlkdebugEventList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlkdebugInjectErrorOptionsList(BlkdebugInjectErrorOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlkdebugInjectErrorOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlkdebugInjectErrorOptions(BlkdebugInjectErrorOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlkdebugInjectErrorOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlkdebugSetStateOptionsList(BlkdebugSetStateOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlkdebugSetStateOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlkdebugSetStateOptions(BlkdebugSetStateOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlkdebugSetStateOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsBlkdebugList(BlockdevOptionsBlkdebugList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsBlkdebugList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsBlkdebug(BlockdevOptionsBlkdebug *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsBlkdebug(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsBlkverifyList(BlockdevOptionsBlkverifyList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsBlkverifyList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsBlkverify(BlockdevOptionsBlkverify *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsBlkverify(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_QuorumReadPatternList(QuorumReadPatternList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_QuorumReadPatternList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsQuorumList(BlockdevOptionsQuorumList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsQuorumList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsQuorum(BlockdevOptionsQuorum *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsQuorum(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptionsList(BlockdevOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevOptions(BlockdevOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevRefList(BlockdevRefList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevRefList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevRef(BlockdevRef *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevRef(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockErrorActionList(BlockErrorActionList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockErrorActionList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PreallocModeList(PreallocModeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PreallocModeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BiosAtaTranslationList(BiosAtaTranslationList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BiosAtaTranslationList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevSnapshotInternalList(BlockdevSnapshotInternalList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevSnapshotInternalList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BlockdevSnapshotInternal(BlockdevSnapshotInternal *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevSnapshotInternal(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TraceEventStateList(TraceEventStateList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TraceEventStateList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TraceEventInfoList(TraceEventInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TraceEventInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TraceEventInfo(TraceEventInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TraceEventInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_LostTickPolicyList(LostTickPolicyList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_LostTickPolicyList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NameInfoList(NameInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NameInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NameInfo(NameInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NameInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_KvmInfoList(KvmInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_KvmInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_KvmInfo(KvmInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_KvmInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RunStateList(RunStateList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RunStateList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_StatusInfoList(StatusInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_StatusInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_StatusInfo(StatusInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_StatusInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_UuidInfoList(UuidInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_UuidInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_UuidInfo(UuidInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_UuidInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevInfoList(ChardevInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevInfo(ChardevInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevBackendInfoList(ChardevBackendInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevBackendInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevBackendInfo(ChardevBackendInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevBackendInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DataFormatList(DataFormatList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DataFormatList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_EventInfoList(EventInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_EventInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_EventInfo(EventInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_EventInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationStatsList(MigrationStatsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationStatsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationStats(MigrationStats *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationStats(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_XBZRLECacheStatsList(XBZRLECacheStatsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_XBZRLECacheStatsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_XBZRLECacheStats(XBZRLECacheStats *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_XBZRLECacheStats(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationStatusList(MigrationStatusList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationStatusList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationInfoList(MigrationInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationInfo(MigrationInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationCapabilityList(MigrationCapabilityList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationCapabilityList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationCapabilityStatusList(MigrationCapabilityStatusList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationCapabilityStatusList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationCapabilityStatus(MigrationCapabilityStatus *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationCapabilityStatus(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationParameterList(MigrationParameterList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationParameterList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationParametersList(MigrationParametersList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationParametersList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MigrationParameters(MigrationParameters *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationParameters(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MouseInfoList(MouseInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MouseInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MouseInfo(MouseInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MouseInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CpuInfoList(CpuInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CpuInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CpuInfo(CpuInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CpuInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_IOThreadInfoList(IOThreadInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_IOThreadInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_IOThreadInfo(IOThreadInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_IOThreadInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetworkAddressFamilyList(NetworkAddressFamilyList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetworkAddressFamilyList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncBasicInfoList(VncBasicInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncBasicInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncBasicInfo(VncBasicInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncBasicInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncServerInfoList(VncServerInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncServerInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncServerInfo(VncServerInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncServerInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncClientInfoList(VncClientInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncClientInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncClientInfo(VncClientInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncClientInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncInfoList(VncInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncInfo(VncInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncPrimaryAuthList(VncPrimaryAuthList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncPrimaryAuthList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncVencryptSubAuthList(VncVencryptSubAuthList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncVencryptSubAuthList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncInfo2List(VncInfo2List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncInfo2List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_VncInfo2(VncInfo2 *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncInfo2(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceBasicInfoList(SpiceBasicInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceBasicInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceBasicInfo(SpiceBasicInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceBasicInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceServerInfoList(SpiceServerInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceServerInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceServerInfo(SpiceServerInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceServerInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceChannelList(SpiceChannelList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceChannelList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceChannel(SpiceChannel *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceChannel(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceQueryMouseModeList(SpiceQueryMouseModeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceQueryMouseModeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceInfoList(SpiceInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SpiceInfo(SpiceInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BalloonInfoList(BalloonInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BalloonInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_BalloonInfo(BalloonInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BalloonInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciMemoryRangeList(PciMemoryRangeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciMemoryRangeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciMemoryRange(PciMemoryRange *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciMemoryRange(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciMemoryRegionList(PciMemoryRegionList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciMemoryRegionList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciMemoryRegion(PciMemoryRegion *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciMemoryRegion(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciBusInfoList(PciBusInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciBusInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciBusInfo(PciBusInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciBusInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciBridgeInfoList(PciBridgeInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciBridgeInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciBridgeInfo(PciBridgeInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciBridgeInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciDeviceClassList(PciDeviceClassList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciDeviceClassList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciDeviceClass(PciDeviceClass *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciDeviceClass(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciDeviceIdList(PciDeviceIdList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciDeviceIdList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciDeviceId(PciDeviceId *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciDeviceId(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciDeviceInfoList(PciDeviceInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciDeviceInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciDeviceInfo(PciDeviceInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciDeviceInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciInfoList(PciInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PciInfo(PciInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_AbortList(AbortList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_AbortList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Abort(Abort *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Abort(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TransactionActionList(TransactionActionList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TransactionActionList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TransactionAction(TransactionAction *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TransactionAction(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ObjectPropertyInfoList(ObjectPropertyInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ObjectPropertyInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ObjectPropertyInfo(ObjectPropertyInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ObjectPropertyInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ObjectTypeInfoList(ObjectTypeInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ObjectTypeInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ObjectTypeInfo(ObjectTypeInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ObjectTypeInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DevicePropertyInfoList(DevicePropertyInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DevicePropertyInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DevicePropertyInfo(DevicePropertyInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DevicePropertyInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DumpGuestMemoryFormatList(DumpGuestMemoryFormatList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DumpGuestMemoryFormatList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DumpGuestMemoryCapabilityList(DumpGuestMemoryCapabilityList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DumpGuestMemoryCapabilityList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_DumpGuestMemoryCapability(DumpGuestMemoryCapability *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DumpGuestMemoryCapability(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevNoneOptionsList(NetdevNoneOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevNoneOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevNoneOptions(NetdevNoneOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevNoneOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetLegacyNicOptionsList(NetLegacyNicOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetLegacyNicOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetLegacyNicOptions(NetLegacyNicOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetLegacyNicOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_StringList(StringList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_StringList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_String(String *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_String(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevUserOptionsList(NetdevUserOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevUserOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevUserOptions(NetdevUserOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevUserOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevTapOptionsList(NetdevTapOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevTapOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevTapOptions(NetdevTapOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevTapOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevSocketOptionsList(NetdevSocketOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevSocketOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevSocketOptions(NetdevSocketOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevSocketOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevL2TPv3OptionsList(NetdevL2TPv3OptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevL2TPv3OptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevL2TPv3Options(NetdevL2TPv3Options *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevL2TPv3Options(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevVdeOptionsList(NetdevVdeOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevVdeOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevVdeOptions(NetdevVdeOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevVdeOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevDumpOptionsList(NetdevDumpOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevDumpOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevDumpOptions(NetdevDumpOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevDumpOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevBridgeOptionsList(NetdevBridgeOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevBridgeOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevBridgeOptions(NetdevBridgeOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevBridgeOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevHubPortOptionsList(NetdevHubPortOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevHubPortOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevHubPortOptions(NetdevHubPortOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevHubPortOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevNetmapOptionsList(NetdevNetmapOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevNetmapOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevNetmapOptions(NetdevNetmapOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevNetmapOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevVhostUserOptionsList(NetdevVhostUserOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevVhostUserOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevVhostUserOptions(NetdevVhostUserOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevVhostUserOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetClientOptionsList(NetClientOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetClientOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetClientOptions(NetClientOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetClientOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetLegacyList(NetLegacyList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetLegacyList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetLegacy(NetLegacy *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetLegacy(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NetdevList(NetdevList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NetdevList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Netdev(Netdev *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Netdev(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InetSocketAddressList(InetSocketAddressList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InetSocketAddressList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InetSocketAddress(InetSocketAddress *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InetSocketAddress(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_UnixSocketAddressList(UnixSocketAddressList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_UnixSocketAddressList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_UnixSocketAddress(UnixSocketAddress *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_UnixSocketAddress(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SocketAddressList(SocketAddressList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SocketAddressList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_SocketAddress(SocketAddress *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SocketAddress(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MachineInfoList(MachineInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MachineInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MachineInfo(MachineInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MachineInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CpuDefinitionInfoList(CpuDefinitionInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CpuDefinitionInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CpuDefinitionInfo(CpuDefinitionInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CpuDefinitionInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_AddfdInfoList(AddfdInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_AddfdInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_AddfdInfo(AddfdInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_AddfdInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_FdsetFdInfoList(FdsetFdInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_FdsetFdInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_FdsetFdInfo(FdsetFdInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_FdsetFdInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_FdsetInfoList(FdsetInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_FdsetInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_FdsetInfo(FdsetInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_FdsetInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TargetInfoList(TargetInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TargetInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TargetInfo(TargetInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TargetInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_QKeyCodeList(QKeyCodeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_QKeyCodeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_KeyValueList(KeyValueList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_KeyValueList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_KeyValue(KeyValue *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_KeyValue(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevFileList(ChardevFileList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevFileList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevFile(ChardevFile *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevFile(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevHostdevList(ChardevHostdevList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevHostdevList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevHostdev(ChardevHostdev *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevHostdev(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevSocketList(ChardevSocketList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevSocketList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevSocket(ChardevSocket *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevSocket(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevUdpList(ChardevUdpList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevUdpList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevUdp(ChardevUdp *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevUdp(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevMuxList(ChardevMuxList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevMuxList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevMux(ChardevMux *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevMux(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevStdioList(ChardevStdioList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevStdioList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevStdio(ChardevStdio *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevStdio(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevSpiceChannelList(ChardevSpiceChannelList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevSpiceChannelList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevSpiceChannel(ChardevSpiceChannel *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevSpiceChannel(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevSpicePortList(ChardevSpicePortList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevSpicePortList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevSpicePort(ChardevSpicePort *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevSpicePort(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevVCList(ChardevVCList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevVCList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevVC(ChardevVC *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevVC(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevRingbufList(ChardevRingbufList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevRingbufList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevRingbuf(ChardevRingbuf *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevRingbuf(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevDummyList(ChardevDummyList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevDummyList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevDummy(ChardevDummy *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevDummy(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevBackendList(ChardevBackendList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevBackendList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevBackend(ChardevBackend *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevBackend(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevReturnList(ChardevReturnList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevReturnList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ChardevReturn(ChardevReturn *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevReturn(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TpmModelList(TpmModelList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TpmModelList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TpmTypeList(TpmTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TpmTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TPMPassthroughOptionsList(TPMPassthroughOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TPMPassthroughOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TPMPassthroughOptions(TPMPassthroughOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TPMPassthroughOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TpmTypeOptionsList(TpmTypeOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TpmTypeOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TpmTypeOptions(TpmTypeOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TpmTypeOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TPMInfoList(TPMInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TPMInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_TPMInfo(TPMInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TPMInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_AcpiTableOptionsList(AcpiTableOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_AcpiTableOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_AcpiTableOptions(AcpiTableOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_AcpiTableOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CommandLineParameterTypeList(CommandLineParameterTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandLineParameterTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CommandLineParameterInfoList(CommandLineParameterInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandLineParameterInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CommandLineParameterInfo(CommandLineParameterInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandLineParameterInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CommandLineOptionInfoList(CommandLineOptionInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandLineOptionInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_CommandLineOptionInfo(CommandLineOptionInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandLineOptionInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_X86CPURegister32List(X86CPURegister32List *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_X86CPURegister32List(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_X86CPUFeatureWordInfoList(X86CPUFeatureWordInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_X86CPUFeatureWordInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_X86CPUFeatureWordInfo(X86CPUFeatureWordInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_X86CPUFeatureWordInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RxStateList(RxStateList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RxStateList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RxFilterInfoList(RxFilterInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RxFilterInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RxFilterInfo(RxFilterInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RxFilterInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputButtonList(InputButtonList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputButtonList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputAxisList(InputAxisList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputAxisList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputKeyEventList(InputKeyEventList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputKeyEventList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputKeyEvent(InputKeyEvent *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputKeyEvent(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputBtnEventList(InputBtnEventList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputBtnEventList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputBtnEvent(InputBtnEvent *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputBtnEvent(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputMoveEventList(InputMoveEventList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputMoveEventList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputMoveEvent(InputMoveEvent *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputMoveEvent(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputEventList(InputEventList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputEventList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_InputEvent(InputEvent *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_InputEvent(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NumaOptionsList(NumaOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NumaOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NumaOptions(NumaOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NumaOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NumaNodeOptionsList(NumaNodeOptionsList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NumaNodeOptionsList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_NumaNodeOptions(NumaNodeOptions *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NumaNodeOptions(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_HostMemPolicyList(HostMemPolicyList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_HostMemPolicyList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MemdevList(MemdevList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MemdevList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_Memdev(Memdev *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_Memdev(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PCDIMMDeviceInfoList(PCDIMMDeviceInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PCDIMMDeviceInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_PCDIMMDeviceInfo(PCDIMMDeviceInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PCDIMMDeviceInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MemoryDeviceInfoList(MemoryDeviceInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MemoryDeviceInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_MemoryDeviceInfo(MemoryDeviceInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MemoryDeviceInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ACPISlotTypeList(ACPISlotTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ACPISlotTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ACPIOSTInfoList(ACPIOSTInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ACPIOSTInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_ACPIOSTInfo(ACPIOSTInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ACPIOSTInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_WatchdogExpirationActionList(WatchdogExpirationActionList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_WatchdogExpirationActionList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_IoOperationTypeList(IoOperationTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_IoOperationTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestPanicActionList(GuestPanicActionList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestPanicActionList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerSwitchList(RockerSwitchList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerSwitchList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerSwitch(RockerSwitch *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerSwitch(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerPortDuplexList(RockerPortDuplexList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerPortDuplexList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerPortAutonegList(RockerPortAutonegList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerPortAutonegList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerPortList(RockerPortList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerPortList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerPort(RockerPort *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerPort(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlowKeyList(RockerOfDpaFlowKeyList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowKeyList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlowKey(RockerOfDpaFlowKey *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowKey(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlowMaskList(RockerOfDpaFlowMaskList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowMaskList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlowMask(RockerOfDpaFlowMask *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowMask(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlowActionList(RockerOfDpaFlowActionList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowActionList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlowAction(RockerOfDpaFlowAction *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowAction(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlowList(RockerOfDpaFlowList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaFlow(RockerOfDpaFlow *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlow(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaGroupList(RockerOfDpaGroupList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaGroupList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_RockerOfDpaGroup(RockerOfDpaGroup *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaGroup(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

