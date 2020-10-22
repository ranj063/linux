/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2020 Intel Corporation. All rights reserved.
 */

/*
 * Extended manifest is a place to store metadata about firmware, known during
 * compilation time - for example firmware version or used compiler.
 * Given information are read on host side before firmware startup.
 * This part of output binary is not signed.
 */

#ifndef __CAVS_FIRMWARE_EXT_MANIFEST_H__
#define __CAVS_FIRMWARE_EXT_MANIFEST_H__

/* In ASCII  $AE1 */
#define CAVS_EXT_MAN_MAGIC_NUMBER    0x31454124

#define MAX_MODULE_NAME_LEN     8
#define MAX_FW_BINARY_NAME      8
#define DEFAULT_HASH_SHA256_LEN 32
#define CAVS18_FW_HDR_OFFSET	0x2000

struct cavs_ext_manifest_hdr {
	u32 id;
	u32 len;
	u16 version_major;
	u16 version_minor;
	u32 entries;
} __packed;

struct FwImgFlags {
	uint32_t    _rsvd0 : 31;
	uint32_t    tp : 1;
} __packed;

struct CavsFwBinaryHeader {
	// This part must be unchanged to be backward compatible with SPT-LP ROM
	uint32_t    id;
	uint32_t    len;                 // sizeof(CavsFwBinaryHeader) in bytes
	uint8_t     name[MAX_FW_BINARY_NAME];
	uint32_t    preload_page_count;         // number of pages of preloaded image
	struct FwImgFlags    fw_image_flags;
	uint32_t    feature_mask;
	uint16_t    major_version;
	uint16_t    minor_version;
	uint16_t    hotfix_version;
	uint16_t    build_version;
	uint32_t    num_module_entries;

	// This part may change to contain any additional data for BaseFw
	// that is skipped by ROM
	uint32_t    hw_buf_base_addr;
	uint32_t    hw_buf_length;
	uint32_t    load_offset; // This value is used by ROM
} __packed;

struct ModuleType {
	uint32_t     load_type : 4; // MT_BUILTIN, MT_LOADABLE
	uint32_t     auto_start : 1; // 0 - manually created, 1 - created by Module Manager
	uint32_t     domain_ll : 1; // support LL domain
	uint32_t     domain_dp : 1; // support DP domain
	uint32_t     lib_code : 1; // determines if module is place holder for common library code
	uint32_t     _rsvd : 24;
} __packed;

struct SegmentFlags {
	uint32_t    contents : 1;
	uint32_t    alloc : 1;
	uint32_t    load : 1;
	uint32_t    readonly : 1;
	uint32_t    code : 1;
	uint32_t    data : 1;
	uint32_t    _rsvd0 : 2;

	uint32_t    type : 4;
	uint32_t    _rsvd1 : 4;

	uint32_t    length : 16; // segment length in pages
} __packed;

struct SegmentDesc {
	struct SegmentFlags    flags;
	uint32_t        v_base_addr;
	uint32_t        file_offset;
} __packed;

struct ModuleEntry {
	uint32_t    id;
	uint8_t     name[MAX_MODULE_NAME_LEN];
	uint8_t     uuid[16];
	struct ModuleType  type;
	uint8_t     hash[DEFAULT_HASH_SHA256_LEN];
	uint32_t    entry_point;
	uint16_t    cfg_offset;
	uint16_t    cfg_count;
	uint32_t    affinity_mask;
	uint16_t    instance_max_count;
	uint16_t    instance_stack_size;
	struct SegmentDesc segments[3];
} __packed;

struct ModuleConfig {
	uint32_t    par[4];             // module parameters
	uint32_t    is_bytes;           // actual size of instance .bss (bytes)
	uint32_t    cps;                // cycles per second
	uint32_t    ibs;                // input buffer size (bytes)
	uint32_t    obs;                // output buffer size (bytes)
	uint32_t    module_flags;       // flags, res for future use
	uint32_t    cpc;                // cycles per single run
	uint32_t    obls;               // output block size, res for future use
} __packed;

struct CavsFwBinaryDesc {
	struct CavsFwBinaryHeader  header;
	struct ModuleEntry         module_entries[0];
	struct ModuleConfig        module_config[0];
} __packed;
#endif /* __CAVS_FIRMWARE_EXT_MANIFEST_H__ */
