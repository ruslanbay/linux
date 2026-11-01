/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2026 Intel Corporation */

#ifndef IPU4P_FW_ISYS_H
#define IPU4P_FW_ISYS_H

#include <linux/build_bug.h>
#include <linux/stddef.h>
#include <linux/types.h>

#include "ipu6-fw-isys.h"

#define IPU4P_ISYS_MAX_STREAMS			8U
#define IPU4P_ISYS_CSI2_NPORTS			8U
#define IPU4P_MAX_IPINS				4U
#define IPU4P_MAX_OPINS				6U
#define IPU4P_NOF_SRAM_BLOCKS_MAX		8U
#define IPU4P_DEVICE_GDA_NR_PAGES		128U
#define IPU4P_DEVICE_GDA_VIRT_FACTOR		32U
#define IPU4P_N_MAX_PROXY_SEND_QUEUES		1U
#define IPU4P_N_MAX_DEV_SEND_QUEUES		1U
#define IPU4P_N_MAX_PROXY_RECV_QUEUES		1U
#define IPU4P_DEV_SEND_QUEUE_SIZE		8U
#define IPU4P_ISYS_SIZE_RECV_QUEUE		40U
#define IPU4P_ISYS_SIZE_SEND_QUEUE		40U
#define IPU4P_ISYS_SIZE_PROXY_RECV_QUEUE	5U
#define IPU4P_ISYS_SIZE_PROXY_SEND_QUEUE	5U
#define IPU4P_ISYS_OPEN_RETRY			2000
#define IPU4P_ISYS_CLOSE_RETRY			2000

#define IPU4P_BASE_PROXY_SEND_QUEUES		0U
#define IPU4P_BASE_DEV_SEND_QUEUES		1U
#define IPU4P_BASE_MSG_SEND_QUEUES		2U
#define IPU4P_BASE_PROXY_RECV_QUEUES		0U
#define IPU4P_BASE_MSG_RECV_QUEUES		1U
#define IPU4P_N_MAX_RECV_QUEUES			2U
#define IPU4P_N_MAX_SEND_QUEUES \
	(IPU4P_BASE_MSG_SEND_QUEUES + IPU4P_ISYS_MAX_STREAMS)

#define IPU4P_ISYS_CROPPING_LOCATION_MAX	4U
#define IPU4P_ISYS_RESOLUTION_INFO_MAX		2U

#define IPU4P_FW_ISYS_USE_NO_ISL_NO_ISA		0
#define IPU4P_FW_ISYS_LINK_OFFLINE		0

struct ipu4p_fw_isys_resolution {
	u32 width;
	u32 height;
};

struct ipu4p_fw_isys_isa_cfg {
	struct ipu4p_fw_isys_resolution
		isa_res[IPU4P_ISYS_RESOLUTION_INFO_MAX];
	u32 cfg_fields;
};

struct ipu4p_fw_isys_cropping {
	s32 top_offset;
	s32 left_offset;
	s32 bottom_offset;
	s32 right_offset;
};

struct ipu4p_fw_isys_input_pin_info {
	struct ipu4p_fw_isys_resolution input_res;
	u8 dt;
	u8 mipi_store_mode;
	u8 bits_per_pix;
	u8 mapped_dt;
};

/**
 * struct ipu4p_fw_isys_output_pin_info - IPU4P output pin configuration
 *
 * Unlike the IPU6 ABI, IPU4P ends this structure at reserve_compression;
 * snoopable and sensor_type are not part of the IPU4P firmware wire format.
 *
 * @link_id: PPG link; zero selects offline capture
 */
struct ipu4p_fw_isys_output_pin_info {
	struct ipu4p_fw_isys_resolution output_res;
	u32 stride;
	u32 watermark_in_lines;
	u32 payload_buf_size;
	u8 send_irq;
	u8 input_pin_id;
	u8 pt;
	u8 ft;
	u8 link_id;
	u8 reserve_compression;
};

struct ipu4p_fw_isys_stream_cfg {
	/* Preserve the firmware ABI slot; ISA is disabled by this driver. */
	struct ipu4p_fw_isys_isa_cfg isa_cfg;
	struct ipu4p_fw_isys_cropping crop[IPU4P_ISYS_CROPPING_LOCATION_MAX];
	struct ipu4p_fw_isys_input_pin_info input_pins[IPU4P_MAX_IPINS];
	struct ipu4p_fw_isys_output_pin_info output_pins[IPU4P_MAX_OPINS];
	u32 compfmt;
	u8 nof_input_pins;
	u8 nof_output_pins;
	u8 send_irq_sof_discarded;
	u8 send_irq_eof_discarded;
	u8 send_resp_sof_discarded;
	u8 send_resp_eof_discarded;
	u8 src;
	u8 vc;
	u8 isl_use;
};

struct ipu4p_fw_isys_output_pin_payload {
	u64 out_buf_id;
	u32 addr;
	u32 compress;
};

struct ipu4p_fw_isys_param_pin {
	u64 param_buf_id;
	u32 addr;
};

struct ipu4p_fw_isys_frame_buff_set {
	struct ipu4p_fw_isys_output_pin_payload
		output_pins[IPU4P_MAX_OPINS];
	struct ipu4p_fw_isys_param_pin process_group_light;
	u8 send_irq_sof;
	u8 send_irq_eof;
	u8 send_irq_capture_ack;
	u8 send_irq_capture_done;
	u8 send_resp_sof;
	u8 send_resp_eof;
	u8 frame_counter;
};

struct ipu4p_fw_isys_error_info {
	u32 error;
	u32 error_details;
};

struct ipu4p_fw_isys_resp_info {
	u64 buf_id;
	struct ipu4p_fw_isys_output_pin_payload pin;
	struct ipu4p_fw_isys_param_pin process_group_light;
	struct ipu4p_fw_isys_error_info error_info;
	u32 timestamp[2];
	u8 stream_handle;
	u8 type;
	u8 pin_id;
	u8 acc_id;
	u8 frame_counter;
	u8 written_direct;
};

struct ipu4p_fw_isys_proxy_error_info {
	u32 error;
	u32 error_details;
};

struct ipu4p_fw_isys_proxy_resp_info {
	u32 request_id;
	struct ipu4p_fw_isys_proxy_error_info error_info;
};

struct ipu4p_fw_isys_send_token {
	u64 buf_handle;
	u32 payload;
	u16 send_type;
	u16 stream_id;
};

struct ipu4p_fw_isys_recv_token {
	struct ipu4p_fw_isys_resp_info resp_info;
};

struct ipu4p_fw_isys_proxy_send_token {
	u32 request_id;
	u32 region_index;
	u32 offset;
	u32 value;
};

struct ipu4p_fw_isys_proxy_recv_token {
	struct ipu4p_fw_isys_proxy_resp_info proxy_resp_info;
};

struct ipu4p_fw_isys_buffer_partition {
	u32 num_gda_pages[IPU4P_ISYS_MAX_STREAMS];
};

struct ipu4p_fw_isys_fw_config {
	struct ipu4p_fw_isys_buffer_partition buffer_partition;
	u32 num_send_queues[N_IPU6_FW_ISYS_QUEUE_TYPE];
	u32 num_recv_queues[N_IPU6_FW_ISYS_QUEUE_TYPE];
};

/* Keep these assertions in sync with the IPU4P firmware ABI. */
static_assert(sizeof(struct ipu4p_fw_isys_output_pin_payload) == 16);
static_assert(offsetof(struct ipu4p_fw_isys_output_pin_payload, addr) == 8);
static_assert(sizeof(struct ipu4p_fw_isys_send_token) == 16);
static_assert(offsetof(struct ipu4p_fw_isys_send_token, payload) == 8);
static_assert(sizeof(struct ipu4p_fw_isys_output_pin_info) == 28);
static_assert(offsetof(struct ipu4p_fw_isys_output_pin_info, link_id) == 24);
static_assert(offsetof(struct ipu4p_fw_isys_output_pin_info,
		       reserve_compression) == 25);
static_assert(sizeof(struct ipu4p_fw_isys_stream_cfg) == 316);
static_assert(offsetof(struct ipu4p_fw_isys_stream_cfg, output_pins) == 132);
static_assert(offsetof(struct ipu4p_fw_isys_stream_cfg, compfmt) == 300);
static_assert(offsetof(struct ipu4p_fw_isys_stream_cfg, src) == 310);
static_assert(offsetof(struct ipu4p_fw_isys_frame_buff_set,
		       process_group_light) == 96);
static_assert(offsetof(struct ipu4p_fw_isys_resp_info,
		       process_group_light) == 24);

#endif /* IPU4P_FW_ISYS_H */
