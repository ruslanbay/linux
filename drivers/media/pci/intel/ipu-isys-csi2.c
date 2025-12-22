// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2018 Intel Corporation

#include <linux/device.h>
#include <linux/module.h>

#include <media/ipu-isys.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#include "ipu.h"
#include "ipu-bus.h"
#include "ipu-buttress.h"
#include "ipu-isys.h"
#include "ipu-isys-subdev.h"
#include "ipu-isys-video.h"
#include "ipu-platform-regs.h"

#define CREATE_TRACE_POINTS
#define IPU_SOF_SEQID_TRACE
#define IPU_EOF_SEQID_TRACE
#include "ipu-trace-event.h"

static const u32 csi2_supported_codes_pad_sink[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_YUYV10_1X20,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8,
	MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8,
	MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
	MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR14_1X14,
	MEDIA_BUS_FMT_SGBRG14_1X14,
	MEDIA_BUS_FMT_SGRBG14_1X14,
	MEDIA_BUS_FMT_SRGGB14_1X14,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	0,
};

static const u32 csi2_supported_codes_pad_source[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_YUYV10_1X20,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR14_1X14,
	MEDIA_BUS_FMT_SGBRG14_1X14,
	MEDIA_BUS_FMT_SGRBG14_1X14,
	MEDIA_BUS_FMT_SRGGB14_1X14,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	0,
};

static const u32 csi2_supported_codes_pad_meta[] = {
	MEDIA_BUS_FMT_FIXED,
	0,
};

static const u32 *csi2_supported_codes[NR_OF_CSI2_PADS];

static struct v4l2_subdev_internal_ops csi2_sd_internal_ops = {
	.open = ipu_isys_subdev_open,
	.close = ipu_isys_subdev_close,
};

int ipu_isys_csi2_get_link_freq(struct ipu_isys_csi2 *csi2, s64 *link_freq)
{
	struct media_pipeline *mp = media_entity_pipeline(&csi2->asd.sd.entity);
	struct ipu_isys_pipeline *pipe = container_of(mp,
						      struct ipu_isys_pipeline,
						      pipe);
	struct v4l2_subdev *ext_sd =
		media_entity_to_v4l2_subdev(pipe->external->entity);
	struct device *dev = &csi2->isys->adev->dev;
	unsigned int bpp, lanes;
	s64 ret;

	if (!ext_sd) {
		WARN_ON(1);
		return -ENODEV;
	}

	bpp = ipu_isys_mbus_code_to_bpp(csi2->asd.ffmt->code);
	lanes = csi2->nlanes;

	ret = v4l2_get_link_freq(ext_sd->ctrl_handler, bpp, lanes * 2);
	if (ret < 0) {
		dev_err(dev, "can't get link frequency (%lld)\n", ret);
		return ret;
	}

	dev_dbg(dev, "link freq of %s is %lld\n", ext_sd->name, ret);
	*link_freq = ret;

	return 0;
}

static int ipu_get_frame_desc_entry_by_dt(struct v4l2_subdev *sd,
					  struct v4l2_mbus_frame_desc_entry
					  *entry, u8 data_type)
{
	struct v4l2_mbus_frame_desc desc = {
		.num_entries = V4L2_FRAME_DESC_ENTRY_MAX,
	};
	int rval, i;

	rval = v4l2_subdev_call(sd, pad, get_frame_desc, 0, &desc);
	if (rval)
		return rval;

	for (i = 0; i < desc.num_entries; i++) {
		if (desc.entry[i].bus.csi2.dt != data_type)
			continue;
		*entry = desc.entry[i];
		return 0;
	}

	return -EINVAL;
}

static int subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
			   struct v4l2_event_subscription *sub)
{
	struct ipu_isys_csi2 *csi2 = to_ipu_isys_csi2(sd);

	dev_dbg(&csi2->isys->adev->dev, "subscribe event(type %u id %u)\n",
		sub->type, sub->id);

	switch (sub->type) {
	case V4L2_EVENT_FRAME_SYNC:
		return v4l2_event_subscribe(fh, sub, 10, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_core_ops csi2_sd_core_ops = {
	.subscribe_event = subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

/*
 * The input system CSI2+ receiver has several
 * parameters affecting the receiver timings. These depend
 * on the MIPI bus frequency F in Hz (sensor transmitter rate)
 * as follows:
 *	register value = (A/1e9 + B * UI) / COUNT_ACC
 * where
 *	UI = 1 / (2 * F) in seconds
 *	COUNT_ACC = counter accuracy in seconds
 *	For IPU4,  COUNT_ACC = 0.125 ns
 *
 * A and B are coefficients from the table below,
 * depending whether the register minimum or maximum value is
 * calculated.
 *				       Minimum     Maximum
 * Clock lane			       A     B     A     B
 * reg_rx_csi_dly_cnt_termen_clane     0     0    38     0
 * reg_rx_csi_dly_cnt_settle_clane    95    -8   300   -16
 * Data lanes
 * reg_rx_csi_dly_cnt_termen_dlane0    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane0   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane1    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane1   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane2    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane2   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane3    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane3   85    -2   145    -6
 *
 * We use the minimum values of both A and B.
 */

#define DIV_SHIFT	8

static uint32_t calc_timing(s32 a, int32_t b, int64_t link_freq, int32_t accinv)
{
	return accinv * a + (accinv * b * (500000000 >> DIV_SHIFT)
			     / (int32_t)(link_freq >> DIV_SHIFT));
}

static int
ipu_isys_csi2_calc_timing(struct ipu_isys_csi2 *csi2,
			  struct ipu_isys_csi2_timing *timing, uint32_t accinv)
{
	__s64 link_freq;
	int rval;

	rval = ipu_isys_csi2_get_link_freq(csi2, &link_freq);
	if (rval)
		return rval;

	timing->ctermen = calc_timing(CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_A,
				      CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_B,
				      link_freq, accinv);
	timing->csettle = calc_timing(CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_A,
				      CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_B,
				      link_freq, accinv);
	dev_dbg(&csi2->isys->adev->dev, "ctermen %u\n", timing->ctermen);
	dev_dbg(&csi2->isys->adev->dev, "csettle %u\n", timing->csettle);

	timing->dtermen = calc_timing(CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_A,
				      CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_B,
				      link_freq, accinv);
	timing->dsettle = calc_timing(CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_A,
				      CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_B,
				      link_freq, accinv);
	dev_dbg(&csi2->isys->adev->dev, "dtermen %u\n", timing->dtermen);
	dev_dbg(&csi2->isys->adev->dev, "dsettle %u\n", timing->dsettle);

	return 0;
}

#define CSI2_ACCINV	8

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ipu_isys_csi2 *csi2 = to_ipu_isys_csi2(sd);
	struct media_pipeline *mp = media_entity_pipeline(&sd->entity);
	struct ipu_isys_pipeline *ip = container_of(mp,
						    struct ipu_isys_pipeline,
						    pipe);
	struct ipu_isys_csi2_config *cfg;
	struct v4l2_subdev *ext_sd;
	struct ipu_isys_csi2_timing timing = {0};
	unsigned int nlanes;
	int rval;

	dev_dbg(&csi2->isys->adev->dev, "csi2 s_stream %d for entity %s\n",
		enable, sd->entity.name);

	if (!ip->external->entity) {
		WARN_ON(1);
		return -ENODEV;
	}
	ext_sd = media_entity_to_v4l2_subdev(ip->external->entity);
	cfg = v4l2_get_subdev_hostdata(ext_sd);

	if (!enable) {
		ipu_isys_csi2_set_stream(sd, timing, 0, enable);
		return 0;
	}

	ip->has_sof = true;

	nlanes = cfg->nlanes;

	dev_dbg(&csi2->isys->adev->dev, "lane nr %d.\n", nlanes);

	rval = ipu_isys_csi2_calc_timing(csi2, &timing, CSI2_ACCINV);
	if (rval)
		return rval;

	rval = ipu_isys_csi2_set_stream(sd, timing, nlanes, enable);

	return rval;
}

static bool csi2_has_route(struct media_entity *entity, unsigned int pad0,
			   unsigned int pad1, int *stream)
{
	if (pad0 == CSI2_PAD_META || pad1 == CSI2_PAD_META)
		return true;
	return ipu_isys_subdev_has_route(entity, pad0, pad1, stream);
}

static const struct v4l2_subdev_video_ops csi2_sd_video_ops = {
	.s_stream = set_stream,
};

static int get_metadata_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *fmt)
{
	struct media_pad *pad =
	    media_pad_remote_pad_first(&sd->entity.pads[CSI2_PAD_SINK]);
	struct v4l2_mbus_frame_desc_entry entry;
	int rval;

	if (!pad)
		return -EINVAL;

	rval =
	    ipu_get_frame_desc_entry_by_dt(media_entity_to_v4l2_subdev
					   (pad->entity), &entry,
					   IPU_ISYS_MIPI_CSI2_TYPE_EMBEDDED8);

	if (!rval) {
		fmt->format.width =
		    entry.two_dim.width * entry.bpp / BITS_PER_BYTE;
		fmt->format.height = entry.two_dim.height;
		fmt->format.code = entry.pixelcode;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	return rval;
}

static int ipu_isys_csi2_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	if (fmt->pad == CSI2_PAD_META)
		return get_metadata_fmt(sd, state, fmt);
	return ipu_isys_subdev_get_ffmt(sd, state, fmt);
}

static int ipu_isys_csi2_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	if (fmt->pad == CSI2_PAD_META)
		return get_metadata_fmt(sd, state, fmt);
	return ipu_isys_subdev_set_ffmt(sd, state, fmt);
}

static int __subdev_link_validate(struct v4l2_subdev *sd,
				  struct media_link *link,
				  struct v4l2_subdev_format *source_fmt,
				  struct v4l2_subdev_format *sink_fmt)
{
	struct ipu_isys_pipeline *ip = container_of(media_entity_pipeline(&sd->entity),
						    struct ipu_isys_pipeline,
						    pipe);

	if (source_fmt->format.field == V4L2_FIELD_ALTERNATE)
		ip->interlaced = true;

	return ipu_isys_subdev_link_validate(sd, link, source_fmt, sink_fmt);
}

static const struct v4l2_subdev_pad_ops csi2_sd_pad_ops = {
	.link_validate = __subdev_link_validate,
	.get_fmt = ipu_isys_csi2_get_fmt,
	.set_fmt = ipu_isys_csi2_set_fmt,
	.enum_mbus_code = ipu_isys_subdev_enum_mbus_code,
	.set_routing = ipu_isys_subdev_set_routing,
	.get_routing = ipu_isys_subdev_get_routing,
};

static struct v4l2_subdev_ops csi2_sd_ops = {
	.core = &csi2_sd_core_ops,
	.video = &csi2_sd_video_ops,
	.pad = &csi2_sd_pad_ops,
};

static struct media_entity_operations csi2_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.has_route = csi2_has_route,
};

static void csi2_set_ffmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *ffmt =
		__ipu_isys_get_ffmt(sd, state, fmt->pad,
				    fmt->which);

	if (fmt->format.field != V4L2_FIELD_ALTERNATE)
		fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->pad == CSI2_PAD_SINK) {
		*ffmt = fmt->format;
		if (fmt->stream)
			return;
		ipu_isys_subdev_fmt_propagate(
			sd, state, &fmt->format, NULL,
			IPU_ISYS_SUBDEV_PROP_TGT_SINK_FMT,
			fmt->pad, fmt->which);
		return;
	}

	if (fmt->pad == CSI2_PAD_META) {
		struct v4l2_mbus_framefmt *ffmt =
			__ipu_isys_get_ffmt(sd, state, fmt->pad,
					    fmt->which);
		struct media_pad *pad = media_pad_remote_pad_first(
			&sd->entity.pads[CSI2_PAD_SINK]);
		struct v4l2_mbus_frame_desc_entry entry;
		int rval;

		if (!pad) {
			ffmt->width = 0;
			ffmt->height = 0;
			ffmt->code = 0;
			return;
		}

		rval = ipu_get_frame_desc_entry_by_dt(
				media_entity_to_v4l2_subdev(pad->entity),
				&entry,
				IPU_ISYS_MIPI_CSI2_TYPE_EMBEDDED8);

		if (!rval) {
			ffmt->width = entry.two_dim.width * entry.bpp
			    / BITS_PER_BYTE;
			ffmt->height = entry.two_dim.height;
			ffmt->code = entry.pixelcode;
			ffmt->field = V4L2_FIELD_NONE;
		}

		return;
	}
	if (sd->entity.pads[fmt->pad].flags & MEDIA_PAD_FL_SOURCE) {
		ffmt->width = fmt->format.width;
		ffmt->height = fmt->format.height;
		ffmt->field = fmt->format.field;
		ffmt->code =
		    ipu_isys_subdev_code_to_uncompressed(fmt->format.code);
		return;
	}

	WARN_ON(1);
}

void ipu_isys_csi2_cleanup(struct ipu_isys_csi2 *csi2)
{
	if (!csi2->isys)
		return;

	v4l2_device_unregister_subdev(&csi2->asd.sd);
	ipu_isys_subdev_cleanup(&csi2->asd);
	csi2->isys = NULL;
}

static void csi_ctrl_init(struct v4l2_subdev *sd)
{
	struct ipu_isys_csi2 *csi2 = to_ipu_isys_csi2(sd);

	static const struct v4l2_ctrl_config cfg = {
		.id = V4L2_CID_IPU_STORE_CSI2_HEADER,
		.name = "Store CSI-2 Headers",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
	};

	csi2->store_csi2_header = v4l2_ctrl_new_custom(&csi2->asd.ctrl_handler,
						       &cfg, NULL);
}

int ipu_isys_csi2_init(struct ipu_isys_csi2 *csi2,
		       struct ipu_isys *isys,
		       void __iomem *base, unsigned int index)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = CSI2_PAD_SINK,
		.format = {
			   .width = 4096,
			   .height = 3072,
			  },
	};
	int i, rval, src;

	dev_dbg(&isys->adev->dev, "csi-%d base = 0x%lx\n", index,
		(unsigned long)base);
	csi2->isys = isys;
	csi2->base = base;
	csi2->index = index;

	csi2->asd.sd.entity.ops = &csi2_entity_ops;
	csi2->asd.ctrl_init = csi_ctrl_init;
	csi2->asd.isys = isys;
	init_completion(&csi2->eof_completion);
	rval = ipu_isys_subdev_init(&csi2->asd, &csi2_sd_ops, 0,
				    NR_OF_CSI2_PADS,
				    NR_OF_CSI2_SOURCE_PADS,
				    NR_OF_CSI2_SINK_PADS, 0,
				    CSI2_PAD_SOURCE,
				    CSI2_PAD_SINK);
	if (rval)
		goto fail;

	csi2->asd.pad[CSI2_PAD_SINK].flags |= MEDIA_PAD_FL_MUST_CONNECT;

	src = index;
	csi2->asd.source = IPU_FW_ISYS_STREAM_SRC_CSI2_PORT0 + src;
	csi2_supported_codes[CSI2_PAD_SINK] = csi2_supported_codes_pad_sink;

	for (i = 0; i < NR_OF_CSI2_SOURCE_PADS; i++)
		csi2_supported_codes[i + 1] = csi2_supported_codes_pad_source;
	csi2->asd.supported_codes = csi2_supported_codes;
	csi2->asd.set_ffmt = csi2_set_ffmt;

	csi2->asd.sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	csi2->asd.sd.internal_ops = &csi2_sd_internal_ops;
	snprintf(csi2->asd.sd.name, sizeof(csi2->asd.sd.name),
		 IPU_ISYS_ENTITY_PREFIX " CSI-2 %u", index);
	v4l2_set_subdevdata(&csi2->asd.sd, &csi2->asd);

	rval = v4l2_device_register_subdev(&isys->v4l2_dev, &csi2->asd.sd);
	if (rval) {
		dev_info(&isys->adev->dev, "can't register v4l2 subdev\n");
		goto fail;
	}

	mutex_lock(&csi2->asd.mutex);
	__ipu_isys_subdev_set_ffmt(&csi2->asd.sd, NULL, &fmt);
	mutex_unlock(&csi2->asd.mutex);

	return 0;

fail:
	ipu_isys_csi2_cleanup(csi2);

	return rval;
}

void ipu_isys_csi2_sof_event(struct ipu_isys_csi2 *csi2)
{
	struct ipu_isys_pipeline *ip = NULL;
	struct v4l2_event ev = {
		.type = V4L2_EVENT_FRAME_SYNC,
	};
	struct video_device *vdev = csi2->asd.sd.devnode;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&csi2->isys->lock, flags);
	csi2->in_frame = true;

	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (csi2->isys->pipes[i] &&
		    csi2->isys->pipes[i]->csi2 == csi2) {
			ip = csi2->isys->pipes[i];
			break;
		}
	}

	/* Pipe already vanished */
	if (!ip) {
		spin_unlock_irqrestore(&csi2->isys->lock, flags);
		return;
	}

	ev.u.frame_sync.frame_sequence = atomic_inc_return(&ip->sequence) - 1;
	spin_unlock_irqrestore(&csi2->isys->lock, flags);

	v4l2_event_queue(vdev, &ev);
	dev_dbg(&csi2->isys->adev->dev,
		"sof_event::csi2-%i sequence: %i\n",
		csi2->index, ev.u.frame_sync.frame_sequence);
}

void ipu_isys_csi2_eof_event(struct ipu_isys_csi2 *csi2)
{
	struct ipu_isys_pipeline *ip = NULL;
	unsigned long flags;
	unsigned int i;
	u32 frame_sequence;

	spin_lock_irqsave(&csi2->isys->lock, flags);
	csi2->in_frame = false;
	if (csi2->wait_for_sync)
		complete(&csi2->eof_completion);

	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (csi2->isys->pipes[i] &&
		    csi2->isys->pipes[i]->csi2 == csi2) {
			ip = csi2->isys->pipes[i];
			break;
		}
	}

	if (ip) {
		frame_sequence = atomic_read(&ip->sequence);
		spin_unlock_irqrestore(&csi2->isys->lock, flags);

		dev_dbg(&csi2->isys->adev->dev,
			"eof_event::csi2-%i sequence: %i\n",
			csi2->index, frame_sequence);
		return;
	}

	spin_unlock_irqrestore(&csi2->isys->lock, flags);
}

/* Call this function only _after_ the sensor has been stopped */
void ipu_isys_csi2_wait_last_eof(struct ipu_isys_csi2 *csi2)
{
	unsigned long flags, tout;

	spin_lock_irqsave(&csi2->isys->lock, flags);

	if (!csi2->in_frame) {
		spin_unlock_irqrestore(&csi2->isys->lock, flags);
		return;
	}

	reinit_completion(&csi2->eof_completion);
	csi2->wait_for_sync = true;
	spin_unlock_irqrestore(&csi2->isys->lock, flags);
	tout = wait_for_completion_timeout(&csi2->eof_completion,
					   IPU_EOF_TIMEOUT_JIFFIES);
	if (!tout)
		dev_err(&csi2->isys->adev->dev,
			"csi2-%d: timeout at sync to eof\n",
			csi2->index);
	csi2->wait_for_sync = false;
}

struct ipu_isys_buffer *ipu_isys_csi2_get_short_packet_buffer(struct
							      ipu_isys_pipeline
							      *ip)
{
	struct ipu_isys_buffer *ib;
	struct ipu_isys_private_buffer *pb;
	struct ipu_isys_mipi_packet_header *ph;

	if (list_empty(&ip->short_packet_incoming))
		return NULL;
	ib = list_last_entry(&ip->short_packet_incoming,
			     struct ipu_isys_buffer, head);
	pb = ipu_isys_buffer_to_private_buffer(ib);
	ph = (struct ipu_isys_mipi_packet_header *)pb->buffer;

	/* Fill the packet header with magic number. */
	ph->word_count = 0xffff;
	ph->dtype = 0xff;

	dma_sync_single_for_cpu(&ip->isys->adev->dev, pb->dma_addr,
				sizeof(*ph), DMA_BIDIRECTIONAL);
	return ib;
}
