// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * APM X-Gene SoC Hardware Monitoring Driver
 *
 * Copyright (c) 2016, Applied Micro Circuits Corporation
 * Authors: Loc Ho <lho@apm.com>
 *          Hoan Tran <hotran@apm.com>
 *
 * This driver provides the following features:
 *  - Retrieve CPU total power (uW)
 *  - Retrieve IO total power (uW)
 *  - Retrieve SoC temperature (milli-degree C) and alarm
 */

#include <linux/acpi.h>
#include <linux/dma-mapping.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <acpi/pcc.h>

/* SLIMpro message defines */
#define MSG_TYPE_DBG			0
#define MSG_TYPE_ERR			7
#define MSG_TYPE_PWRMGMT		9

#define MSG_TYPE(v)			(((v) & 0xF0000000) >> 28)
#define MSG_TYPE_SET(v)			(((v) << 28) & 0xF0000000)
#define MSG_SUBTYPE(v)			(((v) & 0x0F000000) >> 24)
#define MSG_SUBTYPE_SET(v)		(((v) << 24) & 0x0F000000)

#define DBG_SUBTYPE_SENSOR_READ		4
#define SENSOR_RD_MSG			0x04FFE902
#define SENSOR_RD_EN_ADDR(a)		((a) & 0x000FFFFF)
#define PMD_PWR_REG			0x20
#define PMD_PWR_MW_REG			0x26
#define SOC_PWR_REG			0x21
#define SOC_PWR_MW_REG			0x27
#define SOC_TEMP_REG			0x10

#define TEMP_NEGATIVE_BIT		8
#define SENSOR_INVALID_DATA		BIT(15)

#define PWRMGMT_SUBTYPE_TPC		1
#define TPC_ALARM			2
#define TPC_GET_ALARM			3
#define TPC_CMD(v)			(((v) & 0x00FF0000) >> 16)
#define TPC_CMD_SET(v)			(((v) << 16) & 0x00FF0000)
#define TPC_EN_MSG(hndl, cmd, type) \
	(MSG_TYPE_SET(MSG_TYPE_PWRMGMT) | \
	 MSG_SUBTYPE_SET(hndl) | TPC_CMD_SET(cmd) | type)

/*
 * Arbitrary retries in case the remote processor is slow to respond
 * to PCC commands
 */
#define PCC_NUM_RETRIES			500

#define ASYNC_MSG_FIFO_SIZE		16
#define MBOX_OP_TIMEOUTMS		1000

#define WATT_TO_mWATT(x)		((x) * 1000)
#define mWATT_TO_uWATT(x)		((x) * 1000)
#define CELSIUS_TO_mCELSIUS(x)		((x) * 1000)

#define to_xgene_hwmon_dev(cl)		\
	container_of(cl, struct xgene_hwmon_dev, mbox_client)

enum xgene_hwmon_version {
	XGENE_HWMON_V1 = 0,
	XGENE_HWMON_V2 = 1,
};

/* Structure to hold response message from SLIMpro */
struct slimpro_resp_msg {
	u32 msg;
	u32 param1;
	u32 param2;
} __packed;

/* Main device structure */
struct xgene_hwmon_dev {
	struct device		*dev;
	struct mbox_chan	*mbox_chan;
	struct pcc_mbox_chan	*pcc_chan;
	struct mbox_client	mbox_client;
	int			mbox_idx;

	spinlock_t		kfifo_lock;
	struct mutex		rd_mutex;
	struct completion	rd_complete;
	int			resp_pending;
	struct slimpro_resp_msg sync_msg;

	struct work_struct	workq;
	struct kfifo_rec_ptr_1	async_msg_fifo;

	struct device		*hwmon_dev;
	bool			temp_critical_alarm;

	phys_addr_t		comm_base_addr;
	void			*pcc_comm_addr;
	u64			usecs_lat;
};

/*
 * This function tests and clears a bitmask then returns its old value
 */
static u16 xgene_word_tst_and_clr(u16 *addr, u16 mask)
{
	u16 ret, val;

	val = le16_to_cpu(READ_ONCE(*addr));
	ret = val & mask;
	val &= ~mask;
	WRITE_ONCE(*addr, cpu_to_le16(val));

	return ret;
}

/*
 * Perform a read operation using PCC interface
 */
static int xgene_hwmon_pcc_rd(struct xgene_hwmon_dev *ctx, u32 *msg)
{
	struct acpi_pcct_shared_memory *generic_comm_base = ctx->pcc_comm_addr;
	u32 *ptr = (void *)(generic_comm_base + 1);
	int rc, i;
	u16 val;

	mutex_lock(&ctx->rd_mutex);
	init_completion(&ctx->rd_complete);
	ctx->resp_pending = true;

	/* Write signature for subspace */
	WRITE_ONCE(generic_comm_base->signature,
		   cpu_to_le32(PCC_SIGNATURE | ctx->mbox_idx));

	/* Write to the shared command region */
	WRITE_ONCE(generic_comm_base->command,
		   cpu_to_le16(MSG_TYPE(msg[0]) | PCC_CMD_GENERATE_DB_INTR));

	/* Flip CMD COMPLETE bit */
	val = le16_to_cpu(READ_ONCE(generic_comm_base->status));
	val &= ~PCC_STATUS_CMD_COMPLETE;
	WRITE_ONCE(generic_comm_base->status, cpu_to_le16(val));

	/* Copy the message to the PCC comm space */
	for (i = 0; i < sizeof(struct slimpro_resp_msg) / 4; i++)
		WRITE_ONCE(ptr[i], cpu_to_le32(msg[i]));

	/* Ring the doorbell */
	rc = mbox_send_message(ctx->mbox_chan, msg);
	if (rc < 0) {
		dev_err(ctx->dev, "Mailbox send error %d\n", rc);
		goto err;
	}

	if (!wait_for_completion_timeout(&ctx->rd_complete,
					 usecs_to_jiffies(ctx->usecs_lat))) {
		dev_err(ctx->dev, "Mailbox operation timed out\n");
		rc = -ETIMEDOUT;
		goto err;
	}

	/* Check for error message */
	if (MSG_TYPE(ctx->sync_msg.msg) == MSG_TYPE_ERR) {
		rc = -EINVAL;
		goto err;
	}

	msg[0] = ctx->sync_msg.msg;
	msg[1] = ctx->sync_msg.param1;
	msg[2] = ctx->sync_msg.param2;

err:
	if (rc < 0)
		dev_err(ctx->dev, "PCC read operation failed, error %d\n", rc);

	mbox_chan_txdone(ctx->mbox_chan, 0);
	ctx->resp_pending = false;
	mutex_unlock(&ctx->rd_mutex);
	return rc;
}

/*
 * Perform a read operation using either PCC or SLIMpro mailbox
 */
static int xgene_hwmon_rd(struct xgene_hwmon_dev *ctx, u32 *msg)
{
	int rc;

	mutex_lock(&ctx->rd_mutex);
	init_completion(&ctx->rd_complete);
	ctx->resp_pending = true;

	rc = mbox_send_message(ctx->mbox_chan, msg);
	if (rc < 0) {
		dev_err(ctx->dev, "Mailbox send error %d\n", rc);
		goto err;
	}

	if (!wait_for_completion_timeout(&ctx->rd_complete,
					 msecs_to_jiffies(MBOX_OP_TIMEOUTMS))) {
		dev_err(ctx->dev, "Mailbox operation timed out\n");
		rc = -ETIMEDOUT;
		goto err;
	}

	/* Check for error message */
	if (MSG_TYPE(ctx->sync_msg.msg) == MSG_TYPE_ERR) {
		rc = -EINVAL;
		goto err;
	}

	msg[0] = ctx->sync_msg.msg;
	msg[1] = ctx->sync_msg.param1;
	msg[2] = ctx->sync_msg.param2;

err:
	if (rc < 0)
		dev_err(ctx->dev, "Mailbox read operation failed, error %d\n", rc);

	ctx->resp_pending = false;
	mutex_unlock(&ctx->rd_mutex);
	return rc;
}

/*
 * Perform a register map read operation
 */
static int xgene_hwmon_reg_map_rd(struct xgene_hwmon_dev *ctx, u32 addr,
				  u32 *data)
{
	u32 msg[3];
	int rc;

	msg[0] = SENSOR_RD_MSG;
	msg[1] = SENSOR_RD_EN_ADDR(addr);
	msg[2] = 0;

	if (acpi_disabled)
		rc = xgene_hwmon_rd(ctx, msg);
	else
		rc = xgene_hwmon_pcc_rd(ctx, msg);

	if (rc < 0) {
		dev_err(ctx->dev, "Failed to read register map, error %d\n", rc);
		return rc;
	}

	/*
	 * Check if sensor data is valid.
	 * Bits 0-14 are valid data bits, so if any of them are set, data is valid.
	 */
	if (!(msg[1] & ~SENSOR_INVALID_DATA)) {
		dev_err(ctx->dev, "Invalid sensor data received\n");
		return -ENODATA;
	}

	*data = msg[1];

	return 0;
}

/* Get notification message from hardware monitoring */
static int xgene_hwmon_get_notification_msg(struct xgene_hwmon_dev *ctx,
					    u32 *amsg)
{
	u32 msg[3];
	int rc;

	msg[0] = ...; // Populate the message parameters

	rc = xgene_hwmon_pcc_rd(ctx, msg);
	if (rc < 0)
		return rc;

	*amsg = msg[0];

	return 0;
}

/* Remove platform device from the system */
static int xgene_hwmon_remove(struct platform_device *pdev)
{
	struct xgene_hwmon_dev *ctx = platform_get_drvdata(pdev);

	// Clean up operations

	return 0;
}

/* Initialization of the platform driver */
static int xgene_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xgene_hwmon_dev *ctx;
	int ret;

	// Initialization code

	return ret;
}

static const struct of_device_id xgene_hwmon_of_match[] = {
	{ .compatible = "apm,xgene-hwmon", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, xgene_hwmon_of_match);

static struct platform_driver xgene_hwmon_driver = {
	.probe	= xgene_hwmon_probe,
	.remove	= xgene_hwmon_remove,
	.driver	= {
		.name	= "xgene-hwmon",
		.of_match_table = xgene_hwmon_of_match,
	},
};
module_platform_driver(xgene_hwmon_driver);

MODULE_DESCRIPTION("APM X-Gene SoC Hardware Monitoring Driver");
MODULE_AUTHOR("Applied Micro Circuits Corporation");
MODULE_LICENSE("GPL");

