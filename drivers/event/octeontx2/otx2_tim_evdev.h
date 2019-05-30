/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(C) 2019 Marvell International Ltd.
 */

#ifndef __OTX2_TIM_EVDEV_H__
#define __OTX2_TIM_EVDEV_H__

#include <rte_event_timer_adapter.h>

#include "otx2_dev.h"

#define OTX2_TIM_EVDEV_NAME otx2_tim_eventdev

struct otx2_tim_evdev {
	struct rte_pci_device *pci_dev;
	struct otx2_mbox *mbox;
	uint16_t nb_rings;
	uintptr_t bar2;
};

void otx2_tim_init(struct rte_pci_device *pci_dev, struct otx2_dev *cmn_dev);

#endif /* __OTX2_TIM_EVDEV_H__ */
