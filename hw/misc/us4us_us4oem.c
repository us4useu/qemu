/*
 * Us4us Us4oem virtual device.
 *
 * Based on the edu device made by Jiri Slaby 2012-2015
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"

#define TYPE_PCI_US4OEM_DEVICE "us4oem"
typedef struct Us4OEMState Us4OEMState;
DECLARE_INSTANCE_CHECKER(Us4OEMState, US4OEM, TYPE_PCI_US4OEM_DEVICE)

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100
#define DMA_START       0x40000
#define DMA_SIZE        4096
#define DMA_MASK        (~0ull) // 64 bits

#define VENDOR_ID       0x1172 // Altera?
#define DEVICE_ID       0xe005
#define REVISION_ID     0x1  // currently doesn't matter

struct Us4OEMState {
    PCIDevice pdev;
    MemoryRegion pcidma_region;
    MemoryRegion us4oem_region;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define US4OEM_STATUS_COMPUTING    0x01
#define US4OEM_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define US4OEM_DMA_RUN             0x1
#define US4OEM_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define US4OEM_DMA_FROM_PCI       0
# define US4OEM_DMA_TO_PCI         1
#define US4OEM_DMA_IRQ             0x4
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    QEMUTimer dma_timer;
    char dma_buf[DMA_SIZE];
    uint64_t dma_mask;
};

static bool us4oem_msi_enabled(Us4OEMState *us4oem)
{
    return msi_enabled(&us4oem->pdev);
}

static void us4oem_raise_irq(Us4OEMState *us4oem, uint32_t val)
{
    us4oem->irq_status |= val;
    if (us4oem->irq_status) {
        if (us4oem_msi_enabled(us4oem)) {
            msi_notify(&us4oem->pdev, 0);
        } else {
            pci_set_irq(&us4oem->pdev, 1);
        }
    }
}

static void us4oem_lower_irq(Us4OEMState *us4oem, uint32_t val)
{
    us4oem->irq_status &= ~val;

    if (!us4oem->irq_status && !us4oem_msi_enabled(us4oem)) {
        pci_set_irq(&us4oem->pdev, 0);
    }
}

static bool within(uint64_t addr, uint64_t start, uint64_t end)
{
    return start <= addr && addr < end;
}

static void us4oem_check_range(uint64_t addr, uint64_t size1, uint64_t start,
                uint64_t size2)
{
    uint64_t end1 = addr + size1;
    uint64_t end2 = start + size2;

    if (within(addr, start, end2) &&
            end1 > addr && within(end1, start, end2)) {
        return;
    }

    hw_error("us4oem: DMA range 0x%016"PRIx64"-0x%016"PRIx64
             " out of bounds (0x%016"PRIx64"-0x%016"PRIx64")!",
            addr, end1 - 1, start, end2 - 1);
}

static dma_addr_t us4oem_clamp_addr(const Us4OEMState *us4oem, dma_addr_t addr)
{
    dma_addr_t res = addr & us4oem->dma_mask;

    if (addr != res) {
        printf("us4oem: clamping DMA %#.16"PRIx64" to %#.16"PRIx64"!\n", addr, res);
    }

    return res;
}

static void us4oem_dma_timer(void *opaque)
{
    Us4OEMState *us4oem = opaque;
    bool raise_irq = false;

    if (!(us4oem->dma.cmd & US4OEM_DMA_RUN)) {
        return;
    }

    if (US4OEM_DMA_DIR(us4oem->dma.cmd) == US4OEM_DMA_FROM_PCI) {
        uint64_t dst = us4oem->dma.dst;
        us4oem_check_range(dst, us4oem->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START;
        pci_dma_read(&us4oem->pdev, us4oem_clamp_addr(us4oem, us4oem->dma.src),
                us4oem->dma_buf + dst, us4oem->dma.cnt);
    } else {
        uint64_t src = us4oem->dma.src;
        us4oem_check_range(src, us4oem->dma.cnt, DMA_START, DMA_SIZE);
        src -= DMA_START;
        pci_dma_write(&us4oem->pdev, us4oem_clamp_addr(us4oem, us4oem->dma.dst),
                us4oem->dma_buf + src, us4oem->dma.cnt);
    }

    us4oem->dma.cmd &= ~US4OEM_DMA_RUN;
    if (us4oem->dma.cmd & US4OEM_DMA_IRQ) {
        raise_irq = true;
    }

    if (raise_irq) {
        us4oem_raise_irq(us4oem, DMA_IRQ);
    }
}

static void dma_rw(Us4OEMState *us4oem, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && (us4oem->dma.cmd & US4OEM_DMA_RUN)) {
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        timer_mod(&us4oem->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

// BAR REGION READS.
// PCIDMA MEMORY REGION (512 Bytes)
static uint64_t us4oem_pcidma_region_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    printf("Reading pcidma address: %lu, size: %u\n", addr, size);
    printf("NOTE: us4oem firmware currently doesn't use device DMA space."
           "This simulator will always return 0.\n");
    return 0ull;
}

static void us4oem_pcidma_region_write(void *opaque, hwaddr addr, uint64_t val,
                                       unsigned size)
{
    printf("Writing pcidma address: %lu, value: %lu, size: %u\n", addr, val, size);
    printf("NOTE: us4oem firmware currently doesn't use device DMA space.\n");
}

static const MemoryRegionOps us4oem_pcidma_region_ops = {
    .read = us4oem_pcidma_region_read,
    .write = us4oem_pcidma_region_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        // How many bytes can be accessed in a single read/write
        // Asumming now, that us4oem's max size is 4 bytes.
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

// ARRIA MAIN MEMORY REGION (64 MiB)
static uint64_t us4oem_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    printf("Reading us4oem address: %lu, size: %u\n", addr, size);
    Us4OEMState* us4oem = US4OEM(opaque);
    uint64_t val = 0;

    // TODO read firmware
    // TODO read tx firmware

    switch (addr) {
    case 0x00:
        val = 0x010000edu;
        break;
    case 0x04:
        val = us4oem->addr4;
        break;
    case 0x08:
        qemu_mutex_lock(&us4oem->thr_mutex);
        val = us4oem->fact;
        qemu_mutex_unlock(&us4oem->thr_mutex);
        break;
    case 0x20:
        val = qatomic_read(&us4oem->status);
        break;
    case 0x24:
        val = us4oem->irq_status;
        break;
    case 0x80:
        dma_rw(us4oem, false, &val, &us4oem->dma.src, false);
        break;
    case 0x88:
        dma_rw(us4oem, false, &val, &us4oem->dma.dst, false);
        break;
    case 0x90:
        dma_rw(us4oem, false, &val, &us4oem->dma.cnt, false);
        break;
    case 0x98:
        dma_rw(us4oem, false, &val, &us4oem->dma.cmd, false);
        break;
    }
    return val;
}

static void us4oem_mem_write(void *opaque, hwaddr addr, uint64_t val,
                                       unsigned size)
{
    Us4OEMState *us4oem = opaque;

    if (addr < 0x80 && size != 4) {
        return;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        return;
    }

    switch (addr) {
    case 0x04:
        us4oem->addr4 = ~val;
        break;
    case 0x08:
        if (qatomic_read(&us4oem->status) & US4OEM_STATUS_COMPUTING) {
            break;
        }
        /* US4OEM_STATUS_COMPUTING cannot go 0->1 concurrently, because it is only
         * set in this function and it is under the iothread mutex.
         */
        qemu_mutex_lock(&us4oem->thr_mutex);
        us4oem->fact = val;
        qatomic_or(&us4oem->status, US4OEM_STATUS_COMPUTING);
        qemu_cond_signal(&us4oem->thr_cond);
        qemu_mutex_unlock(&us4oem->thr_mutex);
        break;
    case 0x20:
        if (val & US4OEM_STATUS_IRQFACT) {
            qatomic_or(&us4oem->status, US4OEM_STATUS_IRQFACT);
        } else {
            qatomic_and(&us4oem->status, ~US4OEM_STATUS_IRQFACT);
        }
        break;
    case 0x60:
        us4oem_raise_irq(us4oem, val);
        break;
    case 0x64:
        us4oem_lower_irq(us4oem, val);
        break;
    case 0x80:
        dma_rw(us4oem, true, &val, &us4oem->dma.src, false);
        break;
    case 0x88:
        dma_rw(us4oem, true, &val, &us4oem->dma.dst, false);
        break;
    case 0x90:
        dma_rw(us4oem, true, &val, &us4oem->dma.cnt, false);
        break;
    case 0x98:
        if (!(val & US4OEM_DMA_RUN)) {
            break;
        }
        dma_rw(us4oem, true, &val, &us4oem->dma.cmd, true);
        break;
    }
}

static const MemoryRegionOps us4oem_region_ops = {
    .read = us4oem_mem_read,
    .write = us4oem_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        // How many bytes can be accessed in a single read/write
        // Asumming now, that us4oem's max size is 4 bytes.
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * We purposely use a thread, so that users are forced to wait for the status
 * register.
 */
static void *us4oem_fact_thread(void *opaque)
{
    Us4OEMState *us4oem = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&us4oem->thr_mutex);
        while ((qatomic_read(&us4oem->status) & US4OEM_STATUS_COMPUTING) == 0 &&
                        !us4oem->stopping) {
            qemu_cond_wait(&us4oem->thr_cond, &us4oem->thr_mutex);
        }

        if (us4oem->stopping) {
            qemu_mutex_unlock(&us4oem->thr_mutex);
            break;
        }

        val = us4oem->fact;
        qemu_mutex_unlock(&us4oem->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&us4oem->thr_mutex);
        us4oem->fact = ret;
        qemu_mutex_unlock(&us4oem->thr_mutex);
        qatomic_and(&us4oem->status, ~US4OEM_STATUS_COMPUTING);

        if (qatomic_read(&us4oem->status) & US4OEM_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            us4oem_raise_irq(us4oem, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void pci_us4oem_realize(PCIDevice *pdev, Error **errp)
{
    Us4OEMState *us4oem = US4OEM(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    timer_init_ms(&us4oem->dma_timer, QEMU_CLOCK_VIRTUAL, us4oem_dma_timer, us4oem);

    qemu_mutex_init(&us4oem->thr_mutex);
    qemu_cond_init(&us4oem->thr_cond);
    qemu_thread_create(&us4oem->thread, "us4oem", us4oem_fact_thread,
                       us4oem, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&us4oem->pcidma_region,
                          OBJECT(us4oem),
                          &us4oem_pcidma_region_ops,
                          us4oem,
                          "us4oem-pcidma-region",
                          512);
    pci_register_bar(pdev,
                     0,
                     (PCI_BASE_ADDRESS_MEM_TYPE_64|PCI_BASE_ADDRESS_MEM_PREFETCH),
                     &us4oem->pcidma_region);
    memory_region_init_io(&us4oem->us4oem_region,
                          OBJECT(us4oem),
                          &us4oem_region_ops,
                          us4oem,
                          "us4oem-main-region",
                          64 * MiB);
    pci_register_bar(pdev,
                     4,
                     (PCI_BASE_ADDRESS_MEM_TYPE_64|PCI_BASE_ADDRESS_MEM_PREFETCH),
                     &us4oem->us4oem_region);
}

static void pci_us4oem_uninit(PCIDevice *pdev)
{
    Us4OEMState *us4oem = US4OEM(pdev);

    qemu_mutex_lock(&us4oem->thr_mutex);
    us4oem->stopping = true;
    qemu_mutex_unlock(&us4oem->thr_mutex);
    qemu_cond_signal(&us4oem->thr_cond);
    qemu_thread_join(&us4oem->thread);

    qemu_cond_destroy(&us4oem->thr_cond);
    qemu_mutex_destroy(&us4oem->thr_mutex);

    timer_del(&us4oem->dma_timer);
    msi_uninit(pdev);
}

static void us4oem_instance_init(Object *obj)
{
    Us4OEMState *us4oem = US4OEM(obj);

    us4oem->dma_mask = DMA_MASK;
    object_property_add_uint64_ptr(obj, "dma_mask",
                                   &us4oem->dma_mask, OBJ_PROP_FLAG_READWRITE);
}

static void us4oem_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_us4oem_realize;
    k->exit = pci_us4oem_uninit;
    k->vendor_id = VENDOR_ID;
    k->device_id = DEVICE_ID;
    k->revision = REVISION_ID;
    k->class_id = PCI_BASE_CLASS_SIGNAL_PROCESSING;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void pci_us4oem_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo us4oem_info = {
        .name          = TYPE_PCI_US4OEM_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(Us4OEMState),
        .instance_init = us4oem_instance_init,
        .class_init    = us4oem_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&us4oem_info);
}
type_init(pci_us4oem_register_types)
