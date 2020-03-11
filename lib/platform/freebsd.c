/*
 * Microsemi Switchtec(tm) PCIe Management Library
 * Copyright (c) 2017, Microsemi Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#if defined( __FreeBSD__) || (defined(__linux__) && LINUX_DRIVERLESS)

#include "../switchtec_priv.h"
#include "../crc.h"
#include "switchtec/switchtec.h"
#include "gasops.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#ifdef __FreeBSD__
//#include <sys/pciio.h>
#include <sys/sysctl.h>
#endif
#include <pciaccess.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#include <stdarg.h>

#define MAX_SW 16

typedef uint64_t dma_t;

struct pcisel {
  uint8_t pc_domain;
  uint8_t pc_bus;
  uint8_t pc_dev;
  uint8_t pc_func;
};

struct switchtec_fbsd {
	struct switchtec_dev dev;
	struct pcisel pcisel;
  int gas_map_cnt;
};

#include "mmap_gas.h"

#define CMD_GET_CAP  0xE0
#define CMD_GAS_WRITE  0xEA
#define CMD_GET_WRITE_STATUS  0xE2
#define CMD_GAS_WRITE_WITH_STATUS  0xE8
#define CMD_GAS_READ  0xE9

#define MAX_RETRY_COUNT  100
#define MAX_STATUS_GET_RETRY  50
#define PEC_BYTE_COUNT  1
#define TWI_ENHANCED_MODE  0x80
#define GAS_TWI_MRPC_ERR  0x20
#define DATA_TAIL_BYTE_COUNT  2

#define to_switchtec_fbsd(d)  \
	((struct switchtec_fbsd *) \
	 ((char *)d - offsetof(struct switchtec_fbsd, dev)))


#include "mmap_gas.h"

void fatal(const char *err, ...) {
  va_list ap;
  va_start(ap, err);

  fprintf(stderr, "FATAL:");
  vfprintf(stderr, err, ap);
  fprintf(stderr, "\n");
  exit(1);
}

void xioctl(int fd, unsigned cmd, void *arg) {
  int rc = ioctl(fd, cmd, arg);
  if (rc < 0) fatal("ioctl 0x%x failed: errno=%s", cmd, strerror(errno));
}


static void * iomap(size_t page, size_t size) {
  int fd;
  void *ptr;
  assert(!(page & 4095));
  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd == -1)
	  fatal("/dev/mem");
  
  ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page);
  if(ptr == MAP_FAILED) {
    perror("mmap");
    exit(1);
  }
  //fprintf(stderr, "/dev/mem mapped %lx -> %p\n", (long)page, ptr);
  return ptr;
}

static void pci_init(void) {
  static int opened = 0;
  if (!opened) {
    int rc = pci_system_init();
    if (rc)
      fatal("pci_system_init()");
    opened = 1;
  }
}

static uint64_t bar_addr(struct pcisel sel) {
  uint64_t ret;
  uint32_t data[2];
  struct pci_device *dev = pci_device_find_by_slot(sel.pc_domain,sel.pc_bus, sel.pc_dev, sel.pc_func);
  if (!dev)
    fatal("Cannot get dev handle for switchtec");
  
  int rc =  pci_device_cfg_read_u32(dev, data+0, 0x10);
  if (rc)
    fatal("pci_device_cfg_read_u32");
  rc =  pci_device_cfg_read_u32(dev, data+1, 0x14);
  if (rc)
    fatal("pci_device_cfg_read_u32");
  
  ret = ((uint64_t)data[1] << 32) + (data[0] & ~0x1f);
  return ret;
}



/*
 * GAS map maps the hardware registers into user memory space.
 * Needless to say, this can be very dangerous and should only
 * be done if you know what you are doing. Any register accesses
 * that use this will remain unsupported by Microsemi unless it's
 * done within the switchtec user project or otherwise specified.
 */
static gasptr_t fbsd_gas_map(struct switchtec_dev *dev, int writeable,
			      size_t *map_size)
{
	int ret;
	void *map;
	ssize_t msize;

	struct switchtec_fbsd *idev = to_switchtec_fbsd(dev);
	struct pcisel sel = idev->pcisel;
	dma_t addr = bar_addr(sel);
	idev->gas_map_cnt += 1;
	if (idev->gas_map_cnt > 1) {
	  if (map_size)
	    *map_size = dev->gas_map_size;
	  return dev->gas_map;
	}
	if (dev->gas_map != NULL)
	  fatal("gas_map called twice");
	//fprintf(stderr, "gas_map (%02x:%02x.%d): %lx\n", sel.pc_bus, sel.pc_dev, sel.pc_func, addr);
	msize = 4 * 1024 * 1024;
	struct pci_device *pdev = pci_device_find_by_slot(sel.pc_domain,sel.pc_bus, sel.pc_dev, sel.pc_func);
	if (!dev)
	  fatal("pci_gas_map: cannot get dev handle for switchtec");
	if (0) {
	  int rc = pci_device_map_range(pdev, addr, msize, PCI_DEV_MAP_FLAG_WRITABLE, &map);
	  if (rc) fatal("pci_device_map_range");
	} else {
	  map = iomap(addr, msize);
	}
	
	dev->gas_map = (gasptr_t)map;
	dev->gas_map_size = msize;

	if (map_size)
	  *map_size = msize;
	ret = gasop_access_check(dev);
	if (ret) {
		errno = ENODEV;
		goto unmap_and_exit;
	}
	return (gasptr_t)map;

unmap_and_exit:
	munmap(map, msize);
	return SWITCHTEC_MAP_FAILED;
}

static void fbsd_gas_unmap(struct switchtec_dev *dev, gasptr_t map)
{
  struct switchtec_fbsd *idev = to_switchtec_fbsd(dev);
  if (map != dev->gas_map || idev->gas_map_cnt <= 0)
    fatal("map != gas_map");
  idev->gas_map_cnt -= 1;
  if (idev->gas_map_cnt == 0)
      munmap((void *)map, dev->gas_map_size);
}


static void fbsd_close(struct switchtec_dev *dev) {
  fbsd_gas_unmap(dev, dev->gas_map);
}

static const struct switchtec_ops i2c_ops = {
	.close = fbsd_close,
	.gas_map = fbsd_gas_map,
	.gas_unmap = fbsd_gas_unmap,
	
	.cmd = gasop_cmd,
	.get_device_id = gasop_get_device_id,
	.get_fw_version = gasop_get_fw_version,
	.pff_to_port = gasop_pff_to_port,
	.port_to_pff = gasop_port_to_pff,
	.flash_part = gasop_flash_part,
	.event_summary = gasop_event_summary,
	.event_ctl = gasop_event_ctl,
	.event_wait_for = gasop_event_wait_for,

	.gas_read8 = mmap_gas_read8,
	.gas_read16 = mmap_gas_read16,
	.gas_read32 = mmap_gas_read32,
	.gas_read64 = mmap_gas_read64,
	.gas_write8 = mmap_gas_write8,
	.gas_write16 = mmap_gas_write16,
	.gas_write32 = mmap_gas_write32,
	.gas_write32_no_retry = mmap_gas_write32,
	.gas_write64 = mmap_gas_write64,
	.memcpy_to_gas = mmap_memcpy_to_gas,
	.memcpy_from_gas = mmap_memcpy_from_gas,
	.write_from_gas = mmap_write_from_gas,
};

static int sw_pci_list(struct pcisel matches[MAX_SW]) {
  const struct  pci_id_match match = { .vendor_id = 0x11f8, .device_id = 0x8531,
				   .subvendor_id = PCI_MATCH_ANY, .subdevice_id = PCI_MATCH_ANY,
				   .device_class = 0x050000, .device_class_mask = 0xff0000};

  pci_init();
  struct pci_device_iterator * iter = pci_id_match_iterator_create(&match);
  if (!iter)
    fatal("pci_slot_match_iterator_create");
  //  fprintf(stderr, "gasop_cmd=%p\n", gasop_cmd);
  int i;
  for (i = 0;i < MAX_SW; i++) {
    struct pci_device *dev = pci_device_next(iter);
    if (!dev)
      break;
    matches[i].pc_domain = dev->domain_16;
    matches[i].pc_bus = dev->bus;
    matches[i].pc_dev = dev->dev;
    matches[i].pc_func = dev->func;
  }
  return i;
}

struct switchtec_dev *switchtec_open_i2c(const char *path, int i2c_addr)
{
	fprintf(stderr, "%s not implemented\n", __func__);
	exit(1);
}

struct switchtec_dev *switchtec_open_i2c_by_adapter(int adapter, int i2c_addr)
{
	fprintf(stderr, "%s not implemented\n", __func__);
	exit(1);
}


struct switchtec_dev *switchtec_open_by_path(const char *path)
{
	fprintf(stderr, "%s not implemented\n", __func__);
	exit(1);
}

struct switchtec_dev *switchtec_open_by_pci_addr(int domain, int bus, int device, int func)
{
  struct pcisel pcisel = {.pc_domain = domain, .pc_bus = bus,.pc_dev = device, .pc_func = func};
	struct switchtec_fbsd *idev = malloc(sizeof(*idev));
	if (!idev)
		return NULL;

#ifdef __FreeBSD__
	int rc = sysctlbyname("dev.stfc.0.%parent", NULL, 0, NULL, 0);
	if (rc != -1 || errno != ENOENT)
	  fatal("switchtec required stfc driver to be unloadded");
#elif defined(__linux__)
	struct stat s;
	if (stat("/sys/module/switchtec", &s) == 0 || errno != ENOENT) {
	  fatal("switchtec required switchtec driver to be unloaded");
	}
#endif

	pci_init();
	
	memset(idev, 0, sizeof(*idev));
	
	idev->dev.ops = &i2c_ops;
	idev->pcisel = pcisel;
	idev->dev.gas_map = NULL;
	fbsd_gas_map(&idev->dev, 1, NULL);
	
	gasop_set_partition_info(&idev->dev);
	
	return &idev->dev;
}

struct switchtec_dev *switchtec_open_by_index(int index)
{
	struct pcisel matches[MAX_SW];
	int n = sw_pci_list(matches);
	if (index >= n) {
		return NULL;
	}
	struct pcisel sel = matches[index];
	return switchtec_open_by_pci_addr(sel.pc_domain, sel.pc_bus, sel.pc_dev, sel.pc_func);
	
}

const char *platform_strerror(void)
{
	return "Success";
}

int switchtec_list(struct switchtec_device_info **devlist)
{
	struct pcisel matches[MAX_SW];
	int i, n;
	struct switchtec_device_info *dl;

	n = sw_pci_list(matches);
	
	dl = *devlist = calloc(n, sizeof(struct switchtec_device_info));
	if (!dl) {
	  fprintf(stderr, "ERROR: No more memory\n");
	  exit(1);
	}
	memset(dl, 0, n * sizeof(struct switchtec_device_info));

	for (i = 0; i < n; i++) {
		snprintf(dl[i].name, sizeof(dl[i].name),
			 "switchtec%d", i);
		snprintf(dl[i].pci_dev, sizeof(dl[i].pci_dev), "0000:%02x:%02x.%d",
			 matches[i].pc_bus, matches[i].pc_dev,
			 matches[i].pc_func);
		struct switchtec_dev *dev = switchtec_open_by_index(i);
		if (!dev) fatal("cannot open switchtec dev from list");
		
		struct sys_info_regs si;
		dev->ops->memcpy_from_gas(dev, &si, &dev->gas_map->sys_info, sizeof(si));
		
		snprintf(dl[i].path, sizeof(dl[i].path),
			 "/dev/switchtec%d", i);

		snprintf(dl[i].product_id, sizeof(dl[i].product_id),
			 "%.16s", si.product_id);

		snprintf(dl[i].product_rev, sizeof(dl[i].product_rev),
			 "%.4s", si.product_revision);
		dev->ops->get_fw_version(dev, dl[i].fw_version, sizeof(dl[i].fw_version));
		
		switchtec_close(dev);
	}

	return n;
}

#endif
