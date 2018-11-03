/*
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 *  Copyright 1999-2002 Michael Klein <michael(dot)klein(at)puffin(dot)lb(dot)shuttle(dot)de>
 *  Copyright 1997-2005 Joe Forster <sta(at)c64(dot)org> (Device Detection Code)
 *  Copyright 1997-2005 Wolfgang Moser (http://d81.de)   (Device Detection Code)
 *  Copyright 2000-2005 Markus Brenner                   (Parallel Burst Routines)
 *  Copyright 2000-2005 Pete Rittwage                    (Parallel Burst Routines)
 *  Copyright 2005      Tim Schürmann                    (Parallel Burst Routines)
 *  Copyright 2005-2006,2009 Spiro Trikaliotis           (Parallel Burst Routines)
 *  Copyright 2007-2009 Frédéric Brière                  (Adjustments on newer Linux kernels, abstraction from real hardware)
 *  Copyright 2009      Arnd Menge <arnd(at)jonnz(dot)de> (Parallel Burst Routines)
 *  Copyright 2018      Felix Palmen <felix(at)palmen-it(dot)de> (FreeBSD port)
 *
 */

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/types.h>
#include <sys/systm.h>

#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/bus.h>
#include <sys/malloc.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/ppbus/ppbconf.h>
#include "ppbus_if.h"
#include <dev/ppbus/ppbio.h>

#define CBM_NAME "cbm"

#define IEC_DATA   1
#define IEC_CLOCK  2
#define IEC_ATN    4
#define IEC_RESET  8

/* lpt output lines */
#define ATN_OUT    0x01
#define CLK_OUT    0x02
#define DATA_OUT  0x04
#define RESET     0x08

/* lpt input lines */
#define ATN_IN     0x10
#define CLK_IN     0x20
#define DATA_IN    0x40

#define GET(line)        ((POLL() & line) == 0 ? 1 : 0)
#define SET(line)        (CTRL_WRITE(sc->sc_out_eor ^ \
				(sc->sc_out_bits |= line)))
#define RELEASE(line)    (CTRL_WRITE(sc->sc_out_eor ^ \
				(sc->sc_out_bits &= ~(line))))
#define SET_RELEASE(s,r) (CTRL_WRITE(sc->sc_out_eor ^ \
				(sc->sc_out_bits = \
				 (sc->sc_out_bits | (s)) & ~(r))))

#define POLL()           (ppb_rstr(ppbus))
#define XP_READ()        (ppb_rdtr(ppbus))
#define XP_WRITE(c)      (ppb_wdtr(ppbus,c))
#define CTRL_READ()      (ppb_rctr(ppbus))
#define CTRL_WRITE(c)    (ppb_wctr(ppbus,c))

#define set_data_forward() do { CTRL_WRITE(CTRL_READ() & 0xdf); \
                                 sc->sc_data_reverse = 0; } while (0)
#define set_data_reverse() do { CTRL_WRITE(CTRL_READ() | 0x20); \
                                 sc->sc_data_reverse = 1; } while (0)
#define disable_irq() CTRL_WRITE(CTRL_READ() & 0xef)
#define enable_irq() CTRL_WRITE(CTRL_READ() | 0x10)
#define timeout_us(us) pause(CBM_NAME, hz/1000000 * us)

static d_open_t cbm_open;
static d_close_t cbm_close;
static d_read_t cbm_read;
static d_write_t cbm_write;
static d_ioctl_t cbm_ioctl;

static int lp = 0;

struct cbm_data {
    int sc_irq_rid;
    struct resource *sc_irq_resource;
    void *sc_irq_cookie;
    device_t sc_device;
    struct cdev *sc_cdev;
    int sc_cable;
    int sc_reset;
    int sc_hold_clk;
    unsigned char sc_out_bits;
    unsigned char sc_out_eor;
    int sc_busy;
    int sc_data_reverse;
    volatile int sc_cbm_irq_count;
};

static struct cdevsw cbm_cdevsw = {
    .d_version = D_VERSION,
    .d_open = cbm_open,
    .d_close = cbm_close,
    .d_read = cbm_read,
    .d_write = cbm_write,
    .d_ioctl = cbm_ioctl,
    .d_name = CBM_NAME
};

static devclass_t cbm_devclass;

/* forward references for parallel burst routines */
int cbm_parallel_burst_read_track(struct cbm_data *sc, device_t ppbus,
	unsigned char *buffer);
int cbm_parallel_burst_read_track_var(struct cbm_data *sc, device_t ppbus,
	unsigned char *buffer);
int cbm_parallel_burst_write_track(struct cbm_data *sc, device_t ppbus,
	unsigned char *buffer, int length);
unsigned char cbm_parallel_burst_read(struct cbm_data *sc, device_t ppbus);
int cbm_parallel_burst_write(struct cbm_data *sc, device_t ppbus,
	unsigned char c);
int cbm_handshaked_read(struct cbm_data *sc, device_t ppbus, int toggle);
int cbm_handshaked_write(struct cbm_data *sc, device_t ppbus,
	char data, int toggle);

static int
check_if_bus_free(struct cbm_data *sc, device_t ppbus)
{
    int ret = 0;
    do
    {
	RELEASE(ATN_OUT | CLK_OUT | DATA_OUT | RESET);
	timeout_us(100);
	SET(ATN_OUT);
	timeout_us(100);
	if (!GET(DATA_IN)) break;
	RELEASE(ATN_OUT);
	timeout_us(100);
	if (!GET(DATA_IN)) ret = 1;
    } while (0);

    RELEASE(ATN_OUT | CLK_OUT | DATA_OUT | RESET);
    return ret;
}

static void
wait_for_free_bus(struct cbm_data *sc, device_t ppbus)
{
    int i = 1;
    while (1)
    {
	if (check_if_bus_free(sc, ppbus))
	{
	    device_printf(sc->sc_device, "bus is free!\n");
	    break;
	}
	if (++i == 1000)
	{
	    device_printf(sc->sc_device, "timeout waiting for free bus\n");
	    break;
	}
	timeout_us(1000);
    }
}

static void do_reset(struct cbm_data *sc, device_t ppbus)
{
    device_printf(sc->sc_device, "resetting devices\n");
    RELEASE(DATA_OUT | ATN_OUT | CLK_OUT);
    set_data_forward();
    disable_irq();
    SET(RESET);
    pause(CBM_NAME, hz/10);
    RELEASE(RESET);
    device_printf(sc->sc_device, "waiting for free bus...\n");
    wait_for_free_bus(sc, ppbus);
}

static int
cbm_open(struct cdev *dev, int oflags __unused, int devtype __unused,
	struct thread *td __unused)
{
    struct cbm_data *sc = dev->si_drv1;
    device_t ppbus = device_get_parent(sc->sc_device);

    if (sc->sc_busy) return EBUSY;

    // TODO: waitq

    sc->sc_busy = 1;
    if (sc->sc_hold_clk) SET(CLK_OUT);

    return 0;
}

static int
cbm_close(struct cdev *dev, int fflag __unused, int devtype __unused,
	struct thread *td __unused)
{
    struct cbm_data *sc = dev->si_drv1;
    device_t ppbus = device_get_parent(sc->sc_device);

    if (!sc->sc_hold_clk) RELEASE(CLK_OUT);
    return 0;
}

static int
cbm_write(struct cdev *dev, struct uio *uio, int ioflag)
{
    (void)dev;
    (void)uio;
    (void)ioflag;
    return 0;
}

static int
cbm_read(struct cdev *dev, struct uio *uio, int ioflag)
{
    (void)dev;
    (void)uio;
    (void)ioflag;
    return 0;
}

static int
cbm_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
	struct thread *td)
{
    (void)dev;
    (void)cmd;
    (void)data;
    (void)fflag;
    (void)td;
    return 0;
}

static void
cbm_intr(void *arg)
{
}

static void
cbm_identify(driver_t *driver, device_t parent)
{
    TUNABLE_INT("cbm.lp", &lp);
    int unit = device_get_unit(parent);
    if (unit != lp) return;
    device_t dev = device_find_child(parent, CBM_NAME, -1);
    if (!dev) BUS_ADD_CHILD(parent, 0, CBM_NAME, -1);
}

static int
cbm_probe(device_t dev)
{
    device_set_desc(dev, "Serial CBM bus driver");
    return BUS_PROBE_SPECIFIC;
}

static int
cbm_attach(device_t dev)
{
    struct cbm_data *sc = device_get_softc(dev);
    int error = 0;

    sc->sc_irq_rid = 0;
    sc->sc_irq_resource = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE | RF_SHAREABLE);

    if (!sc->sc_irq_resource)
    {
	device_printf(dev, "unable to allocate interrupt resource\n");
	return ENXIO;
    }

    error = bus_setup_intr(dev, sc->sc_irq_resource,
	    INTR_TYPE_TTY | INTR_MPSAFE, 0, cbm_intr,
	    sc, &sc->sc_irq_cookie);
    if (error)
    {
	bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		sc->sc_irq_resource);
	device_printf(dev, "unable to register interrupt handler\n");
	return error;
    }

    struct make_dev_args args;
    make_dev_args_init(&args);
    args.mda_flags = MAKEDEV_WAITOK | MAKEDEV_CHECKNAME;
    args.mda_devsw = &cbm_cdevsw;
    args.mda_uid = UID_ROOT;
    args.mda_gid = GID_OPERATOR;
    args.mda_mode = 0600;
    error = make_dev_s(&args, &sc->sc_cdev, CBM_NAME);
    if (error)
    {
	bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		sc->sc_irq_resource);
	device_printf(dev, "unable to create character device\n");
	return error;
    }
    sc->sc_cdev->si_drv1 = sc;

    sc->sc_device = dev;

    sc->sc_cable = -1;
    sc->sc_reset = 1;
    sc->sc_hold_clk = 1;
    TUNABLE_INT_FETCH("cbm.cable", &sc->sc_cable);
    TUNABLE_INT_FETCH("cbm.reset", &sc->sc_reset);
    TUNABLE_INT_FETCH("cbm.hold_clk", &sc->sc_hold_clk);

    device_t ppbus = device_get_parent(dev);
    ppb_lock(ppbus);
    error = ppb_request_bus(ppbus, dev, PPB_WAIT | PPB_INTR);
    if (error)
    {
	ppb_unlock(ppbus);
	destroy_dev(sc->sc_cdev);
	bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		sc->sc_irq_resource);
	device_printf(dev, "unable to own parallel port\n");
	return error;
    }

    device_printf(dev, "parallel port is mine now\n");

    const char *msg = "";
    if (sc->sc_cable < 0)
    {
	unsigned char in = GET(ATN_IN);
	unsigned char out = (CTRL_READ() & ATN_OUT) ? 1 : 0;
	sc->sc_cable = (in != out);
	msg = " (auto)";
    }

    sc->sc_out_eor = sc->sc_cable ? 0xcb : 0xc4;

    device_printf(dev, "using %s cable%s\n",
	    sc->sc_cable ? "active (XA1541)" : "passive (XM1541)", msg);

    sc->sc_cbm_irq_count = 0;

    sc->sc_out_bits = (CTRL_READ() ^ sc->sc_out_eor) &
	(DATA_OUT | CLK_OUT | ATN_OUT | RESET);

    if ((sc->sc_reset < 0 && (sc->sc_out_bits & RESET)) || sc->sc_reset > 0)
	do_reset(sc, ppbus);

    sc->sc_busy = 0;

    RELEASE(DATA_OUT | ATN_OUT | CLK_OUT);
    set_data_forward();
    disable_irq();

    pause(CBM_NAME, hz/20);

    ppb_unlock(ppbus);

    return 0;
}

static int
cbm_detach(device_t dev)
{
    int error = 0;
    struct cbm_data *sc = device_get_softc(dev);

    destroy_dev(sc->sc_cdev);

    device_t ppbus = device_get_parent(dev);
    ppb_lock(ppbus);
    error = ppb_release_bus(ppbus, dev);
    ppb_unlock(ppbus);

    bus_teardown_intr(dev, sc->sc_irq_resource, sc->sc_irq_cookie);
    bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
	    sc->sc_irq_resource);

    return error;
}

static device_method_t cbm_methods[] = {
    DEVMETHOD(device_identify, cbm_identify),
    DEVMETHOD(device_probe, cbm_probe),
    DEVMETHOD(device_attach, cbm_attach),
    DEVMETHOD(device_detach, cbm_detach),
    { 0, 0 }
};

static driver_t cbm_driver = {
    CBM_NAME,
    cbm_methods,
    sizeof(struct cbm_data)
};

DRIVER_MODULE(cbm, ppbus, cbm_driver, cbm_devclass, 0, 0);
MODULE_DEPEND(cbm, ppbus, 1, 1, 1);

/* 
        And here are the functions, used by parallel burst 
        (they are all called by the ioctl-function)
*/

int cbm_parallel_burst_read_track(struct cbm_data *sc, device_t ppbus,
	unsigned char *buffer)
{
	int i, byte;
	register_t saveintr;

	saveintr = intr_disable();

	for (i = 0; i < 0x2000; i += 1) {
		byte = cbm_handshaked_read(sc, ppbus, i & 1);
		if (byte == -1) {
			intr_restore(saveintr);
			return 0;
		}
		buffer[i] = byte;
	}

	cbm_parallel_burst_read(sc, ppbus);
	intr_restore(saveintr);
	return 1;
}

int cbm_parallel_burst_read_track_var(struct cbm_data *sc, device_t ppbus,
	unsigned char *buffer)
{
	int i, byte;
	register_t saveintr;

	saveintr = intr_disable();

	for (i = 0; i < 0x2000; i += 1) {
		byte = cbm_handshaked_read(sc, ppbus, i & 1);
		if (byte == -1) {
			intr_restore(saveintr);
			return 0;
		}
		buffer[i] = byte;
		if (byte == 0x55)
			break;
	}

	cbm_parallel_burst_read(sc, ppbus);
	intr_restore(saveintr);
	return 1;
}

int cbm_parallel_burst_write_track(struct cbm_data *sc, device_t ppbus,
	unsigned char *buffer, int length)
{
	int i;
	register_t saveintr;

	saveintr = intr_disable();

	for (i = 0; i < length; i++) {
		if (cbm_handshaked_write(sc, ppbus, buffer[i], i & 1)) {
			/* timeout */
			intr_restore(saveintr);
			return 0;
		}
	}
	cbm_handshaked_write(sc, ppbus, 0, i & 1);
	cbm_parallel_burst_read(sc, ppbus);
	intr_restore(saveintr);
	return 1;
}

unsigned char cbm_parallel_burst_read(struct cbm_data *sc, device_t ppbus)
{
	int rv = 0;

	RELEASE(DATA_OUT | CLK_OUT);
	SET(ATN_OUT);
	DELAY(20);		/* 200? */
	while (GET(DATA_IN)) ;
	/* linux rv = inportb(parport); */
	if (!sc->sc_data_reverse) {
		XP_WRITE(0xff);
		set_data_reverse();
	}
	rv = XP_READ();
	DELAY(5);
	RELEASE(ATN_OUT);
	DELAY(10);
	while (!GET(DATA_IN)) ;
	return rv;
}

int cbm_parallel_burst_write(struct cbm_data *sc, device_t ppbus,
	unsigned char c)
{
	RELEASE(DATA_OUT | CLK_OUT);
	SET(ATN_OUT);
	DELAY(20);
	while (GET(DATA_IN)) ;
	/* linux PARWRITE(); */
	if (sc->sc_data_reverse)
		set_data_forward();
	XP_WRITE(c);
	/* linux outportb(parport, arg); */
	DELAY(5);
	RELEASE(ATN_OUT);
	DELAY(20);
	while (!GET(DATA_IN)) ;
	/* linux PARREAD(); */
	if (!sc->sc_data_reverse) {
		XP_WRITE(0xff);
		set_data_reverse();
	}
	XP_READ();
	return 0;
}

#define TO_HANDSHAKED_READ  3300000
#define TO_HANDSHAKED_WRITE 3300000

int cbm_handshaked_read(struct cbm_data *sc, device_t ppbus, int toggle)
{
	static int oldvalue = -1;
	int returnvalue = 0;
	int returnvalue2, returnvalue3, timeoutcount;
	int to = 0;

	RELEASE(DATA_IN);	/* not really needed? */

	/* linux
	   RELEASE(DATA_OUT);
	   DELAY(2);
	   GET(DATA_IN); */

	if (!toggle) {
		while (GET(DATA_IN))
			if (to++ > TO_HANDSHAKED_READ)
				return -1;
	} else {
		while (!GET(DATA_IN))
			if (to++ > TO_HANDSHAKED_READ)
				return -1;
	}

	timeoutcount = 0;

	returnvalue3 = XP_READ();
	returnvalue2 = ~returnvalue3;	/* ensure to read once more */

	do {
		if (++timeoutcount >= 8) {
			device_printf(sc->sc_device,
			     "Triple-Debounce TIMEOUT: 0x%02x, 0x%02x, 0x%02x (%d, 0x%02x)\n",
			     returnvalue, returnvalue2, returnvalue3,
			     timeoutcount, oldvalue);
			break;
		}
		returnvalue = returnvalue2;
		returnvalue2 = returnvalue3;
		returnvalue3 = XP_READ();
	} while ((returnvalue != returnvalue2)
		 || (returnvalue != returnvalue3));

	oldvalue = returnvalue;

	return returnvalue;
}

int cbm_handshaked_write(struct cbm_data *sc, device_t ppbus,
	char data, int toggle)
{
	int to = 0;

	RELEASE(CLK_IN);

	if (!toggle) {
		while (GET(DATA_IN))
			if (to++ > TO_HANDSHAKED_WRITE)
				return 1;
	} else {
		while (!GET(DATA_IN))
			if (to++ > TO_HANDSHAKED_WRITE)
				return 1;
	}
	/* linux outportb(parport, data); */
	if (sc->sc_data_reverse)
		set_data_forward();
	XP_WRITE(data);
	return 1;
}

