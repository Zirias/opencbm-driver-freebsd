# OpenCBM parallel port driver for FreeBSD

This is the driver needed for parallel port cables (XM-1541, XA-1541) with
OpenCBM, ported to FreeBSD. If you want to use it, please build OpenCBM from
this port:

<https://github.com/Zirias/zfbsd-ports/tree/master/new/archivers/opencbm>

This port contains a few patches needed for it to work with this driver.

Then, make sure you have the FreeBSD sources installed and just type `make`
to build this driver, which will create an `opencbm.ko` you can load with
`kldload`.

The original options of the Linux driver are supported by tunables:

- `cbm.lp`: Which parallel port number to use, defaults to 0
- `cbm.cable`: -1 is autodetect (default), 0 is passive (XM-1541),
               1 is active (XA-1541)
- `cbm.reset`: whether to reset the CBM bus when driver is loaded:
               -1 reset if corresponding line was set before, 0 don't reset,
	       1 always reset (default)
- `cbm.hold_clk`: 0 release clock line when idle, 1 always hold it
                  (strictly compliant, default)


