# EPICS Support For Beckhoff EK9000

This is a device support module for the EK9000, which couples EtherCAT terminals (specifically the ELXXXX terminals from Beckhoff) to a modbus network.
Because it uses modbus, it's runnable on any IOC without needing a kernel module or other support software.

[Documentation](https://github.com/slaclab/epics-ek9000/wiki/Manual)

[DOE Code](https://www.osti.gov/doecode/biblio/75566)

[doi:10.11578/dc.20220707.2](https://doi.org/10.11578/dc.20220707.2)

## Dependencies

* [modbus](https://github.com/epics-modules/modbus) R3-0 or later
* [asyn](https://github.com/epics-modules/asyn) (R4-36 is recommended for modbus R3-0, but earlier versions may work)

## Building

C++11 is **required** as of R1.7.4

Build like any other EPICS module:
```
make
```

## Contributing

If you've found an issue, you can open an issue on this repository and I'll hopefully be able to resolve it fairly quickly.

If you'd like to contribute to this, you can open a pull request with changes, and I'll hopefully get back to you as soon as possible.

If you have any questions or comments about the module, you can reach me at jeremy.lorelli.1337@gmail.com

## Known Issues

The EK9000 doesn't really make figuring out a register map easy, so please keep these things in mind:

* If you install an unsupported terminal, the device support module may fail to correctly determine the register map, resulting in weirdness and stuff.
* If you place an unsupported analog ONLY terminal AFTER all other slaves on the EK9000, the register map will be correct. This is because the device maps all analog terminals in order of their position on the rail. Digital terminals are mapped to coils, and therefore, not affected by weirdness in the holding/input register space.
* If you place an unsupported digital ONLY terminal AFTER all other slaves on the EK9000, the register map will also be correct.
* If you change the Pdo mapping for analog input terminals to something other than standard, the analog IO AND register mapping will not work correctly.
To recap:
* Only supported terminals (EL10XX, EL11XX, EL20XX, EL30XX, EL31XX, EL40XX) should be placed on the rail first. Unsupported terminals can be placed after the ones you are trying to access through EPICS.
* Analog input Pdo mapping should ALWAYS be standard, or else the module will not work.

## Supported Terminals

See [SupportedDevices.md](SupportedDevices.md) for an auto-generated list of
the supported terminals.

At least some of the following terminal families are supported:

* EL10XX (EL1004, etc.)
* EL11XX (EL1124, etc.)
* EL20XX (EL2008, etc.)
* EL21XX
* EL30XX (EL3064, etc.)
* EL31XX (EL3174, etc.)
* EL40XX (EL4004, etc.)
* EL41XX (EL4104, etc.)

Any other terminals are not generally supported, but a few exceptions exist.
Support for motor terminals (EL70XX) is currently being developed.

## Copyright Notice:

COPYRIGHT © SLAC National Accelerator Laboratory. All rights reserved. This work is supported [in part] by the U.S. Department of Energy, Office of Basic Energy Sciences under contract DE-AC02-76SF00515.

## Usage Restrictions:

Neither the name of the Leland Stanford Junior University, SLAC National Accelerator Laboratory, U.S. Department of Energy nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

