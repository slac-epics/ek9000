# EPICS Device Support for the EK9000

This is the manual for the EK9000 device support module for EPICS 3.14 and later.

If you don't know already, the EK9000 is a low-cost Modbus to EtherCAT coupler. It allows you to control EtherCAT terminals (such as the EL3064 analog input) using a simple Modbus interface, which can be used on virtually any IOC with internet access.

## Building and Installing

This device support module was built and tested with EPICS R3.14, but should function fine in EPICS 7 or any earlier versions of EPICS.

This module depends on the EPICS modbus module (R3-0 or later) and subsequently requires the asyn driver.

Once you clone this module to your workspace, you'll need to change some of the configuration Makefiles to reflect your personal environment. Once done, building is pretty easy:

```bash
make
```

You can run the test IOC with `./ek9000Test st.cmd` or with `./st.cmd` from the `iocBoot` directory.

## How It Works

Although the EK9000 is pretty easy to access with Modbus, it has some odd quirks and shortcomings that made making this module somewhat difficult, so I thought I'd include an explanation of how the module functions.

### Initialization

There are two steps to initialization: startup script functions and the actual record initialization.

In the startup script, the function `ek9000Configure(name, ip, port, slave_count)` is used to create an EK9000 device at the specified ip and port with the specified number of slaves.
Internally, it allocates memory for terminal structures and holds some important parameters.

Once the device is created, it will try to connect to the device. If it can't connect, the records and other startup script functions (such as `ek9000ConfigureTerminal`) will fail.

#### Startup script

After this, you can use `ek9000ConfigureTerminal(ek9000_name, record_name, type, slave_number)` to tell the driver to expect a terminal attached to the specified ek9000 with the given type, record name and slave position. The parameter `record_name` is the **base** name of the record that this terminal is supposed to be attached to.
Type is an integral value representing the terminal type, for example, 3064, which is found in the string EL3064.
`slave_number` simply represents the position on the rail: keep in mind, this is a 1-based index.

It's important to make sure that the type of terminal that you specify actually matches what's on the rail.
The module **WILL** verify that they match, and throw an error if they don't.
This also means that you can't initialize terminals without a connection to the device.

After those functions are called, you can load a database or a substitutions file that contains the records you want to be used by terminals attached to that ek9000. It's important that you ONLY load these records after you've registered all terminals to the device.

#### Record initialization

During record initialization, terminals with the `DTYP` field set to `EL10XX`, `EL11XX`, etc. will initiate a search for the terminal they're supposed to be attached to.
The name of the record usually will be suffixed with a `:x`, where x is an integer representing the channel that the record represents.
The string preceding the `:x` acts as the base record name, which you specified in your init script.
I decided to do this, because it made sense to me.. Comments on this design decision are welcome.

For example, if you use the template `EL1002.template`, and substitute all the stuff you want to, the records `MyTerminal1:1` and `MyTerminal1:2` will be created.
Given that you ran `ek9000ConfigureTerminal("EK9K1", "MyTerminal1", 1002, 1)` or something similar in your ioc init script, the records will initialize successfully.

Once the record initializes, it associates some private data with itself which contains a pointer to the descriptor of the terminal and some other stuff.

#### Final initialization

The final thing the module does is compute a register map for the terminals inside of the EK9000's register space.
The EK9000 will map the PDO (program data objects) from each slave into it's Modbus register space. Digital inputs will be mapped to discrete input coils and digital outputs will be mapped to output coils. Analog inputs get mapped to input registers and analog outputs are mapped to output registers.

Unfortunately, there is no way to ask the EK9000 for a register map, instead we have to use a register space (Starting at 0x6000) to figure out where each terminal is located on the rail.
This doesn't tell us anything about the size of the PDOs for each slave, nor does it tell us about the PDO types. Thus, all PDO sizes are hard-coded into the module (well, they're actually in an auto generated header).
Devices with multiple PDO types, for example the EL3064 which has a compact and a standard PDO, can't really have their PDO sizes predicted, as they can be set to who knows what. In the next release of the module, a remedy for this is planned (it will actually involve a big restructure of the initialization commands). For all intents and purposes, the module will assume all devices have a PDO set to standard (if they have multiple PDO types).

With this in mind, the module will loop through all registered terminals and determine the register map in the same way the EK9000 does.
It will store this info with a terminal's descriptor, which will then be used by the reading and writing functions.

### Writing to a digital output or reading from a digital input

Using the channel that is determined from the record name, the module will compute a coil index by adding the base coil address to the channel number minus 1. 

Reading from a digital input terminal is done in a similar way, except we read from input coils

### Reading/Writing to analog terminals

Once again, using the channel that is determined from the record name, the module computes which registers need to be read with the following equation:

```c
register = base_register + ((channel-1) * channel_size)
```

In the above example, `channel_size` refers to the size of each channel in modbus registers. Since modbus registers are 16-bits wide, `channel_size` will equal 2, since each channel is 4 bytes wide by default

## Supported Terminals

This module supports most digital and analog I/O terminals, but a few are not supported. You can use the following terminals normally with your EK9000:

* EL10XX and EL11XX digital input terminals
* EL20XX and EL21XX digital output terminals
* EL30XX and EL31XX analog input terminals
* EL40XX and EL41XX analog output terminals

The following terminals are not quite supported, and can't be accessed from the device support module:

* Anything not specified in the above list
* EL7XXX motor terminals
* EL6XXX and EL5XXX terminals

If you have an unsupported terminal, but want to use it on the same rail as other terminals that you'd like to access from an IOC, keep the following rules in mind:

* If you place an unsupported terminal before any slaves that you want to access from EPICS, the register map the device support module computes will be incorrect.
* On analog I/O terminals that have multiple PDO mapping types, if you change the PDO mapping from anything **except standard**, your analog I/O will be horribly broken and the register map will be wrong.
* When placing an unsupported terminal on the same rail as other terminals to be accessed by an EPICS IOC, place it last on the rail.

Here's a visual example of these rules:
| Slave Number | Terminal Type | Accessible by EPICS IOC |
|---|---|---|
| 1 | EL3064 | Yes |
| 2 | EL2008 | Yes |
| 3 | EL7047 | No |
| 4 | EL3064 | No |
| 5 | EL2008 | No |

In the above table, all terminals that are placed after the **unsupported** slave (number 3), cannot be _properly_ accessed by the IOC.
In the above example, slave 4, which is placed after the EL7047, will accidentally get mapped to the address space of slave 3, the unsupported terminal, since the module doesn't know the PDO sizes of the EL7047.

## Startup Script Example

Here's an example of how a rail and IOC might be set up.

Lets assume the rail looks something like this:
| Slave number | Terminal Type |
|---|---|
| 1 | EL3064 |
| 2 | EL2008 |
| 3 | EL3154 |
| 4 | EL1004 |
| 5 | EL1008 |

The initialization script might look something like this:

```bash
...
ek9000Configure("EK9K1", "192.168.1.3", 502, 5)
ek9000ConfigureTerminal("EK9K1", "MyTerminal1", 3064, 1)
ek9000ConfigureTerminal("EK9K1", "MyTerminal2", 2008, 2)
ek9000ConfigureTerminal("EK9K1", "MyTerminal3", 3154, 3)
ek9000ConfigureTerminal("EK9K1", "MyTerminal4", 1004, 4)
ek9000ConfigureTerminal("EK9K1", "MyTerminal5", 1008, 5)
...
dbLoadTemplate("MySubs.substitutions")
...
```

Lets assume that `MySubs.substitutions` contains the following records, which are created from the proper template file:

* MyTerminal1 (Created from EL3064.template)
* MyTerminal2 (Created from EL2008.template)
* MyTerminal3 (Created from EL3154.template)
* MyTerminal4 (Created from EL1004.template)
* MyTerminal5 (Created from EL1008.template)

Given this, you can write to the first digital output on slave 2 using:

```bash
caput MyTerminal1:1  1
```

You can also read from the 3rd analog input on slave 3 using:

```bash
caget MyTerminal3.RVAL
```

## IOCsh Function Documentation

```c
----------------------------------------------------------
ek9000Configure(ek9k, ip, port, num_slaves)

Desc:
	Creates and configures an EK9000 device.

Params:
	[str] ek9k   - The name of the EK9000 to register (must be unique)
	[str] ip     - IP that the EK9000 can be found at
	[int] port   - Port number
	[int] num_slaves - The number of slaves this device will have

----------------------------------------------------------
ek9000ConfigureTerminal(ek9k, record, type, position)

Desc:
	Adds and configures a teminal.

Params:
	[str] ek9k   - The name of the EK9000 this terminal should be attached to
	[str] record - The name of the record to be associated with
	[int] type   - The terminal id, such as 1124 in EL1124
	[int] position - The slave number of this terminal (for example slave #2 will be 2)
----------------------------------------------------------
ek9000Stat(ek9k)

Desc:
	Displays stats about the given ek9000.

Params:
	[str] ek9k   - The name of the ek9000
----------------------------------------------------------
ek9000EnableDebug(ek9k)

Desc:
	Enable very verbose debug logging.

Params:
	[str] ek9k   - The name of the ek9000

----------------------------------------------------------
ek9000DisableDebug(ek9k)

Desc:
	Enables very verbose debug logging.

Params:
	[str] ek9k   - The name of the ek9000

----------------------------------------------------------
```

## Contributing

If you'd like to contribute to this project, feel free to open a pull request at any time!

## Bugs

If you find any bugs, you can open an issue or send me an email, and I'll hopefully get around to fixing it fairly quickly.

## Suggestions or Requests

If you have any suggestions or feature requests, you can either open an issue on the github repository or email me at lorelli@slac.stanford.edu

Usually I'll respond within a day or two.

## Developers

* Jeremy Lorelli (jeremy.lorelli.1337@gmail.com)
