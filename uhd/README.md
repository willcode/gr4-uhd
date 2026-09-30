<!-- Copyright 2026 Jeff Long; SPDX-License-Identifier: MIT -->
# uhd: the USRP source and sink

`capture::uhd::Source` receives from a USRP and `capture::uhd::Sink`
transmits to one, through libuhd's `multi_usrp` interface. Both are GNU Radio
4 blocks of complex float samples. The family word is `uhd`.

## Selecting a unit

`device_parameter` takes every address key libuhd accepts, comma-separated:

```
serial=30D3AB9
type=b200
addr=192.168.10.2
```

An empty selector opens the first unit libuhd finds.

The device list asks libuhd with an empty hint. Where `CAPTURE_UHD_USE_DPDK`
is set in the environment to anything but `0`, the hint carries `use_dpdk=1`,
and a radio reached only through DPDK is listed as well.

## Controls that wait for a start

Four source controls take effect at the next start, and their descriptors
state `applies_at` `start`. Each reaches an open-time setting of the source:

| Control        | Setting             |
|----------------|---------------------|
| `WIRE_FORMAT`  | `wire_format`       |
| `CLOCK_SOURCE` | `clock_source`      |
| `TIME_SOURCE`  | `time_source`       |
| `MASTER_CLOCK` | `master_clock_rate` |

The block keeps a write made while the device is up and opens the device with
it at its next start. A stopped block refuses the write. A write of the
setting that changes it drops the kept choice, and a change of
`device_parameter` drops all four. A program that builds a new block for the
next start passes the choice in the setting: the option's name for the three
lists, and the clock in hertz for `MASTER_CLOCK`, whose control counts
megahertz. An explicit `MASTER_CLOCK` also reaches a running device at once,
except on a radio that states one clock. Every other control of the source and
the sink applies in the running stream. A list control whose entries are
separate points takes an entry alone; the block refuses any other value and
names the entries.

`SAMPLE_RATE` states the rate ladder at the master clock and the wire format in
force, and takes any rate in its range where the clock is programmable.
`MASTER_CLOCK` offers the separate clocks a radio states, or on a clock span
the picks inside it, 5 MHz to 61.44 MHz and the X3xx, N300 or N310 and N320
clocks, up to 250 MHz.
`FREQUENCY` states the tuner's coverage. A write of either stages
`sample_rate` or `frequency`. A source that has not started states the ladder
of the `wire_format` it holds staged or set, so the sc8 rates appear when sc8
is staged.

UHD 4.9 states the clock range of an X3xx, an N3xx or an X4xx as the one
clock it was opened at. On such a radio `MASTER_CLOCK` offers the clocks UHD
4.9's manual states for the product, for every FPGA image that takes the clock
in force. The manual does not name the NI-2974; UHD's X3xx driver runs it as
an X310 motherboard, and it takes the X3xx clocks. The label names the clock
in force.

| Product             | Clocks, MHz                                |
|---------------------|--------------------------------------------|
| X300, X310, NI-2974 | 184.32, 200; 200 alone with a TwinRX       |
| N300, N310          | 122.88, 125, 153.6                         |
| N320, N321          | 200, 245.76, 250                           |
| X410, 200 MHz FPGA  | 245.76, 250                                |
| X410, 400 MHz FPGA  | 491.52, 500                                |
| X440                | 125, 307.2, 327.68, 360, 368.64, 400, 500, |
|                     | and 1000, 2000 on an xx_1600 image         |

A pick there applies at the next open alone, through `master_clock_rate`, and
the rate list shows the new clock's rates after that start. Any open that
fails while it carries a pick from this list drops the pick and names it in
its message, and the start after it opens with the `master_clock_rate`
setting. A clock a running device already took on a span radio stays pinned
through a failed open. When the device runs another clock than the pick, the
source says so and names one cause: an open that finds the device already open
in the same process keeps the clock of the first open. The table copies UHD's
facts and drifts where a later UHD changes them. A product outside the table,
or a clock in force that the table does not hold for the product, gets no
`MASTER_CLOCK`.

`ANTENNA` lists the connectors the unit names and appears where the unit
names more than one. A write of it stages `rx_antennae` on the source and
`tx_antennae` on the sink. The source's default is the connector in force,
and the sink's is the connector the open found. Where the unit states a
list, the source refuses an `rx_antennae` name outside it, names the listed
ones on standard error and leaves the connector and the setting as they
are. A unit that states no list takes or refuses the name the source writes.

`GAIN_MODE` appears where the frontend carries an automatic gain control, at
the `gain_mode` the source holds, and a write of it stages `gain_mode`. The
source's reading `rx_analog_bandwidth` states the analog filter the device
settled on in hertz, the fixed filter of a frontend with no `BANDWIDTH`
control included.

## The transport and the threads

The source passes `recv_frame_size`, `num_recv_frames` and `recv_buff_size`
to libuhd as the caller gives them, and zero leaves each to libuhd. The sink
does the same with its `send_` settings. The frames hold `num_recv_frames`
times the streamer's frame over the rate. UHD 4.9's default on a B2xx is 16
frames of 2040 samples in sc16:

| Rate       | Time 16 x 2040 samples hold |
|------------|-----------------------------|
| 61.44 MS/s | 0.53 ms                     |
| 2 MS/s     | 16 ms                       |

An N2xx takes 32 frames and a socket buffer of `recv_buff_size` bytes. UHD
asks for 50e6 bytes on Linux: 0.5 s at 25 MS/s in sc16. The source's reading
`rx_transport_depth`, beside `rx_overflows`, states the frames, the socket
buffer and the time each holds at the rate in force.

`thread_priority` and `cpu` place the source's receive thread and the sink's
transmit thread. A priority above 0 and at most 1 asks for the real-time
round-robin class at that fraction of its range; 0 leaves the thread as the
system starts it. A `cpu` of 0 or more pins the thread to that processor; -1
leaves it. The start reads both. A placement the system refuses is a line on
standard error, and the stream starts anyway. The real-time class needs
`CAP_SYS_NICE` or an `RLIMIT_RTPRIO` above zero.

A rate change and a stop stop the stream and drain it. The drain ends on the
stream's end-of-burst marker, on a receive that finds nothing, or after
0.3 s, whichever comes first.

## What it needs

- libuhd 4.0 or later, with its headers.
- GNU Radio 4 core; the block library also needs its block-library macros,
  `GnuRadioBlockLib`.

The family needs libuhd 4.0 or later at the configure. With it, the build
makes the family's block library and adds the cases in `test/`. The cases open
no device. `CAPTURE_HW_UHD=ON` adds the cases that open a radio, labeled
`hardware`; `CAPTURE_HW_UHD_ARGS` in the environment selects the unit they
open.
