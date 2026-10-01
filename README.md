<!-- Copyright 2026 Jeff Long; SPDX-License-Identifier: MIT -->
# gr4-uhd

USRP blocks for GNU Radio 4 over UHD. The package builds a block library that
GNU Radio 4 loads as a plugin, with two blocks:

- `capture::uhd::Source` receives complex samples from a USRP.
- `capture::uhd::Sink` transmits complex samples through a USRP.

It also builds a small discovery library that lists the attached USRPs and
states the control surface of one of them, for a program that holds no block
type.

A copy script produces this tree from libcapture. `uhd/` holds the blocks,
their registration header, their cases and their `CMakeLists.txt`. `common/`
holds the headers the blocks build on, the discovery library's source, the
case support and `cmake/capture.cmake`, the build logic `uhd/CMakeLists.txt`
calls. `extract/uhd.imported` lists every file the copy placed; a change to
one of them is made in libcapture. This copy is of libcapture's master branch
as of 2026-09-30, at "Bound the USRP sweep arms' delivery gaps by the gap
watch's threshold", its last change to these files.

## Building

The build needs a C++23 compiler, CMake 3.28 or later, UHD 4.0 or later, and a
GNU Radio 4 installation with its block-library CMake package. Point
`CMAKE_PREFIX_PATH` at the GNU Radio 4 prefix and install into the same
prefix:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/opt/gnuradio4 \
      -DCMAKE_INSTALL_PREFIX=/opt/gnuradio4
cmake --build build
cmake --install build
```

The block library must be built against the GNU Radio 4 it is loaded into.

The cases need Boost.UT, a single header. `GR4UHD_UT_INCLUDE_DIR` names the
directory that holds `boost/ut.hpp`. The offline cases ask no device:

```sh
ctest --test-dir build -LE hardware
```

`-DGR4UHD_ENABLE_HARDWARE_TESTS=ON` adds the cases that open a USRP, labeled
`hardware`. `CAPTURE_HW_UHD_ARGS` in the environment selects the unit.

## Loading the blocks

`cmake --install` puts `libgr4-uhd-blocks.so` in the prefix's
`lib64/gnuradio-4/plugins` (the prefix's library directory), where the GNU
Radio 4 tools of that prefix look. A build tree is loaded by naming its plugin
directory:

```sh
grinfo blocks --plugin-dir build/lib/gnuradio-4/plugins
grinfo block capture::uhd::Source --plugin-dir build/lib/gnuradio-4/plugins
```

`GNURADIO4_PLUGIN_DIRECTORIES` names the same directory for every tool. The
tools of a prefix outside the system library path also need that prefix's
library directory on `LD_LIBRARY_PATH`.

Where the GNU Radio 4 core offers its device listing registry, the block
library also registers the USRP family's listing when it loads, and `grinfo`
lists the attached USRPs by label and selector:

```sh
grinfo devices --plugin-dir build/lib/gnuradio-4/plugins
```

The listing opens no unit, and it broadcasts on every interface UHD reaches.
The configure compiles and links a probe of the registry against the core it
found; against a core without it the library registers its blocks alone.

A receive graph for `rungraph`:

```yaml
blocks:
  - id: capture::uhd::Source
    parameters:
      name: src
      device_parameter: addr=192.168.10.2
      sample_rate: 1000000.0
      frequency: !!float64 [915000000.0]
      rx_antennae: [RX2]
  - id: gr::blocks::testing::NullSink<complex<float32>>
    parameters:
      name: sink
connections:
  - [src, out, sink, in]
```

## The device selector

Each block's `device_parameter` setting takes UHD device arguments:
`addr=192.168.10.2`, `type=b200,serial=31A4F21`, and so on. An empty value
lets UHD take the first device it finds. The `device` string a discovery
listing gives for a unit is a valid `device_parameter`.

## The controls, control and sensors properties

Each block answers three properties by message while it runs. A program
writes a message to the scheduler's message input with the block's unique
name as the service and the property as the endpoint. The reply arrives on
the scheduler's message output under the same endpoint and client request id.

- `controls` answers Get with `{controls: [descriptor, ...]}`, the control
  surface of the open device at the rate and frequency in force. Subscribe
  sends the same body again each time the surface changes.
- `control` answers Set of `{id, value}` with `{id, value, requested,
  read_back}`. `value` is the value in force after the write where
  `read_back` is true, and the request, bounded to the control's domain,
  where it is false.
- `sensors` answers Get with `{sensors: [reading, ...]}`. A request carrying
  `{sweep: "full"}` reads every sensor the device has. The first reading is
  the stream state, `rx_stream_state` or `tx_stream_state`, whose `good` is
  true while the device streams. The source also states
  `rx_analog_bandwidth`, the analog filter in force in hertz.

A source whose frontend carries an automatic gain control states `GAIN_MODE`,
a toggle at the `gain_mode` the block holds, and a `control` write of it
stages `gain_mode`.

A block whose unit names more than one connector states `ANTENNA`, an
enumeration of the unit's connectors whose value is the index of one. A
`control` write of it stages `rx_antennae` on the source and `tx_antennae`
on the sink. The reply carries the requested index with `read_back` false,
and the connector reaches the device at the block's next settings pass. The
source refuses an `rx_antennae` name the unit does not list, names the
listed ones on standard error and keeps the connector in force.

A descriptor carries `id`, `label`, `unit`, `kind`, `min`, `max`, `step`,
`default`, `options`, `is_gain` and `applies_at` among its keys.
`applies_at` is `runtime` for a write the running stream takes and `start`
for one the device takes at the next start: the source's `WIRE_FORMAT`,
`CLOCK_SOURCE`, `TIME_SOURCE` and `MASTER_CLOCK`. A reading carries `id`,
`label`, `value` and `good`. `capture/common/Properties.hpp` holds the
functions that read and write these maps.

## The discovery library

`capture/common/DiscoveryLibrary.hpp` declares three functions in
`capture::discovery`, and `libgr4-uhd-discovery.so` defines them over the
USRP family:

- `families()` names the families the library carries, here `uhd` with its
  receive and transmit blocks. It asks no device.
- `listDevices()` lists each attached USRP with its family, its label and
  its `device` selector. The listing broadcasts on every interface UHD
  reaches. With `CAPTURE_UHD_USE_DPDK` set in the environment, it asks UHD
  with `use_dpdk=1`, and lists a radio reached only through DPDK as well.
- `probeControls(family, transmit, ctx)` opens the device `ctx.deviceParams`
  names, at `ctx.sampleRate`, and returns the control surface a running
  block states on its `controls` property, or an error that names the
  cause. A device a running block holds is asked through that block's
  `controls` property instead.

```cmake
find_package(gr4-uhd CONFIG REQUIRED)
target_link_libraries(myprogram PRIVATE gr4-uhd::discovery)
```

`find_package(gr4-uhd)` gives three targets:

- `gr4-uhd::headers` carries the installed include directory, C++23 and
  Threads, and neither GNU Radio 4 nor UHD: a program that compiles the block
  headers links those two itself. Where the configure found a GNU Radio 4
  core that offers `gr::block::describe()`, it also carries
  `CAPTURE_BLOCK_LABELS`, so a program that compiles a block header sees the
  block type the installed block library was built with. Where the core offers
  the device listing registry, it carries `CAPTURE_DEVICE_LISTINGS` as well.
- `gr4-uhd::blocks`, the block library, carries GNU Radio 4 and
  `gr4-uhd::headers` and links UHD privately.
- `gr4-uhd::discovery`, the discovery library, carries `gr4-uhd::headers` and
  links UHD and GNU Radio 4 privately.

```cpp
#include <capture/common/DiscoveryLibrary.hpp>

for (const auto& unit : capture::discovery::listDevices()) {
    std::println("{}  {}", unit.label, unit.device);
}
const auto surface = capture::discovery::probeControls(
    "uhd", false, {.deviceParams = "addr=192.168.10.2",
                   .sampleRate = 1.0e6, .centerFreq = 915.0e6});
```

## Current GNU Radio 4 releases

Two behaviors of GNU Radio 4 as released today reach these blocks:

- A list setting in a graph file, such as `frequency`, `rx_gains` or
  `tx_gains`, loads only when tagged `!!float64`, as in the graph above. An
  untagged list is refused as "not a tensor of float64".
- `rungraph --set` stages its values after the graph has started its blocks'
  settings, and a block applies them at its first work call, after its
  start. A setting the blocks read when they open the device, such as
  `device_parameter`, keeps the graph file's value. Give such settings in
  the graph file until `rungraph` applies `--set` before the start.

## License

gr4-uhd is licensed under the MIT license. Each file carries its own copyright
line and SPDX-License-Identifier header, and `LICENSE` carries the license text.

gr4-uhd links libuhd. libuhd's own license, GPL-3.0-or-later, governs the
distribution of a binary that includes it, and a packager accounts for that
when shipping such a binary. The source files here stay MIT.
