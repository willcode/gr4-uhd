<!-- Copyright 2026 Jeff Long; SPDX-License-Identifier: MIT -->
# capture/uhd — the USRP source and sink

The family contracts are in `capture/common/KNOWLEDGE.md`. This file holds the USRP's own facts.

**purpose**
`capture::uhd::Source` and `capture::uhd::Sink` drive a USRP through UHD 4's `multi_usrp`
interface, its property tree and its metadata error codes. The two are peers.
`capture::uhd::Device.hpp` holds, once, every body the two share and every body that
describes the device rather than one direction, each taking a `Direction` where the two
halves differ: the truth record and the one reader that fills it, the probe that opens a unit
to run that reader, the device listing and the device-string test, the address-key writer,
the reference and timing writes, the unchanged-write test, the wire-format list, the
read-back carrier and the tune test, the master-clock read taken as two calls, the clock a
device fixed at the open, the rate ladder and the link-rate readers, the test whether a rate
write settled the rate and whether the filter follows it, the gain-element and filter
descriptors with the element-name check, the gain bound and the line naming a moved control
value, the filter and coverage appliers, the test for a stop that arrived during the open,
the reference-lock wait with the live predicate the waits default to, and the sensor sweep. A name a consumer
calls on a block — `sampleRates`, `sampleRatesFor`, `optionIndexFor`, `probeTruth`,
`enumerateDevices`, `describeControls`, `SensorSweep` — stays on it and forwards to the
shared body.

**algorithm**
The sink's drain is the block's own thread, blocking in `send()` at the sample rate, and what
it places is a `tx_streamer` burst of CF32. The work call clips each sample into the queue in
one pass, the only pass before the send, and a send reads the queue in place: the whole
contiguous span it holds, never across its end and at most 10 ms of samples at the rate in
force in whole frames, one frame at the least. UHD cuts the span into frames itself. The
queue stays because `send()` blocks while the device's buffers are full, and the work call
runs on the scheduler's thread with the other blocks of its job. The drain's one timed wait
is the send's own timeout: on an empty queue or a park it waits on a doorbell the work call
rings after each write and a pause, a resume, a rate change and a stop ring after they change
its state. The SAMPLE_RATE descriptor of either block is the ladder of the master clock and the
wire format the block holds, anyInRange where the clock is programmable, and FREQUENCY is the
tuner coverage the truth states. An unstarted block's controls property probes the unit, which
opens it for the length of the probe. The transmit control surface is probed from the
device, as the receive one is, by the one reader both blocks call with their own direction, and
`setElementGain()` writes one element of a distributed gain. The connector control of either
block, and the overall gain `setElementGain()` takes for an empty name in either block, write
through their settings, `rx_antennae`, `tx_antennae`, `rx_gains` and `tx_gains`, and reach the
device at the next settings sweep, so a replay of the settings map carries the value a control
chose. Either block states its connector control where the unit names more than one connector,
one option per name. The source's default is the connector in force, and a landed write moves it
and sends the moved surface to the controls subscribers from the control thread; the sink's
default is the connector the open found. Where the unit states a list, the source refuses an
`rx_antennae` name outside it before any device call, names the listed ones on standard error and
leaves the setting holding the refused name; a unit that states no list has the name written. Every read the shared reader makes but the
firmware and FPGA version pair is guarded alone, so a model that refuses one still states the
rest: the range of the master clock and the clock in force, a filter range and the width in
force, a list of sources and the source in force, and each gain element apart from the others.
Both records state the link rate UHD gives, one link carrying both directions; the
receive record carries three fields more, the three optional frontend corrections, which the
receive frontend alone states. The link rate is the double UHD keeps at
`/mboards/0/link_max_rate`, and the ceiling it sets is that figure over the wire bytes per
sample, the comparison UHD's own `multi_usrp` makes when it builds a streamer, with nothing
taken off. UHD 4.9 creates the node as a double for a USRP1, a B100, a B2xx and an N2xx, from
each driver's own constant. An X3xx creates none. A device an MPM daemon runs creates it as a
`size_t` holding 125 MB/s whatever its link, an X4xx on a 100 Gbit link included, and no check
of UHD's reads it. Neither states a link here: the ceiling is the radio's own fastest rate,
and the line naming the unit says the link rate is unknown and names that ceiling. A unit
whose link UHD states carries the figure in the same line. The selector, its address and the
transport it names decide nothing.

The source's consumer thread drains the staged settings, receives and publishes. Each receive
lands in a reservation on the output edge and asks for whole frames of the streamer's own size,
2040 samples on a B205mini and 363 on an N210: the room the edge has, capped at a quarter of the
edge and at 20 ms of the rate in force, and never below one frame. Below one frame of room the
thread waits on the graph's progress counter, which the reader moves when it takes samples, for
at most the time the stream takes to fill half the edge and at most 0.1 s; it then applies
staged settings and asks again. A receive of whole frames ends on a frame boundary, so each
receive carries the stamp of its own first sample. The cap at a quarter of the edge lets receives
land while the reader takes the rest, and the thread then seldom waits for room. A receive of the
whole edge leaves no room for the next until the reader has taken everything, and on a B205mini
at 8 MS/s into a 64 Ki-sample edge the transport then overflows and loses more than half the
stream. At 61.44 MS/s into a 64 Ki-sample edge a quarter-edge receive waited for room 0 times a
second against 160 for a half-edge receive; the overflows of the two differ by less than their
spread from run to run. A fixed sleep in place of the wait moves at most one edge per sleep once
the edge fills: at 61.44 MS/s into a 4 Ki-sample edge a 200 µs sleep lost 6 to 11 percent of the
stream, and the wait 0.1 to 0.2 percent. The samples of a receive reach the edge when the reservation's span is
released at the end of the pass, and the next pass moves the progress counter, so a reader it
wakes finds them. On an edge whose size the framework did not round to a page, the release also
copies the receive into the buffer's mirror: 0.43 to 0.66 ms for a 375,000-sample receive into a
1,500,000-sample edge at 30 MS/s. In ten 20 s runs at 30 MS/s with UHD's default transport, no
pass spent 1 ms outside the receive and the wait, and none overflowed. Each receive does no locked operation that its case does not need: the
restage flag is read before it is exchanged, the shed count is added only when it is not zero,
the shortfall test reads the clock only once the stream has delivered what its window holds at
the rate or after a gap, and the time of the first sample, with its device-stamp conversion, is
read only when the block emits timing tags; the wall clock is read only for a block with no
device time. The frequency a landed tune holds is written back into the setting on this thread,
the one thread that writes the block's settings. While the
stream runs, a frequency, a gain, an automatic gain mode and an antenna reach the device on a
control thread of the block's own, one write at a time in the order given, the last write of
a key replacing one not yet begun. A rate change runs on the consumer thread once that thread
has settled, because it stops the stream, empties the transport and starts it again. The wait for
the control thread ends after 1 s, and the rate is then written behind the write in flight, the
two serialized on the control mutex. The drain of the stopped stream, at a rate change and at a
stop, ends on the stream's end-of-burst marker, on a receive that waits 50 ms and finds nothing,
or after 0.3 s. Both bounds are durations, so a drain at 2 GS/s takes what the transport holds as
one at 250 kS/s does; a count of receives stops at the same number of samples at every rate. A
B205mini going from 61.44 to 30.72 MS/s drained 14,147 samples in 0.07 ms and an N210 going from
25 to 12.5 MS/s 7,647 samples in 0.11 ms, each ended by the marker; a teardown drained 9,305 and
3,092 samples the same way.

The source's receive thread and the sink's transmit thread take `thread_priority` and `cpu` where
the caller sets them, at the thread's start: a priority above 0 and at most 1 asks for the
real-time round-robin class at that fraction of its range, and a cpu of 0 or more pins the thread
to that processor. Unset, the thread keeps the class and the processors the system gave it, the
process's fast-core narrowing included. The system refuses the real-time class without
`CAP_SYS_NICE` or an `RLIMIT_RTPRIO` above zero, and a processor outside the process's set; a
refusal is a line on standard error and the start goes on. The control threads and the sink's
asynchronous thread keep what the system gives them.

The transport's depth is the caller's: the blocks pass `num_recv_frames`, `recv_buff_size` and the
frame sizes through and choose none. The source's reading `rx_transport_depth`, beside
`rx_overflows`, states the frames in use, the caller's or UHD 4.9's default for the kind the root
of the property tree names (16 on a B2xx or a B100, 32 on an N2xx), the streamer's frame, and the
time the frames hold at the rate in force, and for an N2xx the socket buffer UHD asks for, 50e6
bytes on Linux. It reads "16 frames of 2040 samples, 0.531 ms at 61.44 MS/s, UHD's default" on a
B205mini and "32 frames of 363 samples, 0.465 ms at 25 MS/s" with a socket buffer of 500 ms on an
N210. An X3xx or an MPM device sizes its frames from the link and the socket buffer, and the
reading then states the frame alone.

**guarantees**
- A retune never stops the receive, whether or not a program subscribes to `controls`: the
  consumer thread reads the control surface without waiting for the tune. A B205mini tune
  takes about 100 ms, which the device cannot buffer at 2 MS/s. The frequency tag goes on the
  first sample stamped at or after the device clock read once the tune returned, at its offset
  inside the receive that holds it, and never on a sample taken before the tune. The setting is written back from the newest request alone and
  reaches the parameter map at once. An overflow while a tune is open carries the cause
  `retune`.
- The source measures a shed as the difference between the stamp a block arrived at and the
  stamp expected of it, in the nearest whole number of samples at the rate in force. After a
  live rate change a B2xx can take its samples a fraction of a sample off that rate's grid,
  half a sample at 8 MS/s from 32 MHz, and that fraction is no shed. A shed the stamps measure
  that no overflow, lost packet or rate change recorded is marked with the cause
  `missing samples`, or `retune` while a tune is open.
- A USRP has no register for a frequency correction, so the block pre-compensates one on
  transmit as well as on receive.
- Opening a USRP raises whenever no unit answers the address, and so does parsing an address a
  device string cannot hold, so a start whose open failed is the ordinary start of a transmit
  graph with no radio attached.
- A USRP states one overall gain range and distributes it across the elements itself. Either
  block bounds an overall gain to that range and writes the bounded gain back into `rx_gains` or
  `tx_gains` on the thread that drains its settings, as it writes a refused center back into
  `frequency`. The source's control thread writes the bounded gain to the device.
- A start on a clock source other than internal waits up to 1.5 s for the reference to lock
  before the stream begins, in either direction, and says whether it locked; a reference that
  never locks costs the whole wait and the stream starts anyway. A stop made during the sink's
  wait ends it within one poll, the start then builds no streamer and starts no thread.
- A stop hands the streamer and the device to a releasing thread and returns without waiting
  for the release, which costs 1147 ms on an N210 and 16 ms on a B205mini. A source start that
  fails after its open, a thread the system refuses included, joins its control thread and
  hands the device over the same way before it throws. Every open in the
  process waits for the releases in flight first, the probe's included, because a unit is
  exclusive. A block's destructor does not wait, so a caller that destroys the block at every
  stop stops as fast as one that keeps it; a caller that states the device free waits through
  `Sink::awaitReleases()` first. Every such wait returns only once each release handed off
  before it has ended, one another caller is joining included. The process joins what is left
  at exit, ahead of the vendor library's own teardown. A source and a sink sharing one unit each hand their references
  over, and the unit is released once, after both.
- Both blocks read the device's own sensors through one sweep: the motherboard's under the
  names the unit gives them, and their own frontend's, a transmit frontend's under `tx_` ids,
  because the two frontends of one unit name their synthesizer locks alike. A graph that
  transmits alone sees the reference lock and `tx_lo_locked`. A lock reading counts as failed
  only where the selection in force uses it: `ref_locked` on a clock source other than
  internal, `mimo_locked` on a MIMO clock or time source. A reading that does not apply is
  published as the device states it, marked good and kept off the glance rows. The lock
  readings and the temperature carry a person's label in place of the unit's terse name, and
  the clock and time selections the controls state are no glance rows. Each block reads every sensor once to
  find the cheap ones, about 1 s on an N210 with a GPSDO, whose `gps_time` waits for the next
  time sentence: the receive block in its start on an idle device, keeping a read of up to
  5 ms, and the sink at its first sweep, outside its control mutex and beside a streaming
  drain, keeping a read of up to 50 ms. A transmit start costs the reference wait where one
  applies and nothing for the sensors.
- UHD looks for an N-series GPSDO at the open only while the unit's firmware does not hold its
  do-not-look mark, and UHD writes that mark itself after an open that found no GPSDO or could
  not identify the unit. The mark stays until the unit is power-cycled, and every open until
  then publishes no GPS sensor and offers `gpsdo` as neither a clock nor a time source. Where
  the enumeration succeeds, the unit's EEPROM names a GPSDO and the open published no `gps_`
  sensor, the enumeration says so on standard error and both blocks publish a failed `gpsdo`
  reading in both sweeps, whose id a caller looking for `gps_` readings does not take for a
  GPS sensor. The same state follows a search the GPSDO did not answer and an EEPROM naming a
  GPSDO the unit does not carry, and the line and the reading name the power cycle only for
  an open at which UHD logged no GPSDO detection.
- The transmit rate is chosen as the receive rate is. A fixed clock is divided by a whole
  number in either direction and an odd one runs the CIC alone, so the sink snaps a request to
  the nearest even interpolation the link carries in the wire format in force, and states the
  rate it chose: an N210 asked for 8 MS/s runs 8.333 MS/s, interpolation 12, rather than
  7.692 MS/s on the CIC alone, and one asked for 30.72 MS/s runs 25 MS/s, the highest even
  interpolation under the 31.25 MS/s that UHD's 125 MB/s for the link carries in sc16. A fixed clock is a radio that states exactly one master-clock rate;
  an X4xx under UHD 4.9.0 states the one it was opened at and keeps it while it runs. A
  radio that states several is a variable clock, as a programmable one is. A radio whose
  clock is programmable takes the request as it stands up to the link ceiling UHD states in
  the wire format in force, and the ceiling above it, the snap named on standard error,
  except where the clock is fixed: UHD 4.9's B200 turns its automatic clock off for as long as
  the device stays open wherever the open address carries `master_clock_rate`, and still
  states its whole clock range, so both blocks take the clock the device reads at the open as
  fixed and cut the ladder from it. The device's own `auto_tick_rate` node says whether the
  clock is fixed where the tree carries one, a B2xx doing so, and the open address says it
  otherwise.
- The rate and the analog filter are read back after the write, UHD coercing a request to the
  nearest valid value. A filter write the device refuses leaves the width the block states at
  none until a write lands, in both directions. A rate write whose read-back fails is not
  taken as written: the block states no rate and moves no filter, and the next replay writes
  the rate and reads it again.
- A USRP's analog filter sits ahead of a digital converter that filters to the rate in either
  direction, so the images of the rate never reach it. The block offers the filter across the
  whole range the frontend states at any rate, and the rate applier asks for the rate clamped
  into that range, which is wider than the rate below the range's bottom, 200 kHz on a
  B205mini. A WBX's filter is 40 MHz at every rate and takes no write.
- A USRP states its coverage as the tuner's range widened by half the analog filter, within the
  digital converter's reach, and UHD clips a request outside it to the nearest edge. A B205mini's
  tuner stops at 50 MHz, and at 2 MS/s it reaches 49 MHz on the 2 MHz filter a block writes and
  34 MHz on the 56 MHz filter it comes up with. Both blocks test a tune against the coverage read
  at the rate and the filter in force, read it again after a filter write, and state it in the
  truth record; the probe writes the rate and the filter the block will write before it reads.
- The transmit queue's floor is the larger of eight of the streamer's own maximum send and
  4096 samples. The floor binds at and below four times that floor, which is 65280 S/s for a
  B205mini's 2040 samples and 16384 S/s for a streamer whose eight sends stay under 4096; a
  USRP runs down to a fifth of a megasample, above both crossings, so the quarter-second window
  holds at every rate one of these radios runs. 2^19 samples would be 2.6 s of queue at that
  fifth of a megasample, and every wait the block makes is sized from the queue.
- The sink's `queue_duration` and `queue_max_bytes` state the queue's bound, a quarter of a
  second and 4 GiB by default. The start and each rate change read them. The storage a rate
  change needs is allocated before the control mutex is taken, from the rate the request snaps
  to, and the storage it replaces is freed after the mutex is let go; a device that coerces
  the write to a rate of another power of two makes the resize allocate under the mutex. The
  start sizes the queue outside the mutex, before the drain runs.
- A play-out waits for the queue bounded by the capacity the rate in force asks for, which is
  under 750 ms for a full queue at every rate a USRP runs. A drain that refused the park keeps
  the queue's sample count rather than its length of time, so a block taken from a fast rate to
  a slow one holds a queue far longer than the new rate's own quarter second until that drain
  comes back. The drain asks for the park again at the head of its own loop while the queue
  outsizes the rate in force, and the work call takes the resize behind that answer, counting
  what it throws away under the rate change the park was refused for and saying so once. The
  bound holds every wait made in between inside that quarter second and its margin.
- The transmit queue is CF32, eight bytes a sample: 128 MiB at 61.44 MS/s and 4 MiB at 2 MS/s.
  A resize writes no element of it. Sizing 128 MiB takes about 0.02 ms on the bench host,
  and a zero-fill took 75 ms.
- The drain places the queue itself, so a stop waits for the queue alone and a stop asked for
  while the carrier is held ends at once, the parked drain taking nothing out of the queue.
- A pause ends the burst, which keys the radio down, and keeps the queue. It throws nothing
  away and the count of what a pause discarded stays at zero; a stop after one counts what is
  left as unsent. The block goes on consuming under a pause: a work call stages into the
  queue until it is full, and the resume sends what the queue holds.
- A rate change made while the carrier is held finds the queue still full and the parked drain
  placing nothing, so the play-out returns at once and the resize throws the whole queue away
  under the rate change's own count.
- A hole in a burst keys the radio down: the device transmits nothing and reports an event
  carrying no length, so the block counts the events rather than samples. The device's codes
  are bit flags, one message carrying several, and each is tested alone. A packet lost on the
  link is counted apart from a host that stopped feeding the radio, as on receive, and a packet
  that arrived after the time it carried is counted as late.
- One thread sends at a time on a streamer, and the thread reading the device's asynchronous
  channel is started after the streamer and joined before it goes. An end of burst the
  streamer did not place leaves the burst flag up and says so, because the radio stays keyed.
- A stop waits for the device to acknowledge the end of burst it placed, because the
  acknowledgment is the one statement that the tail left the radio. The device acknowledges
  the ends in the order they were placed, so the stop waits for the count of acknowledgments
  to reach the count of ends placed in the run: a pause's or a rate change's acknowledgment
  arriving during the stop answers for that earlier burst alone. The wait is bounded by the
  play-out of what the device can hold: the `send_buff_size` the open address states, which
  an N-series takes as its flow-control window, and 1 MiB where it states none, about a second
  at 250 kS/s in sc16. UHD states no device transmit buffer in its property tree or on a
  streamer. The block states whether the acknowledgment came or the end could not be
  placed at all, and a stop that gave up or left the carrier up states what the device can
  still hold as a bound beside the counts.
- The USRP source and sink read their non-gain controls back through `controlValue`.
- The source's `WIRE_FORMAT`, `CLOCK_SOURCE`, `TIME_SOURCE` and `MASTER_CLOCK` state that a
  write applies at the next start: the block keeps the write, and the start alone reads the
  wire format, the reference and timing selections and the converter clock. An explicit clock
  also reaches the running device at once and moves the rate with it, except on a radio that
  states one clock; the automatic entry waits for the next open. Every other control of either
  block applies in the running stream. A write that changes the setting a kept write shadows
  (`wire_format`, `clock_source`, `time_source`, `master_clock_rate`) drops that kept write,
  and a change of `device_parameter` to another value drops all four. A write of the value a
  setting already holds reaches no settings hook and drops nothing.
- The truth records whether the master clock range UHD states is one continuous span: a
  single sub-range whose start is below its stop. On such a radio `MASTER_CLOCK` is
  `anyInRange`, with `min` and `max` the span in MHz, and a clock between two entries is
  written as it stands. Separate clocks or one sub-range of a single point leave it false, and
  the list then takes its entries alone: the bound refuses any other value, names the entries
  on standard error and moves nothing onto an entry, so a clock stored under another radio is
  never written as the nearest clock of this one. A clock written by hand makes the truth a
  fixed clock with no span until the next open.
- A radio that states separate clocks is offered those clocks, and each joins the rate offer as
  the rate it reaches at decimation 1, cut at a link ceiling UHD states. A span is offered the
  picks inside it: 5 to 61.44 MHz for a B2xx or an E3xx converter, and above that the clocks UHD
  4.9's X3xx clock driver has a mode for (120, 184.32 and 200 MHz, `x300_clock_ctrl.cpp`), the
  clocks an N300 or N310 takes (122.88, 125 and 153.6 MHz, `MAGNESIUM_RADIO_RATES`) and those an
  N320 takes (200, 245.76 and 250 MHz, `RHODIUM_RADIO_RATES`).
- UHD 4.9's own X3xx, N3xx and X4xx radios state the one clock they were opened at, so each is a
  fixed clock. Its rate offer climbs past the last static pick, 25 MS/s, by the decimations of a
  power of two up to the ladder's top: 50, 100 and 200 MS/s on a 200 MHz clock UHD states no link
  for. An X3xx and an X4xx keep that clock for the whole session; an N3xx can change it while
  open, but still states the one clock.
- A table keyed by the name `get_mboard_name` returns gives the clocks such a radio takes at the
  open, one set per kind of FPGA image, from the UHD 4.9.0.1 manual: 184.32 and 200 MHz for an
  X300, an X310 or an NI-2974, which `x300_mboard_type.cpp` classes as an X310 motherboard;
  122.88, 125 and 153.6 MHz for an `n300` or `n310`; 200, 245.76 and 250 MHz for an `n320`,
  which an N321 also states; 245.76 and 250 MHz, or 491.52 and 500 MHz, for an `x410` by image;
  and for an `x440` the validated clocks of its xx_200, xx_400 and xx_1600 images.
  `MASTER_CLOCK` offers every clock of the sets that hold the clock in force, after the
  automatic entry, and its label names the clock in force. An X440 at 125 MHz, a clock every
  image takes, is offered the clocks of all three. An X3xx whose frontend UHD names as a TwinRX
  is offered 200 MHz alone: the manual allows no other clock with it, and at 184.32 MHz the
  daughterboard clock is not one the TwinRX takes.
- A pick on such a radio is an entry of that list, pinned and carried as `master_clock_rate` by
  the next open. Nothing is written to the running device, and the rate offer stays the ladder
  of the clock in force until that open, because UHD states a fixed clock's rates for the clock
  it runs alone. An open that fails, for any reason, while it carries a pin this list made
  drops the pin and names it in the failure, so the next start opens with the
  `master_clock_rate` setting. A pin a live write put on a span radio's running device stays
  through a failed open: the device ran that clock, and an unplugged or busy unit is no cause
  to forget it. An open whose device runs another clock than the pin names the difference and
  one cause of it: UHD hands a second open in one process the device already open, with the
  first open's keys. A driver coercing a clock inside its range gives the same difference. A radio stating one clock that the table
  does not hold for its product is offered no clock control.
- The table leaves out 120 MHz on an X3xx: UHD 4.9.0.1's device arguments take 184.32 to 200
  MHz and refuse it at the open. It leaves out the X410 clocks MPM's clock policy takes beyond
  the manual's: 122.88 and 125 MHz and a 200 MHz legacy clock. A TwinRX behind a channel other
  than the one read is not seen. The table is a copy of UHD's facts and drifts where a later UHD
  changes them; a clock it offers that the UHD in use refuses fails the next open, which then
  drops it.
- The device list asks libuhd with the empty hint, or with `use_dpdk=1` where the caller asks
  for DPDK or `CAPTURE_UHD_USE_DPDK` is on in the environment. UHD 4.9.0 starts DPDK for an
  MPM device or an X3xx over Ethernet only for a hint that carries the key, so a radio reached
  only through DPDK is listed only then. No search filters a unit on the key.

**fails**
UHD's transport teardown raises `uhd::exception` for a device that has gone. The teardown
guard catches it, so a destroyed block leaks the handle and the process goes on.

**domain**
Two USRPs share a vendor library and nothing else: one names a single gain element and the
other three, one has an automatic gain loop and the other none, one programs its converter
clock per rate and the other divides a fixed clock, one carries a filter from 200 kHz to
56 MHz and the other a fixed 40 MHz, and one keeps its correction enables under the
daughterboard while the other keeps them under the converter core.

A full sensor sweep waits on a disciplined oscillator's serial link for most of a second. The
transmit queue at 61.44 MS/s is 128 MiB of CF32, and a rate change down replaces the storage,
which gives that memory back.

The USRP families UHD 4.9.0.1 builds support for, the links each offers, the fastest rate per
channel, the channel count, and the link rate UHD states in `/mboards/0/link_max_rate`. A
letter names the source below the table.

| family | links | fastest rate per channel | channels | `link_max_rate` |
|---|---|---|---|---|
| USRP1 | USB 2 | 64 MHz fixed clock (a); 8 MS/s sc16, 16 MS/s sc8 (a) | 2 RX, 2 TX; 4 RX image (a) | double, 32 MB/s (h) |
| B100 | USB 2 | 32 to 64 MHz clock (b); 8 MS/s sc16, 16 MS/s sc8 (b) | 1 RX, 1 TX (b) | double, 32 MB/s (h) |
| B200, B210, B200mini, B205mini, B206mini | USB 3 or USB 2 (c) | 61.44 MS/s on one channel, 30.72 on two (c, i) | 1x1; B210 2x2 (c) | double, 500 MB/s on USB 3, 53.248 MB/s on USB 2 (h) |
| E310, E312, E313 | on-device DMA; 1 GbE RJ-45 at 4.4 MS/s (d) | 61.44 MS/s clock (d, j) | 2 (d) | size_t 125 MB/s for every unit, never compared (h) |
| E320 | SFP+ 1 or 10 GbE; 1 GbE RJ-45 at 6.4 MS/s (d) | 61.44 MS/s clock (d, j) | 2 (d) | size_t 125 MB/s, never compared (h) |
| N200, N210, USRP2 | 1 GbE only (e) | 100 MHz fixed clock; 25 MHz sc16, 50 MHz sc8 (e) | 2 RX DDC, 1 TX DUC (e) | double, 125 MB/s (h) |
| N300, N310 | 2 SFP+, 1 or 10 GbE (f) | 122.88, 125 or 153.6 MHz clock (f, j) | N300 2, N310 4 (f) | size_t 125 MB/s, never compared (h) |
| N320, N321 | 2 SFP+, 1 or 10 GbE; QSFP+ 2x 10 GbE (f) | 200, 245.76 or 250 MHz clock; up to 250 MS/s (f, j) | 2 (f) | size_t 125 MB/s, never compared (h) |
| X300, X310 | 2 SFP+, 1 or 10 GbE; PCIe gen1 x4 over MXI (g) | 200 or 184.32 MHz clock; 200 MS/s over 10 GbE (g) | 2 (g) | none (h) |
| X410 | 2 QSFP28, 4x 10 GbE or 100 GbE; 1 GbE RJ-45 (k) | 245.76 or 250 MHz clock on the 200 images, 491.52 or 500 on the 400 image (k) | 4 (k) | size_t 125 MB/s, never compared (h) |
| X440 | as X410 (k) | 125 MS/s to 2.048 GS/s by image; 500 MS/s on the 400 images, 2.048 GS/s on the 1600 images (k) | 8; 2 on the 1600 images (k) | size_t 125 MB/s, never compared (h) |

Sources, all at UHD tag v4.9.0.1: (a) `host/docs/usrp1.dox`; (b) `host/docs/usrp_b100.dox`;
(c) `host/docs/usrp_b200.dox`; (d) `host/docs/usrp_e3xx.dox`; (e) `host/docs/usrp2.dox`;
(f) `host/docs/usrp_n3xx.dox`; (g) `host/docs/usrp_x3x0.dox`; (h) the node's creator in
`host/lib/usrp`: `usrp1/usrp1_impl.cpp`, `b100/b100_impl.cpp`, `b200/b200_impl.cpp`,
`usrp2/usrp2_impl.cpp` and `mpmd/mpmd_prop_tree.cpp`, and no creator under `x300/`;
(i) `b200/b200_impl.cpp`, the master clock limit halved for two channels; (j) the MPM
daughterboard managers `mpm/python/usrp_mpm/dboard_manager/ad936x_db.py` (220 kHz to
61.44 MHz), `magnesium.py` and `rhodium.py` (the clock rates each accepts); (k)
`host/docs/usrp_x4xx.dox`. The component list in `host/lib/CMakeLists.txt` names every family
above.

The four kinds that create the node as a double are the four whose streamer compares it: the
USRP1, the B100, the B2xx and the N2xx. Each figure is the line's own rate. UHD keeps per-link
figures for the other kinds inside the transport and puts none of them in the tree: an X3xx's
Ethernet manager holds a 1 and a 10 GbE figure for buffer sizes, and an MPM link reports its
own `link_rate`, 1 GbE where it states none.

A B205mini at 61.44 MS/s fed at the wall clock's pace in 16384-sample chunks took 30,100 sends
a second one frame at a time and 140 to 620 over the span. The drain's CPU time held at 4.0 to
4.2 ms per million samples either way: UHD's conversion and its transport cost that, not the
block. The device reported 14 to 61 underruns in 20 s in both arms, following the feed's own
stalls.

**rejected**
Leaving the transmit bound to the device library. UHD's `fc32` to `sc16` conversion saturates
on the SSE path and casts on the generic one.
