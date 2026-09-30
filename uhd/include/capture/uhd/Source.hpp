/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/Block.hpp>
#ifdef CAPTURE_BLOCK_LABELS
#include <gnuradio-4.0/BlockAttributes.hpp>
#endif
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Source.hpp>
#include <capture/uhd/Device.hpp>

#include <uhd/device.hpp>
#include <uhd/exception.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp/subdev_spec.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <format>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <source_location>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace capture::uhd {

/*| role: the USRP source for Ettus radios, over libuhd.
    contract: the settings below, receives made into the output edge itself, and a
        device-truth control surface, probed from the device rather than read from a static
        table, because USRP models differ in gain elements, antennas and coverage.
    frame: UHD's rx_streamer::recv() is a pull API and multi_usrp control calls are safe from
        another thread while recv() runs, so this block needs no callback-and-ring split: one
        consumer thread drains staged settings, receives whole frames into a reservation on the
        output edge and publishes it, and a control thread of the block's own makes the control
        calls those settings ask for while the stream runs.
    invariant: the work thread never waits on a device call that another thread makes, apart
        from a change of the device's clocking. A settings pass stages the tune, the gain, the
        automatic gain mode and the antenna on the control thread, and its surface check reads
        the surface values from a snapshot rather than under _ctrlMutex. The exceptions are
        these: a rate change settles the control thread and writes the rate itself behind a
        marked break; the rate a master-clock write restages takes _ctrlMutex; so does the
        clock offset read once a device-clock set latches; and so does a pass carrying
        device_parameter, master_clock_rate, wire_format, clock_source or time_source, which
        clears the pins those settings shadow and waits behind a tune in progress.
    verified-by: controls.uhd-a-settings-pass-never-waits-on-a-tune
    invariant: control calls are serialized by _ctrlMutex, which the sensor sweep deliberately
        does not hold across its device reads: one sweep takes the better part of a second on a
        GPSDO, and a mutex held that long on a repeating timer leaves every tune and gain write
        waiting for it.
    invariant: start() owns the device handle alone and reads it without that mutex. The
        framework holds one mutex across a block's whole start sweep and the same one across its
        stop sweep, so the two hooks never run together, and _deviceUp stays false until the
        last line of start(), which keeps every other holder out. Every holder after that
        line takes the mutex and re-tests _deviceUp under it.
    trap: UHD throws on errors and every call here is wrapped. A transient control failure logs
        and continues, because emitErrorMessage() from a settings handler kills the graph; only
        a start() failure is fatal, and a dead stream surfaces through the work() liveness check
        after at least 5 s of receives that delivered nothing, whatever each one's error arm
        said. At least, rather than about: the recv timeout applies to every single call inside
        recv() and not to the whole of it, so fifty receives at 0.1 s each is the floor and the
        real deadline is longer.
    why: USRPs have no fractional-ppm register, carrying a TCXO or a GPSDO instead, so ppm is
        applied as a scaled tune through capture::correctedTuneHz, which states the convention.
        The sign is verified over the air: a positive setting says the reference runs fast.
*/
struct Source : gr::Block<Source> {
    using Description = gr::Doc<R"(USRP source via libuhd.
fc32 host samples over the sc16 or sc8 wire format. The device list asks libuhd with
use_dpdk=1 where CAPTURE_UHD_USE_DPDK is set in the environment.)">;

    gr::PortOut<std::complex<float>> out;

    static constexpr const char* kDevice = "uhd"; // the family name a device listing and a probe carry
    static constexpr bool        kListingOpens = false; // the listing asks libuhd, which opens no unit
    static constexpr double      kDefaultRateHz      = 2'000'000.0;
    static constexpr double      kDefaultFrequencyHz = 100'000'000.0;
    /*| why: the clock_source and time_source settings take a 10 MHz reference and a PPS from
            outside the unit. The block declares ingests/time for them. It leaves the unit's
            reference and PPS outputs as the unit sets them, and declares no emits/time.
    */
#ifdef CAPTURE_BLOCK_LABELS
    static constexpr auto attributes = gr::block::describe(1U, gr::block::labels::family(kDevice, "USRP radios driven through libuhd."),
        gr::block::labels::role::source, gr::block::labels::holds::device, gr::block::labels::ingests::rf, gr::block::labels::ingests::time);
#endif
    gr::Annotated<std::string, "device", gr::Visible, gr::Doc<"informational; \"uhd\"">>                          device = std::string(kDevice);
    gr::Annotated<std::string, "device_parameter", gr::Visible, gr::Doc<"UHD addr kwargs; serial=<s>,type=b200">> device_parameter;
    gr::Annotated<double, "sample_rate", gr::Unit<"Hz">, gr::Visible, gr::Doc<"requested rate (snapped by UHD)">>  sample_rate = kDefaultRateHz;
    gr::Annotated<std::vector<double>, "frequency", gr::Unit<"Hz">, gr::Visible, gr::Doc<"center frequency">>     frequency = std::vector<double>{kDefaultFrequencyHz};
    gr::Annotated<std::vector<double>, "rx_gains", gr::Unit<"dB">, gr::Visible, gr::Doc<"overall RX gain">>       rx_gains  = std::vector<double>{35.0};
    gr::Annotated<bool, "gain_mode", gr::Doc<"hardware AGC on/off">>                             gain_mode = false;
    gr::Annotated<double, "frequency_correction", gr::Unit<"ppm">, gr::Doc<"applied as a scaled tune">>           frequency_correction = 0.0;
    gr::Annotated<std::vector<std::string>, "rx_antennae", gr::Doc<"receive connector name">>                     rx_antennae;
    gr::Annotated<bool, "emit_timing_tags", gr::Doc<"emit wall-clock timing tags">>                               emit_timing_tags = true;
    gr::Annotated<float, "tag_interval", gr::Unit<"s">, gr::Doc<"minimum interval between timing tags">>          tag_interval     = 1.0f;
    gr::Annotated<std::string, "trigger_name", gr::Doc<"tag trigger_name">>                                       trigger_name     = std::string("SDR_WALLCLOCK");
    // The nine settings below are read by start() alone; withDeviceArgs states why.
    gr::Annotated<std::string, "wire_format", gr::Doc<"'sc16' or 'sc8'">>                                         wire_format       = std::string("sc16");
    gr::Annotated<std::string, "clock_source", gr::Doc<"reference to discipline the device by; empty keeps its own">> clock_source;
    gr::Annotated<std::string, "time_source", gr::Doc<"source of the timing edge; empty keeps its own">>              time_source;
    gr::Annotated<double, "master_clock_rate", gr::Unit<"Hz">, gr::Doc<"0 lets UHD choose">>                      master_clock_rate = 0.0;
    gr::Annotated<double, "recv_frame_size", gr::Unit<"B">, gr::Doc<"0 lets UHD choose">>                         recv_frame_size   = 0.0;
    /*| contract: the transport's depth is the caller's to set, and zero leaves it to UHD. The
            frames hold num_recv_frames x the streamer's frame over the rate: UHD's default on a
            B2xx is 16 frames of 2040 samples in sc16, 0.53 ms at 61.44 MS/s and 16 ms at
            2 MS/s. An N2xx takes 32 frames and a socket buffer of recv_buff_size bytes, which
            UHD asks as 50e6 on Linux, 0.5 s at 25 MS/s in sc16. The reading rx_transport_depth
            states the depth in use at the rate in force.
    */
    gr::Annotated<double, "num_recv_frames", gr::Doc<"receive frames; 0 lets UHD choose (B2xx 16, N2xx 32)">>      num_recv_frames   = 0.0;
    gr::Annotated<double, "recv_buff_size", gr::Unit<"B">, gr::Doc<"socket buffer bytes; 0 lets UHD choose (N2xx 50e6)">> recv_buff_size = 0.0;
    /*| contract: where the receive thread runs, the caller's to set as the transport is. Unset,
            the thread keeps what the system gives it. placeOwnThread states the scale, and a
            placement the system refuses is a line on standard error, never a failed start.
        verified-by: threading.an-asked-placement-is-applied-or-named
        verified-by: controls.uhd-thread-placement-is-the-callers
    */
    gr::Annotated<double, "thread_priority", gr::Doc<"receive thread real-time priority, 0 to 1; 0 keeps the system's">> thread_priority = 0.0;
    gr::Annotated<double, "cpu", gr::Doc<"receive thread's processor; -1 keeps the system's choice">>             cpu               = -1.0;

    GR_MAKE_REFLECTABLE(Source, out, device, device_parameter, sample_rate, frequency, rx_gains, gain_mode, frequency_correction, rx_antennae, emit_timing_tags, tag_interval, trigger_name, wire_format, clock_source, time_source, master_clock_rate, recv_frame_size, num_recv_frames, recv_buff_size, thread_priority, cpu);

    ::uhd::usrp::multi_usrp::sptr _usrp;
    ::uhd::rx_streamer::sptr      _stream;
    mutable std::mutex          _ctrlMutex; // serializes every UHD control call, control thread against consumer thread; mutable so the const read-back path can lock too
    std::mutex                  _teardownMutex; // one hardwareTeardown() owner at a time — see its doc
    mutable std::mutex          _sensorMutex;   // one sensor sweep at a time, and the lifetime bound on the handle a sweep snapshots — see readSensors
    std::atomic<bool>           _deviceUp{false};
    ConsumerThread              _consumer; // recv + publish; RAII-joined: a stop can land while a B2xx loads firmware in multi_usrp::make
    /*| frame: the thread the frequency, the gain, the automatic gain mode and the antenna reach
            a streaming device on, one write at a time in the order the settings gave them. The
            consumer thread stages each write here and goes back to its receive.
        why: a B205mini retune takes about 100 ms. Made on the consumer thread, the call left
            the device undrained for that long, the device overflowed on most retunes, and 59
            tunes at 2 MS/s shed 2.8 s of signal behind 14 markers.
    */
    ControlWorker               _control;
    static constexpr int        kControlFrequency = 0;
    static constexpr int        kControlGain      = 1;
    static constexpr int        kControlAgc       = 2;
    static constexpr int        kControlAntenna   = 3;
    /*| frame: what the control thread hands back about one tune: the request's number, the
            frequency the tuner holds afterwards, and the device clock read after the tune
            returned, which a tune that moved nothing carries none of.
        invariant: every sample the device stamps after that reading was taken at the new
            frequency, so the frequency tag goes on the first of them and never before it.
    */
    struct TuneResult {
        std::uint64_t                      request = 0;
        double                             heldHz  = 0.0;
        std::optional<::uhd::time_spec_t>  after;
    };
    std::mutex                  _tuneMutex; // guards _tuneResults between the two threads
    std::vector<TuneResult>     _tuneResults;
    std::atomic<std::uint64_t>  _tuneResultsPosted{0};
    std::uint64_t               _tuneRequests = 0; // consumer thread: the number of the newest frequency request
    std::atomic<int>            _retunesOpen{0};   // tunes begun whose frequency tag is not yet placed
    std::atomic<bool>           _streamDead{false}; // consumer saw a persistently dead stream
    std::atomic<std::uint64_t>  _overflows{0};
    std::atomic<std::uint64_t>  _seqErrors{0}; // gaps the link lost rather than the buffer overran
    /*| frame: the holes this block made itself, by emptying the transport across a rate
            change, and which kind the last hole was. A marker names the cause
            so a reader can tell a radio that shed samples from a block that dropped them on
            purpose.
    */
    std::atomic<std::uint64_t>  _rateBreaks{0};
    static constexpr int        kGapBufferOverrun = 0;
    static constexpr int        kGapLostPacket    = 1;
    static constexpr int        kGapRateChange    = 2;
    static constexpr int        kGapRetune        = 3;
    static constexpr int        kGapMissingSamples = 4;
    std::atomic<int>            _lastGapCause{kGapBufferOverrun};
    /*| contract: the name a gap marker carries for one of the causes above, and "overflow" for
            a value naming none of them.
        frame: the words the marker's rx_overflow_cause key carries, so a reader tells a radio
            that shed samples from a link that lost a packet and from a block that emptied the
            transport itself.
        verified-by: controls.uhd-a-gap-marker-names-its-cause
    */
    static const char* gapCauseName(int cause) {
        switch (cause) {
        case kGapLostPacket: return "lost packet";
        case kGapRateChange: return "rate change";
        case kGapRetune: return "retune";
        case kGapMissingSamples: return "missing samples";
        default: return "overflow";
        }
    }
    /*| contract: the cause an overflow report names: a packet the link lost where the report
            says so, a retune where one is open, and the radio's own overrun otherwise. A tune
            is open from the control thread's write until its frequency tag is placed on the
            first sample stamped after it.
        verified-by: controls.uhd-an-overflow-during-a-retune-is-named-for-it
    */
    static int overflowCause(bool outOfSequence, bool retuneOpen) {
        if (outOfSequence) {
            return kGapLostPacket;
        }
        return retuneOpen ? kGapRetune : kGapBufferOverrun;
    }
    /*| contract: the cause the gap marker of one block names, or nothing where the block follows
            the one before it without a gap. A gap a counter recorded since the last marker names
            the latest counted cause. A shed the block's own stamps measure that no counter
            recorded names a retune where one is open, and "missing samples" otherwise.
        verified-by: controls.uhd-a-shed-no-counter-explains-is-marked
    */
    static std::optional<int> gapMarkerFor(bool counted, int lastCause, std::uint64_t shed, bool retuneOpen) {
        if (counted) {
            return lastCause;
        }
        if (shed > 0UL) {
            return retuneOpen ? kGapRetune : kGapMissingSamples;
        }
        return std::nullopt;
    }
    std::atomic<std::uint64_t>  _dropped{0}; // samples the radio shed, from the stream's own stamps
    /*| frame: what the open device said about itself, read once by start(). The same record a
            caller probes while stopped, so the block's own answers and a probe's agree.
        invariant: guarded by _ctrlMutex. Every write of it, of _rateActualHz and of _bwCurHz
            is followed by publishSurfaceLocked() under the same lock.
    */
    DeviceTruth              _openTruth;
    double                      _bwMinHz = 0.0, _bwMaxHz = 0.0; // analog filter range (read at start)
    double                      _bwCurHz = 0.0;                 // what the filter holds, read at start and after every write
    std::vector<std::pair<double, double>> _freqRanges;         // tuner coverage at the rate in force
    double                      _appliedFreqHz = 0.0;           // the frequency the tuner holds, in the frame a caller asks in
    /*| frame: the request and the correction the frequency applier last worked from, guarded
            by _ctrlMutex. A control path retunes from these rather than from the settings
            members, which the settings drain rewrites on the consumer thread under a mutex no
            control path holds — and for a vector setting the read can follow the reassignment
            that freed the buffer.
    */
    double                      _requestedFreqHz = 0.0;
    double                      _ppmApplied      = 0.0;
    double                      _tunedRfHz     = 0.0;           // the RF frequency the last tune reached, which an LO offset moves
    double                      _loOffsetHz    = 0.0;           // guarded by _ctrlMutex
    double                      _rateActualHz  = 0.0;           // what the device answered for the rate last written
    double                      _refLockWaitMs = 0.0;           // how long start() waited for the reference, 0 on an internal clock
    SensorNames                 _sensorNames;   // enumerated and timed once at start()
    std::vector<std::string>    _clockSources;  // reference and timing option lists, read once at start()
    std::vector<std::string>    _timeSources;
    std::size_t                 _maxRecvSamples = 0UZ; // the frame UHD chose, from the streamer
    mutable std::mutex          _drainMutex; // guards _lastDrain, written by the consumer thread or the teardown
    double                      _recvFramesAsked = 0.0; // num_recv_frames in the open address, zero for UHD's own; guarded by _ctrlMutex
    double                      _recvBuffAsked   = 0.0; // recv_buff_size in the open address, zero for UHD's own; guarded by _ctrlMutex
    std::atomic<bool>           _deviceTime{false};    // tag from the device clock rather than the host's
    /*| frame: what the block knows about the device clock. A set takes effect on the next
            pulse-per-second edge where the time source carries one, up to a second later, and
            at once where none does, so between the write and the effect the device clock
            still reads the seconds since the device came up. _clockSetPending says a set is
            in flight, _clockTargetSec is the whole second it latches, _clockLatchDeadlineNs
            when the block gives the edge up, and _deviceClockSet says a block stamped at or
            past that second has arrived.
        why: a timestamp taken in that window names 1970, and the stamps the stream's shed is
            measured by jump by the whole difference between the two clocks.
    */
    std::atomic<bool>           _clockSetPending{false};
    std::atomic<bool>           _deviceClockSet{false};
    std::atomic<std::int64_t>   _clockTargetSec{0};
    std::atomic<std::uint64_t>  _clockLatchDeadlineNs{0};
    /*| frame: the host clock minus the device clock in nanoseconds, measured when the stream
            starts and again whenever a device-clock set takes effect. It carries a device
            timestamp into the host's epoch, so a tag names the sample the device stamped
            rather than the moment the host drained it.
    */
    std::atomic<std::int64_t>   _deviceToHostNs{0};
    bool                        _agcSupported = false; // whether the frontend carries an AGC at all
    bool                        _agcUnavailableLogged = false; // one line per open, not one per write
    bool                        _agcOn        = false; // guarded by _ctrlMutex; gates manual gain writes
    // The gain_mode the block holds, as its start and its settings drain took it, for
    // controlSurface(), which takes no lock.
    std::atomic<bool>           _surfaceAgc{false};
    /*| frame: what each applier last put on the device, emptied by start() because a fresh
            multi_usrp holds none of the previous device state. Guarded by _ctrlMutex.
    */
    std::optional<double>       _freqWrittenHz, _loOffsetWrittenHz, _rateWrittenHz, _gainWrittenDb, _bwWrittenHz;
    std::optional<bool>         _agcWritten;
    std::optional<std::string>  _antennaWritten;
    /*| frame: the wire format and the master clock in force, and what a control pinned them
            to. Both are read at open — the format when the streamer is built, the clock as a
            device-address key — so a control write outlives the stream it was made during and
            the next start takes it. Unpinned, the settings decide. Guarded by _ctrlMutex.
    */
    std::string                 _wireFormat = "sc16";
    std::optional<std::string>  _wireFormatPinned;
    std::optional<double>       _masterClockPinnedHz;
    /*| frame: whether the clock pin came from a product table's list, on a radio stating one
            clock, and no device has taken that clock yet. A pin a live write put on the device
            leaves it false. Guarded by _ctrlMutex.
    */
    bool                        _masterClockPinFromTable = false;
    /*| frame: the converter clock an explicit write put on the device, empty until one was
            made. It is not the same as the pinned value: selecting the automatic entry asks
            the next open to choose, and UHD's own choice does not come back while the device
            stays open, so the clock a write put there is still the one every rate is derived
            from. Guarded by _ctrlMutex.
    */
    std::optional<double>       _clockInForceHz;
    // A master-clock write moved every rate the radio reaches, so the consumer thread owes
    // the settings a rate that clock does reach.
    std::atomic<bool>           _restageRate{false};
    /*| frame: the reference and the timing selection a control asked for, which the next
            start writes before it configures anything else. Guarded by _ctrlMutex.
    */
    std::optional<std::string>  _clockSourcePinned, _timeSourcePinned;
    std::vector<std::pair<std::string, double>> _heldGains; // element gains asked for, guarded by _ctrlMutex
    std::uint64_t               _lastTagTimeNs = 0;
    std::vector<std::complex<float>> _recvBuf; // the drain's buffer, on the consumer thread
    OutputState                   _outputState;
    ControlProperties     _controlProperties{this};
    Snapshot<SurfaceValues>       _surfaceValues; // what controlSurface() reads, published by publishSurfaceLocked()
    TeardownGuard<Source>         _teardown{this}; // the last member, so its teardown runs first

    // ---- the controls, control and sensors properties -----------------------------------

    std::optional<gr::Message> controlProperty(std::string_view name, gr::Message message) { return answerControlProperty(*this, name, std::move(message)); }

    /*| contract: the control surface of the device this block holds, at the rate and the
            frequency in force. The controls property answers with it, and the settings pass
            reads it without _ctrlMutex. */
    std::vector<ControlDesc> controlSurface() {
        const SurfaceValues v = _surfaceValues.read();
        return describeControls(v.truth, {.deviceParams = {}, .sampleRate = v.rateHz, .centerFreq = 0.0, .gainMode = _surfaceAgc.load(std::memory_order_acquire)}, v.filterHz, v.clockHz, v.wireFormat);
    }

    // Hand the values controlSurface() reads to the snapshot. Called with _ctrlMutex held, or
    // by start() before the device is up.
    void publishSurfaceLocked() { _surfaceValues.publish({.truth = _openTruth, .rateHz = _rateActualHz, .filterHz = _bwCurHz, .clockHz = _clockInForceHz.value_or(0.0), .wireFormat = _wireFormat}); }

    /*| contract: the value one non-gain control holds, in the unit its descriptor states: the
            filter the device settled on, the LO offset tuned with, the device-clock tagging,
            and for the four open-time selections the one the running device was opened with,
            which a staged write moves only at the next start. Nothing for the two corrections,
            whose state the block does not read back, and nothing for the antenna, whose write
            reaches the device at the next settings drain.
        frame: a selection is an index into its own list, as the descriptor states it. */
    std::optional<double> controlValue(const std::string& id) {
        std::lock_guard lock(_ctrlMutex);
        const auto indexIn = [](const std::vector<std::string>& names, const std::string& name) -> std::optional<double> {
            const auto at = std::ranges::find(names, name);
            return at == names.end() ? std::nullopt : std::optional<double>(static_cast<double>(at - names.begin()));
        };
        if (id == "BANDWIDTH") {
            return _bwCurHz / 1e6;
        }
        if (id == "LO_OFFSET") {
            return _loOffsetHz / 1e6;
        }
        if (id == "DEVICE_TIME") {
            return _deviceTime.load(std::memory_order_acquire) ? 1.0 : 0.0;
        }
        if (id == "WIRE_FORMAT") {
            return indexIn(wireFormats(), _wireFormat);
        }
        if (id == "MASTER_CLOCK") {
            return _clockInForceHz.value_or(_openTruth.mclkHz) / 1e6;
        }
        if (id == "CLOCK_SOURCE") {
            return indexIn(_clockSources, _openTruth.clockSource);
        }
        if (id == "TIME_SOURCE") {
            return indexIn(_timeSources, _openTruth.timeSource);
        }
        return std::nullopt;
    }

    // ---- discovery + device truth (no claim / brief exclusive claim) -----------------

    // A value a device string can spell, which is the test the listing applies to every
    // address key it puts into one.
    static bool deviceStringSafe(std::string_view value) { return capture::uhd::deviceStringSafe(value); }

    // (label, device string) pairs for a caller's device list. The transmit block offers the
    // same list, so the body sits in the header both blocks include.
    static std::vector<std::pair<std::string, std::string>> enumerateDevices() { return deviceList(); }

    // The record probeTruth fills and describeControls reads, for a caller that names neither.
    using Truth = DeviceTruth;

    /*| contract: open the device briefly, while the DSP is stopped, and read its receive
            control surface. False when the device cannot be opened. See the trap on the shared
            body for what the rate argument buys.
    */
    static bool probeTruth(const std::string& kwargs, double sampleRateHz, DeviceTruth& truth) {
        return capture::uhd::probeTruth(kwargs, sampleRateHz, truth, Direction::Receive, "uhd::Source", [](::uhd::usrp::multi_usrp& usrp, DeviceTruth& into) { readReceiveExtrasInto(usrp, into); });
    }

    /*| contract: the two property-tree roots the receive chain of channel 0 hangs off: the
            daughterboard's own frontend first, then the motherboard's receive frontend core.
            Both empty where the device names no subdevice.
        why: the daughterboard slot and the frontend name differ per model — a B2xx answers
            A:A and an N-series with a WBX answers A:0 — so a path written out as a literal
            reads as absent on whichever model it was not written for, and the facility it
            asks about then looks missing when it is there. The two roots differ too: the
            gain and its automatic loop belong to the daughterboard, while the digital
            corrections belong to the converter core on the motherboard, and which of the two
            carries a given facility is a property of the model.
    */
    static std::pair<std::string, std::string> rxFrontendRoots(::uhd::usrp::multi_usrp& usrp) {
        try {
            const auto spec = usrp.get_rx_subdev_spec(0);
            if (spec.empty()) {
                return {};
            }
            return {std::format("/mboards/0/dboards/{}/rx_frontends/{}", spec.at(0).db_name, spec.at(0).sd_name), std::format("/mboards/0/rx_frontends/{}", spec.at(0).db_name)};
        } catch (const std::exception&) {
            return {};
        }
    }

    /*| contract: the full path of one optional receive facility — "gain/agc/enable",
            "dc_offset/enable", "iq_balance/enable" — under whichever of the two frontend
            roots carries it, or empty where neither does.
        frame: empty is the answer that decides a control is not offered, because UHD takes
            the write for a facility that is not there, warns and changes nothing.
        verified-by: uhd.correction-defaults
    */
    static std::string rxFacilityPath(::uhd::usrp::multi_usrp& usrp, std::string_view leaf) {
        const auto [dboard, converter] = rxFrontendRoots(usrp);
        try {
            auto tree = usrp.get_device()->get_tree();
            for (const std::string& root : {dboard, converter}) {
                if (root.empty()) {
                    continue;
                }
                if (const std::string path = root + "/" + std::string(leaf); tree->exists(path)) {
                    return path;
                }
            }
        } catch (const std::exception&) { // no property tree reachable
        }
        return {};
    }

    /*| contract: read one open device's whole receive surface into truth, replacing it: the
            shared record both directions carry, and the three fields the receive frontend alone
            states.
        frame: the one place that says what the receive truth is, so a probe of a stopped device
            and a running block's own record of the device it opened are the same record.
    */
    static void readTruthInto(::uhd::usrp::multi_usrp& usrp, DeviceTruth& truth) {
        capture::uhd::readTruthInto(usrp, truth, Direction::Receive);
        readReceiveExtrasInto(usrp, truth);
    }

    /*| contract: which of the three optional corrections this unit's receive frontend carries,
            added to a record the shared reader already filled.
        frame: the three facilities hang off the receive frontend's own property-tree roots, so
            they belong to the receive record alone.
    */
    static void readReceiveExtrasInto(::uhd::usrp::multi_usrp& usrp, DeviceTruth& truth) {
        truth.hasAgc           = !rxFacilityPath(usrp, "gain/agc/enable").empty();
        truth.hasDcOffsetCorr  = !rxFacilityPath(usrp, "dc_offset/enable").empty();
        truth.hasIqBalanceCorr = !rxFacilityPath(usrp, "iq_balance/enable").empty();
    }

    // The same probe at whatever rate the device came up at, for a caller with no rate to
    // state. See the trap on the call above.
    static bool probeTruth(const std::string& kwargs, DeviceTruth& truth) { return probeTruth(kwargs, 0.0, truth); }

    bool deviceUp() const { return _deviceUp.load(std::memory_order_acquire); }

    // True from a stream that stopped delivering while the device was up to the next start.
    bool streamDead() const { return _streamDead.load(std::memory_order_acquire); }

    /*| contract: I/Q samples the radio shed before this block saw them, since the stream
            started. It only rises within one stream, so a caller watches the difference
            between two readings.
        frame: samples, not packets and not overflow events. UHD reports an overflow as an
            event with no count, and only a count says how much of the signal is gone.
        measured: a 300 ms stall at 8 MS/s on a B205mini sheds about 2.3 million samples
            against one overflow event.
        verified-by: controls.uhd-counts-what-the-device-shed
    */
    std::uint64_t droppedSamples() const { return _dropped.load(std::memory_order_relaxed); }
    // Markers the output port had no room for, each a gap the stream never reported.
    std::size_t droppedTags() const { return droppedTagCount(*this); }


    // UHD overflow events since the stream started. The samples they cost are
    // droppedSamples(); an overflow event carries no count of its own.
    std::uint64_t overflowEvents() const { return _overflows.load(std::memory_order_relaxed); }

    /*| contract: gaps whose packet never arrived, since the stream started, counted apart
            from the gaps a full buffer caused.
        frame: UHD marks both as an overflow and sets out_of_sequence on the second. They
            have different causes and different answers: a buffer runs over when the host
            stops draining, and a packet goes missing on the link between the radio and the
            host, which no amount of host attention fixes.
        measured: a stalled consumer on a B205mini moves the overflow counter and leaves this
            one at zero; the same stall on an N210 over Ethernet moves this one.
        verified-by: uhd.overflow-under-a-stalled-consumer
    */
    std::uint64_t sequenceErrors() const { return _seqErrors.load(std::memory_order_relaxed); }

    /*| contract: what the device said about itself when this block opened it, with the tuner
            coverage the rate and the filter in force give, which is the coverage a tune is
            tested against. An empty record while the block is down.
        verified-by: uhd.accessors-after-a-stop
        verified-by: controls.uhd-the-stated-coverage-is-the-coverage-in-force
    */
    DeviceTruth deviceTruth() const {
        std::lock_guard lock(_ctrlMutex);
        DeviceTruth     t = _openTruth;
        if (!_freqRanges.empty()) {
            t.freqRanges = _freqRanges;
        }
        return t;
    }

    /*| contract: the samples one receive frame holds, as UHD sized it from the transport
            settings and the link. Zero before the streamer exists and after it has gone.
        verified-by: uhd.accessors-after-a-stop
    */
    std::size_t recvFrameSamples() const {
        std::lock_guard lock(_ctrlMutex);
        return _maxRecvSamples;
    }

    /*| contract: the analog filter width the last device this block opened settled on, in
            Hz, read from it at the open and after every write. Zero before the first open.
            It survives a stop, unlike the rest of the record, because a caller drawing the
            control surface of a stopped radio has nowhere else to read the filter it holds.
        why: UHD coerces a bandwidth to the nearest valid value rather than to the next value
            at or above the request, so on a frontend with a stepped filter the value a
            descriptor would state from the request disagrees with the hardware.
        verified-by: uhd.accessors-after-a-stop
    */
    double analogBandwidthHz() const {
        std::lock_guard lock(_ctrlMutex);
        return _bwCurHz;
    }

    /*| contract: the wire format the running streamer was built with, which is the packing
            the rates in force are carried in. A control's choice reaches the streamer at the
            next start, so between a write and that start the two differ.
        verified-by: uhd.wire-format
    */
    std::string wireFormatInForce() const {
        std::lock_guard lock(_ctrlMutex);
        return _wireFormat;
    }

    /*| contract: the RF frequency the last accepted tune reached, in Hz, and zero while the
            block is down. An LO offset moves it away from the center frequency by the
            offset; without one the two agree.
        verified-by: uhd.accessors-after-a-stop
    */
    double tunedRfHz() const {
        std::lock_guard lock(_ctrlMutex);
        return _tunedRfHz;
    }

    // How long start() waited for the reference to lock, in milliseconds. Zero where the
    // clock source in force was internal, which is waited for not at all.
    double refLockWaitMs() const { return _refLockWaitMs; }

    /*| contract: the samples missing between the stamp a block was expected at and the stamp
            it arrived at: the difference of the two stamps at rateHz, as the nearest whole
            number of samples. A block less than half a sample late, on time or early sheds
            nothing.
        frame: a USRP timestamps every packet, so the stamps say where a block belongs in the
            stream and the gap between two blocks measures what the radio shed between them.
        why: the device takes a sample on every decimation-th tick of its master clock. After
            a live rate change those ticks can sit a fraction of a sample off the grid of
            whole sample periods counted from the device's epoch. A stamp rounded to that grid
            on its own rounds a half up on one block and down on the next, and each return
            upward reads as one sample shed. Two stamps of one stream sit the same fraction
            off that grid, so their difference is a whole number of samples at any rate, the
            master clock's own included.
        measured: a B205mini at 8 MS/s from a 32 MHz clock, after a change from 1 MS/s, took
            its samples two ticks, half a sample, off the 8 MS/s grid. Rounded one by one, 75
            of 733 stamps each read as one sample shed. The whole master-clock ticks show none
            lost.
        trap: the count is the nearest whole number. The setting a block is measured at can
            differ from the rate the device holds by up to half a hertz, which moves a long
            block's difference by a small fraction of a sample either way, and the whole part
            of one sample less that fraction reads as none.
        trap: a block arrives before the stamp expected of it after a rate change, because the
            expectation was formed at the previous rate. An early arrival is not a negative
            gap to subtract.
        verified-by: controls.uhd-counts-what-the-device-shed
        verified-by: controls.uhd-stamps-off-the-sample-grid-count-only-lost-samples
    */
    static std::uint64_t shedBetween(const ::uhd::time_spec_t& expected, const ::uhd::time_spec_t& arrived, double rateHz) {
        const double samples = (arrived - expected).get_real_secs() * rateHz;
        if (!(samples >= 0.5)) { // a value that is not a number fails the test as well
            return 0UL;
        }
        return static_cast<std::uint64_t>(std::min(std::round(samples), 0x1p63)); // 2^63 bounds the conversion
    }

    // The rate picks for a caller holding no probe, by the kind the device string names.
    static std::vector<double> sampleRates(const std::string& kwargs) { return capture::uhd::sampleRates(kwargs); }

    // The shortfall a stream may carry before the block says the host is not keeping up, and
    // the window it is measured over.
    static constexpr double kShortfallFraction = 0.05;
    static constexpr double kShortfallWindowS  = 10.0;

    /*| contract: the fraction of what the rate in force should have delivered that did not
            arrive over a window, negative where more arrived than that rate accounts for,
            and nothing where the window or the rate says nothing.
        why: the ladder states what the radio reaches, and UHD states a link rate for some
            kinds and none for others. A stated rate is the line's own and not what this host
            sustains, so the delivered count against the rate in force is the one statement
            that holds whatever the link is.
        measured: a B205mini asked for 61.44 MS/s over USB 3 delivers about 83 percent of it
            on this host.
        verified-by: controls.uhd-reports-a-rate-the-host-cannot-carry
    */
    static std::optional<double> deliveryShortfall(std::uint64_t delivered, double seconds, double rateHz) {
        if (!(seconds > 0.0) || !(rateHz > 0.0)) {
            return std::nullopt;
        }
        const double expected = rateHz * seconds;
        return (expected - static_cast<double>(delivered)) / expected;
    }

    /*| contract: the delivered count at which the shortfall test next reads the clock, from the
            count and the seconds elapsed at this reading: the count the rate in force adds
            before kShortfallWindowS has passed. The next receive reads it once the window
            has passed, and at a rate that is not a positive number.
        why: the test needs the clock only where the window can close. A stream that keeps
            its rate reaches this count as the window closes. A stream that falls short of its
            rate has gaps, and the first block after each gap reads the clock as well.
        verified-by: controls.uhd-reports-a-rate-the-host-cannot-carry
    */
    static std::uint64_t shortfallNextCheck(std::uint64_t delivered, double elapsedS, double rateHz) {
        const double ahead = (kShortfallWindowS - elapsedS) * rateHz;
        if (!(ahead >= 1.0)) { // a value that is not a number fails the test as well
            return delivered;
        }
        return delivered + static_cast<std::uint64_t>(std::min(ahead, 0x1p62)); // 2^62 bounds the sum
    }

    // The rates to offer a caller for the radio a probe described, ascending.
    static std::vector<double> sampleRatesFor(const DeviceTruth& t, std::string_view wireFormat = "sc16") { return capture::uhd::sampleRatesFor(t, wireFormat); }

    /*| contract: the LO offset to apply, in Hz, bounded by half the rate in force at each
            end, which is the bound the descriptor states. A rate of zero or a value that is
            not a number leaves no bound to apply and answers zero.
        why: ControlDesc states that the bound is the family's as well as the caller's, and a
            caller holding the block directly applies none. An offset past half the rate puts
            the LO spur outside the band the host receives, and the spur it was moved out of
            the way of comes back.
        verified-by: controls.uhd-bounds-the-lo-offset
    */
    static double boundedLoOffsetHz(double offsetHz, double rateHz) {
        if (offsetHz != offsetHz || !(rateHz > 0.0)) { // a value that is not a number fails the first test
            return offsetHz == offsetHz ? offsetHz : 0.0;
        }
        const double edge = rateHz / 2.0;
        return std::clamp(offsetHz, -edge, edge);
    }

    /*| contract: when the first sample of a block was taken, in nanoseconds. A device
            timestamp names that sample directly; without one the wall clock is read once the
            block has arrived and the block's own duration comes off it.
        why: the device clock is the one the samples are counted against, so a tag taken from
            it carries neither the backlog correction nor the host's own scheduling jitter.
            It means nothing until the device time has been set, and the toggle that turns
            this on sets it first.
        invariant: a timestamp at or below zero, which a device whose clock was never set
            produces, falls back to the wall clock.
        contract: wallNs is the wall-clock reading or a callable returning it, and a callable
            runs only where the block has no device time.
        verified-by: controls.uhd-tag-time-comes-from-the-device
    */
    template <typename WallNs>
    static std::uint64_t blockTagNs(std::optional<std::int64_t> deviceNs, WallNs&& wallNs, std::size_t nSamples, double rateHz) {
        if (deviceNs.has_value() && *deviceNs > 0) {
            return static_cast<std::uint64_t>(*deviceNs);
        }
        if constexpr (std::invocable<WallNs>) {
            return blockStartNs(wallNs(), nSamples, rateHz);
        } else {
            return blockStartNs(wallNs, nSamples, rateHz);
        }
    }

    /*| contract: the timestamp to tag a block with, from the device's own stamp for its first
            sample. With deviceEpoch the stamp stands, which is the device clock a caller
            asked for; without it the stamp is carried into the host's epoch by toHostNs.
            Nothing where the block carried no stamp, or where the carried value is not a
            time.
        why: the device stamps the sample rather than the delivery, so a tag taken this way
            carries neither the transport's backlog nor the host's scheduling jitter. The
            toggle then chooses whose epoch the tag names and nothing else.
        verified-by: controls.uhd-tag-time-comes-from-the-device
    */
    static std::optional<std::int64_t> deviceStampNs(std::optional<std::int64_t> deviceNs, bool deviceEpoch, std::int64_t toHostNs) {
        if (!deviceNs.has_value()) {
            return std::nullopt;
        }
        if (deviceEpoch) {
            return deviceNs;
        }
        const std::int64_t host = *deviceNs + toHostNs;
        return host > 0 ? std::optional<std::int64_t>{host} : std::nullopt;
    }

    /*| contract: the samples one block's stamp says the radio shed since the block before it,
            and nothing where the expectation cannot speak: none yet, one formed at another
            rate, or a device-clock set in flight.
        why: setting the device clock moves it from the seconds since the device came up to a
            wall-clock time, about 1.79e9 seconds further on, and the stamp difference across
            that jump is that number times the rate — 3.6e15 samples at 2 MS/s. The stream
            lost nothing; the clock it was measured against moved.
        verified-by: controls.uhd-holds-the-tick-grid-across-a-clock-set
    */
    static std::uint64_t shedForBlock(bool haveExpected, bool clockPending, double rateNow, double expectedRate, const ::uhd::time_spec_t& expected, const ::uhd::time_spec_t& arrived) {
        if (!haveExpected || clockPending || rateNow != expectedRate) {
            return 0UL;
        }
        return shedBetween(expected, arrived, rateNow);
    }

    // How long the block waits for a device-clock set to reach the stream, in milliseconds.
    // A pulse-per-second edge is at most a second away; this is that and a margin.
    static constexpr double kClockLatchLimitMs = 2500.0;

    enum class ClockLatch { Pending, Latched, Abandoned };

    /*| contract: what one block's own seconds say about a device-clock set in flight: the
            clock has reached the second it was asked to latch, the wait for it has run out,
            or neither yet.
        frame: the latch is read from the stream's own stamps, so it costs no device call
            and it names the first block the new clock reached.
        verified-by: controls.uhd-holds-the-tick-grid-across-a-clock-set
    */
    static ClockLatch clockLatchState(std::int64_t blockSeconds, std::int64_t targetSeconds, std::uint64_t nowNs, std::uint64_t deadlineNs) {
        if (blockSeconds >= targetSeconds) {
            return ClockLatch::Latched;
        }
        return nowNs >= deadlineNs ? ClockLatch::Abandoned : ClockLatch::Pending;
    }

    /*| frame: the receives with no samples in them that mean the stream is dead. At the
            0.1 s timeout each one carries, that is five seconds of silence — long enough to
            cover a B2xx spinning its stream path up at the first start.
        trap: at least five seconds, not about five: the timeout applies to every internal
            call inside recv rather than to the whole of it, so the real deadline is longer.
    */
    static constexpr int    kDeadReceives  = 50;
    static constexpr double kRecvTimeoutS  = 0.1;

    /*| contract: the samples one receive asks for: the room the output edge has, capped at
            a quarter of the edge (kRecvEdgeShares) and at kRecvWindowS of the rate in force,
            in whole frames, and never below one frame. Zero where the room holds less than
            one frame or the frame is zero, and the consumer loop then waits for the graph to
            move, bounded by roomWaitBound, and asks again.
        frame: UHD fills a receive from as many frames as it needs, and a frame holds 2040
            samples on a B205mini and 363 on an N210 at the transport sizes this block
            accepts. A request of whole frames ends every receive on a frame boundary, so
            each receive starts at a frame's first sample and carries that frame's stamp.
        frame: the receive lands in the output edge itself. The room is the space the edge's
            writer can reserve, and a reservation is all or nothing, so the request never
            exceeds it. The window bounds how long one receive holds the reservation before
            its samples reach the graph.
        why: a receive that fills the whole edge leaves no room for the next one until the
            reader has taken everything, and the device's transport overflows during that
            wait. A quarter of the edge lets receives land while the reader takes the rest,
            and the consumer thread seldom waits for room, which is time spent away from the
            transport.
        measured: on a B205mini at 8 MS/s into a 64 Ki-sample edge, receives that filled the
            edge lost 23.4 million of 40 million samples in 5 s over 31 overflows; receives of
            half the edge lost none.
        measured: on a B205mini at 61.44 MS/s into a 64 Ki-sample edge, receives of a
            quarter of the edge waited for room 0 times a second, and receives of half the
            edge 160 times a second. The overflows, 5.8 and 7.8 per 20 s run over twelve and
            nine runs, differ by less than their spread from run to run. A quarter costs
            1 percent more user cycles per sample.
        measured: on a B205mini into an edge of 50 ms of the rate, a receive held 9, 78 and
            294 frames at 1, 8 and 30 MS/s, and the receives per second were 54, 50 and 50,
            against 61, 488 and 1830 for 16384-sample receives. The consumer thread spent
            15.6, 5.9 and 5.5 million user cycles per million samples against 16.1, 6.2 and
            6.4 million, the best of three to six 20 s runs of each.
        verified-by: controls.uhd-a-receive-asks-for-whole-frames
        verified-by: uhd.transport-sizing
    */
    static constexpr double      kRecvWindowS    = 0.02;
    static constexpr std::size_t kRecvEdgeShares = 4UZ;
    static std::size_t recvRequest(std::size_t roomSamples, std::size_t edgeSamples, std::size_t frameSamples, double rateHz) {
        if (frameSamples == 0UZ || roomSamples < frameSamples) {
            return 0UZ;
        }
        std::size_t  frames    = std::min(roomSamples, edgeSamples / kRecvEdgeShares) / frameSamples;
        const double capFrames = rateHz > 0.0 ? std::floor(rateHz * kRecvWindowS / static_cast<double>(frameSamples)) : 0.0; // a rate that is not a number fails the test
        if (capFrames < static_cast<double>(frames)) {
            frames = static_cast<std::size_t>(capFrames);
        }
        return std::max(frames, 1UZ) * frameSamples;
    }

    /*| contract: the liveness counter after one receive: zero where samples arrived, one
            more where none did, whatever the error arm said.
        why: a counter that only a timeout raises, and that every other arm clears, never
            reaches the deadline for a device stuck on a bad packet, a failed alignment or a
            broken chain: work() then reports OK for ever while nothing leaves the block.
        verified-by: controls.uhd-a-silent-receive-counts-whatever-it-says
    */
    static int livenessAfterReceive(int consecutive, std::size_t nSamples) { return nSamples > 0UZ ? 0 : consecutive + 1; }

    /*| contract: whether this error code is the first of its kind in the list, adding it
            where it is. One list per stream.
        why: an arm that prints one line per iteration lets a device stuck on a bad packet
            fill a terminal at a thousand lines a second. One line per distinct code says as
            much and stops.
        verified-by: controls.uhd-a-silent-receive-counts-whatever-it-says
    */
    static bool firstSightOf(std::vector<int>& seen, int code) {
        if (std::ranges::find(seen, code) != seen.end()) {
            return false;
        }
        seen.push_back(code);
        return true;
    }

    /*| contract: the control surface from probed truth — gain elements, ranges and
            bandwidth differ per USRP model and daughterboard, so there is no static table.
            The caller owns the probe cache.
        frame: heldBandwidthHz above zero carries the width the analog filter holds on a
            running block, and it becomes the BANDWIDTH descriptor's default there. Zero
            leaves the default at the rate-derived value the rate applier programs, the only
            value a caller drawing a surface for a stopped device has.
        frame: clockInForceHz is the master clock a running block holds, zero where UHD
            chooses. Without it the clock is the one an open with ctx.deviceParams fixes, as a
            probe reads it.
        frame: wireFormat is the packing the ladder is cut for and the WIRE_FORMAT default,
            the format a running block holds. Empty takes ctx.wireFormat, the format an
            unstarted block holds staged or set, and a format the family does not name is sc16.
    */
    static std::vector<ControlDesc> describeControls(const DeviceTruth& t, const TruthContext& ctx, double heldBandwidthHz = 0.0, std::optional<double> clockInForceHz = std::nullopt,
                                                     std::string_view wireFormat = {}) {
        const std::string        format = validWireFormat(std::string(wireFormat.empty() ? std::string_view(ctx.wireFormat) : wireFormat));
        std::vector<ControlDesc> out    = gainElementControls(t);
        /*| frame: the receive connector, as an index into the unit's own list, defaulting to
                the connector in force. A unit that names one connector or none states no
                control, as the sink does.
            verified-by: controls.uhd-states-the-antennas-the-unit-names
        */
        if (t.antennas.size() > 1UZ) {
            ControlDesc c;
            c.id          = "ANTENNA";
            c.label       = "RX connector";
            c.kind        = 3;
            c.min         = 0.0;
            c.max         = static_cast<double>(t.antennas.size() - 1UZ);
            c.step        = 1.0;
            c.options     = t.antennas;
            const auto it = std::ranges::find(t.antennas, t.antenna);
            c.defValue    = it == t.antennas.end() ? 0.0 : static_cast<double>(std::distance(t.antennas.begin(), it));
            c.isGain      = false;
            out.push_back(std::move(c));
        }
        if (auto bandwidth = bandwidthControl(t, ctx, heldBandwidthHz); bandwidth.has_value()) {
            out.push_back(std::move(*bandwidth));
        }
        /*| frame: each automatic correction is stated only where the frontend's property tree
                carries its enable. A model without one takes the write, logs a warning and
                changes nothing, so a toggle offered for it would be a control that does
                nothing.
            measured: a B205mini comes up with both corrections already running. Its property
                tree reads dc_offset/enable and iq_balance/enable true before anything writes
                them, and the DC bin of a received frame sits below -120 dBFS as the device
                comes up against about -60 dBFS with the corrections written off. Two runs
                read -123.0 and -125.9 dBFS as it came up against -62.0 and -60.0 dBFS off,
                so the 60 dB gap is the reading and the last decibel of it is not. An N210
                with a WBX carries the DC offset enable and no IQ balance enable.
            verified-by: uhd.correction-defaults
        */
        for (const auto& [id, label, present] : {std::tuple{"DC_CORR", "Auto DC offset corr", t.hasDcOffsetCorr}, std::tuple{"IQ_CORR", "Auto IQ balance corr", t.hasIqBalanceCorr}}) {
            if (!present) {
                continue;
            }
            ControlDesc c;
            c.id       = id;
            c.label    = label;
            c.kind     = 2;
            c.min      = 0.0;
            c.max      = 1.0;
            c.step     = 1.0;
            c.defValue = 1.0;
            c.isGain   = false;
            out.push_back(std::move(c));
        }
        /*| frame: the frontend's automatic gain loop, stated where its property tree carries
                the loop's enable, at the mode the context holds.
        */
        if (t.hasAgc) {
            out.push_back(gainModeControl(ctx.gainMode));
        }
        /*| frame: the reference and the timing selection, each an index into the device's own
                option list, defaulting to what the device is running on. The ids are
                UHD's own. Both reach the device at the next start. Both descriptors state the
                start, and both labels say so.
        */
        const auto addEnum = [&out](const char* id, const char* label, const std::vector<std::string>& options, const std::string& current) {
            if (options.empty()) {
                return;
            }
            ControlDesc c;
            c.id          = id;
            c.label       = label;
            c.kind        = 3;
            c.min         = 0.0;
            c.max         = static_cast<double>(options.size() - 1UZ);
            c.step        = 1.0;
            c.options     = options;
            const auto it = std::ranges::find(options, current);
            c.defValue    = it == options.end() ? 0.0 : static_cast<double>(std::distance(options.begin(), it));
            c.isGain      = false;
            c.appliesAt   = AppliesAt::Start;
            out.push_back(std::move(c));
        };
        addEnum("CLOCK_SOURCE", "Clock source (next start)", t.clockSources, t.clockSource);
        addEnum("TIME_SOURCE", "Time source (next start)", t.timeSources, t.timeSource);
        /*| frame: the LO offset the tuner carries, in MHz, bounded by half the rate in force
                at each end because an offset past that puts the LO spur outside the band the
                host receives and the spur it was moved out of the way of comes back.
            measured: a B205mini at 915 MHz with a 1 MHz offset answers 915 MHz from
                get_rx_freq while its RF frontend sits at 916 MHz, and the DC bin falls from
                -58 to -95 dBFS.
        */
        if (ctx.sampleRate > 0.0) {
            ControlDesc c;
            c.id          = "LO_OFFSET";
            c.label       = "LO offset";
            c.unit        = "MHz";
            c.kind        = 0;
            c.max         = ctx.sampleRate / 2.0 / 1e6;
            c.min         = -c.max;
            c.step        = 0.1;
            c.defValue    = 0.0;
            c.isGain      = false;
            c.rateDerived = true;
            out.push_back(std::move(c));
        }
        /*| frame: timing tags taken from the device clock rather than the host's. Off by
                default because the device clock means nothing until it has been set, and
                turning this on sets it.
        */
        ControlDesc deviceTime;
        deviceTime.id       = "DEVICE_TIME";
        deviceTime.label    = "Device-time tags";
        deviceTime.kind     = 2;
        deviceTime.min      = 0.0;
        deviceTime.max      = 1.0;
        deviceTime.step     = 1.0;
        deviceTime.defValue = 0.0;
        deviceTime.isGain   = false;
        out.push_back(std::move(deviceTime));
        /*| frame: the packing the samples take on the link between the radio and the host.
                sc8 sends eight bits per part instead of sixteen, which halves the bytes and
                doubles the rate a link of a given speed carries, and costs the lower eight
                bits of every sample. The label says that cost, because a caller choosing a
                rate the link could not otherwise reach is trading dynamic range for it.
            trap: the stream argument is fixed when the streamer is built, so a change here
                reaches the radio at the next start, and the descriptor states the start.
            measured: an N210 delivers nothing at 50 MS/s in sc16 and delivers it in sc8.
        */
        ControlDesc wire;
        wire.id       = "WIRE_FORMAT";
        wire.label    = "Wire format (sc8 halves dynamic range)";
        wire.kind     = 3;
        wire.min      = 0.0;
        wire.max      = static_cast<double>(wireFormats().size() - 1UZ);
        wire.step     = 1.0;
        wire.options.assign(wireFormats().begin(), wireFormats().end());
        wire.defValue  = static_cast<double>(std::ranges::find(wireFormats(), format) - wireFormats().begin());
        wire.isGain    = false;
        wire.appliesAt = AppliesAt::Start;
        out.push_back(std::move(wire));
        /*| frame: the converter clock, in MHz, on a radio that lets one be picked, with a
                first entry of zero meaning the choice UHD makes for itself. Every reachable
                rate is that clock over a decimation, so the clock decides the whole ladder.
            trap: an explicit write turns UHD's own choice off for as long as the device
                stays open, and only the next open restores it, which is why the automatic
                entry takes effect at the next start rather than at once.
            contract: the descriptor states the start. The next open is the one point at which
                every entry is in force, the automatic one included. On a radio stating a span
                or separate clocks, an explicit clock also reaches a running device at once and
                moves the rate with it.
            measured: a B205mini states 220 kHz to 61.44 MHz and runs 16 or 32 MHz when it
                chooses for itself.
            contract: a device whose clock range is one continuous span takes any clock in
                it, so the descriptor is anyInRange and min and max state that span in MHz.
                Otherwise min and max are the ends of the list and only its entries are
                offered.
            verified-by: controls.uhd-a-clock-span-takes-any-clock-in-it
            contract: a radio stating one clock is offered the clocks its product takes at
                the open, and the label names the clock in force. A clock picked there applies
                at the next open alone, through the master_clock_rate device argument.
            why: the rate offer of such a radio stays the ladder of the clock in force until
                that open, which reads the rates of the picked clock. UHD states the rates of a
                fixed-clock radio for the clock it runs, and the ladder of any other clock
                would be a guess at the decimations the image carries.
            verified-by: controls.uhd-a-product-table-offers-the-clocks-of-a-fixed-clock-radio
        */
        {
            const auto ladder = masterClockLadderMHz(t);
            if (!ladder.empty()) {
                ControlDesc clock;
                clock.id         = "MASTER_CLOCK";
                clock.label      = offersOneClockRate(t) ? std::format("Master clock ({:g} MHz in force; a pick applies at the next start)", t.mclkMinHz / 1e6) : std::string("Master clock");
                clock.unit       = "MHz";
                clock.kind       = 4;
                clock.listValues = ladder;
                clock.anyInRange = t.mclkContinuous;
                clock.min        = clock.anyInRange ? t.mclkMinHz / 1e6 : ladder.front();
                clock.max        = clock.anyInRange ? t.mclkMaxHz / 1e6 : ladder.back();
                clock.step       = 1.0;
                clock.defValue   = 0.0;
                clock.isGain     = false;
                clock.appliesAt  = AppliesAt::Start;
                out.push_back(std::move(clock));
            }
        }
        appendLadderAndCoverage(out, t, clockInForceHz.value_or(clockInForceAtOpen(::uhd::device_addr_t(ctx.deviceParams), t)), format, kDefaultRateHz, kDefaultFrequencyHz);
        return out;
    }

    // The entry an enum control's value selects out of a list of count options.
    static std::optional<std::size_t> optionIndexFor(double value, std::size_t count) { return capture::uhd::optionIndexFor(value, count); }

    // ---- device-truth control surface ------------------------------------------------

    /*| contract: the gain writes leaving the automatic mode makes, in order: the overall
            gain the block carries, then every element a caller asked for while the loop had
            the gain path that the open device names. An empty element name is the overall
            gain.
        why: the element write comes last so that the value a caller moved outlives the loop
            that overrode it. The overall gain alone would put the device back on whatever
            rx_gains last carried, while the control the caller moved still reads the value
            it was moved to.
        trap: the element names are a property of the model, so a name held from another
            radio is not written to this one. UHD throws once per element for a name the
            device does not name, which is the check setElementGain already makes.
        verified-by: controls.uhd-agc-hands-the-gain-back
    */
    static std::vector<std::pair<std::string, double>> agcReleaseWrites(double overallDb, std::span<const std::pair<std::string, double>> held, std::span<const DeviceTruth::GainElement> elements) {
        std::vector<std::pair<std::string, double>> writes{{std::string{}, overallDb}};
        for (const auto& [name, dB] : held) {
            if (std::ranges::any_of(elements, [&name](const DeviceTruth::GainElement& g) { return g.name == name; })) {
                writes.emplace_back(name, dB);
            }
        }
        return writes;
    }

    /*| contract: write one gain element by the name the device gave it, bounded by the range
            that element states; an element the open device does not list is refused. The
            overall gain, the empty name, is bounded by the overall range, which UHD distributes
            across the elements itself, and goes through rx_gains: true once the setting is
            staged, the value reaching the device at the consumer thread's next settings drain,
            and a refusal there named on standard error alone.
        why: the overall gain and rx_gains name one thing. A write of the device alone leaves the
            setting where it was, and the next drain carrying rx_gains, the replay after a start
            among them, puts the setting's gain back.
        verified-by: uhd.gain
        verified-by: controls.uhd-the-overall-gain-writes-through-its-setting
    */
    bool setElementGain(const std::string& element, double dB) {
        gr::property_map through;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp);
            if (!guard) {
                return false;
            }
            if (!gainElementNamed(_openTruth, element, "uhd::Source")) {
                return false;
            }
            const double bounded = boundedGainFor(_openTruth, element, dB, "uhd::Source");
            if (element.empty()) {
                through = {{"rx_gains", std::vector<double>{bounded}}};
            } else {
                holdElementGain(element, bounded);
                if (_agcOn) {
                    return true; // the loop owns the gain path; the held value is written when it lets go
                }
                try {
                    _usrp->set_rx_gain(bounded, element, 0);
                    return true;
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "[uhd::Source] set_rx_gain(%s, %.1f): %s\n", element.c_str(), dB, e.what());
                    return false;
                }
            }
        }
        /*| trap: staged with the control mutex free. The settings drain takes the settings
                machinery's mutex and then this block's control mutex, so a stage made under the
                control mutex takes the two in the opposite order. */
        std::ignore = this->settings().setStaged(through);
        return true;
    }

    // The gain one element holds, or the overall gain for an empty name.
    double elementGain(const std::string& element) const {
        /*| trap: unlike the other sources this reads back from the device instead of a cached
                atomic, so it needs the same guard as the setters: teardown resets _usrp under
                _ctrlMutex, and an unsynchronized shared_ptr read against that reset races the
                refcount control block and not merely the pointer.
        */
        CtrlGuard guard(_ctrlMutex, _deviceUp);
        if (!guard) {
            return 0.0;
        }
        try {
            return element.empty() ? _usrp->get_rx_gain(0) : _usrp->get_rx_gain(element, 0);
        } catch (const std::exception&) {
            return 0.0;
        }
    }

    /*| contract: the setting a control writes through, with the value it writes: ANTENNA as
            rx_antennae, the connector the index names. Empty for every other control and for
            an index past the unit's list.
        why: as the overall gain and rx_gains, for the reason setElementGain states.
        verified-by: controls.uhd-the-antenna-writes-through-its-setting
    */
    static gr::property_map settingForControl(const DeviceTruth& t, const std::string& id, double bounded) {
        if (id == "ANTENNA") {
            if (const auto index = optionIndexFor(bounded, t.antennas.size()); index.has_value()) {
                return {{"rx_antennae", std::vector<std::string>{t.antennas[*index]}}};
            }
        }
        return {};
    }

    /*| contract: the line refusing a receive connector the unit does not list, naming the
            ones it lists, and nothing for a connector it lists. A unit that states no list
            refuses nothing, and the device takes or refuses the name itself.
        verified-by: controls.uhd-refuses-an-antenna-the-unit-does-not-offer
    */
    static std::optional<std::string> antennaRefusal(std::span<const std::string> offered, const std::string& wanted) {
        if (offered.empty() || std::ranges::find(offered, wanted) != offered.end()) {
            return std::nullopt;
        }
        std::string names;
        for (const auto& name : offered) {
            names += names.empty() ? name : ", " + name;
        }
        return std::format("[uhd::Source] the antenna {} is not one this unit offers ({}); the antenna stays where it is", wanted, names);
    }

    /*| contract: stage ANTENNA through rx_antennae, the index bounded by the surface of the
            open device: true once the setting is staged, the connector reaching the device at
            the consumer thread's next settings drain, and a refusal there named on standard
            error alone. False for a block that is down, for an index past the unit's list and
            for a unit that states no ANTENNA.
        trap: staged with the control mutex free, for the reason setElementGain states.
    */
    bool stageAntennaControl(double value) {
        gr::property_map through;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp);
            if (!guard) {
                return false;
            }
            const TruthContext ctx{.deviceParams = {}, .sampleRate = _rateActualHz, .centerFreq = 0.0};
            const auto         bounded = boundedControlValue(describeControls(_openTruth, ctx, _bwCurHz), "ANTENNA", value, "uhd::Source");
            if (!bounded.has_value()) {
                return false;
            }
            through = settingForControl(_openTruth, "ANTENNA", *bounded);
        }
        if (through.empty()) {
            return false;
        }
        std::ignore = this->settings().setStaged(through);
        return true;
    }

    /*| contract: apply one non-gain control by id: ANTENNA, an index into the unit's own
            connector list, staged through rx_antennae as stageAntennaControl states; BANDWIDTH
            in MHz, the analog frontend filter; DC_CORR and IQ_CORR, the frontend's automatic
            corrections; CLOCK_SOURCE and TIME_SOURCE, each an index into the device's own
            option list; LO_OFFSET in MHz, which retunes; DEVICE_TIME, which sets the device
            clock and moves the timing tags onto it; WIRE_FORMAT, an index into the format
            list; MASTER_CLOCK in MHz, zero meaning the choice UHD makes. False for a control
            this device does not have.
        frame: WIRE_FORMAT, CLOCK_SOURCE, TIME_SOURCE and the automatic MASTER_CLOCK entry
            are stream and open-time arguments, so they are staged and take effect at the
            next start.
        contract: a staged value is named on standard error only where it moves what the
            next start takes, the value pinned before or else the one in force. A caller
            replays its stored controls after every start, and a replay prints nothing.
        verified-by: controls.uhd-a-replay-of-the-selection-in-force-says-nothing
        trap: the reference and the timing selection are not written to a streaming device,
            for the reason applyReferenceSources states. Writing a time source of external
            while the clock source is already external throws as well, UHD refusing one
            reference as both, and a throw at the next start leaves the device on the selection
            it had.
    */
    bool setControl(const std::string& id, double value) {
        if (id == "ANTENNA") {
            return stageAntennaControl(value);
        }
        CtrlGuard guard(_ctrlMutex, _deviceUp);
        if (!guard) {
            return false;
        }
        const TruthContext ctx{.deviceParams = {}, .sampleRate = _rateActualHz, .centerFreq = 0.0};
        const auto         bounded = boundedControlValue(describeControls(_openTruth, ctx, _bwCurHz), id, value, "uhd::Source");
        if (!bounded.has_value()) {
            return false; // an id the surface of the open device does not state, or an index past its list
        }
        const double want = *bounded;
        try {
            if (id == "BANDWIDTH") {
                if (_bwMaxHz <= _bwMinHz) {
                    return false; // a fixed filter takes no write
                }
                applyBandwidthLocked(want * 1e6);
                refreshFreqRanges(); // the coverage moves with the filter
                return true;
            }
            if (id == "DC_CORR") {
                if (!_openTruth.hasDcOffsetCorr) {
                    return false;
                }
                _usrp->set_rx_dc_offset(want >= 0.5, 0);
                return true;
            }
            if (id == "IQ_CORR") {
                if (!_openTruth.hasIqBalanceCorr) {
                    return false;
                }
                _usrp->set_rx_iq_balance(want >= 0.5, 0);
                return true;
            }
            if (id == "WIRE_FORMAT") {
                const auto index = optionIndexFor(want, wireFormats().size());
                if (!index.has_value()) {
                    return false;
                }
                /*| trap: the pinned format alone. _wireFormat is the format the running
                        streamer was built with, and the rate ladder in force is that
                        format's: moving it under the stream would offer a rate the link
                        cannot carry in the packing it is actually sending. */
                const std::string& format = wireFormats()[*index];
                const bool         moves  = format != _wireFormatPinned.value_or(_wireFormat);
                _wireFormatPinned         = format;
                if (moves) {
                    std::fprintf(stderr, "[uhd::Source] the wire format %s takes effect at the next start\n", format.c_str());
                }
                return true;
            }
            if (id == "MASTER_CLOCK") {
                return applyMasterClockLocked(want * 1e6);
            }
            if (id == "CLOCK_SOURCE" || id == "TIME_SOURCE") {
                const bool  clock = id == "CLOCK_SOURCE";
                const auto& names = clock ? _clockSources : _timeSources;
                const auto  at    = optionIndexFor(want, names.size());
                if (!at.has_value()) {
                    return false;
                }
                auto&              pinned = clock ? _clockSourcePinned : _timeSourcePinned;
                const std::string& name   = names[*at];
                const bool         moves  = name != pinned.value_or(clock ? _openTruth.clockSource : _openTruth.timeSource);
                pinned                    = name;
                if (moves) {
                    std::fprintf(stderr, "[uhd::Source] the %s source %s takes effect at the next start\n", clock ? "clock" : "time", name.c_str());
                }
                return true;
            }
            if (id == "LO_OFFSET") {
                const double offset  = boundedLoOffsetHz(want * 1e6, _rateActualHz);
                const double previous = _loOffsetHz;
                _loOffsetHz           = offset;
                if (_requestedFreqHz > 0.0) {
                    tuneLocked(_requestedFreqHz, _ppmApplied, false);
                    // A refused tune leaves the tuner where it was, so the offset it was
                    // refused for is not the offset in force either.
                    if (_loOffsetWrittenHz.value_or(previous) != offset) {
                        _loOffsetHz = previous;
                        return false;
                    }
                }
                return true;
            }
            if (id == "DEVICE_TIME") {
                const bool on = want >= 0.5;
                _deviceTime.store(on, std::memory_order_release);
                if (on) {
                    setDeviceTimeLocked();
                }
                return true;
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Source] setControl(%s, %.2f): %s\n", id.c_str(), value, e.what());
            return false;
        }
        return false;
    }

    // ---- live sensors ----------------------------------------------------------------

    // The sweep kinds, named on the block the way a caller asks for them.
    using SensorSweep = capture::uhd::SensorSweep;

    /*| contract: read the sensors the open device publishes, plus the reference and timing
            state that is not a sensor but belongs beside them. False, publishing nothing, when
            no device is up or the device goes down mid-sweep. Full is every sensor; Brief is
            the ones start() measured as cheap, and the default is Full so a caller that says
            nothing gets what it always got.
        frame: these are control-plane reads over the same transport the settings appliers use,
            so they are safe beside a running stream. They are also slow: a GPSDO answers its
            GPS sensors from the oscillator's 1 Hz serial link, where the epoch-time sensor
            waits for the next NMEA sentence by construction, so one full sweep costs the
            better part of a second.
        measured: on an N210 with a GPSDO a full sweep is about 700 ms and a brief one about
            4 ms; on a B205mini the two are the same 4 ms, every sensor there being cheap.
        trap: holding the control mutex across that, on a sweep a timer repeats, would put the
            mutex in a caller's way for a large fraction of all wall-clock time, and every tune,
            gain write and gain read-back landing in that window would wait out the rest of the
            sweep.
        why: so the control mutex is taken only to snapshot the device handle and the sensor
            names, and every blocking read runs outside it. The snapshot is a shared handle, so
            the device cannot be destroyed while a read is in flight; _sensorMutex bounds how
            long that snapshot can exist, because teardown takes it before releasing the device
            and so cannot return while a sweep still holds one. Clearing _deviceUp is teardown's
            other half of that handshake: it is the sweep's abort signal, checked between reads,
            so teardown waits for one sensor rather than a whole sweep.
    */
    bool readSensors(std::vector<SensorReading>& out, SensorSweep sweepKind = SensorSweep::Full) const {
        out.clear();
        std::lock_guard             sweep(_sensorMutex);
        ::uhd::usrp::multi_usrp::sptr usrp;
        SensorNames                   names;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp);
            if (!guard) {
                return false;
            }
            usrp  = _usrp; // shared: the device outlives a teardown that races this sweep
            names = _sensorNames;
        }
        if (usrp == nullptr || !readDeviceSensors(out, *usrp, names, Direction::Receive, sweepKind, [this] { return _deviceUp.load(std::memory_order_acquire); })) {
            return false;
        }
        appendReadings(out);
        /*| frame: what the unit is, which no sensor reports, read once when the device was
                opened, so it costs a poll nothing. It is a diagnostic.
        */
        if (const std::string what = identity(); !what.empty()) {
            out.push_back({.id = "device_model", .label = "Device", .value = what, .good = true, .brief = false});
        }
        return true;
    }

    /*| contract: what this stream has lost, beside the device's own sensors, because a receiver
            polls the reading set and has nowhere else to read it, and the analog filter the
            device settled on. The overflows, the dropped samples and the filter are glance rows,
            and the lost packets and markers are diagnostics.
        frame: the filter is a reading because a frontend with a fixed filter states its width
            and no control reaches it.
        frame: the three counts are different quantities. UHD raises one overflow event however
            much of the signal went with it, only the sample count says how much that was, and
            rx_sequence_errors counts packets the link lost rather than a buffer that ran over.
    */
    void appendReadings(std::vector<SensorReading>& out) const {
        out.push_back({.id = "rx_overflows", .label = "Overflows", .value = std::format("{}", overflowEvents()), .good = true, .brief = true});
        if (const std::string depth = recvDepth().describe(); !depth.empty()) {
            out.push_back({.id = "rx_transport_depth", .label = "Receive transport", .value = depth, .good = true, .brief = false});
        }
        out.push_back({.id = "rx_dropped_samples", .label = "Dropped samples", .value = std::format("{}", droppedSamples()), .good = true, .brief = true});
        out.push_back({.id = "rx_sequence_errors", .label = "Lost packets", .value = std::format("{}", sequenceErrors()), .good = true, .brief = false});
        out.push_back({.id = "rx_dropped_markers", .label = "Lost markers", .value = std::format("{}", droppedTags()), .good = true, .brief = false});
        if (const double filterHz = analogBandwidthHz(); filterHz > 0.0) {
            out.push_back(analogBandwidthReading(filterHz));
        }
    }

    /*| contract: the receive transport in use: the frames and socket buffer the open address
            asked for or UHD's defaults for the kind, the frame the streamer chose, and the rate
            in force. No frame before the streamer exists or after it has gone.
        verified-by: controls.uhd-the-receive-depth-is-stated-in-time
    */
    RecvDepth recvDepth() const {
        std::lock_guard lock(_ctrlMutex);
        return recvDepthFor(_recvFramesAsked, _recvBuffAsked, _openTruth.deviceKind, _maxRecvSamples, _rateActualHz, _wireFormat);
    }

    // The unit's own name, as the device stated it when this block opened it.
    std::string identity() const {
        std::lock_guard lock(_ctrlMutex);
        return _openTruth.describe();
    }

    // ---- lifecycle -------------------------------------------------------------------

    // The address this block opens the device with, the settings and any pinned master
    // clock folded into the caller's selector.
    ::uhd::device_addr_t openAddress() const {
        return ::uhd::device_addr_t(withDeviceArgs(device_parameter.value, _masterClockPinnedHz.value_or(master_clock_rate.value), Direction::Receive, recv_frame_size, num_recv_frames, recv_buff_size));
    }

    /*| contract: the words a failed open adds for a clock pin a product table's list made, which
            the open carried, with the pin cleared, so the next start opens with the
            master_clock_rate setting. Empty, and nothing cleared, where the pin is none, the
            automatic entry, or a clock a live write already put on the device. Called with
            _ctrlMutex held.
        why: a table clock the loaded image or the daughterboard refuses fails every open that
            carries it, and a stopped block refuses every control write. A pin nothing clears
            would keep the block from starting again. A clock the device already ran is not the
            cause of a failed open, and an unplugged unit or one another process holds keeps it.
        verified-by: controls.uhd-a-failed-open-clears-the-clock-pin
    */
    std::string clearClockPinAfterFailedOpenLocked() {
        if (!_masterClockPinFromTable || !_masterClockPinnedHz.has_value() || *_masterClockPinnedHz <= 0.0) {
            return {};
        }
        const double pinned = *_masterClockPinnedHz;
        _masterClockPinnedHz.reset();
        _masterClockPinFromTable = false;
        return std::format("; the master clock pinned at {:g} MHz is cleared, and the next start opens with the master_clock_rate setting", pinned / 1e6);
    }

    /*| contract: the line naming a pinned master clock the open did not put in force: the pin
            above zero and the clock the device reads more than a hertz from it. Empty
            otherwise, and where the device read no clock.
        why: a difference has more than one cause. UHD hands a second open in one process the
            device already open, with the first open's keys, so a pin reaches the radio only at
            an open that opens it; and a driver can coerce a clock inside its range to the
            nearest one it reaches.
        verified-by: controls.uhd-a-table-clock-applies-at-the-next-open
    */
    static std::string clockPinMissedLine(double pinnedHz, double readHz) {
        if (pinnedHz <= 0.0 || readHz <= 0.0 || std::abs(pinnedHz - readHz) <= 1.0) {
            return {};
        }
        return std::format("[uhd::Source] the device runs a {:g} MHz master clock, not the pinned {:g} MHz; one cause: UHD hands a second open in one process the device already open, with the first open's keys", readHz / 1e6,
                           pinnedHz / 1e6);
    }

    void start() {
        noteOutputAtStart(*this);
        _lastTagTimeNs = 0;
        _streamDead.store(false, std::memory_order_release);
        _overflows.store(0, std::memory_order_relaxed);
        _seqErrors.store(0, std::memory_order_relaxed);
        _rateBreaks.store(0, std::memory_order_relaxed);
        _dropped.store(0, std::memory_order_relaxed);
        // A fresh multi_usrp holds none of the previous device's state, so nothing an
        // applier wrote before is still in force.
        _freqWrittenHz.reset();
        _loOffsetWrittenHz.reset();
        _rateWrittenHz.reset();
        _gainWrittenDb.reset();
        _bwWrittenHz.reset();
        _agcWritten.reset();
        _antennaWritten.reset();
        // Including the elements a caller moved while an automatic loop held the gain path:
        // the next device may not name them, and it holds none of this one's state.
        _heldGains.clear();
        _rateActualHz         = 0.0;
        _bwCurHz              = 0.0;
        _tunedRfHz            = 0.0;
        _refLockWaitMs        = 0.0;
        _maxRecvSamples       = 0UZ;
        _agcUnavailableLogged = false;
        _clockSetPending.store(false, std::memory_order_release);
        _deviceClockSet.store(false, std::memory_order_release);
        _deviceToHostNs.store(0, std::memory_order_relaxed);
        _restageRate.store(false, std::memory_order_release);
        _clockInForceHz.reset();
        publishSurfaceLocked();
        {
            std::lock_guard tunes(_tuneMutex);
            _tuneResults.clear();
        }
        _tuneResultsPosted.store(0, std::memory_order_release);
        _tuneRequests = 0;
        _retunesOpen.store(0, std::memory_order_release);
        _wireFormat           = validWireFormat(_wireFormatPinned.value_or(wire_format.value));
        _surfaceAgc.store(gain_mode.value, std::memory_order_release);
        awaitReleases();
        try {
            _usrp = ::uhd::usrp::multi_usrp::make(openAddress());
        } catch (const std::exception& e) {
            _usrp.reset();
            std::string cleared;
            {
                std::lock_guard lock(_ctrlMutex);
                cleared = clearClockPinAfterFailedOpenLocked();
            }
            throwStartFailure("uhd::Source", std::format("uhd make({}): {}{}", device_parameter.value, e.what(), cleared));
        }
        {
            // The address parsed for the open above, so it parses here.
            const ::uhd::device_addr_t address = openAddress();
            std::lock_guard            lock(_ctrlMutex);
            _recvFramesAsked = addressNumber(address, "num_recv_frames");
            _recvBuffAsked   = addressNumber(address, "recv_buff_size");
        }

        if (stopArrivedDuringOpen(this->state())) {
            _usrp.reset();
            return;
        }

        /*| invariant: the reference and the timing selection are written before anything
                else this start configures, and the device time after them, which is the
                order the vendor states.
        */
        {
            std::lock_guard lock(_ctrlMutex);
            applyReferenceSourcesLocked();
        }

        /*| invariant: the whole control surface is read here, once, from the device this
                block opened, and every applier and descriptor below reads it rather than
                asking the device again. The gain element names, the analog filter, the
                reference and timing lists and the rate set all differ per model and per
                daughterboard.
        */
        std::string truthFailure;
        {
            std::lock_guard lock(_ctrlMutex);
            try {
                readTruthInto(*_usrp, _openTruth);
            } catch (const std::exception& e) {
                /*| invariant: caught here rather than left to escape, so the device is
                        handed off and the record cleared before the start throws. A block
                        whose start threw is never given stop().
                */
                truthFailure = e.what();
                _openTruth   = {};
            }
            if (const std::string missed = clockPinMissedLine(_masterClockPinnedHz.value_or(0.0), _openTruth.mclkHz); !missed.empty()) {
                std::fprintf(stderr, "%s\n", missed.c_str());
            }
            _bwMinHz      = _openTruth.bwMinHz;
            _bwMaxHz      = _openTruth.bwMaxHz;
            _bwCurHz      = _openTruth.bwCurHz;
            _agcSupported = _openTruth.hasAgc;
            _clockSources = _openTruth.clockSources;
            _timeSources  = _openTruth.timeSources;
            takeClockInForceLocked();
            publishSurfaceLocked();
        }
        if (!truthFailure.empty()) {
            failStartOnOpenDevice(std::format("uhd read the control surface of {}: {}", device_parameter.value, truthFailure));
        }

        // The coverage is read per rate by applySampleRate below; this is where the tuner
        // sits until a request is accepted, and what a refused request forwards.
        _freqRanges.clear();
        try {
            _appliedFreqHz = _usrp->get_rx_freq(0);
        } catch (const std::exception&) {
            _appliedFreqHz = 0.0;
        }

        // The set of sensors stays fixed while the device is open. The names are
        // enumerated once, and each tick reads the values alone.
        _sensorNames = sensorNamesOf(*_usrp, Direction::Receive, "uhd::Source");

        applySampleRate();
        applyAntenna();
        applyFrequency();
        applyAgc(); // before gains: manual gain writes are skipped while AGC holds them
        applyGain();
        if (_deviceTime.load(std::memory_order_acquire)) {
            std::lock_guard lock(_ctrlMutex);
            setDeviceTimeLocked();
        }
        _refLockWaitMs = capture::uhd::waitForReference(*_usrp, "uhd::Source").waitedMs;

        try {
            ::uhd::stream_args_t streamArgs("fc32", _wireFormat);
            streamArgs.channels = {0};
            _stream             = _usrp->get_rx_stream(streamArgs);
            {
                std::lock_guard lock(_ctrlMutex);
                _maxRecvSamples = _stream->get_max_num_samps();
                // The two clocks read beside each other, which carries every device
                // timestamp into the host's epoch for as long as this stream runs.
                measureClockOffsetLocked();
            }
            ::uhd::stream_cmd_t cmd(::uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
            cmd.stream_now = true;
            _stream->issue_stream_cmd(cmd);
        } catch (const std::exception& e) {
            failStartOnOpenDevice(std::format("uhd rx stream: {}", e.what()));
        }

        _deviceUp.store(true, std::memory_order_release);
        startThreads();
        offerControlProperties(*this);
    }

    /*| contract: end a start that has opened the device. The control thread and the consumer
            thread are joined, the streamer and the device go to the releasing thread as a stop
            hands them over, and a StartFailure carrying reason leaves the call. The next open
            waits for the release.
        verified-by: controls.uhd-a-thread-that-does-not-start-ends-the-start
    */
    [[noreturn]] void failStartOnOpenDevice(std::string_view reason, std::source_location where = std::source_location::current()) {
        std::ignore = hardwareTeardown();
        throwStartFailure("uhd::Source", reason, where);
    }

    /*| contract: start the control thread and then the consumer thread on the open device. A
            thread the system does not start ends the start through failStartOnOpenDevice, which
            joins the control thread where it is running.
        verified-by: controls.uhd-a-thread-that-does-not-start-ends-the-start
    */
    void startThreads() {
        try {
            _control.start("uhdctl", this->name.value);
            _consumer.clearStop();
            _consumer.thread = std::thread([this, priority = thread_priority.value, processor = cpu.value] {
                nameStreamingThread("uhd", this->name.value);
                std::ignore = placeStreamingThread("uhd::Source", "receive", priority, processor);
                consumerLoop();
            });
        } catch (const std::system_error& e) {
            failStartOnOpenDevice(std::format("uhd could not start a thread: {}", e.what()));
        }
    }

    /*| contract: put the device on the reference and the timing selection this block was
            given — the choice a control pinned where one was made, the settings otherwise —
            and read both back where either was asked for. An empty name leaves the device on
            its own selection. Called with _ctrlMutex held, from start() alone, before the
            streamer exists and before the device time is set, where applyReferenceSources
            says the writes belong.
    */
    void applyReferenceSourcesLocked() {
        const std::string wantClock = _clockSourcePinned.value_or(clock_source.value);
        const std::string wantTime  = _timeSourcePinned.value_or(time_source.value);
        if (wantClock.empty() && wantTime.empty()) {
            return;
        }
        applyReferenceSources(*_usrp, wantClock, wantTime, "uhd::Source");
        try {
            std::fprintf(stderr, "[uhd::Source] the device is disciplined by the %s reference and timed by %s\n", _usrp->get_clock_source(0).c_str(), _usrp->get_time_source(0).c_str());
        } catch (const std::exception&) { // a radio that states no selection
        }
    }

    void stop() {
        if (hardwareTeardown()) {
            this->publishEoS();
        }
    }

    /*| contract: idempotent and serialized. It returns true to exactly one caller per open
            device — the one that released it — so exactly one end-of-stream tag follows.
        why: also run by _teardown's destructor when stop() never fired. Serialized because two
            threads can request a stop at once, a liveness-detected failure on the scheduler
            thread and an external stop on a caller's, and unserialized they would double-join
            one thread.
        measured: the caller waits about half a second in the worst case and a few
            milliseconds in the ordinary one — the consumer's receive in flight, at most
            0.1 s, and then the drain, at most 0.3 s. A receiver reaches stop() from the
            thread that serves its interface, so that wait is the interface's.
        trap: longer only where one receive overruns its own timeout, which the vendor
            allows: the timeout applies to every internal call inside recv rather than to the
            whole of it.
        verified-by: uhd.stop-latency
    */
    bool hardwareTeardown() {
        std::lock_guard teardown(_teardownMutex);
        const bool hadDevice = _usrp != nullptr;
        _deviceUp.store(false, std::memory_order_release);
        _consumer.requestStop();
        // Ahead of the join: a consumer waiting for the control thread to settle is let go.
        _control.stop();
        _consumer.joinIfRunning();
        if (_stream != nullptr) {
            std::ignore = stopAndDrainStream();
        }
        /*| invariant: a sensor sweep is waited out before the device is handed to the releasing
                thread. A sweep reads through a shared handle it snapshotted, and a snapshot that
                outlived this call would hold the device open past the release the next start()
                waits for, and USRPs are exclusive.
            frame: _deviceUp is already false, so an in-flight sweep aborts at its next read and
                this waits for one sensor rather than a whole sweep.
        */
        std::lock_guard sensors(_sensorMutex);
        std::lock_guard lock(_ctrlMutex);
        releaseLater(std::move(_stream), std::move(_usrp));
        /*| invariant: the accessors answer what they promise for a block that is down. The
                record, the frame the streamer chose and the frequency the tuner reached all
                belong to a device this block has released.
            frame: the analog filter width is the exception and is kept on purpose: a caller
                drawing a control surface for a stopped radio has to state the width the
                last device settled on, and has nowhere else to read it. The accessor says
                so.
        */
        _openTruth      = {};
        _maxRecvSamples = 0UZ;
        _tunedRfHz      = 0.0;
        publishSurfaceLocked();
        return hadDevice;
    }

    gr::work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!gr::lifecycle::isActive(this->state()) || stopWhenOutputIsUnconnected(*this)) {
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        if (_deviceUp.load(std::memory_order_acquire) && _streamDead.load(std::memory_order_acquire)) {
            this->emitErrorMessage("work()", "UHD stream dead (persistent recv timeouts — unplug?)");
            this->requestStop();
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        return {requestedWork, 0UZ, gr::work::Status::OK};
    }

    // ---- staged settings (drained on the consumer thread) ----------------------------

    /*| contract: runs on the consumer thread, which drains the staged settings. The frequency,
            the gain, the automatic gain mode and the antenna are copied out of the settings
            here and staged on the control thread; a rate change runs here, after the control
            thread has settled, because it stops the stream, drains it and starts it again.
        contract: the gain is bounded here, and a gain the bound moved is written back into
            rx_gains here, before the control thread writes the bounded value to the device.
        invariant: nothing forwards the frequency from here. The consumer thread places the
            frequency tag on the first sample the tune reaches, from what the control thread
            hands back.
        verified-by: controls.uhd-a-retune-forwards-nothing-ahead-of-its-samples
    */
    void settingsChanged(const gr::property_map& oldSettings, gr::property_map& newSettings, gr::property_map& forwardSettings) {
        const ControlSurfaceCheck<Source> surfaceCheck{*this}; // sends a moved surface once the body has let go of its locks
        if (newSettings.contains("gain_mode")) {
            _surfaceAgc.store(gain_mode.value, std::memory_order_release);
        }
        clearPinsFor(oldSettings, newSettings);
        // The atomic rather than the handle: the handle is written under the control mutex
        // this test does not hold, and reading it here races that write.
        if (!_deviceUp.load(std::memory_order_acquire)) {
            return;
        }
        if (newSettings.contains("frequency") || newSettings.contains("frequency_correction")) {
            forwardSettings.erase(std::pmr::string("frequency"));
            if (!frequency->empty()) {
                const std::uint64_t request = ++_tuneRequests;
                _control.post(kControlFrequency, [this, request, want = frequency->front(), ppm = frequency_correction.value] { retune(request, want, ppm); });
            }
        }
        if (newSettings.contains("sample_rate")) {
            // The writes given before this one reach the device first, within a bound.
            if (!_control.settleFor(std::chrono::duration<double>(kControlSettleLimitS))) {
                std::fprintf(stderr, "[uhd::Source] a control write still runs after %.1f s; the rate is written behind it\n", kControlSettleLimitS);
            }
            applySampleRate();
            forwardSettings.insert_or_assign(std::pmr::string("sample_rate"), sample_rate.value);
        }
        if (newSettings.contains("rx_gains") && !rx_gains->empty()) {
            _control.post(kControlGain, [this, want = boundGainSetting()] { applyGainOf(want); });
        }
        if (newSettings.contains("gain_mode")) {
            const std::optional<double> overall = rx_gains->empty() ? std::nullopt : std::optional<double>(rx_gains->front());
            _control.post(kControlAgc, [this, on = gain_mode.value, overall] {
                applyAgcOf(on);
                if (!on) {
                    applyGainReleaseOf(overall); // the overall gain, then every element the loop was holding
                }
            });
        }
        if (newSettings.contains("rx_antennae") && !rx_antennae->empty() && !rx_antennae->front().empty()) {
            _control.post(kControlAntenna, [this, want = rx_antennae->front()] {
                std::ignore = applyAntennaOf(want);
                publishControlSurface(*this); // a landed write moves the surface's default
            });
        }
    }

    /*| contract: drop the pins a settings write makes stale: every pin where device_parameter
            moves to another value, and the pin a setting shadows where the settings pass
            carries that setting, master_clock_rate, wire_format, clock_source or time_source.
            Runs whether or not the device is up.
        frame: the settings pass carries a setting whose staged value differs from the value
            it holds; a write of the value held reaches no hook and clears nothing.
        why: a pin outlives the stream it was made during and shadows its setting at the next
            start. A pin made for one radio fails the open of another: 184.32 MHz fails on an
            N310 and on a B2xx. A write of the setting states what the caller wants next.
        verified-by: controls.uhd-a-settings-write-clears-the-pins
    */
    void clearPinsFor(const gr::property_map& oldSettings, const gr::property_map& newSettings) {
        const auto deviceMoved = [&] {
            if (!newSettings.contains("device_parameter")) {
                return false;
            }
            const auto old = oldSettings.find("device_parameter");
            return old == oldSettings.end() || old->second.value_or(std::string_view{}) != std::string_view(device_parameter.value);
        }();
        const auto shadowed = [&newSettings](std::string_view key) { return newSettings.contains(key); };
        if (!deviceMoved && std::ranges::none_of(std::array<std::string_view, 4>{"master_clock_rate", "wire_format", "clock_source", "time_source"}, shadowed)) {
            return; // the control mutex is taken only for a clear, so a pass of other settings never waits on a tune
        }
        std::lock_guard lock(_ctrlMutex);
        if (deviceMoved || newSettings.contains("master_clock_rate")) {
            _masterClockPinnedHz.reset();
            _masterClockPinFromTable = false;
        }
        if (deviceMoved || newSettings.contains("wire_format")) {
            _wireFormatPinned.reset();
        }
        if (deviceMoved || newSettings.contains("clock_source")) {
            _clockSourcePinned.reset();
        }
        if (deviceMoved || newSettings.contains("time_source")) {
            _timeSourcePinned.reset();
        }
    }

    // ---- device-specific appliers (log-and-continue: see class comment) --------------

    void applyFrequency() {
        if (frequency->empty()) {
            return;
        }
        std::lock_guard lock(_ctrlMutex);
        if (_usrp == nullptr) {
            return; // teardown resets the handle under this mutex
        }
        // Called by start() alone, before the consumer thread drains any setting; a retune
        // while streaming takes the values the drain copied out, and every control path reads
        // the mirrors.
        _requestedFreqHz = frequency->front();
        _ppmApplied      = frequency_correction;
        tuneLocked(_requestedFreqHz, _ppmApplied, true);
    }

    /*| contract: tune to the request given, under the correction given and the LO offset in
            force, and with forwardSetting, put the frequency the tuner holds into the
            frequency setting. Called with _ctrlMutex held and a live device.
        contract: true where the tuner was written and took the write.
        why: forwardSetting is false for a write from any thread but the one that drains the
            settings: a control written from a caller's own thread, and a retune on the control
            thread. The setting is the block's, written by the thread that drains the settings,
            and a second writer of it would be a race. The request and the correction are
            arguments for the same reason: those threads pass the mirrors the applier left or
            the values the drain copied, never the settings members themselves.
        trap: an LO offset of zero is tuned without one rather than with an offset of zero.
            The two are different requests: tune_request_t(f, 0) pins the RF frontend at f
            and leaves the residue to the digital mixer, where tune_request_t(f) lets UHD
            place the frontend where it likes.
        why: the reference correction scales the center and not the offset. The two-argument
            tune request pins the frontend at the center plus the offset and lets the digital
            down-converter take the rest, and that converter's oscillator runs at the tick rate
            the same reference sets. Both stages are therefore realized a factor of one plus
            the fraction away from the value they were commanded at, and the offset the mixer
            removes moves with the frontend that carries it: the net center lands on the
            request whatever the offset is. An offset scaled here would move the net center by
            the offset times the fraction instead.
        trap: the correction fixes the center and nothing else. The sample rate carries the
            same error and the block does not scale the rate it states, so a caller reading its
            axis from the nominal rate places a component of true offset b at b divided by one
            plus the fraction: 100 Hz low at an offset of 1 MHz and 100 ppm. See
            tuneFrequencyFor.
    */
    bool tuneLocked(double requested, double ppm, bool forwardSetting) {
        const auto f = tuneFrequencyFor(requested, ppm, _freqRanges, _loOffsetHz);
        if (!f) {
            std::fprintf(stderr, "[uhd::Source] %.0f Hz with an LO offset of %.0f Hz puts the tuner outside this radio's coverage; it stays where it is\n", requested, _loOffsetHz);
            if (forwardSetting) {
                frequency = std::vector<double>{_appliedFreqHz};
            }
            return false;
        }
        bool moved = false;
        if (writeNeeded(_freqWrittenHz, *f) || writeNeeded(_loOffsetWrittenHz, _loOffsetHz)) {
            try {
                const auto result  = _loOffsetHz == 0.0 ? _usrp->set_rx_freq(::uhd::tune_request_t(*f), 0) : _usrp->set_rx_freq(::uhd::tune_request_t(*f, _loOffsetHz), 0);
                _tunedRfHz         = result.actual_rf_freq;
                _freqWrittenHz     = *f;
                _loOffsetWrittenHz = _loOffsetHz;
                _appliedFreqHz     = tunedFrequencyFrom(_usrp->get_rx_freq(0), ppm);
                moved              = true;
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[uhd::Source] set_rx_freq(%.0f) failed: %s (continuing)\n", *f, e.what());
            }
        }
        if (forwardSetting) {
            frequency = std::vector<double>{_appliedFreqHz};
        }
        return moved;
    }

    /*| contract: tune on the control thread to the request given, under the correction given,
            and hand the consumer thread the frequency the tuner holds afterwards and the device
            clock read once the tune returned. request numbers the frequency request, so the
            consumer thread writes the setting back from the newest one alone.
        frame: a request the coverage refuses moves nothing and hands back the frequency in
            force, which the setting returns to. A tune made while a device-clock set is in
            flight carries no clock reading, because the stamps that follow may belong to
            either clock, and its tag goes on the next block.
    */
    void retune(std::uint64_t request, double want, double ppm) {
        _retunesOpen.fetch_add(1, std::memory_order_acq_rel);
        TuneResult result;
        result.request = request;
        {
            std::lock_guard lock(_ctrlMutex);
            if (_usrp == nullptr) {
                _retunesOpen.fetch_sub(1, std::memory_order_acq_rel);
                return; // teardown resets the handle under this mutex
            }
            _requestedFreqHz = want;
            _ppmApplied      = ppm;
            const bool moved = tuneLocked(want, ppm, false);
            result.heldHz    = _appliedFreqHz;
            if (moved && !_clockSetPending.load(std::memory_order_acquire)) {
                try {
                    result.after = _usrp->get_time_now(0);
                } catch (const std::exception&) { // no reading: the tag goes on the next block
                }
            }
        }
        {
            std::lock_guard tunes(_tuneMutex);
            _tuneResults.push_back(std::move(result));
        }
        _tuneResultsPosted.fetch_add(1, std::memory_order_acq_rel);
    }

    // The shared filter write on this block's device. Called with _ctrlMutex held and a live
    // device.
    void applyBandwidthLocked(double hz) {
        applyBandwidth(*_usrp, Direction::Receive, hz, _bwWrittenHz, _bwCurHz, "uhd::Source");
        publishSurfaceLocked();
    }

    /*| contract: put the converter clock on the rate asked for, in Hz, and re-derive
            everything that hangs off it — the stated rate set, the clock in force, and the
            rate the consumer thread is then owed. Zero pins the automatic choice, which the
            next start restores. Called with _ctrlMutex held and a live device.
        why: every reachable rate is the clock over a decimation, so a clock written under a
            running stream moves the rate as well.
        trap: this runs on whichever thread called setControl, which in a receiver is the one
            that serves the interface. It therefore writes the device and nothing else: the
            rate setting is the block's, written by the appliers on the consumer thread, and
            a second writer of it would be a race. The flag asks that thread for the write
            and for the forward that carries the new rate downstream.
        measured: a B205mini put on a 30.72 MHz clock reaches 7.68 MS/s exactly. UHD's own
            choice does not come back while the device stays open: the explicit write clears
            the automatic flag inside the driver and only a fresh open sets it again, which
            is why the clock in force outlives a request for the automatic entry.
        contract: a clock within a hertz of the clock in force is pinned and answers true with no
            device write and no restage of the rate.
        why: a caller replays its stored clock after every start, and a clock the open address
            carried is in force from the first sample. Written again, it resets the rate cache
            and restages the rate, and the consumer thread then stops the stream, marks a rate
            break and names a rate move at every start for a rate that did not move.
        verified-by: controls.uhd-a-replay-of-the-clock-in-force-writes-nothing
    */
    bool applyMasterClockLocked(double hz) {
        if (hz <= 0.0) {
            // No clock in force is UHD's own choice, which the open left automatic.
            const bool moves         = _masterClockPinnedHz.value_or(_clockInForceHz.value_or(0.0)) != 0.0;
            _masterClockPinnedHz     = 0.0;
            _masterClockPinFromTable = false;
            if (moves) {
                std::fprintf(stderr, "[uhd::Source] the automatic master clock takes effect at the next start; this device keeps the clock it was given\n");
            }
            return true;
        }
        /*| contract: on a radio stating one clock, a clock its product takes is pinned for the
                next open and the device is not written. The bound has already refused a value
                that is not an entry. A pin that moves the next open is named on standard error,
                with the one case in which the next start does not apply it.
            why: UHD's manual names these clocks as values of the master_clock_rate argument at
                the open, and states that an X3xx and an X4xx keep the clock they were opened at
                for the whole session.
            trap: UHD hands a second open in one process the device already open, with the
                first open's keys. A pin reaches the radio only at an open that opens it, and
                the start compares the clock it reads with the pin.
            verified-by: controls.uhd-a-table-clock-applies-at-the-next-open
        */
        if (offersOneClockRate(_openTruth)) {
            const auto ladder = masterClockLadderMHz(_openTruth);
            if (std::ranges::none_of(ladder, [hz](double mhz) { return mhz > 0.0 && std::abs(mhz * 1e6 - hz) < 1.0; })) {
                return false;
            }
            const bool moves         = std::abs(hz - _masterClockPinnedHz.value_or(_openTruth.mclkMinHz)) >= 1.0;
            _masterClockPinnedHz     = hz;
            _masterClockPinFromTable = true;
            if (moves) {
                std::fprintf(stderr, "[uhd::Source] the master clock %g MHz takes effect at the next start that opens the device; a device another block of this process holds open keeps its clock\n", hz / 1e6);
            }
            return true;
        }
        if (_clockInForceHz.has_value() && std::abs(*_clockInForceHz - hz) < 1.0) {
            _masterClockPinnedHz     = hz;
            _masterClockPinFromTable = false;
            return true;
        }
        try {
            _usrp->set_master_clock_rate(hz, 0);
            _masterClockPinnedHz     = hz;
            _masterClockPinFromTable = false;
            _openTruth.mclkHz    = _usrp->get_master_clock_rate(0);
            _clockInForceHz      = _openTruth.mclkHz;
            _openTruth.rates.clear();
            for (const auto& r : _usrp->get_rx_rates(0)) {
                _openTruth.rates.emplace_back(r.start(), r.stop());
            }
            publishSurfaceLocked();
            _rateWrittenHz.reset(); // the clock moved under it, so the rate has to be written again
            _restageRate.store(true, std::memory_order_release);
            return true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Source] set_master_clock_rate(%.0f): %s (the clock stays where it is)\n", hz, e.what());
            return false;
        }
    }

    /*| contract: ask the settings for the rate the clock now in force reaches nearest the
            rate the radio was running, so that the consumer thread performs the write and
            the settings machinery forwards the new rate downstream. Called on the consumer
            thread, holding no lock while it stages.
        trap: staged outside the control mutex. The settings drain takes the settings
            machinery's mutex and then this block's control mutex, so staging under the
            control mutex is the opposite order and the two deadlock.
        verified-by: uhd.master-clock
    */
    void restageRateForClock() {
        double want = 0.0;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp);
            if (!guard) {
                return;
            }
            want = _rateActualHz > 0.0 ? rateToWrite(_rateActualHz) : 0.0;
        }
        if (want > 0.0) {
            std::ignore = this->settings().setStaged({{"sample_rate", want}});
        }
    }

    /*| contract: whether a time source disciplines the device clock with a pulse per second,
            which is the edge a whole second can be latched on.
        frame: external is a PPS input, gpsdo is the oscillator's own pulse, mimo is the
            pulse of the radio upstream, and UHD spells an external source it drives itself
            with underscores. internal and none have no edge to wait for.
        verified-by: controls.uhd-device-time-from-the-pps
    */
    static bool timeSourceHasPps(std::string_view source) { return source == "external" || source == "_external_" || source == "gpsdo" || source == "mimo"; }

    /*| contract: the whole seconds a sensor's own value text carries, or nothing where the
            text holds no number at all.
        why: sensor_value_t::to_int() is declared signed, which is 32 bits here, so a cast to
            a 64-bit second is applied after the narrowing and widens nothing. UTC seconds
            pass that width on 2038-01-19, after which the accessor throws or answers
            negative and a disciplined oscillator's own second is refused with nothing said.
            The sensor carries its value as text, which has no such width.
        verified-by: controls.uhd-reads-a-disciplined-second
    */
    static std::optional<std::int64_t> wholeSecondsFrom(std::string_view text) {
        std::size_t at = 0;
        while (at < text.size() && (text[at] == ' ' || text[at] == '+')) {
            ++at;
        }
        std::int64_t seconds = 0;
        const auto [ptr, ec] = std::from_chars(text.data() + at, text.data() + text.size(), seconds);
        std::ignore          = ptr;
        return ec == std::errc{} ? std::optional<std::int64_t>{seconds} : std::nullopt;
    }

    /*| contract: the whole second to latch on the next pulse: the second after the GPS time
            the oscillator reports where it has a fix, and the second after the host's clock
            otherwise.
        why: a GPSDO with a fix knows UTC to a fraction of a microsecond, and the host clock
            does not, so a device disciplined by one and timestamped from the other carries
            the host's error into every recording. gpsSeconds names the second the GPS was
            in when it was read, so the edge one second later is the start of the next.
        invariant: a GPS time at or below zero, which an oscillator without a fix reports,
            falls back to the host.
        verified-by: controls.uhd-device-time-from-the-pps
    */
    static std::int64_t ppsWholeSecond(bool gpsLocked, std::int64_t gpsSeconds, std::uint64_t hostNs) {
        if (gpsLocked && gpsSeconds > 0) {
            return gpsSeconds + 1;
        }
        return static_cast<std::int64_t>(hostNs / 1'000'000'000UL) + 1;
    }

    /*| contract: set the device clock, so a block's own timestamp reads as a wall-clock
            time. Called with _ctrlMutex held and a live device.
        frame: with a pulse-per-second source in force the device latches a whole second on
            the next edge, which is where two radios align to better than a packet and where
            a disciplined oscillator's own UTC reaches the samples; otherwise the host clock
            goes on at once, within a fraction of a millisecond.
        trap: the next edge is up to a second away, and until it arrives the device clock is
            neither the host's nor UTC: it is the seconds since the device came up. The block
            holds the whole second it asked for and the deadline it waits until, and the
            consumer thread reads the stream's own stamps to see the new clock arrive. Until
            then the timing tags carry the host's epoch and the stamps measure no shed.
        measured: set_time_now on a B205mini landed 0.16 ms from the host clock; an N210 on
            its GPSDO latches the GPS second and get_time_last_pps then reads the same second
            the gps_time sensor does.
    */
    void setDeviceTimeLocked() {
        _deviceClockSet.store(false, std::memory_order_release);
        try {
            const std::uint64_t ns   = wallClockNs();
            const auto          host = ::uhd::time_spec_t(static_cast<std::int64_t>(ns / 1'000'000'000UL), static_cast<double>(ns % 1'000'000'000UL) / 1e9);
            _clockLatchDeadlineNs.store(ns + static_cast<std::uint64_t>(kClockLatchLimitMs * 1e6), std::memory_order_relaxed);
            if (!timeSourceHasPps(_usrp->get_time_source(0))) {
                _usrp->set_time_now(host, 0);
                _clockTargetSec.store(static_cast<std::int64_t>(ns / 1'000'000'000UL), std::memory_order_relaxed);
                _clockSetPending.store(true, std::memory_order_release);
                return;
            }
            bool         gpsLocked  = false;
            std::int64_t gpsSeconds = 0;
            try {
                gpsLocked = _usrp->get_mboard_sensor("gps_locked", 0).to_bool();
                if (gpsLocked) {
                    // Out of the reading's own text, which carries no width, rather than
                    // through the 32-bit accessor beside it.
                    const auto reading = _usrp->get_mboard_sensor("gps_time", 0);
                    gpsSeconds         = wholeSecondsFrom(reading.value).value_or(static_cast<std::int64_t>(reading.to_real()));
                }
            } catch (const std::exception&) { // no GPS discipline on this unit
            }
            if (gpsLocked && gpsSeconds <= 0) {
                std::fprintf(stderr, "[uhd::Source] the oscillator reads locked and its second could not be read; the device clock takes the host's\n");
            }
            const std::int64_t latched = ppsWholeSecond(gpsLocked, gpsSeconds, ns);
            _usrp->set_time_next_pps(::uhd::time_spec_t(latched), 0);
            _clockTargetSec.store(latched, std::memory_order_relaxed);
            _clockSetPending.store(true, std::memory_order_release);
        } catch (const std::exception& e) {
            _clockSetPending.store(false, std::memory_order_release);
            std::fprintf(stderr, "[uhd::Source] setting the device time: %s (continuing)\n", e.what());
        }
    }

    /*| contract: read the offset between the device clock and the host clock into
            _deviceToHostNs. Called with _ctrlMutex held and a live device.
        why: the device stamps every packet with its own clock, and a tag has to name a host
            wall-clock time whether that clock has been set or not. One reading of the two
            clocks beside each other carries every later stamp across.
    */
    void measureClockOffsetLocked() {
        try {
            const std::uint64_t hostNs   = wallClockNs();
            const std::int64_t  deviceNs = _usrp->get_time_now(0).to_ticks(1'000'000'000.0);
            _deviceToHostNs.store(static_cast<std::int64_t>(hostNs) - deviceNs, std::memory_order_relaxed);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Source] reading the device clock: %s (the timing tags fall back to the host clock at the drain)\n", e.what());
        }
    }

    // The same reading taken from the consumer thread, where the device may be going down
    // under it.
    void measureClockOffset() {
        CtrlGuard guard(_ctrlMutex, _deviceUp);
        if (!guard) {
            return;
        }
        measureClockOffsetLocked();
    }

    /*| contract: move a device-clock set that is in flight on by one block: latched once a
            block stamped at or past the second asked for has arrived, abandoned once the
            wait for the edge has run out. Called on the consumer thread.
    */
    void settleDeviceClock(std::int64_t blockSeconds) {
        switch (clockLatchState(blockSeconds, _clockTargetSec.load(std::memory_order_relaxed), wallClockNs(), _clockLatchDeadlineNs.load(std::memory_order_relaxed))) {
        case ClockLatch::Pending: return;
        case ClockLatch::Latched:
            _deviceClockSet.store(true, std::memory_order_release);
            _clockSetPending.store(false, std::memory_order_release);
            measureClockOffset(); // the device clock moved, so the offset measured before it says nothing
            return;
        case ClockLatch::Abandoned:
            _clockSetPending.store(false, std::memory_order_release);
            std::fprintf(stderr, "[uhd::Source] the device clock did not reach the second it was asked to latch within %.0f ms; the timing tags stay in the host's epoch\n", kClockLatchLimitMs);
            return;
        }
    }

    // The shared coverage re-read on this block's device. Called with _ctrlMutex held.
    void refreshFreqRanges() {
        if (_usrp != nullptr) {
            capture::uhd::refreshFreqRanges(*_usrp, Direction::Receive, _freqRanges, "uhd::Source");
        }
    }

    void applySampleRate() {
        std::lock_guard lock(_ctrlMutex);
        if (_usrp == nullptr) {
            return; // teardown resets the handle under this mutex
        }
        applyRateLocked();
    }

    /*| contract: take the clock the open address fixed as the clock in force, so the ladder is that
            clock's from the first sample rather than the range the device states. The caller
            holds _ctrlMutex and has read the truth of the open device.
        verified-by: sink.uhd-an-address-that-pins-the-clock-fixes-the-ladder
    */
    void takeClockInForceLocked() {
        if (const double pinned = clockInForceAtOpen(openAddress(), _openTruth); pinned > 0.0) {
            _clockInForceHz = pinned;
        }
    }

    // The rate to hand this block's device for a request, from the shared ladder. Called with
    // _ctrlMutex held.
    double rateToWrite(double requestHz) const { return capture::uhd::rateToWrite(requestHz, _openTruth, _clockInForceHz.value_or(0.0), _wireFormat, Direction::Receive, "uhd::Source"); }

    /*| contract: the bounds on a drain of a stopped stream, both durations: the whole drain
            takes at most kDrainLimitS, and one receive waits at most kDrainRecvS for a packet.
        why: a duration holds at every rate. The drain takes every sample the transport holds
            that arrives within kDrainLimitS, 18.4 million at 61.44 MS/s and 600 million at
            2 GS/s, where a count of receives stops at the same number of samples at every
            rate and at a faster rate leaves more of the old rate in the transport.
        verified-by: controls.uhd-a-drain-ends-by-duration-at-any-rate
    */
    static constexpr double kDrainRecvS  = 0.05;
    static constexpr double kDrainLimitS = 0.3;

    /*| contract: the longest a rate change waits for the control thread to finish the writes
            given before it. Past it the rate is written behind them, the two serialized on
            _ctrlMutex, and a line says so.
        why: a duration, as the drain's bound is. One control write is a retune, a gain, an
            automatic gain mode or an antenna, and a retune is the slowest of them.
        verified-by: controlworker.a-settle-ends-by-its-bound
    */
    static constexpr double kControlSettleLimitS = 1.0;

    /*| frame: the samples the drain's own buffer holds, one receive's worth and no bound on the
            drain. The drain throws what it takes away, and so does a receive made while the
            block is not running, which lands in the same buffer, one frame at least.
    */
    static constexpr std::size_t kDrainSamples = 16384UZ;

    // What ended a drain of a stopped stream.
    enum class DrainEnd { EndOfBurst, Empty, Deadline, Failed };

    struct DrainResult {
        std::size_t samples = 0UZ;
        DrainEnd    end     = DrainEnd::Empty;
        double      seconds = 0.0; // from the stop to the end of the drain
    };
    DrainResult _lastDrain; // guarded by _drainMutex

    /*| contract: receive through recv until the stream's end-of-burst marker, a receive that
            finds nothing, or limitS on now's clock, whichever comes first, and answer the
            samples taken and which of the three ended it. recv takes the metadata to fill and
            answers the samples received. A receive that reports an overflow and no samples
            goes on to the next one, because packets behind the report are still in flight.
        frame: UHD ends a stopped continuous stream with end_of_burst on its last packet where
            the device marks one. A radio that marks none ends the drain at the first receive
            that waits kDrainRecvS and finds nothing.
        verified-by: controls.uhd-a-drain-ends-by-duration-at-any-rate
    */
    template <typename Recv, typename Now>
    static DrainResult drainStopped(Recv&& recv, Now&& now, double limitS) {
        DrainResult out;
        const auto  began    = now();
        const auto  deadline = began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(limitS));
        out.end              = DrainEnd::Deadline;
        while (now() < deadline) {
            ::uhd::rx_metadata_t md;
            const std::size_t    n = recv(md);
            out.samples += n;
            if (md.end_of_burst) {
                out.end = DrainEnd::EndOfBurst;
                break;
            }
            if (n == 0UZ && md.error_code != ::uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                out.end = DrainEnd::Empty;
                break;
            }
        }
        out.seconds = std::chrono::duration<double>(now() - began).count();
        return out;
    }

    /*| contract: the last drain of a stopped stream this block made, at a rate change or a
            teardown, and a default result before the first. A diagnostic.
        verified-by: uhd.a-drain-at-the-top-rate-ends-by-duration
    */
    DrainResult lastDrain() const {
        std::lock_guard lock(_drainMutex);
        return _lastDrain;
    }

    /*| contract: stop the device stream and drain it through drainStopped into the drain's
            buffer. Answers the samples taken. Called on the one thread that may call recv at
            that moment: the consumer thread with _ctrlMutex held, or the teardown after it has
            joined the consumer thread.
        why: samples already in flight were taken at the old rate. Published behind the new
            rate's tag they are labeled with a rate they were not taken at and timed by it as
            well, which is a hole no marker names. At a teardown the device quiesces before the
            streamer goes.
        verified-by: uhd.rate-change-midstream
    */
    DrainResult stopAndDrainStream() {
        DrainResult out;
        try {
            _stream->issue_stream_cmd(::uhd::stream_cmd_t(::uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS));
            if (_recvBuf.empty()) {
                _recvBuf.resize(kDrainSamples);
            }
            out = drainStopped([this](::uhd::rx_metadata_t& md) { return _stream->recv(_recvBuf.data(), _recvBuf.size(), md, kDrainRecvS); }, [] { return std::chrono::steady_clock::now(); }, kDrainLimitS);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Source] draining the transport: %s (continuing)\n", e.what());
            out.end = DrainEnd::Failed;
        }
        std::lock_guard lock(_drainMutex);
        _lastDrain = out;
        return out;
    }

    // Ask the device to stream continuously from now. Called with _ctrlMutex held.
    void startStreamLocked() {
        ::uhd::stream_cmd_t cmd(::uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        cmd.stream_now = true;
        _stream->issue_stream_cmd(cmd);
    }

    // The rate applier's body, with _ctrlMutex already held and the device live.
    void applyRateLocked() {
        applyRateWith([this](double hz) { return writeRate(*_usrp, Direction::Receive, hz, "uhd::Source"); });
    }

    /*| contract: the rate applier with the device's rate write passed in as write, which answers
            what the write left on the device. The caller holds _ctrlMutex.
        verified-by: sink.uhd-a-start-configures-the-rate-first
    */
    template <typename WriteRate>
    void applyRateWith(WriteRate&& write) {
        const auto want = rateToWrite(static_cast<double>(sample_rate.value));
        if (!writeNeeded(_rateWrittenHz, want)) {
            if (_rateActualHz > 0.0) {
                sample_rate = _rateActualHz; // the rate in force, not the request the replay carried
            }
            return;
        }
        /*| invariant: a rate written to a streaming device is bracketed by a stop and a
                start, with the transport emptied between them, and the discontinuity is
                marked. Before the streamer exists there is nothing to bracket.
        */
        const bool streaming = _stream != nullptr && _deviceUp.load(std::memory_order_acquire);
        if (streaming) {
            const DrainResult drained = stopAndDrainStream();
            _rateBreaks.fetch_add(1, std::memory_order_relaxed);
            _lastGapCause.store(kGapRateChange, std::memory_order_relaxed);
            std::fprintf(stderr, "[uhd::Source] the rate moves to %.0f S/s: %zu samples taken at the old rate were dropped%s and the discontinuity is marked\n", want, drained.samples,
                         drained.end == DrainEnd::Deadline ? ", the drain ending at its bound with the transport still delivering," : "");
        }
        /*| invariant: the setting states the rate the device holds afterwards, read back from
                it whether or not the write went through.
        */
        const RateWrite written = write(want);
        if (rateWriteSettled(written)) {
            _rateWrittenHz = want;
        }
        if (written.heldHz > 0.0) {
            _rateActualHz = written.heldHz;
            publishSurfaceLocked();
            if (std::abs(written.heldHz - static_cast<double>(sample_rate.value)) > 0.5) {
                sample_rate = written.heldHz;
            }
        }
        /*| invariant: the analog filter tracks the rate: UHD coerces a bandwidth to the nearest
                valid value, so this asks for fs itself, clamped into the range the frontend
                states.
            trap: a manual BANDWIDTH write survives only until the next rate change, which
                re-derives it.
        */
        if (filterFollowsRate(written, _bwMinHz, _bwMaxHz)) {
            applyBandwidthLocked(bandwidthFor(_rateActualHz, _bwMinHz, _bwMaxHz));
        }
        if (streaming) {
            try {
                startStreamLocked();
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[uhd::Source] restarting the stream after a rate change: %s\n", e.what());
            }
        }
        refreshFreqRanges();
    }

    /*| contract: bound the overall gain by the range the open device states, write a gain the
            bound moved back into rx_gains, as a refused center is written back into frequency,
            and write the bounded gain to the device. The setting then states the gain the
            device takes, while the automatic loop holds the gain path as well. Called by
            start() alone, before the consumer thread drains any setting.
        verified-by: controls.uhd-a-bounded-gain-is-written-back
    */
    void applyGain() {
        if (!rx_gains->empty()) {
            applyGainOf(boundGainSetting());
        }
    }

    /*| contract: the overall gain rx_gains asks for, bounded by the range the open device
            states, with a gain the bound moved written back into rx_gains. Called on the thread
            that drains the settings, or by start() before that thread runs: the setting is the
            block's, and the control thread takes the bounded value rather than the setting.
        why: the gain range is read without the control mutex, which a retune on the control
            thread holds for the whole of its device call. start() writes the range before the
            consumer thread exists and the teardown clears it after the join, so no write
            overlaps this read.
        verified-by: controls.uhd-the-drain-writes-the-bounded-gain-back
    */
    double boundGainSetting() {
        const double want = boundedGainFor(_openTruth, "", rx_gains->front(), "uhd::Source");
        if (!(want == rx_gains->front())) {
            rx_gains = std::vector<double>{want};
        }
        return want;
    }

    // The overall gain applier, from the gain the settings drain bounded.
    void applyGainOf(double want) {
        std::lock_guard lock(_ctrlMutex);
        if (_usrp == nullptr) {
            return; // teardown resets the handle under this mutex
        }
        if (_agcOn) {
            return;
        }
        /*| trap: an element a caller moved through setElementGain outlives a replay of the
                same overall gain, because the replay is skipped. That is the order the AGC
                release states as well: the more specific value is the one the device ends on.
        */
        if (!writeNeeded(_gainWrittenDb, want)) {
            return;
        }
        try {
            _usrp->set_rx_gain(want, 0); // overall gain, UHD distributes
            _gainWrittenDb = want;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Source] set_rx_gain(%.1f) failed: %s (continuing)\n", want, e.what());
        }
    }

    // Keep one value per gain element, the newest. Called with _ctrlMutex held.
    void holdElementGain(const std::string& element, double dB) {
        for (auto& [name, value] : _heldGains) {
            if (name == element) {
                value = dB;
                return;
            }
        }
        _heldGains.emplace_back(element, dB);
    }

    // Write what leaving the automatic mode owes the device: the overall gain asked for, where
    // one was, then the elements a caller moved while the loop held them.
    void applyGainReleaseOf(std::optional<double> overallDb) {
        std::lock_guard lock(_ctrlMutex);
        if (_usrp == nullptr) {
            return; // teardown resets the handle under this mutex
        }
        const double overall = overallDb.has_value() ? boundedGainFor(_openTruth, "", *overallDb, "uhd::Source") : 0.0;
        for (const auto& [element, dB] : agcReleaseWrites(overall, _heldGains, _openTruth.gains)) {
            if (element.empty() && !overallDb.has_value()) {
                continue;
            }
            try {
                if (element.empty()) {
                    _usrp->set_rx_gain(dB, 0);
                    _gainWrittenDb = dB;
                } else {
                    _usrp->set_rx_gain(dB, element, 0);
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[uhd::Source] set_rx_gain(%s, %.1f) failed: %s (continuing)\n", element.c_str(), dB, e.what());
            }
        }
    }

    /*| contract: put the frontend's automatic gain control into the mode the setting asks
            for, on a model that has one. On a model without one the mode is logged once per
            open and nothing is written.
        why: whether the frontend carries an AGC is read from its property tree when the
            device is opened, not inferred from a failed write: UHD takes the write on a model
            without one, warns and changes nothing, so a write is no test at all and a caller
            would see the mode reported on while the gain stayed manual.
        trap: the vendor header states that a device implementing no automatic gain control
            throws on this call. The catch below stays for the version that does, and for the
            model this bench has none of.
        measured: a B205mini carries one and lifts a -72 dBFS signal to -16 dBFS with zero
            commanded. An N210 with a WBX carries none: on UHD 4.9.0.1 the call returns
            without throwing there and the gain does not move, which is the behavior this
            comment describes rather than the one the header documents.
        verified-by: uhd.agc
    */
    void applyAgc() { applyAgcOf(gain_mode.value); }

    // The automatic gain applier, from the mode the settings drain copied out.
    void applyAgcOf(bool on) {
        std::lock_guard lock(_ctrlMutex);
        if (_usrp == nullptr) {
            return; // teardown resets the handle under this mutex
        }
        if (!_agcSupported) {
            _agcOn = false;
            if (on && !_agcUnavailableLogged) {
                _agcUnavailableLogged = true;
                std::fprintf(stderr, "[uhd::Source] this radio's frontend carries no automatic gain control; the gain stays manual\n");
            }
            return;
        }
        if (!writeNeeded(_agcWritten, on)) {
            return;
        }
        try {
            _usrp->set_rx_agc(on, 0);
            _agcOn      = on;
            _agcWritten = on;
            if (on) {
                // The loop owns the gain path and moves it, so nothing the block wrote
                // before is still in force.
                _gainWrittenDb.reset();
            }
        } catch (const std::exception& e) {
            _agcSupported = false;
            _agcOn        = false;
            if (on) {
                std::fprintf(stderr, "[uhd::Source] set_rx_agc: %s (AGC unavailable)\n", e.what());
            }
        }
    }

    void applyAntenna() {
        if (!rx_antennae->empty() && !rx_antennae->front().empty()) {
            std::ignore = applyAntennaOf(rx_antennae->front());
        }
    }

    /*| contract: the antenna applier, from the name the settings drain copied out. A name
            outside the list the unit states is refused before any device call, in the words
            antennaRefusal gives. A connector the device takes becomes the one in force on the
            surface; the caller sends the moved surface to the controls subscribers once this
            returns.
        contract: false where the name is refused, by the list or by the device, and true
            otherwise: the name written, the name already in force, or a block holding no
            device, which writes nothing. The answer follows setControl's, true for a write
            the block carries out.
        contract: a refused name stays in rx_antennae, and the device and the surface keep the
            connector they held.
        verified-by: controls.uhd-refuses-an-antenna-the-unit-does-not-offer
        verified-by: uhd.antenna, for the surface's default following a landed write
    */
    bool applyAntennaOf(const std::string& want) {
        std::lock_guard lock(_ctrlMutex);
        if (const auto refusal = antennaRefusal(_openTruth.antennas, want); refusal.has_value()) {
            std::fprintf(stderr, "%s\n", refusal->c_str());
            return false;
        }
        if (_usrp == nullptr) {
            return true; // teardown resets the handle under this mutex
        }
        if (!writeNeeded(_antennaWritten, want)) {
            return true;
        }
        try {
            _usrp->set_rx_antenna(want, 0);
            _antennaWritten    = want;
            _openTruth.antenna = want;
            publishSurfaceLocked();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Source] set_rx_antenna(%s): %s (continuing)\n", want.c_str(), e.what());
            return false;
        }
        return true;
    }

    // ---- consumer thread: settings drain + recv + hold-and-retry publish -------------

    /*| contract: where in a block of nSamples the frequency tag of one tune goes: the offset
            of the first sample stamped at or after the clock reading the tune carries, zero
            where the tune carries no reading or the block no stamp, and nothing where every
            sample of the block was taken before the reading. A reading more than
            kTuneTagLimitS past the block's first stamp belongs to a device clock that moved
            under the tune, and the tag goes on the first sample.
        verified-by: controls.uhd-a-retune-tag-lands-on-the-first-sample-it-reached
    */
    static constexpr double kTuneTagLimitS = 2.0;
    static std::optional<std::size_t> tuneTagOffset(std::optional<::uhd::time_spec_t> after, std::optional<::uhd::time_spec_t> first, double rateHz, std::size_t nSamples) {
        if (!after.has_value() || !first.has_value() || !(rateHz > 0.0)) {
            return 0UZ;
        }
        const double aheadS = (*after - *first).get_real_secs();
        if (!(aheadS > 0.0) || aheadS > kTuneTagLimitS) {
            return 0UZ;
        }
        const double at = std::ceil(aheadS * rateHz - 1e-6); // a millionth of a sample absorbs the rounding of two stamps
        if (at >= static_cast<double>(nSamples)) {
            return std::nullopt;
        }
        return static_cast<std::size_t>(at);
    }

    /*| contract: publish one receive from the reservation it landed in: the tags its first
            sample carries, a frequency tag at each offset given, in order, and then the
            nSamples received. An offset behind the one before it goes on that one's sample.
            firstNs returns the time of the first sample and runs only where a timing tag
            needs it. The edge takes the samples when the span is released, and the caller
            moves the graph's progress counter after that.
    */
    template <typename Reservation, typename FirstNs>
    void publishReceived(Reservation& span, std::size_t nSamples, FirstNs&& firstNs, std::span<const std::pair<std::size_t, double>> tags) {
        publishBlockHeadTags(*this, firstNs);
        std::size_t at = 0UZ;
        for (const auto& [offset, hz] : tags) {
            at          = std::max(at, offset);
            auto tagMap = out.makeTagMap();
            gr::tag::put(tagMap, "frequency", hz);
            out.publishTag(std::move(tagMap), at);
            noteDroppedTags(*this);
        }
        span.publish(nSamples);
    }

    void consumerLoop() {
        const std::size_t frame = _stream->get_max_num_samps();
        _recvBuf.resize(std::max(kDrainSamples, frame));
        auto&             writer    = out.streamWriter();
        const auto        keepGoing = [this] { return gr::lifecycle::isActive(this->state()) && !_consumer.stopRequested(); };
        std::uint64_t    gapsSeen            = 0;
        int              consecutiveTimeouts = 0;
        std::vector<int> seenErrors; // one line per error kind per stream
        double             expectedRate        = 0.0;
        ::uhd::time_spec_t expectedAt;
        double             secondsPerSample    = 0.0; // at expectedRate
        bool               haveExpected        = false;
        std::uint64_t delivered           = 0;
        bool          shortfallSaid       = false;
        std::uint64_t shortfallCheckAt    = 0; // the delivered count at which the clock is next read
        const auto    streamStart         = std::chrono::steady_clock::now();
        std::uint64_t tunesTaken          = 0;
        std::vector<TuneResult>                      tunesOpen;
        std::vector<std::pair<std::size_t, double>> frequencyTags;

        bool published = false; // the pass before released a reservation holding samples

        while (!_consumer.stopFlag.load(std::memory_order_acquire)) {
            /*| invariant: a writer span hands its samples to the edge when it is released at
                    the end of the pass, not when publish() is called on it. The graph's
                    progress counter moves here, after the release, so a reader it wakes finds
                    the samples.
            */
            if (published) {
                published = false;
                this->progress->incrementAndGet();
                this->progress->notify_all();
            }
            if (_restageRate.load(std::memory_order_relaxed) && _restageRate.exchange(false, std::memory_order_acq_rel)) {
                restageRateForClock(); // a control moved the converter clock under every rate
            }
            this->applyChangedSettings(); // settingsChanged() -> UHD control calls run on this thread

            /*| frame: a receive lands in a reservation on the output edge, and the samples
                    it returns are published from there. While the block is not running, as
                    between the start and the scheduler's first call, a receive lands in the
                    drain's buffer instead and is not published, which keeps the transport
                    drained.
            */
            const bool        publishing = keepGoing();
            const std::size_t room       = publishing ? writer.available() : _recvBuf.size();
            const std::size_t edge       = publishing ? writer.buffer().size() : 2UZ * room;
            const std::size_t request    = recvRequest(room, edge, frame, static_cast<double>(sample_rate.value));
            if (request == 0UZ) {
                const std::size_t seen = this->progress->value();
                if (!publishing || writer.available() < frame) {
                    awaitProgress(*this, seen, edge);
                }
                continue;
            }
            auto span = writer.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(publishing ? request : 0UZ);
            if (publishing && span.empty()) {
                awaitProgress(*this, this->progress->value(), edge);
                continue;
            }

            ::uhd::rx_metadata_t md;
            std::size_t        nSamples = 0;
            try {
                nSamples = _stream->recv(publishing ? span.data() : _recvBuf.data(), request, md, kRecvTimeoutS);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[uhd::Source] recv: %s\n", e.what());
                _streamDead.store(true, std::memory_order_release);
                return;
            }

            if (md.error_code == ::uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                /*| frame: UHD marks both a buffer that ran over and a packet that never
                        arrived as an overflow, and tells them apart with out_of_sequence.
                        A host that stopped draining causes the first; the link between the
                        radio and the host causes the second.
                */
                if (md.out_of_sequence) {
                    _seqErrors.fetch_add(1, std::memory_order_relaxed);
                } else {
                    _overflows.fetch_add(1, std::memory_order_relaxed);
                }
                _lastGapCause.store(overflowCause(md.out_of_sequence, _retunesOpen.load(std::memory_order_acquire) > 0), std::memory_order_relaxed);
            } else if (md.error_code != ::uhd::rx_metadata_t::ERROR_CODE_NONE && md.error_code != ::uhd::rx_metadata_t::ERROR_CODE_TIMEOUT
                       && firstSightOf(seenErrors, static_cast<int>(md.error_code))) {
                // The vendor's own name for the arm, once per kind per stream.
                std::fprintf(stderr, "[uhd::Source] recv: %s (said once per error kind per stream)\n", md.strerror().c_str());
            }
            /*| invariant: every receive that delivered nothing counts against one deadline,
                    whichever arm it took. A device stuck on a bad packet, a failed alignment
                    or a broken chain delivers nothing just as a timeout does.
            */
            consecutiveTimeouts = livenessAfterReceive(consecutiveTimeouts, nSamples);
            if (consecutiveTimeouts >= kDeadReceives) {
                _streamDead.store(true, std::memory_order_release);
                return;
            }
            if (nSamples == 0) {
                continue;
            }

            /*| invariant: the stamps are measured at the rate in force, so a rate change
                    starts the measure again rather than reading the change as a gap; and no
                    block is measured against an expectation formed before a device-clock set
                    moved the clock the stamps come from.
            */
            const bool    clockPending = _clockSetPending.load(std::memory_order_acquire);
            std::uint64_t shedHere     = 0;
            if (const double rateNow = static_cast<double>(sample_rate.value); md.has_time_spec && rateNow > 0.0) {
                shedHere = shedForBlock(haveExpected, clockPending, rateNow, expectedRate, expectedAt, md.time_spec);
                if (shedHere != 0UL) {
                    _dropped.fetch_add(shedHere, std::memory_order_relaxed);
                }
                if (rateNow != expectedRate) {
                    secondsPerSample = 1.0 / rateNow;
                }
                expectedAt   = md.time_spec + ::uhd::time_spec_t(static_cast<double>(nSamples) * secondsPerSample);
                expectedRate = rateNow;
                haveExpected = true;
            }
            if (clockPending && md.has_time_spec) {
                settleDeviceClock(md.time_spec.get_full_secs());
            }

            /*| frame: one marker on the first delivered block after any gap, naming the latest
                    cause: a buffer that ran over, a packet the link lost, a rate change this
                    block emptied the transport across, or a shed the stamps measure that none
                    of those counted. An overflow report carries no samples, so two gaps before
                    one delivered block share its marker.
            */
            const auto gaps    = _overflows.load(std::memory_order_relaxed) + _seqErrors.load(std::memory_order_relaxed) + _rateBreaks.load(std::memory_order_relaxed);
            const bool gapHere = gaps != gapsSeen || shedHere != 0UL;
            if (const auto cause = gapMarkerFor(gaps != gapsSeen, _lastGapCause.load(std::memory_order_relaxed), shedHere, _retunesOpen.load(std::memory_order_acquire) > 0)) {
                emitOverflowTag(*this, gapCauseName(*cause));
            }
            gapsSeen = gaps;

            /*| frame: what the host actually took, against the rate the radio is running.
                    No header states what a link carries, and a ladder entry the host cannot
                    keep up with otherwise says nothing at all: it just delivers less. One
                    line, once per stream, after a window long enough that a slow start does
                    not decide it. The clock is read where shortfallNextCheck says, and at
                    the first block after a gap.
            */
            delivered += nSamples;
            if (!shortfallSaid && (delivered >= shortfallCheckAt || gapHere)) {
                const double elapsed   = std::chrono::duration<double>(std::chrono::steady_clock::now() - streamStart).count();
                const double rateNowHz = static_cast<double>(sample_rate.value);
                if (elapsed >= kShortfallWindowS) {
                    shortfallSaid = true;
                    if (const auto shortfall = deliveryShortfall(delivered, elapsed, rateNowHz); shortfall.value_or(0.0) > kShortfallFraction) {
                        std::fprintf(stderr, "[uhd::Source] %.1f percent of %.0f S/s did not arrive over the first %.0f s; this host and link do not carry that rate\n", 100.0 * *shortfall, rateNowHz,
                                     elapsed);
                    }
                } else {
                    shortfallCheckAt = shortfallNextCheck(delivered, elapsed, rateNowHz);
                }
            }

            const double rateHz = static_cast<double>(sample_rate.value);

            /*| frame: each tune the control thread finished puts its frequency tag on the
                    first sample it reached, which may fall inside this block or in a later
                    one. The setting is written back from the newest request alone, so an
                    older tune finishing late does not put its frequency back.
            */
            if (const std::uint64_t posted = _tuneResultsPosted.load(std::memory_order_acquire); posted != tunesTaken) {
                tunesTaken = posted;
                std::lock_guard tunes(_tuneMutex);
                std::ranges::move(_tuneResults, std::back_inserter(tunesOpen));
                _tuneResults.clear();
            }
            frequencyTags.clear();
            bool settingMoved = false;
            while (!tunesOpen.empty()) {
                const TuneResult& tune = tunesOpen.front();
                const auto        at   = tuneTagOffset(tune.after, md.has_time_spec ? std::optional<::uhd::time_spec_t>(md.time_spec) : std::nullopt, rateHz, nSamples);
                if (!at.has_value()) {
                    break;
                }
                frequencyTags.emplace_back(*at, tune.heldHz);
                if (tune.request == _tuneRequests) {
                    frequency    = std::vector<double>{tune.heldHz};
                    settingMoved = true;
                }
                _retunesOpen.fetch_sub(1, std::memory_order_acq_rel);
                tunesOpen.erase(tunesOpen.begin());
            }
            if (settingMoved) {
                this->settings().updateActiveParameters();
            }
            if (publishing && keepGoing()) {
                /*| frame: the device stamps every packet with the tick its first sample was
                        taken at, so that stamp is the tag wherever the block carries one: with
                        the toggle on it stands, carrying the device's own epoch, and with the
                        toggle off it is carried into the host's epoch by the offset measured at
                        start. Either way the tag names the sample rather than the delivery, and
                        the transport's backlog — 300 ms of it on an N210 that absorbed a stall
                        — is not in it.
                    trap: a block with no stamp falls back to the clock read after recv()
                        returned, less the block's own duration. That reading is late by whatever
                        the transport held beyond this block, which no reading here can see.
                */
                const auto firstNs = [this, &md, nSamples, rateHz] {
                    std::optional<std::int64_t> stampNs;
                    if (md.has_time_spec) {
                        const bool deviceEpoch = _deviceTime.load(std::memory_order_acquire) && _deviceClockSet.load(std::memory_order_acquire);
                        stampNs = deviceStampNs(md.time_spec.to_ticks(1'000'000'000.0), deviceEpoch, _deviceToHostNs.load(std::memory_order_relaxed));
                    }
                    return blockTagNs(stampNs, wallClockNs, nSamples, rateHz);
                };
                publishReceived(span, nSamples, firstNs, frequencyTags);
                published = true;
            }
        }
    }
};

} // namespace capture::uhd

