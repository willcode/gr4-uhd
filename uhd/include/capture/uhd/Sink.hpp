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

#include <capture/common/Device.hpp>
#include <capture/common/Sink.hpp>
#include <capture/common/ControlDesc.hpp>
#include <capture/uhd/Device.hpp>

#include <uhd/device.hpp>
#include <uhd/exception.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <format>
#include <limits>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace capture::uhd {

/*| role: the USRP transmit sink for Ettus radios, over libuhd.
    contract: the peer of Source. It takes the source's setting names, with the tx_ prefix
        for what a transmitter has, the same device and device_parameter selection, the same
        static device listing, and a control surface probed from the device rather than read
        from a table. The work call stages samples in a ring and the block's own drain thread
        hands them to a tx_streamer.
    frame: UHD's tx_streamer::send() blocks at the sample rate, so the drain paces itself and
        the work call only keeps the ring fed. Beside it a second thread reads the device's
        asynchronous channel and counts the transmit gaps and the burst acknowledgments the
        radio reports.
    invariant: the asynchronous thread is started after the streamer and joined before it goes,
        so it never reads a streamer that has been released.
    invariant: one thread sends at a time. The drain snapshots the streamer once and sends
        under _sendMutex; a burst ended from a settings change or from a teardown takes that
        same mutex, because two sends on one streamer are not safe together.
    trap: UHD throws on errors and every call here is wrapped. A transient control failure logs
        and continues, because emitErrorMessage() from a settings handler kills the graph; only
        a start() failure is fatal, and a drain that has ended surfaces through the work()
        liveness check.
    why: USRPs have no fractional-ppm register, carrying a TCXO or a GPSDO instead, so ppm is
        applied as a scaled tune through capture::correctedTuneHz, which states the convention,
        in the sign Source applies.
    frame: the drain's send() and the asynchronous thread's recv_async_msg() sit beside each
        other on one tx_streamer, which is the arrangement UHD's own transmit examples use. The
        vendor header states the serialization rule for send() alone and says nothing about the
        pair.
*/
struct Sink : gr::Block<Sink> {
    using Description = gr::Doc<R"(USRP sink via libuhd.
fc32 host samples over the sc16 or sc8 wire format, clipped to full scale 1.0 before they
reach the wire. Tags on the input are ignored. The setting names are the USRP source's, with
the tx_ prefix where a transmitter carries the thing, so one caller drives both directions
identically. The device list asks libuhd with use_dpdk=1 where CAPTURE_UHD_USE_DPDK is set in
the environment.)">;

    gr::PortIn<std::complex<float>> in;

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
        gr::block::labels::role::sink, gr::block::labels::holds::device, gr::block::labels::emits::rf, gr::block::labels::ingests::time);
#endif
    gr::Annotated<std::string, "device", gr::Visible, gr::Doc<"device family; 'uhd'">>                                device = std::string(kDevice);
    gr::Annotated<std::string, "device_parameter", gr::Visible, gr::Doc<"UHD addr kwargs; serial=<s>,type=b200">>     device_parameter;
    gr::Annotated<double, "sample_rate", gr::Unit<"Hz">, gr::Visible, gr::Doc<"transmit sample rate">>                sample_rate = kDefaultRateHz;
    gr::Annotated<std::vector<double>, "frequency", gr::Unit<"Hz">, gr::Visible, gr::Doc<"center frequency">>         frequency = std::vector<double>{kDefaultFrequencyHz};
    gr::Annotated<std::vector<double>, "tx_gains", gr::Unit<"dB">, gr::Visible, gr::Doc<"overall transmit gain">>     tx_gains  = std::vector<double>{0.0};
    gr::Annotated<double, "frequency_correction", gr::Unit<"ppm">, gr::Doc<"reference error in parts per million">>   frequency_correction = 0.0;
    gr::Annotated<std::vector<std::string>, "tx_antennae", gr::Doc<"transmit connector name">>                       tx_antennae;
    // The seven settings below are read by start() alone; withDeviceArgs states why.
    gr::Annotated<std::string, "wire_format", gr::Doc<"'sc16' or 'sc8'">>                                             wire_format  = std::string("sc16");
    gr::Annotated<std::string, "clock_source", gr::Doc<"reference the device is disciplined by">>                     clock_source;
    gr::Annotated<std::string, "time_source", gr::Doc<"source of the timing edge">>                                   time_source;
    gr::Annotated<double, "master_clock_rate", gr::Unit<"Hz">, gr::Doc<"converter clock; 0 lets UHD choose">>         master_clock_rate = 0.0;
    gr::Annotated<double, "send_frame_size", gr::Unit<"B">, gr::Doc<"transport frame bytes; 0 lets UHD choose">>      send_frame_size   = 0.0;
    gr::Annotated<double, "num_send_frames", gr::Doc<"transport frames; 0 lets UHD choose">>                          num_send_frames   = 0.0;
    gr::Annotated<double, "send_buff_size", gr::Unit<"B">, gr::Doc<"socket buffer bytes; 0 lets UHD choose">>         send_buff_size    = 0.0;
    // The two below size the transmit queue at the start and at each rate change.
    gr::Annotated<double, "queue_duration", gr::Unit<"s">, gr::Doc<"transmit queue length in time">>                 queue_duration  = QueueBound{}.seconds;
    gr::Annotated<double, "queue_max_bytes", gr::Unit<"B">, gr::Doc<"most bytes the transmit queue takes">>          queue_max_bytes = QueueBound{}.maxBytes;
    /*| contract: where the drain thread runs, read by the start and the caller's to set. Unset,
            the thread keeps what the system gives it. The scale is the source's, and a
            placement the system refuses is a line on standard error, never a failed start.
        verified-by: controls.uhd-thread-placement-is-the-callers
    */
    gr::Annotated<double, "thread_priority", gr::Doc<"transmit thread real-time priority, 0 to 1; 0 keeps the system's">> thread_priority = 0.0;
    gr::Annotated<double, "cpu", gr::Doc<"transmit thread's processor; -1 keeps the system's choice">>                cpu             = -1.0;

    GR_MAKE_REFLECTABLE(Sink, in, device, device_parameter, sample_rate, frequency, tx_gains, frequency_correction, tx_antennae, wire_format, clock_source, time_source, master_clock_rate,
                        send_frame_size, num_send_frames, send_buff_size, queue_duration, queue_max_bytes, thread_priority, cpu);

    // The longest one send may block. It is longer than any chunk the drain hands over, so a
    // send that reaches it has met a device that stopped consuming.
    static constexpr double kSendTimeoutS = 3.0;

    ::uhd::usrp::multi_usrp::sptr _usrp;
    ::uhd::tx_streamer::sptr      _stream;
    mutable std::mutex          _ctrlMutex;     // serializes every UHD control call
    std::mutex                  _sendMutex;     // one sender at a time on the streamer
    std::mutex                  _teardownMutex; // one hardwareTeardown() owner at a time
    mutable std::mutex          _sensorMutex;   // one sensor sweep at a time, and the lifetime bound on the handle a sweep snapshots
    std::mutex                  _startMutex;    // held by a start while its reference wait holds a handle
    std::atomic<bool>           _stopping{false}; // raised by a teardown, lowered by the next start
    std::atomic<bool>           _deviceUp{false};
    std::atomic<bool>           _drainDead{false}; // the drain met a send it could not place
    std::atomic<bool>           _bursting{false};  // a burst is open on the streamer
    std::atomic<bool>           _held{false};      // a pause ended the burst
    /*| role: a rate change asking the drain to end the burst and stand off the ring, and the
            drain's answer that it has.
        why: the settings thread is the block's own worker, and every other block in its job
            waits behind it. Taking the send mutex there puts that whole job behind a send
            against a device that stopped consuming, which is kSendTimeoutS of it. A flag the
            drain reads ends the burst on the one thread that sends and waits on no device.
    */
    std::atomic<bool> _rateBreak{false};
    std::atomic<bool> _drainParked{false};
    /*| role: the drain's one wait between sends. The work call rings it after each write
            into the queue, and every writer of the pause, the rate break and the drain's stop
            flag rings it after the write.
    */
    RingDoorbell _drainBell;
    /*| role: the most samples one send carries at the rate in force, published beside the ring
            for the drain to read once per send. sendCapSamples states the rule.
    */
    std::atomic<std::size_t> _sendCapSamples{1UZ};
    TxCounters        _counters;

    // What the device said about the end of burst the last stop placed.
    enum class BurstEnd { NoneOpen, Acknowledged, Unconfirmed, NotPlaced };
    // The answer of one attempt to end a burst: no burst was open, the end was placed, or the
    // send placed nothing and the carrier stays up.
    enum class EndOfBurst { NoneOpen, Placed, NotPlaced };
    // What the device's asynchronous channel carried beside the gaps, counted since the run
    // started: burst acknowledgments, packets lost on the link and packets that arrived late.
    std::atomic<std::uint64_t> _burstAcks{0};
    std::atomic<std::uint64_t> _sequenceErrors{0};
    std::atomic<std::uint64_t> _latePackets{0};
    std::atomic<std::uint64_t> _endsPlaced{0};       // ends of burst placed since the run started
    std::atomic<std::uint64_t> _unconfirmedBound{0}; // what the last stop left undecided, in samples
    std::atomic<BurstEnd>      _stopBurstEnd{BurstEnd::NoneOpen};

    // What the open device said about itself, and what each applier last put on it. Guarded
    // by _ctrlMutex. Every write of _openTruth, _rateActualHz or _bwCurHz is followed by
    // publishSurfaceLocked() under the same lock.
    DeviceTruth _openTruth;
    double         _rateActualHz  = 0.0;
    double         _clockInForceHz = 0.0; // the clock the open address fixed, zero where UHD chooses
    double         _appliedFreqHz = 0.0;
    /*| invariant: the caller's own request, beside the frequency the tuner reached. The
            commanded value is the request scaled by the reference correction, and at 100 ppm
            and 1 GHz the two are 100 kHz apart, so a caller writing the stated value back
            would have the correction applied twice.
    */
    double                                 _appliedRequestHz = 0.0;
    double                                 _actualGainDb     = 0.0;
    double                                 _bwCurHz = 0.0, _bwMinHz = 0.0, _bwMaxHz = 0.0;
    std::string                            _actualAntenna;
    std::string                            _wireFormat = "sc16";
    std::vector<std::pair<double, double>> _freqRanges;
    std::size_t                            _maxSendSamples = 0UZ;
    std::size_t                            _deviceHeldBytes = kDeviceHeldBytes; // deviceHeldBytesFor the open address
    QueueBound                             _queueBound;                         // queue_duration and queue_max_bytes as the last sizing read them
    /*| role: queue storage a resize takes or gives back under _ctrlMutex. A caller allocates
            it before taking the mutex and frees what comes back after letting the mutex go.
    */
    RingStorage<std::complex<float>>       _queueSpare;
    mutable SensorNames                    _sensorNames;              // timed at the first sweep of a run
    mutable bool                           _sensorNamesTaken = false; // whether that sweep has kept them
    std::atomic<double>                    _refLockWaitMs{0.0};
    std::optional<double>                  _freqWrittenHz, _rateWrittenHz, _gainWrittenDb, _bwWrittenHz;
    std::optional<std::string>             _antennaWritten;

    /*| invariant: the ring is declared before the threads that index it. Members are
            destroyed in reverse declaration order, so a buffer declared after a ConsumerThread
            is freed before its destructor joins the thread, and a teardown that raised — which
            the guard swallows — leaves the drain reading storage that has gone.
    */
    SpscRing<std::complex<float>> _ring{1UZ << 19};
    /*| invariant: the ring's capacity, published where a reader can see it. A rate change
            replaces the ring's storage, so a reading taken on a third thread asks this rather
            than asking the vector being reassigned how large it is.
    */
    std::atomic<std::size_t>         _ringCapacity{1UZ << 19};
    /*| role: the capacity the rate in force asks for, in samples, published beside the ring's
            own. The rate applier writes it whether or not the resize behind it landed.
        invariant: the two are equal for a queue sized at the rate it is running at. A refused
            park leaves the ring's own capacity above this one, which is the one state the
            drain and the work call retry the resize from.
    */
    std::atomic<std::size_t>         _ringWantSamples{1UZ << 19};
    ConsumerThread                   _drain;          // the ring into the streamer
    ConsumerThread                   _async;          // the device's asynchronous channel
    ControlProperties     _controlProperties{this};
    Snapshot<SurfaceValues>       _surfaceValues; // what controlSurface() reads, published by publishSurfaceLocked()
    TeardownGuard<Sink>           _teardown{this}; // the last member, so its teardown runs first

    // ---- the controls, control and sensors properties -----------------------------------

    std::optional<gr::Message> controlProperty(std::string_view name, gr::Message message) { return answerControlProperty(*this, name, std::move(message)); }

    /*| contract: the control surface of the device this block holds, at the rate and the
            frequency in force. The controls property answers with it, and the settings hook
            reads it without _ctrlMutex. */
    std::vector<ControlDesc> controlSurface() {
        const SurfaceValues v = _surfaceValues.read();
        return describeControls(v.truth, {.deviceParams = {}, .sampleRate = v.rateHz, .centerFreq = 0.0}, v.filterHz, v.clockHz, v.wireFormat);
    }

    // Hand the values controlSurface() reads to the snapshot. The caller holds _ctrlMutex.
    void publishSurfaceLocked() { _surfaceValues.publish({.truth = _openTruth, .rateHz = _rateActualHz, .filterHz = _bwCurHz, .clockHz = _clockInForceHz, .wireFormat = _wireFormat}); }

    /*| contract: the value one control holds, in the unit its descriptor states: the filter the
            device settled on, and a gain element's gain read back from the device. Nothing for
            the antenna, whose write takes effect at the next settings drain. */
    std::optional<double> controlValue(const std::string& id) {
        CtrlGuard guard(_ctrlMutex, _deviceUp);
        if (!guard) {
            return std::nullopt;
        }
        if (id == "BANDWIDTH") {
            return _bwCurHz / 1e6;
        }
        if (_usrp == nullptr || !gainElementNamed(_openTruth, id, "uhd::Sink") || id.empty()) {
            return std::nullopt;
        }
        try {
            return _usrp->get_tx_gain(id, 0);
        } catch (const std::exception&) {
            return std::nullopt;
        }
    }

    // ---- discovery and the control surface ----------------------------------------------

    // (label, device string) pairs for a caller's device list, the pairs Source offers.
    static std::vector<std::pair<std::string, std::string>> enumerateDevices() { return deviceList(); }

    // The record probeTruth fills and describeControls reads, for a caller that names neither.
    using Truth = DeviceTruth;

    /*| contract: return once every device this process's USRP blocks handed off at a stop is
            released. A stop returns before the release, and so does the block's destructor; a
            caller that states the device free waits here first.
    */
    static void awaitReleases() { capture::uhd::awaitReleases(); }

    /*| contract: open the device briefly, while the graph is stopped, and read the transmit
            control surface into truth. False when the device cannot be opened. See the trap on
            the shared body for what the rate argument buys.
    */
    static bool probeTruth(const std::string& kwargs, double sampleRateHz, DeviceTruth& truth) { return capture::uhd::probeTruth(kwargs, sampleRateHz, truth, Direction::Transmit, "uhd::Sink"); }

    // Read one open device's transmit surface into truth, replacing it.
    static void readTruthInto(::uhd::usrp::multi_usrp& usrp, DeviceTruth& truth) { capture::uhd::readTruthInto(usrp, truth, Direction::Transmit); }

    /*| contract: the transmit control surface from probed truth — the gain elements, the
            connector and the analog filter. Gain elements, ranges and bandwidth differ per USRP
            model and daughterboard, so there is no static table.
        frame: heldBandwidthHz above zero carries the width the analog filter holds on a running
            block and becomes the filter control's default there. Zero leaves the default at the
            rate-derived value the rate applier programs, the only value a caller drawing a
            surface for a stopped device has.
        frame: clockInForceHz is the master clock a running block holds, zero where UHD
            chooses. Without it the clock is the one an open with ctx.deviceParams fixes.
            wireFormat is the packing the ladder is cut for: the format a running block holds,
            or where empty ctx.wireFormat, the one an unstarted block holds staged or set.
    */
    static std::vector<ControlDesc> describeControls(const DeviceTruth& t, const TruthContext& ctx, double heldBandwidthHz = 0.0, std::optional<double> clockInForceHz = std::nullopt,
                                                     std::string_view wireFormat = {}) {
        std::vector<ControlDesc> out = gainElementControls(t);
        /*| frame: the connector, as an index into the device's own list. A model with one port
                states one entry, which a caller draws as a single choice rather than as a
                control it can move.
        */
        if (t.antennas.size() > 1UZ) {
            ControlDesc c;
            c.id          = "ANTENNA";
            c.label       = "TX connector";
            c.kind        = 3;
            c.min         = 0.0;
            c.max         = static_cast<double>(t.antennas.size() - 1UZ);
            c.step        = 1.0;
            c.options     = t.antennas;
            const auto it = std::ranges::find(t.antennas, chooseAntenna(t.antennas, t.antenna));
            c.defValue    = it == t.antennas.end() ? 0.0 : static_cast<double>(std::distance(t.antennas.begin(), it));
            c.isGain      = false;
            out.push_back(std::move(c));
        }
        if (auto bandwidth = bandwidthControl(t, ctx, heldBandwidthHz); bandwidth.has_value()) {
            out.push_back(std::move(*bandwidth));
        }
        appendLadderAndCoverage(out, t, clockInForceHz.value_or(clockInForceAtOpen(::uhd::device_addr_t(ctx.deviceParams), t)), validWireFormat(std::string(wireFormat.empty() ? std::string_view(ctx.wireFormat) : wireFormat)), kDefaultRateHz, kDefaultFrequencyHz);
        return out;
    }

    /*| contract: the connector to ask the device for: the one named where the device offers
            it, TX/RX where it does not and the device offers TX/RX, and the device's first
            entry otherwise. Empty where the device offers nothing, which leaves the connector
            where it is.
        why: TX/RX rather than whatever the device came up on, because it is the port a B2xx
            transmits from. A model whose ports are named otherwise would transmit from its
            default port with the name written in, which on a multi-port model is a different
            connector than the one the cable is on.
        verified-by: sink.uhd-antenna-and-bandwidth
    */
    static std::string chooseAntenna(const std::vector<std::string>& offered, const std::string& wanted) {
        if (offered.empty()) {
            return {};
        }
        if (std::ranges::find(offered, wanted) != offered.end()) {
            return wanted;
        }
        if (std::ranges::find(offered, std::string("TX/RX")) != offered.end()) {
            return "TX/RX";
        }
        return offered.front();
    }

    /*| contract: the smallest transmit queue this block is driven through, in samples: eight of
            the streamer's own maximum send, and 4096 samples where the streamer states a
            smaller one or none at all.
        why: the queue is a quarter of a second at the rate in force, and at a low rate that is
            fewer samples than a handful of sends place. Eight sends leaves the drain whole
            chunks to hand over on every pass.
        frame: it is not 2^19 samples. That floor is a quarter of a second at 2 MS/s and 2.62 s
            at the 250 kS/s a B205mini also runs, and every wait this block makes is sized from
            the queue. A B205mini's streamer states 2040 samples over USB and an N210's 363 over
            the network.
        verified-by: sink.uhd-the-queue-follows-the-rate
    */
    static std::size_t ringFloorSamples(std::size_t maxSendSamples) {
        constexpr std::size_t kSendsHeld    = 8UZ;
        constexpr std::size_t kLeastSamples = 4096UZ;
        return std::max(kLeastSamples, kSendsHeld * maxSendSamples);
    }

    /*| contract: the most samples one send carries: 10 ms at rateHz, rounded down to whole
            frames of maxSendSamples and never below one frame. One frame where the rate is not
            a positive number.
        why: the drain hands the streamer the whole contiguous span the queue holds, and UHD
            cuts it into frames itself. The block's own work then happens once per send rather
            than once per frame. The cap bounds how long one send keeps the drain from a pause,
            a rate break or a stop, and whole frames leave at most one short packet per send.
        verified-by: sink.uhd-a-send-takes-the-span-up-to-the-cap
    */
    static std::size_t sendCapSamples(double rateHz, std::size_t maxSendSamples) {
        constexpr double  kSendSeconds = 0.010;
        const std::size_t frame        = std::max<std::size_t>(maxSendSamples, 1UZ);
        if (!(rateHz > 0.0)) {
            return frame;
        }
        const auto samples = static_cast<std::size_t>(std::min(rateHz * kSendSeconds, 1.0e15));
        return std::max(frame, samples / frame * frame);
    }

    /*| contract: the bytes the open device can hold between the host and the air: the send
            buffer the open address states, and kDeviceHeldBytes where it states none or one
            that is not a positive number.
        frame: UHD states no device transmit buffer in its property tree or on a tx_streamer.
            The address's send_buff_size is the one figure the open carries. An N-series takes
            it as the window its flow control lets the host run ahead of the transmit DSP, the
            unit's SRAM when the address states none. A network device's host socket holds that
            many bytes ahead of the device.
        verified-by: sink.uhd-the-stop-bound-reads-the-stated-buffer
    */
    static std::size_t deviceHeldBytesFor(const ::uhd::device_addr_t& address) {
        if (!address.has_key("send_buff_size")) {
            return kDeviceHeldBytes;
        }
        double bytes = 0.0;
        try {
            bytes = address.cast<double>("send_buff_size", 0.0);
        } catch (const std::exception&) {
            return kDeviceHeldBytes;
        }
        if (!(bytes >= 1.0) || !std::isfinite(bytes)) {
            return kDeviceHeldBytes;
        }
        return static_cast<std::size_t>(std::min(bytes, 1.0e15));
    }

    /*| contract: why this block cannot start at rateHz, or an empty string where it can. The
            device states its own rate set and UHD coerces inside it, so the one refusal this
            block can make without the device is a rate that is not a positive number.
        trap: a rate that is not positive asks the rate applier for nothing, and the applier is
            the only body that reads the device's coverage. Left to run, the block transmits at
            whatever rate and frequency the previous program left the radio on.
        verified-by: sink.uhd-refuses-a-rate-that-is-not-a-positive-number
    */
    static std::string startRefusal(double rateHz) {
        if (!(rateHz > 0.0)) {
            return std::format("uhd: {:.0f} S/s is not a rate this block can start at", rateHz);
        }
        return {};
    }

    // ---- counters and status --------------------------------------------------------------

    bool deviceUp() const { return _deviceUp.load(std::memory_order_acquire); }

    /*| contract: always zero for this family.
        why: a USRP keys down and transmits nothing at all for a gap, and reports it as an event
            carrying no length, so there is no silence to count. underrunEvents() is the honest
            number here.
        verified-by: uhd.tx-underrun
    */
    std::uint64_t underrunSamples() const { return _counters.underrunSamples.load(std::memory_order_relaxed); }

    /*| contract: transmit gaps the device itself reported since the run started: one per event
            the radio raised, and not a count of samples.
        frame: the events counted are a starved transmitter and a packet that ran out mid-way. A
            packet the link lost is counted by sequenceErrors() instead.
        verified-by: uhd.tx-underrun
    */
    std::uint64_t underrunEvents() const { return _counters.underrunEvents.load(std::memory_order_relaxed); }

    /*| contract: packets the device reported lost on the link from the host since the run
            started, one per message carrying a sequence error in or out of a burst.
        why: counted apart from the underruns, as the source keeps a lost packet apart from an
            overflow. A host that stopped feeding the radio and a link that dropped a packet
            have different answers, and an N210's Ethernet link moves this count.
        verified-by: sink.uhd-counts-transmit-events-by-bit
    */
    std::uint64_t sequenceErrors() const { return _sequenceErrors.load(std::memory_order_relaxed); }

    /*| contract: packets the device reported as arriving after the time they carried, since the
            run started. Every send here is untimed, so a nonzero count names a device that
            reported a late packet on its own.
        verified-by: sink.uhd-counts-transmit-events-by-bit
    */
    std::uint64_t latePackets() const { return _latePackets.load(std::memory_order_relaxed); }

    /*| contract: always zero for this family.
        why: a pause here ends the burst and keeps the queue, so the samples already handed over
            go out when the block resumes rather than being thrown away.
    */
    std::uint64_t discardedAtHoldSamples() const { return _counters.discardedAtHold.load(std::memory_order_relaxed); }

    /*| contract: samples a rate change threw away, counted from the start of the current run.
            They were handed over and never transmitted.
        why: the queue is played out at the rate it was staged at before the new rate is
            written, and the resize behind that play-out hands both ends fresh storage. What
            the play-out could not place therefore goes nowhere, and the path that leaves
            samples there is a device that stopped consuming, which is also a device with no
            rate to send them at.
        verified-by: sink.uhd-a-rate-change-counts-what-it-discards
    */
    std::uint64_t discardedAtRateChangeSamples() const { return _counters.discardedAtRateChange.load(std::memory_order_relaxed); }

    /*| contract: samples the last stop could not get out, from the queue it was still holding
            when its deadline expired. Zero for a stop that played the queue out.
        why: a burst that ended early and one that played whole leave the same silence behind
            them, so nothing else can tell them apart.
    */
    std::uint64_t unsentAtStopSamples() const { return _counters.unsentAtStop.load(std::memory_order_relaxed); }

    /*| contract: ends of burst the device acknowledged since the run started: one per burst whose
            last sample left the radio, a pause, a rate change and a stop each ending one.
        verified-by: sink.uhd-counts-the-burst-acknowledgment
    */
    std::uint64_t burstAcks() const { return _burstAcks.load(std::memory_order_acquire); }

    /*| contract: what the device said about the end of burst the last stop placed: NoneOpen where
            no burst was open at the stop, Acknowledged where the stop's own acknowledgment
            arrived inside burstAckBoundMs, Unconfirmed where it did not, and NotPlaced where the
            send placed no end and the carrier stayed up. A start sets it back to NoneOpen.
        why: the acknowledgment is the one statement that the tail of a burst left the radio.
            unsentAtStopSamples() counts the queue alone, and a stop that releases the device
            without the acknowledgment cannot tell a tail that went out from one still in the
            device's own buffer.
        verified-by: sink.uhd-a-stop-waits-for-its-own-acknowledgment
    */
    BurstEnd stopBurstEnd() const { return _stopBurstEnd.load(std::memory_order_acquire); }

    /*| contract: a bound and not a count: the samples the device can still hold, where the last
            stop gave up its wait for the acknowledgment or could not place its end of burst,
            and zero where the device acknowledged the end or no burst was open. A start sets it
            back to zero.
        frame: deviceHeldBytesFor the open address over the bytes one sample takes in the wire
            format in force. With no send buffer stated that is kDeviceHeldBytes, 262144 samples
            in sc16 and 524288 in sc8.
        why: every count this block states measures samples the block itself held, and a caller
            moving a stream position back by the counts moves back over no air. What the device
            holds is a debt no reading of this block can measure, so it is stated as a bound
            beside the counts and added to none of them.
        verified-by: sink.uhd-a-stop-waits-for-its-own-acknowledgment
    */
    std::uint64_t unconfirmedByFlushBoundSamples() const { return _unconfirmedBound.load(std::memory_order_acquire); }

    // Samples the queue in front of the streamer still holds.
    std::uint64_t queuedSamples() const { return static_cast<std::uint64_t>(ringElements()); }

    // The queue's own fill, bounded by the capacity published beside the ring rather than by
    // the storage a rate change replaces.
    std::size_t ringElements() const { return ringFillBounded(_ring, _ringCapacity.load(std::memory_order_acquire)); }

    // Whether a pause has ended the burst and no resume has followed it.
    bool carrierHeld() const { return _held.load(std::memory_order_acquire); }

    /*| frame: the status readers below take the control mutex with this deadline and answer the
            block-is-down value where they cannot get it. The blocking form waits for whatever
            the mutex is held for, and a rate change holds it through a play-out of up to half
            a second: a window that stops for half a second is worse than a reading that did
            not arrive.
        contract: an empty answer means the reading did not arrive and is asked for again. It
            does not say the device has gone; deviceUp() says that, and it takes no lock.
        verified-by: ctrlguard.a-bounded-wait-gives-up-and-holds-no-lock
    */
    static constexpr std::chrono::milliseconds kStatusWait{2};

    /*| contract: what the device said about itself when this block opened it, with the tuner
            coverage the rate and the filter in force give, as the peer source states it. An
            empty record while the block is down or the control mutex is busy.
        verified-by: controls.uhd-the-stated-coverage-is-the-coverage-in-force
    */
    DeviceTruth deviceTruth() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        if (!guard) {
            return {};
        }
        DeviceTruth t = _openTruth;
        if (!_freqRanges.empty()) {
            t.freqRanges = _freqRanges;
        }
        return t;
    }

    // The analog filter width the device settled on, in Hz, read from it after every write.
    double analogBandwidthHz() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        return guard ? _bwCurHz : 0.0;
    }

    /*| contract: the center frequency the last accepted tune was made for, in the caller's own
            frame, and zero while the block is down.
        why: the tuner is commanded the request scaled by the reference correction, and the
            read-back is carried back out of that scale, so a caller can write the stated value
            back and reach the same frequency.
    */
    double tunedRfHz() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        return guard ? _appliedRequestHz : 0.0;
    }

    /*| contract: the rate the device runs at, in Hz, as the rate applier last read it back, and
            zero while the block is down or the control mutex is busy.
        why: sample_rate carries the request between a settings replay and the applier's
            write-back, so a caller reading the member in that window states the request as the
            rate in force: 8 MS/s for an N210 playing 8.333 MS/s.
        verified-by: sink.uhd-states-the-rate-in-force
    */
    double rateInForceHz() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        return guard ? _rateActualHz : 0.0;
    }

    // The wire format the running streamer was built with. A setting's change reaches the
    // streamer at the next start, so between the two they differ.
    std::string wireFormatInForce() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        return guard ? _wireFormat : std::string{};
    }

    /*| contract: what a status line calls this sink, naming what the device accepted. One line,
            no trailing period, and empty while the block is down or the control mutex is busy.
        trap: every number here is a mirror the appliers left under the control mutex and not a
            settings member.
    */
    std::string describe() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        if (!guard) {
            return {};
        }
        if (_actualAntenna.empty()) {
            return std::format("usrp {:.6f} MHz, gain {:.1f} dB", _appliedRequestHz / 1.0e6, _actualGainDb);
        }
        if (_bwCurHz > 0.0) {
            return std::format("usrp {:.6f} MHz, gain {:.1f} dB, {}, filter {:.3f} MHz", _appliedRequestHz / 1.0e6, _actualGainDb, _actualAntenna, _bwCurHz / 1.0e6);
        }
        return std::format("usrp {:.6f} MHz, gain {:.1f} dB, {}", _appliedRequestHz / 1.0e6, _actualGainDb, _actualAntenna);
    }

    // The unit's own name, as the device stated it when this block opened it.
    std::string identity() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        return guard ? _openTruth.describe() : std::string{};
    }

    // How long start() waited for the reference to lock, in milliseconds. Zero where the
    // clock source in force was internal, which is waited for not at all.
    double refLockWaitMs() const { return _refLockWaitMs.load(std::memory_order_acquire); }

    // Whether a start's reference wait may go on: false from a teardown's first step until the
    // next start.
    bool referenceWaitLive() const { return !_stopping.load(std::memory_order_acquire); }

    // The sweep kinds, named on the block the way a caller asks for them.
    using SensorSweep = capture::uhd::SensorSweep;

    /*| contract: the device's own sensors, read through the sweep both blocks share — the
            motherboard's, the transmit frontend's under tx_ ids with tx_lo_locked among them,
            and the reference and timing selections — and then what only this block knows,
            under tx_ ids: the gaps the radio reported, the packets the link lost and the queue
            in front of the radio, and the unit it opened. False with nothing published while
            the device is down, while the control mutex is busy, or where the device goes down
            mid-sweep. Full is every sensor; Brief is the ones the first sweep of the run
            measured as cheap.
        why: a graph that transmits alone has no receive block beside it to read the device,
            and the reference lock and the transmit synthesizer's lock are the readings that
            say a burst is on frequency.
        trap: the control mutex is held only to snapshot the handle and the name, and every
            device read runs outside it, as in the receive block: a GPSDO's full sweep takes the
            better part of a second. _sensorMutex bounds the life of the snapshot, the teardown
            taking it before it releases the device, and the cleared _deviceUp aborts a sweep
            between two reads.
    */
    bool readSensors(std::vector<SensorReading>& out, SensorSweep sweepKind = SensorSweep::Full) const {
        out.clear();
        std::lock_guard               sweep(_sensorMutex);
        ::uhd::usrp::multi_usrp::sptr usrp;
        std::string                   what;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
            if (!guard) {
                return false;
            }
            usrp = _usrp; // shared: the device outlives a teardown that races this sweep
            what = _openTruth.describe();
        }
        if (usrp == nullptr || !sweepDeviceSensors(out, *usrp, sweepKind)) {
            return false;
        }
        appendReadings(out);
        if (!what.empty()) {
            out.push_back({.id = "device_model", .label = "Device", .value = std::move(what), .good = true, .brief = false});
        }
        return true;
    }

    /*| contract: this block's own counts, under tx_ ids. The underruns, one per gap the radio
            reported, are the glance row; the rest are diagnostics.
    */
    void appendReadings(std::vector<SensorReading>& out) const {
        out.push_back({.id = "tx_underrun_events", .label = "Underruns", .value = std::format("{}", underrunEvents()), .good = true, .brief = true});
        out.push_back({.id = "tx_sequence_errors", .label = "Lost packets", .value = std::format("{}", sequenceErrors()), .good = true, .brief = false});
        out.push_back({.id = "tx_late_packets", .label = "Late packets", .value = std::format("{}", latePackets()), .good = true, .brief = false});
        out.push_back({.id = "tx_discarded_at_rate_change", .label = "Dropped at a rate change", .value = std::format("{} samples", discardedAtRateChangeSamples()), .good = true, .brief = false});
        out.push_back({.id = "tx_unsent_at_stop", .label = "Unsent at stop", .value = std::format("{} samples", unsentAtStopSamples()), .good = true, .brief = false});
        out.push_back({.id = "tx_burst_acks", .label = "Bursts acknowledged", .value = std::format("{}", burstAcks()), .good = true, .brief = false});
        out.push_back({.id = "tx_queued_samples", .label = "Queued", .value = std::format("{} samples", queuedSamples()), .good = true, .brief = false});
    }

    /*| frame: how long one sensor read may take at the first sweep of a run and still belong in
            the brief set, in milliseconds. That sweep runs beside a streaming drain, where a read
            of ref_locked or tx_lo_locked waits behind the samples on the link or behind other
            threads on a loaded host. A GPSDO's records take about 180 ms each and stay out.
        why: a name the timing drops stays out of every brief sweep of the run, and the lock
            readings are the ones a brief sweep exists for.
        verified-by: sink.uhd-a-loaded-host-keeps-the-brief-sensors
    */
    static constexpr double kFirstSweepSensorLimitMs = 50.0;

    /*| contract: the device's part of a sweep on usrp. The first sweep of a run times the sensor
            names outside _ctrlMutex and keeps them, and every sweep reads the names kept. False
            with nothing kept where the device goes down meanwhile or the control mutex stays
            busy. The caller holds _sensorMutex.
        why: timing the names is one read of every sensor, about 700 ms on an N210 with a GPSDO.
            Made in the start under the control mutex, it lengthened every transmit start and
            held a stop behind it, for readings a transmit caller may never ask for.
        verified-by: sink.uhd-times-its-sensors-at-the-first-sweep
    */
    template <typename Usrp>
    bool sweepDeviceSensors(std::vector<SensorReading>& out, Usrp& usrp, SensorSweep sweepKind) const {
        const auto  live = [this] { return _deviceUp.load(std::memory_order_acquire); };
        SensorNames names;
        bool        named = false;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
            if (!guard) {
                return false;
            }
            names = _sensorNames;
            named = _sensorNamesTaken;
        }
        if (!named) {
            names = sensorNamesOf(usrp, Direction::Transmit, "uhd::Sink", kFirstSweepSensorLimitMs, live);
            CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
            if (!guard) {
                return false;
            }
            _sensorNames      = names;
            _sensorNamesTaken = true;
        }
        return readDeviceSensors(out, usrp, names, Direction::Transmit, sweepKind, live);
    }

    // ---- lifecycle --------------------------------------------------------------------------

    // The address this block opens the device with, the settings folded into the selector.
    ::uhd::device_addr_t openAddress() const { return ::uhd::device_addr_t(withDeviceArgs(device_parameter.value, master_clock_rate, Direction::Transmit, send_frame_size, num_send_frames, send_buff_size)); }

    /*| contract: open the device, put it on the rate, the center, the gain, the connector and
            the filter the settings ask for, build the streamer and start the drain. A failure
            releases the device and throws its reason.
    */
    void start() {
        std::ignore = hardwareTeardown();
        _stopping.store(false, std::memory_order_release);
        _counters.reset();
        _drainDead.store(false, std::memory_order_release);
        _bursting.store(false, std::memory_order_release);
        _held.store(false, std::memory_order_release);
        _rateBreak.store(false, std::memory_order_release);
        _drainParked.store(false, std::memory_order_release);
        _refLockWaitMs.store(0.0, std::memory_order_release);
        _burstAcks.store(0, std::memory_order_release);
        _sequenceErrors.store(0, std::memory_order_relaxed);
        _latePackets.store(0, std::memory_order_relaxed);
        _endsPlaced.store(0, std::memory_order_release);
        _unconfirmedBound.store(0, std::memory_order_release);
        _stopBurstEnd.store(BurstEnd::NoneOpen, std::memory_order_release);
        // The queue the previous run left is the queue this one starts from, so the two
        // capacities agree until the streamer has stated its own maximum send.
        _ringWantSamples.store(_ringCapacity.load(std::memory_order_acquire), std::memory_order_release);
        {
            std::lock_guard lock(_ctrlMutex);
            // A fresh multi_usrp holds none of the previous device's state.
            _freqWrittenHz.reset();
            _rateWrittenHz.reset();
            _gainWrittenDb.reset();
            _bwWrittenHz.reset();
            _antennaWritten.reset();
            _rateActualHz     = 0.0;
            _clockInForceHz   = 0.0;
            _appliedFreqHz    = 0.0;
            _appliedRequestHz = 0.0;
            _actualGainDb     = 0.0;
            _bwCurHz          = 0.0;
            _maxSendSamples   = 0UZ;
            _actualAntenna.clear();
            _freqRanges.clear();
            publishSurfaceLocked();
            _sensorNames      = {};
            _sensorNamesTaken = false;
            _wireFormat = validWireFormat(wire_format.value);
        }

        if (const std::string why = startRefusal(sample_rate); !why.empty()) {
            throwStartFailure("uhd::Sink", why);
        }

        awaitReleases();
        try {
            std::lock_guard lock(_ctrlMutex);
            _usrp = ::uhd::usrp::multi_usrp::make(openAddress());
        } catch (const std::exception& e) {
            {
                std::lock_guard lock(_ctrlMutex);
                _usrp.reset();
            }
            throwStartFailure("uhd::Sink", std::format("uhd make({}): {}", device_parameter.value, e.what()));
        }

        if (stopArrivedDuringOpen(this->state())) {
            std::lock_guard lock(_ctrlMutex);
            _usrp.reset();
            return;
        }

        std::string failure;
        {
            std::lock_guard lock(_ctrlMutex);
            if (_usrp == nullptr) {
                return; // a stop took the handle between the state read and this mutex
            }
            applyReferenceSourcesLocked();
            try {
                readTruthInto(*_usrp, _openTruth);
            } catch (const std::exception& e) {
                /*| invariant: caught here rather than left to escape, so the device is released
                        and the record cleared before the start throws. A block whose start
                        threw is never given stop().
                */
                failure    = std::format("uhd read the transmit surface of {}: {}", device_parameter.value, e.what());
                _openTruth = {};
            }
            publishSurfaceLocked();
            if (failure.empty()) {
                failure = configureOpenDeviceLocked([this](double hz) { return writeRate(*_usrp, Direction::Transmit, hz, "uhd::Sink"); });
            }
        }
        if (!failure.empty()) {
            std::ignore = hardwareTeardown();
            throwStartFailure("uhd::Sink", failure);
        }

        /*| invariant: the start waits for the reference before it builds the streamer, and the
                wait holds neither the control mutex nor the teardown's, so a status reader or a
                stop request made meanwhile is not held behind it. The start holds _startMutex
                from the wait through the thread starts, and the wait polls referenceWaitLive().
                A stop lowers that answer before anything else and then takes _startMutex, so it
                meets the start either inside its wait, which ends within one poll and arms
                nothing, or with the threads running, which the stop joins. The stop hands the
                device to the releasing thread, and the next open waits for that release.
            trap: _startMutex is let go before the failure branch below, whose teardown takes it.
            verified-by: sink.uhd-a-stop-ends-the-reference-wait
            verified-by: sink.uhd-a-start-a-stop-ended-arms-nothing
        */
        {
            std::lock_guard               startLock(_startMutex);
            ::uhd::usrp::multi_usrp::sptr usrp;
            {
                std::lock_guard lock(_ctrlMutex);
                usrp = _usrp;
            }
            if (usrp == nullptr) {
                return; // a stop took the handle while the device was being configured
            }
            _refLockWaitMs.store(waitForReference(*usrp, "uhd::Sink", kRefLockLimitMs, 25.0, [this] { return referenceWaitLive(); }).waitedMs, std::memory_order_release);
            usrp.reset();
            failure = armAfterReferenceWait([this](const ::uhd::stream_args_t& args) -> ::uhd::tx_streamer::sptr { return _usrp == nullptr ? nullptr : _usrp->get_tx_stream(args); });
        }
        if (!failure.empty()) {
            std::ignore = hardwareTeardown();
            throwStartFailure("uhd::Sink", failure);
        }
        offerControlProperties(*this);
    }

    /*| contract: the start's steps behind the reference wait, the caller holding _startMutex:
            nothing where a stop ended the wait or took the handle, and otherwise the streamer
            makeStream builds under _ctrlMutex, the queue sized for its maximum send, deviceUp()
            raised, and the asynchronous thread and the drain started. makeStream answers no
            streamer where the block holds no handle. Answers the failure the streamer build
            met, or an empty string.
        why: a stop that ended the wait has lowered referenceWaitLive() and waits on _startMutex
            behind this call. A start that went on would build a streamer and start two threads
            after that stop's joins, and they would run on a snapshot of the streamer, holding
            the transport open, until the next start or the block's end.
        invariant: the asynchronous thread is started after the streamer and joined before it
            goes, so it never reads a streamer that has been released.
        verified-by: sink.uhd-a-start-a-stop-ended-arms-nothing
    */
    template <typename MakeStream>
    std::string armAfterReferenceWait(MakeStream&& makeStream) {
        if (!referenceWaitLive()) {
            return {};
        }
        std::size_t want = 0UZ;
        try {
            std::lock_guard      lock(_ctrlMutex);
            ::uhd::stream_args_t args("fc32", _wireFormat);
            args.channels = {0};
            _stream       = makeStream(args);
            if (_stream == nullptr) {
                return {};
            }
            _maxSendSamples  = _stream->get_max_num_samps();
            _deviceHeldBytes = deviceHeldBytesFor(openAddress());
            takeQueueBoundLocked();
            want             = ringWantSamplesLocked();
            _ringWantSamples.store(want, std::memory_order_release);
            publishSendCapLocked();
        } catch (const std::exception& e) {
            return std::format("uhd tx stream: {}", e.what());
        }
        /*| invariant: the queue is sized outside _ctrlMutex. Neither thread that indexes it runs
                yet, and the work call leaves the queue alone until deviceUp() rises below.
        */
        try {
            resizeRing(_ring, want);
        } catch (const std::exception& e) {
            return std::format("uhd tx queue of {} samples: {}", want, e.what());
        }
        _ringCapacity.store(_ring.buf.size(), std::memory_order_release);
        _deviceUp.store(true, std::memory_order_release);
        _async.start([this] {
            nameStreamingThread("uhdtxev", this->name.value);
            asyncLoop();
        });
        _drain.start([this, priority = thread_priority.value, processor = cpu.value] {
            nameStreamingThread("uhdtx", this->name.value);
            std::ignore = placeStreamingThread("uhd::Sink", "transmit", priority, processor);
            drainLoop();
        });
        return {};
    }

    void stop() { std::ignore = hardwareTeardown(); }

    /*| contract: end the burst and stop sending, without giving up the device. The queue stays
            where it is, so a resume sends what was already handed over.
        frame: the drain ends the burst rather than the thread that asked for the pause, because
            two sends on one streamer are not safe together and the drain may have one in
            flight.
    */
    void pause() {
        _held.store(true, std::memory_order_release);
        wakeDrain();
    }

    // The block sends again, and the next samples open a fresh burst.
    void resume() {
        _held.store(false, std::memory_order_release);
        wakeDrain();
    }

    // Ends the drain's wait, for a caller that has just changed what the drain waits on.
    void wakeDrain() noexcept { _drainBell.ring(); }

    /*| contract: idempotent and serialized. It returns true to exactly one caller per open
            device — the one that released it.
        why: also run by _teardown's destructor when stop() never fired. Serialized because two
            threads can request a stop at once, and unserialized they would double-join one
            thread.
        trap: what the work calls already handed over goes out before the streamer is released,
            and the wait for it is bounded by how long the queue takes to play out. A device
            that has stopped draining would otherwise hold the stop for as long as it stays
            attached. A stop made while the carrier is held waits for nothing: the drain is
            parked and the queue it holds is counted as unsent.
    */
    bool hardwareTeardown() {
        std::lock_guard teardown(_teardownMutex);
        /*| invariant: the reference wait's answer falls first, and the start mutex is taken behind
                it, so a start inside its wait lets go of its snapshot of the handle within one
                poll and the release below is the device's last.
            verified-by: sink.uhd-a-stop-ends-the-reference-wait
        */
        _stopping.store(true, std::memory_order_release);
        {
            std::lock_guard startLock(_startMutex);
        }
        bool hadDevice = false;
        {
            std::lock_guard lock(_ctrlMutex);
            hadDevice = _usrp != nullptr;
        }
        if (hadDevice) {
            playOut();
        }
        _drain.requestStop();
        wakeDrain();
        _drain.joinIfRunning();
        if (hadDevice) {
            _counters.unsentAtStop.store(static_cast<std::uint64_t>(ringElements()), std::memory_order_relaxed);
            /*| invariant: the burst is ended before the streamer goes, so the radio stops keying
                    rather than being left holding the last sample, and the device's
                    acknowledgment of that end is waited for before the streamer and the device
                    are released. The asynchronous thread still reads the channel here, and it is
                    the thread that counts the acknowledgment.
            */
            double      rateHz = 0.0;
            std::string wireFormat;
            std::size_t heldBytes = kDeviceHeldBytes;
            {
                std::lock_guard lock(_ctrlMutex);
                rateHz     = _rateActualHz;
                wireFormat = _wireFormat;
                heldBytes  = _deviceHeldBytes;
            }
            endBurstAtStop(streamerSnapshot(), rateHz, wireFormat, heldBytes);
        }
        _deviceUp.store(false, std::memory_order_release);
        _async.requestStop();
        _async.joinIfRunning();
        /*| invariant: a sensor sweep is waited out before the device is released, so the handle
                it snapshotted does not outlive this call and the next start does not find the
                device still open. _deviceUp is already false, so the sweep aborts at its next
                read and this waits for one sensor rather than a whole sweep.
        */
        std::lock_guard sensors(_sensorMutex);
        std::lock_guard lock(_ctrlMutex);
        releaseLater(std::move(_stream), std::move(_usrp));
        _openTruth        = {};
        _sensorNames      = {};
        _sensorNamesTaken = false;
        _maxSendSamples   = 0UZ;
        _deviceHeldBytes  = kDeviceHeldBytes;
        publishSurfaceLocked();
        return hadDevice;
    }

    /*| contract: a drain that has ended ends the block, the way a source's liveness check ends
            one whose stream has gone.
    */
    gr::work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!gr::lifecycle::isActive(this->state())) {
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        if (_deviceUp.load(std::memory_order_acquire) && _drainDead.load(std::memory_order_acquire)) {
            this->emitErrorMessage("work()", "the transmit stream stopped taking samples");
            this->requestStop();
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        /*| why: here rather than in processBulk, because a work call is made whether or not the
                input edge has samples, and the queue a refused park left oversized is owed a
                resize whichever it is. This thread is the ring's producer either way. */
        takeOwedQueueResize();
        return gr::Block<Sink>::work(requestedWork);
    }

    /*| contract: stage what the ring has room for and consume exactly that. A full ring
            consumes nothing and reports it, so the scheduler offers the same samples again;
            nothing here waits on the device.
        invariant: every sample goes through clipUnit before it reaches the ring. UHD's fc32 to
            sc16 conversion saturates on its SSE path and casts on its generic one, and a part
            past full scale cast into an integer format comes out with its sign turned over, so
            the bound is applied on this side rather than left to whichever path a build
            selected.
        measured: a scratch unit over uhd::convert on UHD 4.9.0.1 saturated: +1.5 and +2.0 both
            converted to 32767 and -1.5 to -32768. Which path a build selects still decides it,
            so the clip stands.
        why: the queue stands between this call and the device because send() blocks for as
            long as the device's buffers are full. This call runs on the scheduler's thread,
            which runs the other blocks of its job, and a send made here would hold all of
            them at the device's pace. The clip and the copy into the queue are one pass, the
            only one the block makes over the samples before send() reads the queue in place.
    */
    gr::work::Status processBulk(gr::InputSpanLike auto& input) noexcept {
        if (!_deviceUp.load(std::memory_order_acquire)) {
            std::ignore = input.consume(input.size()); // no streamer behind the ring
            return gr::work::Status::OK;
        }
        const std::size_t want = input.size();
        if (want == 0UZ) {
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const std::size_t take = stage(input.data(), want);
        std::ignore            = input.consume(take);
        return take == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::OK;
    }

    /*| contract: clip and copy what the queue has room for out of the n samples at samples, ring
            the drain's bell where anything went in, and answer how many went in. The caller is
            the queue's one producer.
        why: the bell wakes a drain waiting on an empty queue. The ring costs a fence and a
            load and no system call while the drain is sending.
        verified-by: sink.uhd-a-write-wakes-the-waiting-drain
    */
    std::size_t stage(const std::complex<float>* samples, std::size_t n) noexcept {
        const std::size_t take = writeWhatFitsWith(_ring, samples, n, [](std::complex<float> s) { return clipUnit(s); });
        if (take > 0UZ) {
            wakeDrain();
        }
        return take;
    }

    /*| contract: frequency, gain and connector reach the device live; a rate change ends the
            burst and rewrites the rate, and the next samples open a fresh burst at it.
    */
    void settingsChanged(const gr::property_map& /*oldSettings*/, gr::property_map& newSettings, gr::property_map& /*forwardSettings*/) {
        const ControlSurfaceCheck<Sink> surfaceCheck{*this}; // sends a moved surface once the body has let go of its locks
        if (!_deviceUp.load(std::memory_order_acquire)) {
            return;
        }
        /*| invariant: the queue a rate change asks for is allocated before _ctrlMutex is taken,
                and the storage it replaces is freed after the mutex is let go. The size comes
                from the rate the request snaps to, and a device that coerces the write to a
                rate of another size makes the resize allocate under the mutex instead.
        */
        RingStorage<std::complex<float>> storage;
        if (newSettings.contains("sample_rate")) {
            storage = queueStorageFor(queueSamplesForRequest());
        }
        std::lock_guard lock(_ctrlMutex);
        if (_usrp == nullptr) {
            return; // teardown resets the handle under this mutex
        }
        if (newSettings.contains("frequency") || newSettings.contains("frequency_correction")) {
            std::ignore = applyFrequencyLocked();
        }
        if (newSettings.contains("sample_rate")) {
            _queueSpare.swap(storage);
            std::ignore = applyRateLocked();
            _queueSpare.swap(storage);
        }
        if (newSettings.contains("tx_gains")) {
            applyGainLocked();
        }
        if (newSettings.contains("tx_antennae")) {
            applyAntennaLocked();
        }
    }

    // ---- the device, under _ctrlMutex ---------------------------------------------------
    /*| trap: a null handle is a stop that took the device. Every body below that calls libuhd
            tests for it under the control mutex the teardown resets it under, because a start
            reads the lifecycle state before it takes that mutex and a stop can land between
            the two.
        verified-by: sink.uhd-a-stop-that-took-the-handle-calls-no-library
    */

    /*| contract: put the device on the reference and the timing selection the settings name,
            before anything else this start configures, where applyReferenceSources says the
            writes belong. An empty name leaves the device on its own selection.
    */
    void applyReferenceSourcesLocked() {
        if (_usrp == nullptr) {
            return;
        }
        applyReferenceSources(*_usrp, clock_source.value, time_source.value, "uhd::Sink");
    }

    /*| contract: put the open device on what the settings ask for, in the order a start owes it:
            the filter range, the width and the coverage the truth read at the open states, the
            clock the open address fixed, the rate through write, and behind a rate that landed
            the connector, the center and the gain. Answers why the start fails, or an empty
            string. The caller holds _ctrlMutex and has read the truth of the open device.
        contract: a rate the device refused leaves sample_rate on the request. The start fails
            and releases the device, so the rate that device held names nothing a caller runs
            on, and a restart of the block asks for the request again.
        why: a transmitter that cannot reach the rate asked for transmits nothing. Left to
            finish, the start arms a burst at the rate the previous program left, with a queue
            sized for another; the rate goes first because the coverage and the queue follow it.
        why: a transmitter that cannot reach the frequency asked for transmits nothing. Left to
            finish, the start arms a burst at whatever the previous program left the
            synthesizer on, while the status line states a frequency the radio is not on.
        verified-by: sink.uhd-a-start-configures-the-rate-first
    */
    template <typename WriteRate>
    std::string configureOpenDeviceLocked(WriteRate&& write) {
        _bwMinHz = _openTruth.bwMinHz;
        _bwMaxHz = _openTruth.bwMaxHz;
        /*| why: the width the device came up with, so a frontend whose filter is fixed in the
                hardware — a WBX states 40 MHz to 40 MHz — states that width rather than nothing.
                The bandwidth applier runs only where the stated range has room in it, so on
                such a frontend it never runs at all. */
        _bwCurHz = _openTruth.bwCurHz;
        publishSurfaceLocked();
        /*| why: the coverage the device stated at the open, so a tune is tested against a real
                span even where the rate applier makes no device call. The applier is otherwise
                the only body that fills this. */
        _freqRanges = _openTruth.freqRanges;
        takeClockInForceLocked();
        const double asked = sample_rate;
        if (!applyRateWith(write)) {
            sample_rate = asked;
            return "uhd: the sample rate was refused";
        }
        applyAntennaLocked();
        if (!applyFrequencyLocked()) {
            return "uhd: the center frequency was refused";
        }
        applyGainLocked();
        return {};
    }

    // The rate applier on this block's device. The caller holds _ctrlMutex.
    bool applyRateLocked() {
        if (_usrp == nullptr) {
            return true;
        }
        return applyRateWith([this](double hz) { return writeRate(*_usrp, Direction::Transmit, hz, "uhd::Sink"); });
    }

    /*| contract: write the rate rateToWrite chooses for the request through write, which answers
            what the write left on the device, and behind it the analog filter the rate asks
            for, and state the rate the device holds in sample_rate. False
            where the device refused the write, which leaves the setting on the rate the device
            still holds. A rate already in force is not written again. A new one plays the queue
            out at the rate it was staged at, asks the drain to stand off the ring and resizes
            the queue for the new rate, whether or not a burst is open; the next samples open a
            fresh burst at it, and the drain ends the burst in flight where there is one.
        verified-by: sink.uhd-a-rate-change-before-the-first-send-resizes-the-queue
        frame: the wait is bounded by how long the queue takes to play out at the rate in
            force. A device that has stopped draining leaves samples the bound could not place;
            the resize throws those away and discardedAtRateChangeSamples() states how many.
        trap: a drain that did not stand off the ring inside its own bound leaves the queue
            where it is. The rate is written alone, the queue keeps the sample count the
            previous rate gave it rather than its length of time, and the line says so. The
            resize is owed from there: the drain asks for the park again at the head of its own
            loop while the queue outsizes the rate in force, and the work call takes the resize
            behind that answer, so the queue holds the wrong length of time for as long as the
            drain stays inside that send. Every wait made in that window is bounded by
            playOutBoundMs, which takes the capacity the rate in force asks for.
        trap: a rate change made while the carrier is held discards the queue the pause kept.
            The play-out returns at once under a hold, a parked drain placing nothing, so the
            resize behind it throws the whole queue away and discardedAtRateChangeSamples()
            carries it. This family's pause counts nothing, so the rate change's counter
            carries the whole sequence.
        why: the samples in the queue are the caller's, handed over and acknowledged, so they
            are not the receive side's drop-and-mark case: that answer is for data a device
            produced. Written under them, the new rate would transmit them at a rate they were
            never sampled at, and carrying them to the new rate is no answer either — the
            device that left them there is one that stopped consuming.
        trap: the burst is ended by the drain and not here. This runs on the block's own worker
            thread, so a send mutex taken here would put every other block in the job behind a
            send against a device that stopped consuming.
        verified-by: sink.uhd-a-rate-change-ends-the-burst
        verified-by: sink.uhd-a-start-configures-the-rate-first
    */
    template <typename WriteRate>
    bool applyRateWith(WriteRate&& write) {
        const double     asked  = sample_rate;
        const double     want   = rateToWrite(asked);
        const RateChange change = rateChangeFor(want, _rateActualHz, _bursting.load(std::memory_order_acquire));
        if (!change.write || !writeNeeded(_rateWrittenHz, want)) {
            if (_rateActualHz > 0.0) {
                sample_rate = _rateActualHz; // the rate in force, not the request a replay carried
            }
            refreshFreqRangesLocked();
            return true;
        }
        /*| why: the queue follows the rate whether or not a burst is open. _bursting rises only
                inside a send, so a rate staged before the first samples flow would skip the
                resize and leave the queue sized for the previous rate: 65536 samples after a
                start at 250 kS/s, which at 61.44 MS/s is 1.07 ms against the scheduler's
                100 ms park. The play-out returns at once for an empty queue and the drain's
                park branch ends a burst only where one is open, so both are asked for
                whenever the rate reaches the device.
            frame: the start sizes the queue itself once the streamer has stated its maximum
                send, so a rate written before the streamer exists leaves the queue to that
                step. The fresh burst the next samples open is this family's restart: a
                transmit streamer carries no stream command of its own.
        */
        const bool haveStreamer = _stream != nullptr;
        bool       parked       = true;
        if (haveStreamer) {
            playOutLocked();
            parked = parkDrainForRateChange();
        }
        const RateWrite written = write(want);
        if (rateWriteSettled(written)) {
            _rateWrittenHz = want;
        }
        if (written.heldHz > 0.0) {
            _rateActualHz = written.heldHz;
            publishSurfaceLocked();
            publishSendCapLocked();
            if (std::abs(written.heldHz - asked) > 0.5) {
                sample_rate = written.heldHz;
            }
        }
        /*| invariant: the transmit filter follows the rate the way the receive filter follows the
                sample rate, so a burst leaves through a filter its own width rather than through
                whatever the device came up with.
        */
        if (filterFollowsRate(written, _bwMinHz, _bwMaxHz)) {
            applyBandwidthLocked(bandwidthFor(_rateActualHz, _bwMinHz, _bwMaxHz));
        }
        if (haveStreamer) {
            // A parked drain stands off the ring and the work call is this same thread, so the
            // ring has neither a producer nor a consumer here.
            takeQueueBoundLocked();
            resizeQueueForRateLocked(parked);
            _rateBreak.store(false, std::memory_order_release);
            wakeDrain();
        }
        refreshFreqRangesLocked();
        return written.landed;
    }

    /*| contract: the rate to hand the device for a request: the request on a radio whose clock is
            programmable, and the nearest rate of the transmit ladder the clock and the link
            reach on a fixed clock or on one the open address fixed. The caller holds _ctrlMutex.
        why: an N210's transmit DSP interpolates its fixed 100 MHz clock by a whole number, and
            an odd one turns both halfbands off: 8 MS/s written as it stands lands on
            7.692 MS/s, interpolation 13, on the CIC alone, and the passband droops across the
            signal on the air.
        verified-by: sink.uhd-snaps-the-rate-as-the-source-does
    */
    double rateToWrite(double requestHz) const { return capture::uhd::rateToWrite(requestHz, _openTruth, _clockInForceHz, _wireFormat, Direction::Transmit, "uhd::Sink"); }

    /*| contract: take the clock the open address fixed as the clock in force, so the transmit
            ladder is that clock's. The caller holds _ctrlMutex and has read the truth of the open
            device.
        verified-by: sink.uhd-an-address-that-pins-the-clock-fixes-the-ladder
    */
    void takeClockInForceLocked() { _clockInForceHz = clockInForceAtOpen(openAddress(), _openTruth); }

    /*| contract: ask the drain to end the burst and stand off the ring, and answer whether the
            ring has a consumer left: true where the drain raised the parked flag, where it has
            died and where no drain is running at all, and false where a live drain did not
            answer inside bound. The caller holds _ctrlMutex and clears _rateBreak once the
            rate is written.
        invariant: the answer waited for is a fresh one. The flag is cleared before the break is
            raised, because the drain raises it for a pause as well, and a flag left standing by
            a pause would answer for a drain that took the resume and is already back in the
            copy.
        frame: bound is one send by default, kSendTimeoutS, read from the streamer's own send
            timeout as the longest a send can hold the drain away from the top of its loop; an
            idle drain wakes on the bell this rings and answers at once. A device that stopped consuming
            is the case this bracket exists for, so that is the bound a rate change owes beside
            the play-out's, and the settings thread holds _ctrlMutex through both.
        why: a wait rather than a shorter deadline, because the resize behind it replaces the
            ring's storage and a live drain reads that storage between its own availability test
            and the end of the send that reads it in place.
        verified-by: sink.uhd-a-rate-change-waits-for-a-fresh-park
    */
    bool parkDrainForRateChange(std::chrono::milliseconds bound = std::chrono::milliseconds(static_cast<long>(kSendTimeoutS * 1000.0))) {
        _drainParked.store(false, std::memory_order_release);
        _rateBreak.store(true, std::memory_order_release);
        wakeDrain();
        const auto deadline = std::chrono::steady_clock::now() + bound;
        for (;;) {
            if (_drainParked.load(std::memory_order_acquire) || _drainDead.load(std::memory_order_acquire) || !_drain.thread.joinable()) {
                return true;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    /*| contract: size the queue for the rate the device took, counting what the resize throws
            away, or keep the queue where the drain did not stand off the ring. The caller
            holds _ctrlMutex, has written the rate, and says whether the park arrived.
        trap: a resize under a live drain reassigns the storage that drain indexes between its
            own availability test and the end of the send that reads it in place. A queue left sized for the
            previous rate holds the wrong length of time; a queue resized under a consumer is
            a fault, so the refused park keeps the queue and says so. This body publishes the
            capacity the rate in force asks for either way, and that publication leaves the
            resize owed.
        contract: storage of the size asked for stays in place. Other storage is replaced by
            _queueSpare where it holds that size and by a fresh allocation otherwise, and the
            storage replaced goes into _queueSpare for the caller to free outside the mutex.
        verified-by: sink.uhd-a-refused-park-keeps-the-queue
        verified-by: sink.uhd-a-rate-change-reuses-the-queue-that-fits
        verified-by: sink.uhd-a-refused-park-is-retried-until-it-lands
    */
    void resizeQueueForRateLocked(bool parked) {
        _ringWantSamples.store(ringWantSamplesLocked(), std::memory_order_release);
        if (!parked) {
            std::fprintf(stderr, "[uhd::Sink] the drain did not stand off the queue inside %.0f s; the rate is written and the queue stays as it is until the drain answers\n", kSendTimeoutS);
            return;
        }
        _counters.discardedAtRateChange.fetch_add(static_cast<std::uint64_t>(resizeRingCountingDiscard(_ring, ringWantSamplesLocked(), _queueSpare)), std::memory_order_relaxed);
        _ringCapacity.store(_ring.buf.size(), std::memory_order_release);
    }

    // The queue capacity the rate in force asks for, in samples. The caller holds _ctrlMutex.
    std::size_t ringWantSamplesLocked() const { return ringSamplesFor(_rateActualHz, ringFloorSamples(_maxSendSamples), _queueBound); }

    // Read queue_duration and queue_max_bytes into the bound every sizing takes. The caller
    // holds _ctrlMutex.
    void takeQueueBoundLocked() { _queueBound = {.seconds = queue_duration, .maxBytes = queue_max_bytes}; }

    // The queue capacity the rate the current request snaps to asks for, in samples. Zero
    // where the control mutex stays busy past the status readers' deadline.
    std::size_t queueSamplesForRequest() const {
        CtrlGuard guard(_ctrlMutex, _deviceUp, kStatusWait);
        if (!guard) {
            return 0UZ;
        }
        return ringSamplesFor(rateToWrite(sample_rate), ringFloorSamples(_maxSendSamples), {.seconds = queue_duration, .maxBytes = queue_max_bytes});
    }

    // Publish the send cap of the rate in force for the drain. The caller holds _ctrlMutex.
    void publishSendCapLocked() { _sendCapSamples.store(sendCapSamples(_rateActualHz, _maxSendSamples), std::memory_order_release); }

    /*| contract: storage for a queue of samples, the size a resize to that capacity takes, and
            empty where the queue already holds that size or samples is zero. The caller holds no lock and hands
            the answer to _queueSpare under _ctrlMutex.
        verified-by: sink.uhd-a-rate-change-reuses-the-queue-that-fits
    */
    RingStorage<std::complex<float>> queueStorageFor(std::size_t samples) const {
        const std::size_t size = ringCapacityFor(samples);
        if (samples == 0UZ || size == _ringCapacity.load(std::memory_order_acquire)) {
            return {};
        }
        return RingStorage<std::complex<float>>(size);
    }

    /*| contract: whether the queue holds more storage than the rate in force asks for, which is
            what a refused park leaves behind. It takes no lock: both figures are published
            beside the ring.
        frame: the two are equal for a queue sized at the rate it runs at, the capacity asked
            for already being rounded up to the power of two the ring indexes by.
        verified-by: sink.uhd-a-refused-park-is-retried-until-it-lands
    */
    bool queueOutsizesRate() const {
        const std::size_t want = _ringWantSamples.load(std::memory_order_acquire);
        return want > 0UZ && _ringCapacity.load(std::memory_order_acquire) > want;
    }

    /*| contract: ask for the park again where the queue outsizes the rate in force, and say
            whether the ask was made. The drain calls this at the head of its own loop, which is
            the one place a break is answered, so the ask reaches a standing-off drain on the
            pass after the send that refused the rate change's own park returned.
        verified-by: sink.uhd-a-refused-park-is-retried-until-it-lands
    */
    bool askParkForOwedResize() {
        if (!queueOutsizesRate()) {
            return false;
        }
        _rateBreak.store(true, std::memory_order_release);
        return true;
    }

    /*| contract: take the resize a refused park left owed, where the drain has since stood off
            the ring. It counts what it throws away under the rate change's own counter and says
            so once, that being the rate change the park was refused for. The caller is the
            ring's producer and holds no lock.
        why: the work call is the ring's one producer and the parked drain is its one consumer,
            so this body is the same bracket the rate applier resizes in. The drain cannot
            resize the ring itself: it is not the producer, and the storage it would replace is
            the storage a work call indexes.
        trap: the control mutex is taken with the status readers' own deadline. A rate change
            holds it through a play-out, and the resize stays owed until a later work call
            rather than putting this one behind that wait.
        verified-by: sink.uhd-a-refused-park-is-retried-until-it-lands
    */
    void takeOwedQueueResize() {
        if (!queueOutsizesRate() || !_drainParked.load(std::memory_order_acquire)) {
            return;
        }
        RingStorage<std::complex<float>> storage = queueStorageFor(_ringWantSamples.load(std::memory_order_acquire));
        CtrlGuard                        guard(_ctrlMutex, _deviceUp, kStatusWait);
        if (!guard || !queueOutsizesRate()) {
            return;
        }
        std::fprintf(stderr, "[uhd::Sink] the queue the refused park left is sized for a faster rate than the %.0f S/s in force; what it holds is discarded\n", _rateActualHz);
        _queueSpare.swap(storage);
        resizeQueueForRateLocked(true);
        _queueSpare.swap(storage);
        _rateBreak.store(false, std::memory_order_release);
        wakeDrain();
    }

    // The shared filter write on this block's device. The caller holds _ctrlMutex.
    void applyBandwidthLocked(double hz) {
        if (_usrp != nullptr) {
            applyBandwidth(*_usrp, Direction::Transmit, hz, _bwWrittenHz, _bwCurHz, "uhd::Sink");
            publishSurfaceLocked();
        }
    }

    // The shared coverage re-read on this block's device. The caller holds _ctrlMutex.
    void refreshFreqRangesLocked() {
        if (_usrp != nullptr) {
            refreshFreqRanges(*_usrp, Direction::Transmit, _freqRanges, "uhd::Sink");
        }
    }

    /*| contract: tune the synthesizer where the settings and the correction in force say. False
            where the request is outside the coverage the device states or the device refused
            the write, which leaves the tuner where it is.
        invariant: the request this block states is written once, from the read-back carried out
            of the correction, and a replay of a value already in force leaves it where the
            write that put it there left it. Written from the raw request as well, the settings
            replay shortly after the start would put that request back over the read-back, and
            describe() and tunedRfHz() would state two different numbers for one tune within a
            run.
        contract: the center the tuner reached goes back into the frequency setting, so a
            refused request does not leave the setting naming a center the radio is not on.
        verified-by: sink.uhd-states-the-frequency-the-caller-asked-for
    */
    bool applyFrequencyLocked() {
        if (frequency->empty()) {
            return false;
        }
        const double request = frequency->front();
        const auto   f       = tuneFrequencyFor(request, frequency_correction, _freqRanges);
        if (!f) {
            std::fprintf(stderr, "[uhd::Sink] %.0f Hz is outside this radio's transmit coverage; the tuner stays where it is\n", request);
            forwardTunedFrequencyLocked();
            return false;
        }
        if (writeNeeded(_freqWrittenHz, *f)) {
            if (_usrp == nullptr) {
                return false;
            }
            try {
                _usrp->set_tx_freq(::uhd::tune_request_t(*f), 0);
                _freqWrittenHz = *f;
                // The read-back carried back into the frame the caller asks in.
                _appliedFreqHz    = _usrp->get_tx_freq(0);
                _appliedRequestHz = tunedFrequencyFrom(_appliedFreqHz, frequency_correction);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[uhd::Sink] set_tx_freq(%.0f) failed: %s (continuing)\n", *f, e.what());
                return false;
            }
        }
        forwardTunedFrequencyLocked();
        return true;
    }

    /*| contract: write the center the tuner reached back into the frequency setting, in the
            caller's own frame. A block that has reached no center yet leaves the setting where
            the caller staged it, having no answer to write. The caller holds _ctrlMutex and
            runs on the block's own worker thread, which is the one thread the settings
            machinery rewrites that member on.
        why: a request outside the coverage is refused and the tuner stays where it is, so a
            setting left naming that request states a center the radio is not on for the rest
            of the run, and a caller drawing its frequency axis from the setting rather than
            from the reader is wrong from there. The peer source writes the same answer back.
        frame: the member is the block's own and the framework's parameter map is a copy of it
            taken twice in each settings sweep, before the block's settings hook and after it.
            The tune a settings change makes runs inside that hook, so the map carries its
            write-back at the end of the same sweep; the tune a start makes runs outside a
            sweep, and its write-back waits for the next one. tunedRfHz() carries either at
            once.
        verified-by: sink.uhd-states-the-frequency-the-caller-asked-for
    */
    void forwardTunedFrequencyLocked() {
        if (_appliedRequestHz > 0.0) {
            frequency = std::vector<double>{_appliedRequestHz};
        }
    }

    /*| contract: bound the overall gain by the range the open device states and write a gain
            the bound moved back into tx_gains, as the peer source does with rx_gains.
        verified-by: controls.uhd-a-bounded-gain-is-written-back
    */
    void applyGainLocked() {
        if (tx_gains->empty()) {
            return;
        }
        const double want = boundedGainFor(_openTruth, "", tx_gains->front(), "uhd::Sink");
        if (!(want == tx_gains->front())) {
            tx_gains = std::vector<double>{want};
        }
        if (_usrp == nullptr || !writeNeeded(_gainWrittenDb, want)) {
            return;
        }
        try {
            _usrp->set_tx_gain(want, 0); // overall gain, UHD distributes it across the elements
            _gainWrittenDb = want;
            _actualGainDb  = _usrp->get_tx_gain(0);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Sink] set_tx_gain(%.1f) failed: %s (continuing)\n", want, e.what());
        }
    }

    void applyAntennaLocked() {
        if (_usrp == nullptr) {
            return;
        }
        const std::string wanted = tx_antennae->empty() ? std::string{} : tx_antennae->front();
        std::string       pick;
        try {
            pick = chooseAntenna(_usrp->get_tx_antennas(0), wanted);
        } catch (const std::exception&) { // a frontend with one connector and no list
        }
        if (pick.empty() || !writeNeeded(_antennaWritten, pick)) {
            return;
        }
        try {
            _usrp->set_tx_antenna(pick, 0);
            _antennaWritten = pick;
            _actualAntenna  = _usrp->get_tx_antenna(0);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Sink] set_tx_antenna(%s): %s (continuing)\n", pick.c_str(), e.what());
        }
    }

    // ---- the writable control surface -------------------------------------------------------

    /*| contract: whether setControl writes this id at a device stating this truth. It answers
            true for exactly the ids describeControls states for that device other than
            SAMPLE_RATE and FREQUENCY, which the control property stages as settings, so a caller
            drawing the surface can write every part of it and nothing the surface leaves out.
        trap: the answer is derived from the surface rather than listed beside it. A model with
            one connector states no ANTENNA control and a frontend whose filter range has equal
            ends states no BANDWIDTH — a WBX states 40 MHz to 40 MHz — and setControl refuses
            both. An answer naming the two whatever the device states offers a caller a fixed
            filter as a slider with nothing at the other end.
        verified-by: sink.uhd-every-control-it-describes-it-writes
    */
    static bool controlIsWritable(const DeviceTruth& t, const std::string& id) {
        const TruthContext ctx{.deviceParams = {}, .sampleRate = 0.0, .centerFreq = 0.0};
        const auto         surface = describeControls(t, ctx);
        return id != kSampleRateControl && id != kFrequencyControl && std::ranges::any_of(surface, [&id](const ControlDesc& c) { return c.id == id; });
    }

    /*| contract: write one gain element at the device, bounded by the range that element
            states. False for an element the open device does not name, which is named on
            standard error beside the names it gives, or for an element write the device
            refused. The overall gain, the empty name, is bounded by the overall range, which
            UHD distributes across the elements itself, and goes through tx_gains as
            settingForControl states for the connector: true once the setting is staged, the
            value reaching the device at the next work call's settings sweep, after the resume
            on a paused graph, and a refusal there named on standard error alone.
        why: a caller that wants one element at a particular value writes it here rather than
            through tx_gains.
        verified-by: sink.uhd-a-control-writes-through-its-setting
    */
    bool setElementGain(const std::string& element, double dB) {
        gr::property_map through;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp);
            if (!guard) {
                return false;
            }
            if (!element.empty()) {
                return setElementGainLocked(element, dB);
            }
            through = {{"tx_gains", std::vector<double>{boundedGainFor(_openTruth, "", dB, "uhd::Sink")}}};
        }
        stageThrough(through);
        return true;
    }

    /*| contract: the setting a control writes through, with the value it writes, for a control
            this block also carries as a setting: ANTENNA as tx_antennae, the connector the index
            names. Empty for a control written at the device directly, and for an index past the
            device's list.
        why: a control and a setting that name one thing are two writers. A control that wrote
            the device alone left the setting where it was, and the next settings sweep carrying
            it, the replay after a start among them, put the setting's value back.
        verified-by: sink.uhd-a-control-writes-through-its-setting
    */
    static gr::property_map settingForControl(const DeviceTruth& t, const std::string& id, double bounded) {
        if (id == "ANTENNA") {
            const auto index = static_cast<std::size_t>(std::lround(bounded));
            if (index < t.antennas.size()) {
                return {{"tx_antennae", std::vector<std::string>{t.antennas[index]}}};
            }
        }
        return {};
    }

    /*| contract: stage a control's setting for the next settings sweep, which writes the device.
        trap: called with the control mutex free. The settings sweep runs this block's settings
            hook, which takes the control mutex, inside the settings machinery's own lock, so a
            stage made under the control mutex takes the two in the opposite order.
    */
    void stageThrough(const gr::property_map& through) { std::ignore = this->settings().setStaged(through); }

    /*| contract: write one control, with the value brought inside the domain the surface states
            for the device that is open. An id the device does not carry, or one the surface does
            not state, is refused rather than written. ANTENNA goes through tx_antennae, as
            settingForControl states: true once the setting is staged, the connector reaching
            the device at the next work call's settings sweep, after the resume on a paused
            graph, and a refusal there named on standard error alone. Every other id is written
            at the device, and false names a write the device refused.
        invariant: every value reaching a UHD call here has been through boundedControlValue, so
            a request past the frontend's range reaches the device bounded rather than coerced,
            and a connector index past the device's list is refused.
        verified-by: sink.uhd-every-control-it-describes-it-writes
    */
    bool setControl(const std::string& id, double value) {
        gr::property_map through;
        {
            CtrlGuard guard(_ctrlMutex, _deviceUp);
            if (!guard || !controlIsWritable(_openTruth, id)) {
                return false;
            }
            const TruthContext ctx{.deviceParams = {}, .sampleRate = _rateActualHz, .centerFreq = 0.0};
            const auto         bounded = boundedControlValue(describeControls(_openTruth, ctx, _bwCurHz), id, value, "uhd::Sink");
            if (!bounded.has_value()) {
                return false; // an id the surface does not state, or an index past the device's list
            }
            const double want = *bounded;
            through           = settingForControl(_openTruth, id, want);
            if (through.empty()) {
                if (_usrp == nullptr || id == "ANTENNA") {
                    return false;
                }
                if (id == "BANDWIDTH") {
                    applyBandwidthLocked(want * 1e6); // the descriptor states MHz
                    refreshFreqRangesLocked();        // the coverage moves with the filter
                    return _bwCurHz > 0.0;
                }
                return setElementGainLocked(id, want);
            }
        }
        stageThrough(through);
        return true;
    }

    // The caller holds _ctrlMutex and re-tested liveness under it.
    bool setElementGainLocked(const std::string& element, double dB) {
        if (_usrp == nullptr || !gainElementNamed(_openTruth, element, "uhd::Sink")) {
            return false;
        }
        const double bounded = boundedGainFor(_openTruth, element, dB, "uhd::Sink");
        try {
            if (element.empty()) {
                _usrp->set_tx_gain(bounded, 0);
                _gainWrittenDb = bounded;
            } else {
                _usrp->set_tx_gain(bounded, element, 0);
            }
            _actualGainDb = _usrp->get_tx_gain(0);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Sink] set_tx_gain(%s, %.1f): %s (continuing)\n", element.c_str(), dB, e.what());
            return false;
        }
        return true;
    }

    // ---- the burst and the drain ------------------------------------------------------------

    ::uhd::tx_streamer::sptr streamerSnapshot() const {
        std::lock_guard lock(_ctrlMutex);
        return _stream;
    }

    /*| contract: whether a parked drain ends the burst on this pass. Once per park: the flag
            the park raises is the record that the attempt was made.
        why: an end of burst is a send, and against a device that stopped consuming it runs the
            whole send timeout, leaves the carrier up and says so. Attempted on every pass, a
            pause against such a device costs one attempt and one line a second for as long as
            the pause lasts, and a stop then joins behind the attempt in flight. The timeout
            is kSendTimeoutS, read from the argument every send here carries; no case drives a
            device that stops consuming.
        verified-by: sink.uhd-a-parked-drain-ends-the-burst-once, which pins the once-per-park
            rule and the fresh park a rate change behind a pause makes.
    */
    static bool endBurstOnThisPass(bool alreadyParked) { return !alreadyParked; }

    /*| contract: end the burst if one is open, and say what the attempt did: NoneOpen where no
            burst was open, Placed where the end went out, and NotPlaced where the send placed
            nothing. The radio stops keying on an end placed; the streamer and the device stay,
            and the next send opens a fresh burst. Every end placed is counted for the stop's
            wait.
        invariant: the streamer is taken as a shared handle, so it stays alive for the send even
            where a teardown resets the block's own pointer underneath.
        trap: a send that placed nothing leaves the radio keyed, so the burst flag goes back up
            and the line says so. UHD returns fewer samples than asked for under a timeout, and
            a flag lowered on such a send tells a teardown the carrier is down when it is not.
        verified-by: sink.uhd-a-stop-waits-for-its-own-acknowledgment
    */
    EndOfBurst endBurstWith(const ::uhd::tx_streamer::sptr& stream) {
        if (stream == nullptr) {
            return EndOfBurst::NoneOpen;
        }
        std::lock_guard send(_sendMutex);
        if (!_bursting.exchange(false, std::memory_order_acq_rel)) {
            return EndOfBurst::NoneOpen;
        }
        try {
            ::uhd::tx_metadata_t md;
            md.start_of_burst = false;
            md.end_of_burst   = true;
            md.has_time_spec  = false;
            const std::complex<float> nothing{};
            if (stream->send(&nothing, 1, md, kSendTimeoutS) == 0UZ) {
                _bursting.store(true, std::memory_order_release);
                std::fprintf(stderr, "[uhd::Sink] the end of burst was not placed after %.0f s; the carrier stays up\n", kSendTimeoutS);
                return EndOfBurst::NotPlaced;
            }
        } catch (const std::exception& e) {
            _bursting.store(true, std::memory_order_release);
            std::fprintf(stderr, "[uhd::Sink] end of burst: %s\n", e.what());
            return EndOfBurst::NotPlaced;
        }
        _endsPlaced.fetch_add(1, std::memory_order_acq_rel);
        return EndOfBurst::Placed;
    }

    /*| contract: end the burst a stop found open on stream and record what the device said about
            it: stopBurstEnd() as that answer states, and unconfirmedByFlushBoundSamples() for a
            stop that gave up its wait or left the carrier up. The wait for the acknowledgment
            ends once burstAcks() reaches the count of ends placed in the run, or at
            burstAckBoundMs, and a stop that gave up says so on standard error.
        why: the device acknowledges the ends in the order they were placed, so the stop's own
            acknowledgment is the one that brings the count up to the ends placed. A pause or a
            rate change ends a burst on the drain, the device acknowledges it once that tail has
            played, and the next burst queues behind it: an acknowledgment counted after the stop
            began can be that earlier burst's.
        verified-by: sink.uhd-a-stop-waits-for-its-own-acknowledgment
    */
    void endBurstAtStop(const ::uhd::tx_streamer::sptr& stream, double rateHz, std::string_view wireFormat, std::size_t heldBytes = kDeviceHeldBytes) {
        BurstEnd said = BurstEnd::NoneOpen;
        switch (endBurstWith(stream)) {
        case EndOfBurst::NoneOpen: break;
        case EndOfBurst::NotPlaced: said = BurstEnd::NotPlaced; break;
        case EndOfBurst::Placed: {
            const auto bound = burstAckBoundMs(rateHz, wireFormat, heldBytes);
            if (waitForBurstAck([this] { return burstAcks(); }, _endsPlaced.load(std::memory_order_acquire), bound)) {
                said = BurstEnd::Acknowledged;
            } else {
                said = BurstEnd::Unconfirmed;
                std::fprintf(stderr, "[uhd::Sink] stopped without the device acknowledging the end of burst inside %lld ms; the tail may not have left the radio\n", static_cast<long long>(bound.count()));
            }
            break;
        }
        }
        const bool undecided = said == BurstEnd::Unconfirmed || said == BurstEnd::NotPlaced;
        _unconfirmedBound.store(undecided ? static_cast<std::uint64_t>(heldBytes / wireBytesPerSample(wireFormat)) : 0ULL, std::memory_order_release);
        _stopBurstEnd.store(said, std::memory_order_release);
    }

    /*| frame: the most an N210 or a B2xx holds between the host and the air, in bytes, and the
            figure a stop takes where the open address states no send buffer. An N210's flow
            control lets the host run the unit's SRAM ahead of the transmit DSP, and UHD 4.9
            states that SRAM as USRP2_SRAM_BYTES, 1 MiB, which is also its default send buffer
            there; a B2xx holds less.
        trap: a model with a larger transmit buffer, an X3xx's DRAM FIFO, holds more than this,
            and a stop on one can give up its wait while the tail still plays unless the address
            states the buffer.
    */
    static constexpr std::size_t kDeviceHeldBytes = 1UZ << 20;

    /*| contract: the longest a stop waits for the device to acknowledge its end of burst: the
            play-out at rateHz of heldBytes, what the device can still hold, in the wire format
            in force, with the margin and the floor every play-out here carries.
        why: the device acknowledges once the last sample has left the radio, so the wait is
            the time the samples it holds take to reach the air: about a second for an N210's
            SRAM at 250 kS/s in sc16, and a few milliseconds past the margin at 61.44 MS/s.
        verified-by: sink.uhd-a-stop-waits-for-the-burst-acknowledgment
    */
    static std::chrono::milliseconds burstAckBoundMs(double rateHz, std::string_view wireFormat, std::size_t heldBytes = kDeviceHeldBytes) {
        return playOutMs(static_cast<std::uint64_t>(heldBytes / wireBytesPerSample(wireFormat)), rateHz);
    }

    /*| contract: wait until acks() reaches target, or until bound has passed, and say whether it
            did. acks is read every millisecond.
        verified-by: sink.uhd-a-stop-waits-for-the-burst-acknowledgment
    */
    template <typename Acks>
    static bool waitForBurstAck(Acks&& acks, std::uint64_t target, std::chrono::milliseconds bound) {
        const auto deadline = std::chrono::steady_clock::now() + bound;
        for (;;) {
            if (acks() >= target) {
                return true;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    /*| contract: let the queue reach the streamer, bounded by how long it takes to play out at
            the rate in force. Called with no lock held, before the drain is joined.
    */
    void playOut() {
        double      rateHz  = 0.0;
        std::size_t maxSend = 0UZ;
        QueueBound  bound;
        {
            std::lock_guard lock(_ctrlMutex);
            rateHz  = _rateActualHz;
            maxSend = _maxSendSamples;
            bound   = _queueBound;
        }
        playOutAt(rateHz, maxSend, bound);
    }

    // The same wait from a caller already holding _ctrlMutex.
    void playOutLocked() { playOutAt(_rateActualHz, _maxSendSamples, _queueBound); }

    /*| contract: the longest a play-out waits for a queue of queuedSamples at rateHz: the queue
            bounded by the capacity that rate asks for, so the deadline is the length of time
            the rate in force gives the queue's own capacity.
        trap: a refused park keeps the queue's sample count rather than its length of time, so
            the queue can hold a faster rate's capacity at a slower rate. Sized from the count
            alone, a block taken from 61.44 MS/s to 250 kS/s with the park refused would set a
            deadline of 67358 ms and hold _ctrlMutex and _teardownMutex for the whole of it
            while every status reader answers empty.
        frame: the bound is the same queue duration every other wait here is sized from, so a
            queue the rate in force asks for is not shortened by it.
        verified-by: sink.uhd-a-play-out-is-bounded-by-the-rate-in-force
    */
    static std::chrono::milliseconds playOutBoundMs(std::size_t queuedSamples, double rateHz, std::size_t maxSendSamples, QueueBound bound = {}) {
        return playOutMs(static_cast<std::uint64_t>(std::min(queuedSamples, ringSamplesFor(rateHz, ringFloorSamples(maxSendSamples), bound))), rateHz);
    }

    /*| trap: a held carrier consumes nothing. The drain parks on _held and takes nothing out of
            the ring, so a wait here cannot end by the queue emptying: it runs its whole
            deadline and leaves the queue where it was. A stop made under a hold is a discard,
            and the count it owes goes out through unsentAtStop; a rate change made under a
            hold is a discard for the same reason, and the resize behind this counts it under
            the rate change.
        verified-by: sink.uhd-a-held-stop-ends-at-once
    */
    void playOutAt(double rateHz, std::size_t maxSendSamples, QueueBound bound = {}) {
        if (_held.load(std::memory_order_acquire)) {
            return;
        }
        const std::size_t queued = ringElements();
        if (queued == 0UZ) {
            return;
        }
        const auto deadline = std::chrono::steady_clock::now() + playOutBoundMs(queued, rateHz, maxSendSamples, bound);
        while (ringElements() > 0UZ && !_drainDead.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    /*| contract: hand the ring to the streamer for as long as the block runs. A send that
            cannot place its samples ends the drain and the block with it, rather than waiting
            on a device that has stopped taking them.
        frame: the blocking send is this family's pace: the work call only keeps the ring fed.
            A pause ends the burst here and leaves the queue where it is, so a resume sends what
            was already handed over.
        frame: a send reads the ring in place: the whole contiguous span the ring holds, never
            across the ring's end, and at most sendCapSamples of the rate in force. The samples
            leave the ring once it returns. A failed send also takes its span off the ring. The
            unsent count a stop reports holds only samples no send was handed.
        frame: the send's own timeout is the drain's one timed wait. An empty ring and a park
            both wait on _drainBell, which the work call rings after each write and which a
            pause, a resume, the rate break and a stop ring after they change.
        trap: a sleep of a fixed length on an empty ring can outlast the device's own buffer.
            A B205mini holds 16 frames of 2040 samples, 0.53 ms at 61.44 MS/s, against a 1 ms
            sleep.
        measured: a B205mini at 61.44 MS/s fed 16384-sample chunks at the wall clock's pace
            took 30,100 sends a second of one frame each, and 140 to 620 sends a second over
            the span. The drain's CPU time stayed at 4.0 to 4.2 ms per million samples either
            way, which is UHD's conversion and transport.
        verified-by: sink.uhd-a-send-takes-the-span-up-to-the-cap
        verified-by: sink.uhd-a-write-wakes-the-waiting-drain
    */
    void drainLoop() {
        const ::uhd::tx_streamer::sptr stream = streamerSnapshot();
        if (stream == nullptr) {
            return;
        }
        while (!_drain.stopRequested()) {
            /*| why: a park a rate change asked for and did not get leaves the queue holding the
                    previous rate's capacity for the rest of the run at the rate in force. The
                    break is raised again here, at the head of the one loop that can answer it,
                    and the work call takes the resize behind that answer. */
            std::ignore = askParkForOwedResize();
            /*| frame: the two ways this loop stands off. A pause keeps the queue and waits for
                    the resume. A rate change has already played the queue out and waits for
                    the rate to be written; the parked flag says the ring has no consumer, so
                    the settings thread can resize it. */
            if (_held.load(std::memory_order_acquire) || _rateBreak.load(std::memory_order_acquire)) {
                if (endBurstOnThisPass(_drainParked.load(std::memory_order_acquire))) {
                    std::ignore = endBurstWith(stream);
                    _drainParked.store(true, std::memory_order_release);
                }
                awaitBell([this] {
                    return !_drain.stopRequested() && (_held.load(std::memory_order_acquire) || _rateBreak.load(std::memory_order_acquire)) && _drainParked.load(std::memory_order_acquire);
                });
                continue;
            }
            if (_drainParked.load(std::memory_order_acquire)) {
                _drainParked.store(false, std::memory_order_release);
            }
            std::size_t       tail = 0;
            const std::size_t have = _ring.available(tail);
            if (have == 0UZ) {
                awaitBell([this] {
                    std::size_t at = 0;
                    return !_drain.stopRequested() && !_held.load(std::memory_order_acquire) && !_rateBreak.load(std::memory_order_acquire) && _ring.available(at) == 0UZ && !queueOutsizesRate();
                });
                continue;
            }
            const std::size_t pos  = tail & _ring.mask();
            const std::size_t take = std::min({have, _sendCapSamples.load(std::memory_order_acquire), _ring.buf.size() - pos});
            const bool        sent = sendChunk(stream, _ring.buf.data() + pos, take);
            _ring.advanceTo(tail + take);
            if (!sent) {
                _drainDead.store(true, std::memory_order_release);
                return;
            }
        }
    }

    // Wait on the drain's bell where idle() still holds once the bell is armed.
    template <typename Idle>
    void awaitBell(Idle&& idle) {
        const std::uint32_t token = _drainBell.arm();
        if (idle()) {
            _drainBell.wait(token);
        } else {
            _drainBell.disarm();
        }
    }

    /*| contract: place the n samples at samples on the streamer, opening a burst where none is
            open. False where the device stopped taking them, which is a send that timed out or
            threw. The burst flag is read once and written once, where the call opens a burst.
    */
    bool sendChunk(const ::uhd::tx_streamer::sptr& stream, const std::complex<float>* samples, std::size_t n) {
        std::lock_guard    send(_sendMutex);
        ::uhd::tx_metadata_t md;
        md.start_of_burst = !_bursting.load(std::memory_order_acquire);
        md.end_of_burst   = false;
        md.has_time_spec  = false;
        std::size_t done  = 0UZ;
        try {
            while (done < n) {
                const std::size_t sent = stream->send(samples + done, n - done, md, kSendTimeoutS);
                if (sent == 0UZ) {
                    std::fprintf(stderr, "[uhd::Sink] the send timed out after %.0f s; the device stopped taking samples\n", kSendTimeoutS);
                    return false;
                }
                done += sent;
                if (md.start_of_burst) {
                    md.start_of_burst = false;
                    _bursting.store(true, std::memory_order_release);
                }
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[uhd::Sink] send: %s\n", e.what());
            return false;
        }
        return true;
    }

    /*| contract: count one message of the device's asynchronous channel, one count per kind it
            carries: a gap, a lost packet, a late packet, a burst acknowledgment.
        frame: the device raises an underflow when the host stopped feeding it, an
            underflow-in-packet when a packet ran out mid-way, a sequence error in or out of a
            burst when a packet never arrived, a time error when a packet arrived after the time
            it carried, and a burst acknowledgment when the last sample of a burst left the
            radio.
        trap: the codes are bit flags and one message can carry several. Compared for equality,
            a message carrying two flags counts as neither.
        verified-by: sink.uhd-counts-transmit-events-by-bit
    */
    void countAsyncEvent(::uhd::async_metadata_t::event_code_t code) {
        using Md = ::uhd::async_metadata_t;
        if ((code & Md::EVENT_CODE_BURST_ACK) != 0) {
            _burstAcks.fetch_add(1, std::memory_order_acq_rel);
        }
        if ((code & (Md::EVENT_CODE_UNDERFLOW | Md::EVENT_CODE_UNDERFLOW_IN_PACKET)) != 0) {
            _counters.underrunEvents.fetch_add(1, std::memory_order_relaxed);
        }
        if ((code & (Md::EVENT_CODE_SEQ_ERROR | Md::EVENT_CODE_SEQ_ERROR_IN_BURST)) != 0) {
            _sequenceErrors.fetch_add(1, std::memory_order_relaxed);
        }
        if ((code & Md::EVENT_CODE_TIME_ERROR) != 0) {
            _latePackets.fetch_add(1, std::memory_order_relaxed);
        }
    }

    /*| contract: take one message off channel, waiting up to timeoutS, and count it. False where
            none arrived; a throw from the channel reaches the caller.
        verified-by: sink.uhd-counts-the-burst-acknowledgment
    */
    template <typename Channel>
    bool takeAsyncMessage(Channel& channel, double timeoutS) {
        ::uhd::async_metadata_t md;
        if (!channel.recv_async_msg(md, timeoutS)) {
            return false;
        }
        countAsyncEvent(md.event_code);
        return true;
    }

    /*| contract: count what the device's asynchronous channel carries until a stop is requested
            of this thread.
        trap: the poll has a timeout of its own so the thread notices a stop rather than parking
            in the device library until the next message, which on an idle transmitter never
            comes.
    */
    void asyncLoop() {
        const ::uhd::tx_streamer::sptr stream = streamerSnapshot();
        if (stream == nullptr) {
            return;
        }
        while (!_async.stopRequested()) {
            try {
                std::ignore = takeAsyncMessage(*stream, 0.1);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[uhd::Sink] async channel: %s (the transmit gap counts stop here)\n", e.what());
                return;
            }
        }
    }
};

} // namespace capture::uhd
