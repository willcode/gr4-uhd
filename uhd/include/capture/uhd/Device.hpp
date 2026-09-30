/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Device.hpp>
#include <capture/common/Env.hpp>

#include <gnuradio-4.0/LifeCycle.hpp>

#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <format>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <ranges>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

/*| role: what a USRP is, which units are attached, and the reads and writes both blocks make.
    why: the receive block and the transmit block probe the same record and offer a caller the
        same device list. Both live here so that neither family has to include the other and
        neither can drift from the other's spelling.
    frame: a body that differs between the two blocks only in whether it says rx or tx takes
        the direction as an argument and lives here once.
*/
namespace capture::uhd {

// The half of a USRP a body asks about.
enum class Direction { Receive, Transmit };

/*| role: probed device truth for a UHD device, with owning strings.
    why: a USRP's control surface is only knowable by asking the device, not from a static
        table: gain elements and their ranges, antennas, tuning ranges and analog bandwidth
        all differ per model and daughterboard. A caller probes once per
        device selection, while stopped and so holding exclusive access, and caches the result.
    frame: one record per direction. The block that filled it asked the device about its own
        direction, so a receive record and a transmit record of one unit differ in every field
        the daughterboard states separately. The measured figures below are the receive side's.
*/
struct DeviceTruth {
    struct GainElement {
        std::string name; // UHD gain element (e.g. B2xx "PGA"); doubles as the config key
        double      min = 0.0, max = 0.0, step = 1.0, current = 0.0;
    };
    std::vector<GainElement> gains;
    /*| frame: the overall gain range, which UHD distributes across the elements above. On a
            model with several elements it is their sum and not any one of them.
        measured: 0 to 76 dB step 1 on a B205mini, whose one element is that range; 0 to 38 dB
            step 0.5 on an N210 with a WBX, whose three elements are 0 to 31.5, 0 to 6 and 0
            to 0.5.
    */
    double                                 gainMinDb = 0.0, gainMaxDb = 0.0, gainStepDb = 1.0;
    std::vector<std::string>               antennas;
    std::string                            antenna; // current selection at probe time
    std::vector<std::pair<double, double>> freqRanges;
    /*| frame: the analog filter range and the width in force. Equal ends are a fixed filter,
            which takes no write: the width the device came up with is the width it keeps.
        measured: 200 kHz to 56 MHz on a B205mini; 40 MHz to 40 MHz on a WBX.
    */
    double bwMinHz = 0.0, bwMaxHz = 0.0, bwCurHz = 0.0;
    /*| frame: the rate sub-ranges the device states for the direction probed, in the order it
            gives them. A sub-range whose ends are equal is one reachable rate; a sub-range with
            room between them is a span.
        trap: on a radio whose master clock is programmable this states the ladder of the
            clock in force, and the ladder moves with the rate, so it is not the set of rates
            the radio can reach. mclkMinHz and mclkMaxHz say which kind of radio this is.
        measured: a B205mini states 256 single points, 31.25 kHz to 16 MHz at a 16 MHz master
            clock and 120 kHz to 61.44 MHz at a 61.44 MHz one.
    */
    std::vector<std::pair<double, double>> rates;
    /*| frame: the master clock range get_master_clock_rate_range states. Equal ends are a
            fixed clock, and under a fixed clock the stated rate set is the radio's own
            ladder rather than one setting's ladder.
        measured: 220 kHz to 61.44 MHz on a B205mini.
    */
    double                   mclkMinHz = 0.0, mclkMaxHz = 0.0;
    /*| frame: whether that range is one continuous span, a single sub-range whose start is
            below its stop, which lets the device run any clock between mclkMinHz and
            mclkMaxHz. Several sub-ranges, the separate clocks a radio offers, or one sub-range
            that is a single point leave it false.
        verified-by: controls.uhd-a-clock-span-takes-any-clock-in-it
    */
    bool                     mclkContinuous = false;
    /*| frame: the separate clocks the range states, in Hz and ascending, where every sub-range
            of it is a single point and there are two or more. Empty for a continuous span and
            for a single clock.
        frame: UHD 4.9's own radios state a single clock or a span. An X3xx, an N3xx or an
            X4xx states the one clock it was opened at, and productClocksMHz states the clocks
            such a radio takes at the next open.
        verified-by: controls.uhd-a-radio-stating-clocks-above-the-converter-picks
    */
    std::vector<double>      mclkPoints;
    double                   mclkHz = 0.0; // the master clock in force
    /*| frame: whether UHD chooses the master clock for the device as it is open now, read from
            the motherboard's own auto_tick_rate node, and empty where the property tree carries
            no such node. A B2xx creates it.
        why: UHD hands a second open in one process the device already open, with the first
            open's keys, so the address one block opened with does not say whether the clock is
            fixed; the node does.
    */
    std::optional<bool>      autoTickRate;
    std::vector<std::string> clockSources;
    std::string              clockSource; // current selection at probe time
    std::vector<std::string> timeSources;
    std::string              timeSource;
    std::string              product; // e.g. "B210"
    /*| frame: the name UHD gives the frontend of the channel read, the information map's
            rx_subdev_name or tx_subdev_name, "TwinRX RX0" on a TwinRX. Empty where the map
            states none.
    */
    std::string              frontend;
    /*| frame: the device kind UHD names at the root of its property tree: "B-Series Device"
            for a B2xx or a B100, "USRP2 / N-Series Device" for an N2xx, "X-Series Device" for an
            X3xx. Empty where the tree names none.
    */
    std::string              deviceKind;
    /*| frame: what the unit says it is. model is the motherboard name, serial its own, and
            the two versions are the firmware and the FPGA image it is running.
        measured: "B205mini" / "30D3AB9" on a B2xx; "N210r4" / "E2R18S9UP" with firmware 12.4
            and FPGA 11.1 on an N-series.
    */
    std::string model;
    std::string serial;
    std::string fwVersion;
    std::string fpgaVersion;
    /*| frame: which of the frontend's optional facilities this unit carries, read from the
            property tree rather than assumed. A model without one takes the write and ignores
            it, logging a warning, so a control offered for it would do nothing.
        measured: a B205mini carries all three; an N210 with a WBX carries the DC offset
            correction alone.
    */
    bool hasAgc           = false;
    bool hasDcOffsetCorr  = false;
    bool hasIqBalanceCorr = false;
    /*| frame: the bytes per second UHD states for the link between the radio and the host,
            or zero where UHD states none. One link carries both directions, so both records
            state it. It cuts the top off a fixed-clock radio's ladder at the rate UHD's own
            check allows, and a zero cuts nothing: the ladder then reaches the radio's own
            fastest rate.
        measured: a B205mini over USB 3 states 500 MB/s and an N210 125 MB/s.
    */
    double linkBytesPerS = 0.0;

    /*| contract: the fastest rate the radio states for itself: the top of its master-clock
            range, or of its stated rate set where that is higher. Zero where it states neither.
    */
    double topRateHz() const {
        double top = mclkMaxHz;
        for (const auto& r : rates) {
            top = std::max(top, r.second);
        }
        return top;
    }

    /*| contract: one line naming the unit: the model, the serial, the firmware and FPGA
            versions where it states them, and the link. The link part states the rate UHD
            gives for it, or says the link rate is unknown and names the ceiling the ladder
            takes instead, the radio's own fastest rate. Empty where nothing was probed.
        verified-by: uhd.hardware-information
        verified-by: controls.uhd-describes-the-unit
        verified-by: controls.uhd-an-mpm-device-is-cut-at-its-own-rate
    */
    std::string describe() const {
        if (model.empty() && serial.empty() && product.empty()) {
            return {};
        }
        std::string out = model.empty() ? product : model;
        if (!serial.empty()) {
            out += " " + serial;
        }
        if (!fwVersion.empty()) {
            out += std::format(", firmware {}", fwVersion);
        }
        if (!fpgaVersion.empty()) {
            out += std::format(", FPGA {}", fpgaVersion);
        }
        if (linkBytesPerS > 0.0) {
            out += std::format(", link {:g} MB/s as UHD states it", linkBytesPerS / 1e6);
        } else {
            out += topRateHz() > 0.0 ? std::format(", link rate unknown, ceiling {:g} MS/s from the radio", topRateHz() / 1e6) : std::string(", link rate unknown");
        }
        return out;
    }
};

/*| frame: what a running block's control surface is computed from: the record of the open
        device, the rate in force and the width the analog filter holds. A block publishes
        these through a Snapshot, and its controlSurface() reads them from there.
*/
struct SurfaceValues {
    DeviceTruth truth;
    double      rateHz     = 0.0;
    double      filterHz   = 0.0;
    double      clockHz    = 0.0; // the master clock in force, zero where UHD chooses
    std::string wireFormat = "sc16";
};

/*| contract: a value may be put into a device string only where it carries neither the
        separator between two pairs nor the one between a key and a value.
    why: a selector is assembled by joining key=value pairs with commas, and a value carrying
        either character parses back as something else: the reader of the string finds a key
        where the value continues, and the selector then names another radio or none.
    verified-by: sink.uhd-listing-pairs-are-the-sources
*/
inline bool deviceStringSafe(std::string_view value) { return value.find(',') == std::string_view::npos && value.find('=') == std::string_view::npos; }

/*| role: one entry of a USRP listing: what a caller sees, what it selects the unit with, and
        the identity two transports answering for one unit share.
    contract: unspellableKey names the address key whose value cannot go into a device string,
        and is empty for an entry a listing keeps.
    trap: the identity is built from every address key an entry carries, rather than from the
        serial and the model. Two units of one model that state no serial share that pair, and
        keying on it lists one of them and drops the other with nothing said.
    verified-by: sink.uhd-listing-pairs-are-the-sources
*/
struct ListingEntry {
    std::string label;
    std::string deviceString;
    std::string identity;
    std::string unspellableKey;
};

inline ListingEntry listingEntryFor(const ::uhd::device_addr_t& addr) {
    ListingEntry   entry;
    const std::string serial  = addr.get("serial", "");
    const std::string product = addr.get("product", addr.get("type", "USRP"));
    entry.deviceString        = "driver=uhd";
    for (const char* key : {"type", "serial", "addr", "name", "resource"}) {
        if (!addr.has_key(key) || addr[key].empty()) {
            continue;
        }
        if (!deviceStringSafe(addr[key])) {
            /*| invariant: an entry the listing drops carries neither an identity nor a
                    selector. Half a spelling would shadow the next unit whose keys begin the
                    same way. */
            entry.unspellableKey = key;
            entry.identity.clear();
            entry.deviceString.clear();
            return entry;
        }
        entry.deviceString += std::format(",{}={}", key, addr[key]);
        entry.identity += std::format("{}={},", key, addr[key]);
    }
    entry.label = serial.empty() ? std::format("USRP {}", product) : std::format("USRP {} {}", product, serial);
    return entry;
}

// The environment switch that adds use_dpdk=1 to the hint a device listing asks libuhd with.
inline constexpr const char* kUseDpdkVariable = "CAPTURE_UHD_USE_DPDK";

/*| contract: the hint a device listing asks libuhd with: use_dpdk=1 where the caller asks for
        DPDK or the switch kUseDpdkVariable is on in the environment, and the empty hint
        otherwise.
    why: UHD 4.9.0's search for MPM devices and for an X3xx over Ethernet starts DPDK, and so
        reaches a radio whose links run through it, only for a hint that carries use_dpdk. No
        search filters a unit on the key, so the hint that carries it lists every unit the empty
        hint lists as well.
    verified-by: controls.uhd-a-listing-asks-for-dpdk-where-asked
*/
inline ::uhd::device_addr_t listingHint(bool callerAsksForDpdk = false) {
    ::uhd::device_addr_t hint;
    if (callerAsksForDpdk || capture::envFlag(kUseDpdkVariable)) {
        hint["use_dpdk"] = "1";
    }
    return hint;
}

// (label, device string) pairs for a caller's device list, enumerated through libuhd with the
// hint listingHint composes. One entry per unit, even where several transports answer for it.
inline std::vector<std::pair<std::string, std::string>> deviceList(bool callerAsksForDpdk = false) {
    std::vector<std::pair<std::string, std::string>> out;
    try {
        std::set<std::string> seen;
        for (const ::uhd::device_addr_t& addr : ::uhd::device::find(listingHint(callerAsksForDpdk))) {
            const ListingEntry entry = listingEntryFor(addr);
            if (!entry.unspellableKey.empty()) {
                std::fprintf(stderr, "[uhd] a USRP whose %s is \"%s\" is left out of the listing: a device string cannot spell a value carrying a comma or an equals sign\n",
                             entry.unspellableKey.c_str(), addr[entry.unspellableKey].c_str());
                continue;
            }
            if (!seen.insert(entry.identity).second) {
                continue;
            }
            out.emplace_back(entry.label, entry.deviceString);
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[uhd] enumerate: %s\n", e.what());
    }
    return out;
}

/*| contract: whether a value has to reach the device. A value never written does; a value
        already in force does not.
    why: the staged settings map is replayed shortly after the stream starts, so every applier
        runs at least twice with unchanged values, and a second rate or frequency write disturbs
        a running radio for nothing.
    frame: the cache each applier keeps is per open device and start() empties it, because a
        fresh multi_usrp holds none of the previous device's state.
    verified-by: controls.uhd-skips-an-unchanged-write
*/
template <typename T>
bool writeNeeded(const std::optional<T>& lastWritten, const T& value) {
    return !lastWritten.has_value() || !(*lastWritten == value);
}

// The wire formats these blocks build a streamer for, in descriptor-option order.
inline const std::vector<std::string>& wireFormats() {
    static const std::vector<std::string> formats{"sc16", "sc8"};
    return formats;
}

// The wire format named, where a streamer is built for it, and sc16 otherwise.
inline std::string validWireFormat(const std::string& wanted) { return std::ranges::find(wireFormats(), wanted) == wireFormats().end() ? std::string("sc16") : wanted; }

/*| contract: the center frequency to state for a tuner that reads heldHz, under the correction
        in force. It is the read-back carried back into the frame the caller asks in, so the two
        agree where no correction is set, and it is the inverse of capture::correctedTuneHz.
    why: UHD coerces a tune to the nearest frequency its synthesizer reaches and says so through
        the read-back, which is the number a caller's frequency axis has to carry. Forwarding
        that read-back unchanged would put the scaled value into the frequency setting and the
        next write of it would scale it a second time: 100 kHz at 1 GHz and 100 ppm.
    verified-by: controls.uhd-forwards-the-frequency-the-tuner-holds
*/
inline double tunedFrequencyFrom(double heldHz, double ppm) { return heldHz * capture::referenceScale(ppm); }

/*| contract: the frequency to hand the tuner for a request, when the coverage the device
        states holds it, and nothing when no range does. The correction is folded in first,
        because the scaled value is the one the tuner has to reach. loOffsetHz moves the RF
        frontend away from the center, and zero moves it nowhere.
    why: refused rather than clamped. UHD coerces a request past an edge to the nearest
        frequency it can reach and reports success, so a 40 MHz request on a radio whose
        coverage starts at 49 MHz tunes 49 MHz, and every frequency axis downstream is out by
        nine megahertz with nothing to say so. A burst goes out on a frequency the caller never
        asked for in the same way.
    frame: a USRP's coverage moves with the analog filter, UHD widening the tuner's range by
        half the filter at each end within the digital converter's reach, and a block's filter
        follows its rate, so the ranges given here are the ones read at the rate and the filter
        in force.
    trap: the vendor header states the opposite — that the stated range "does not include
        the baseband bandwidth" and a caller should assume half the rate more at each end. On
        a model that behaves that way, this refusal costs up to half a rate of band at each
        edge.
    measured: both radios here follow the range they state. An N210 at 6.25 MS/s states a
        bottom of 48.75 MHz, and a direct open asked for 47.75 MHz lands back on 48.75 MHz; a
        B205mini at 8 MS/s states 34 MHz and does the same. On a model that does reach past
        its stated range, the range to test against is the stated one widened by half the
        rate.
    measured: on a B205mini at 2 MS/s the stated coverage is 49 to 6001 MHz; the same radio
        states 42 to 6008 MHz at 16 MS/s.
    frame: the correction is exact for the center and for the center alone. The same
        reference sets the converter's clock, so a stream stated at a nominal rate runs at that
        rate times one plus the fraction. A receive caller reading its axis from the nominal
        rate places a component of true offset b at b divided by one plus the fraction, short
        by b times the fraction: 100 Hz at an offset of 1 MHz and 100 ppm, where the center
        itself is on the request. A component a transmit caller placed at b leaves at b times
        one plus the fraction. Neither block scales the rate it states: a consumer's frequency
        axis is its own.
    verified-by: controls.uhd-frequency-out-of-range-refused
*/
inline std::optional<double> tuneFrequencyFor(double requestHz, double ppm, std::span<const std::pair<double, double>> ranges, double loOffsetHz = 0.0) {
    const auto center = frequencyInRange(capture::correctedTuneHz(requestHz, ppm), ranges);
    if (!center.has_value() || loOffsetHz == 0.0) {
        return center;
    }
    /*| trap: an LO offset moves the RF frontend to the center plus the offset and leaves the
            rest to the digital mixer, so the synthesizer has to reach the frontend frequency
            rather than the center. UHD coerces a frontend frequency past the end of the stated
            coverage without refusing it, which is the silent clamp this whole path exists to
            refuse.
    */
    return frequencyInRange(*center + loOffsetHz, ranges).has_value() ? center : std::nullopt;
}

/*| contract: the device address to open with: the caller's own selector, plus the master clock
        rate and the three transport sizes where they are above zero. A key the caller already
        spelled is left alone, because a device string a person typed says what they meant.
    frame: UHD reads all four when the device is opened and never again, so they are address
        keys rather than controls. The frame size is the bytes one transport frame holds, the
        frame count how many of them the ring has, and the buffer size the bytes the socket
        buffer holds; zero leaves each to UHD. The direction picks the recv or the send
        spelling of the three keys.
    frame: each block has seven settings its start reads and no applier writes: the wire
        format, the stream argument; the master clock rate and the three transport sizes, the
        address keys written here; and the reference and the timing selection, which
        applyReferenceSources writes before anything else is configured. A change to any of
        them reaches the radio at the next start.
    measured: an N210's default socket buffer absorbs a 300 ms stall at 7.7 MS/s without losing
        a sample; the same stall with recv_buff_size at 2 MB loses them.
    verified-by: controls.uhd-transport-sizing
*/
inline std::string withDeviceArgs(std::string kwargs, double masterClockHz, Direction dir, double frameSize, double numFrames, double buffSize) {
    const char* const frameKey  = dir == Direction::Receive ? "recv_frame_size" : "send_frame_size";
    const char* const countKey  = dir == Direction::Receive ? "num_recv_frames" : "num_send_frames";
    const char* const bufferKey = dir == Direction::Receive ? "recv_buff_size" : "send_buff_size";
    for (const auto& [key, value] : {std::pair{"master_clock_rate", masterClockHz}, std::pair{frameKey, frameSize}, std::pair{countKey, numFrames}, std::pair{bufferKey, buffSize}}) {
        if (value <= 0.0 || !kwargsValue(kwargs, key).empty()) {
            continue;
        }
        if (!kwargs.empty()) {
            kwargs += ",";
        }
        kwargs += std::format("{}={:.0f}", key, value);
    }
    return kwargs;
}

// The master clock figures a device states: the range it offers and the rate it is running.
struct MasterClock {
    double minHz = 0.0;
    double maxHz = 0.0;
    double hz    = 0.0;
};

/*| contract: those figures from the range read and the rate read taken apart. A device that
        states no range still states the clock it is running, and one that states a range but
        refuses the rate still states the range.
    why: the two are separate device calls and either raises on a model that does not carry it,
        so neither may lose the other. Taken under one try, a refused range left the rate at
        zero, and the rate names the clock a stated ladder was read at.
    verified-by: controls.uhd-a-refused-clock-range-keeps-the-clock
*/
constexpr MasterClock masterClockFrom(const std::optional<std::pair<double, double>>& range, const std::optional<double>& rateHz) {
    return {.minHz = range.has_value() ? range->first : 0.0, .maxHz = range.has_value() ? range->second : 0.0, .hz = rateHz.value_or(0.0)};
}

// ---- the rate ladder ---------------------------------------------------------------------

// The picks offered on a radio whose master clock is programmable, every one of which UHD
// reaches by choosing a clock for it.
inline std::vector<double> programmableClockRates() { return {250e3, 500e3, 1000e3, 2000e3, 4000e3, 8000e3, 16000e3, 30720e3, 56000e3, 61440e3}; }

/*| role: discrete rate picks per USRP kind, taken from the type= key of the device string.
    why: static truth rather than a probe: probing from a caller's own thread would freeze it
        on a replugged USRP while firmware loads, and the answer is a property of the model
        rather than of the unit.
    frame: an N2xx, type=usrp2, has one 100 MHz master clock, an integer decimator on receive
        and an integer interpolator on transmit, so every reachable rate is 100 MHz / N. An even
        N turns a halfband on and an N divisible by four turns both on; an odd N runs the CIC
        alone and droops across the passband, in either direction. The ladder is therefore
        restricted to even divisors. The ceiling is the link and not the FPGA: UHD states
        125 MB/s for the kind's Gigabit Ethernet link, which carries 31.25 MS/s at the four
        bytes an sc16 sample takes, and the highest even divisor under that is 25 MS/s, while
        the decimator itself reaches 50 MS/s. Hence 250 k to 25 M, every entry exact and
        halfband-filtered.
    trap: every other kind gets the picks of a programmable master clock, the B2xx's and the
        E3xx's, whose converter tops out at 61.44 MHz. UHD snaps a pick to the nearest rate the
        radio reaches. Only a probe states the rates of an X3xx, an N3xx or an X4xx.
    why: this is the answer for a caller holding no probe. Where a probe exists,
        sampleRatesFor() states the ladder from what the device itself said.
*/
inline std::vector<double> sampleRates(const std::string& kwargs) {
    if (kwargsValue(kwargs, "type") == "usrp2") {
        return {250e3, 500e3, 1000e3, 2000e3, 2500e3, 5000e3, 6250e3, 10000e3, 12500e3, 25000e3};
    }
    return programmableClockRates();
}

// The deepest decimation a USRP's digital down-converter runs. It sets the lowest rate a
// given master clock reaches.
inline constexpr int kMaxDecimation = 256;

// Bytes one I/Q sample takes on the wire in the format named. sc8 is the only format narrower
// than the sc16 default, and it carries eight bits per part instead of sixteen.
constexpr std::size_t wireBytesPerSample(std::string_view wireFormat) { return wireFormat == "sc8" ? 2UZ : 4UZ; }

/*| contract: the fastest rate per channel a link of linkBytesPerS carries in the wire format
        named: the bytes per second over the bytes one sample takes, with nothing taken off.
    why: UHD compares the same two figures. Where a motherboard's property tree holds
        link_max_rate as a double, _check_link_rate in UHD 4.9.0.1's host/lib/usrp/multi_usrp.cpp
        warns when a streamer is made whose channels' summed rate passes link_max_rate over the
        wire bytes per sample. Each block streams one channel, so the sum is its own rate.
    verified-by: controls.uhd-the-link-ceiling-is-the-rate-uhd-states
*/
inline double linkCeilingHz(std::string_view wireFormat, double linkBytesPerS) { return linkBytesPerS / static_cast<double>(wireBytesPerSample(wireFormat)); }

/*| contract: the receive frames UHD 4.9 opens a data transport with where the open address
        states no num_recv_frames, for the device kind the root of the property tree names: 16
        for a B2xx or a B100 over USB, 32 for an N2xx over UDP, and zero for every other kind.
    frame: b200_impl.cpp and b100_impl.cpp take 16 where the address states none, and
        usrp2_impl.cpp's DEFAULT_NUM_FRAMES is 32. An X3xx and an MPM device size their frames
        from the link and the socket buffer, which this function does not state.
    verified-by: controls.uhd-the-receive-depth-is-stated-in-time
*/
inline std::size_t uhdDefaultRecvFrames(std::string_view deviceKind) {
    if (deviceKind == "B-Series Device") {
        return 16UZ;
    }
    return deviceKind == "USRP2 / N-Series Device" ? 32UZ : 0UZ;
}

/*| contract: the receive socket buffer UHD 4.9 asks for where the open address states no
        recv_buff_size: for an N2xx, 50e6 bytes on Linux and Windows and 1e6 on macOS and BSD,
        as usrp2_impl.cpp sets it. Zero for every other kind.
    trap: the system may grant less than a socket buffer asks for, and UHD then warns at the
        open. The figure is the request.
*/
inline double uhdDefaultRecvBuffBytes(std::string_view deviceKind) {
    if (deviceKind != "USRP2 / N-Series Device") {
        return 0.0;
    }
#if defined(__APPLE__) || defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
    return 1e6;
#else
    return 50e6;
#endif
}

/*| role: the depth of the receive transport in use, in frames and in time at a rate.
    contract: frames is the caller's num_recv_frames, or UHD's default for the kind where
        uhdDefaultRecvFrames states one, or zero. bufferBytes is the caller's recv_buff_size or
        UHD's default for the kind, or zero. The times are the depth over the rate: frames of
        frameSamples each, and the socket buffer's bytes over the wire bytes a sample takes.
    frame: UHD's default holds 16 x 2040 samples on a B2xx at its default frame size in sc16,
        32640 samples: 0.53 ms at 61.44 MS/s and 16 ms at 2 MS/s.
    verified-by: controls.uhd-the-receive-depth-is-stated-in-time
*/
struct RecvDepth {
    std::size_t frames        = 0UZ;
    bool        framesDefault = false;
    std::size_t frameSamples  = 0UZ;
    double      bufferBytes   = 0.0;
    bool        bufferDefault = false;
    double      rateHz        = 0.0;
    std::size_t wireBytes     = 4UZ;

    double framesSeconds() const { return rateHz > 0.0 ? static_cast<double>(frames * frameSamples) / rateHz : 0.0; }
    double bufferSeconds() const { return rateHz > 0.0 ? bufferBytes / (rateHz * static_cast<double>(wireBytes)) : 0.0; }

    /*| contract: the reading's text: the frames, their samples and the time they hold at the
            rate, then the socket buffer where one is known. Empty where no frame size is known.
    */
    std::string describe() const {
        if (frameSamples == 0UZ) {
            return {};
        }
        const std::string at   = rateHz > 0.0 ? std::format(" at {:g} MS/s", rateHz / 1e6) : std::string();
        const auto        time = [](double seconds) { return seconds < 1.0 ? std::format("{:.3g} ms", seconds * 1e3) : std::format("{:.3g} s", seconds); };
        std::string       out = frames > 0UZ ? std::format("{} frames of {} samples", frames, frameSamples) : std::format("frames of {} samples, as many as UHD chose", frameSamples);
        if (frames > 0UZ && rateHz > 0.0) {
            out += std::format(", {}{}", time(framesSeconds()), at);
        }
        if (frames > 0UZ) {
            out += framesDefault ? ", UHD's default" : ", as set";
        }
        if (bufferBytes > 0.0) {
            out += std::format("; socket buffer {:.0f} bytes", bufferBytes);
            if (rateHz > 0.0) {
                out += std::format(", {}{}", time(bufferSeconds()), at);
            }
            out += bufferDefault ? ", UHD's request" : ", as set";
        }
        return out;
    }
};

// The depth from the two settings as the caller gave them, zero or less leaving each to UHD.
inline RecvDepth recvDepthFor(double askedFrames, double askedBuffBytes, std::string_view deviceKind, std::size_t frameSamples, double rateHz, std::string_view wireFormat) {
    RecvDepth out;
    out.frameSamples = frameSamples;
    out.rateHz       = rateHz > 0.0 ? rateHz : 0.0;
    out.wireBytes    = wireBytesPerSample(wireFormat);
    if (askedFrames >= 1.0 && std::isfinite(askedFrames)) {
        out.frames = static_cast<std::size_t>(askedFrames);
    } else {
        out.frames        = uhdDefaultRecvFrames(deviceKind);
        out.framesDefault = out.frames > 0UZ;
    }
    if (askedBuffBytes >= 1.0 && std::isfinite(askedBuffBytes)) {
        out.bufferBytes = askedBuffBytes;
    } else {
        out.bufferBytes   = uhdDefaultRecvBuffBytes(deviceKind);
        out.bufferDefault = out.bufferBytes > 0.0;
    }
    return out;
}

// One key of an open address as a number, zero where it is absent or not a number.
inline double addressNumber(const ::uhd::device_addr_t& address, const std::string& key) {
    if (!address.has_key(key)) {
        return 0.0;
    }
    try {
        return address.cast<double>(key, 0.0);
    } catch (const std::exception&) {
        return 0.0;
    }
}

/*| contract: the bytes per second UHD states for the link between the device and the host:
        the double on /mboards/0/link_max_rate, and zero where the node is missing or holds
        another type. The selector and the transport decide nothing here.
    frame: UHD 4.9.0.1 creates the node as a double for four kinds, each from its driver's own
        constant: 125 MB/s for a USRP2 or N2xx, 500 MB/s for a B2xx on USB 3 and 53.248 MB/s
        on USB 2, and 32 MB/s for a B100 or a USRP1. An X3xx creates no node. A device an MPM
        daemon runs (E3xx, N3xx, X4xx) creates it as a size_t holding 125000000 whatever its
        link, and no check of UHD's reads it. UHD's check reads the node as a double, and so
        does this read, which sets that node aside.
    measured: a B205mini over USB 3 states 500000000 and an N210 125000000.
    verified-by: uhd.property-tree-link-rate
    verified-by: controls.uhd-the-link-ceiling-is-the-rate-uhd-states
*/
template <typename Usrp>
double statedLinkBytesPerS(Usrp& usrp) {
    try {
        auto tree = usrp.get_device()->get_tree();
        if (tree->exists("/mboards/0/link_max_rate")) {
            return tree->template access<double>("/mboards/0/link_max_rate").get();
        }
    } catch (const std::exception&) { // no tree, or a node of another type
    }
    return 0.0;
}

/*| contract: the entry of ladder nearest requestHz, or the request itself where the ladder is
        empty.
    why: a fixed-clock radio reaches only its own divisors, and UHD lands a request between two
        of them on whichever it likes — 8 MS/s on a 100 MHz clock becomes 7.692 MS/s, decimation
        13, which is odd and runs the CIC alone. Snapping first puts the radio on a rate the
        block chose and a caller can be told about.
    verified-by: controls.uhd-rate-ladder-from-the-device
*/
inline double snapToLadder(double requestHz, std::span<const double> ladder) {
    if (ladder.empty()) {
        return requestHz;
    }
    double best = ladder.front();
    for (const double r : ladder) {
        if (std::abs(r - requestHz) < std::abs(best - requestHz)) {
            best = r;
        }
    }
    return best;
}

/*| contract: whether the radio offers exactly one master-clock rate, which makes it a
        fixed-clock radio: a stated clock range whose ends meet above zero. A range with room
        between its ends, continuous or several discrete rates, is a variable clock, and UHD
        chooses a clock for each request.
    frame: UHD 4.9.0 states the master-clock range of the first radio on the motherboard. An
        X4xx states the one rate it was opened at and keeps that rate while it runs; a B2xx and
        an E3xx state their converter's whole range.
    verified-by: controls.uhd-several-clock-rates-are-a-variable-clock
*/
inline bool offersOneClockRate(const DeviceTruth& t) { return t.mclkMinHz > 0.0 && t.mclkMaxHz <= t.mclkMinHz; }

/*| contract: every rate a fixed-clock radio reaches in the wire format named: the single points
        of its stated set whose decimation of the clock is even, cut at what the link carries
        where a link rate is known. Ascending, and empty for a radio whose clock is programmable
        or whose stated set holds no single point.
    why: an odd decimation runs the CIC alone, the halfband filter staying out of the chain, and
        the passband droops across its own width. An odd interpolation does the same on
        transmit: the transmit DSP turns both halfbands off for one and runs the CIC alone. UHD
        says as much on a programmable clock too, where it picks the clock to keep the division
        even.
    trap: a fixed-clock radio's decimator also states rates its link cannot carry, so the stated
        set is cut at the link ceiling for the wire format asked for. The format moves the
        ceiling: sc8 is half the bytes and so twice the rate.
    frame: a rate a caller states outright is snapped to the nearest entry here, while
        sampleRatesFor states the shorter list a caller chooses from.
    measured: an N210 states 255 sub-ranges up to 50 MS/s against a fixed 100 MHz clock, of
        which sc16 carries to 25 MS/s and sc8 to 50 MS/s.
    verified-by: controls.uhd-rate-ladder-from-the-device
*/
inline std::vector<double> reachableRatesFor(const DeviceTruth& t, std::string_view wireFormat = "sc16") {
    std::vector<double> out;
    if (!offersOneClockRate(t)) {
        return out;
    }
    const double ceiling = t.linkBytesPerS > 0.0 ? linkCeilingHz(wireFormat, t.linkBytesPerS) : std::numeric_limits<double>::max();
    for (const auto& [lo, hi] : t.rates) {
        if (lo != hi || lo <= 0.0 || lo > ceiling) {
            continue;
        }
        const double decimation = t.mclkMinHz / lo;
        const double whole      = std::round(decimation);
        if (std::abs(decimation - whole) < 1e-6 && (whole == 1.0 || std::fmod(whole, 2.0) == 0.0)) {
            out.push_back(lo);
        }
    }
    std::ranges::sort(out);
    return out;
}

/*| contract: the truth the rate decisions read: what the device said, with a converter clock an
        explicit write put on it standing in for the range it stated.
    why: a clock written by hand does not move again while the device stays open — UHD's own
        choice is cleared inside the driver and only a fresh open sets it — so the radio is a
        fixed-clock radio until then and its reachable rates are that clock's even decimations.
        Left on the stated range, the block takes the programmable branch, snaps nothing, and
        UHD reaches the request by whatever decimation it likes: 2 MS/s on a pinned 30.72 MHz
        clock lands on 2.048 MS/s, decimation 15, which runs the CIC alone and droops across
        its own passband.
    verified-by: controls.uhd-a-pinned-clock-is-a-fixed-clock
*/
inline DeviceTruth ladderTruthFor(DeviceTruth truth, double clockInForceHz) {
    if (clockInForceHz > 0.0) {
        truth.mclkMinHz      = clockInForceHz;
        truth.mclkMaxHz      = clockInForceHz;
        truth.mclkHz         = clockInForceHz;
        truth.mclkContinuous = false;
        truth.mclkPoints.clear();
    }
    return truth;
}

/*| contract: the rates to offer a caller for the radio a probe described, ascending. Empty only
        where the probe described nothing.
    frame: where the master clock is programmable the rate set the device states is the ladder
        of the clock in force and moves with the rate, so the picks above are the ladder and the
        clock range bounds them: the top is the fastest clock, offered itself where no pick
        reaches it, and the bottom is the slowest clock over the deepest decimation. A link
        ceiling UHD states in the wire format named lowers the top to that ceiling, offered
        itself in the same way: 13.312 MS/s in sc16 for a B2xx on USB 2.
    trap: the picks stop at 61.44 MS/s, the top of a B2xx's or an E3xx's converter. A radio
        whose clock runs faster reaches its fastest clock at decimation 1, and an offer cut at
        the last pick would hide every rate above 61.44 MS/s. The top and every separate clock
        the radio states up to it are offered as well.
    frame: where the master clock is fixed the radio reaches every even decimation of it, which
        on a 100 MHz clock is 191 entries between 195 kS/s and 25 MS/s, and the ones below about
        1 MS/s sit less than a percent apart. A list drawn over all of them cannot be aimed by
        hand, so what is offered is the static picks for the kind that lie on that ladder, plus
        the ladder's own top. A rate a caller states outright is still snapped to the nearest
        entry of the whole ladder.
    measured: a B205mini states 256 single points, 31.25 kHz to 16 MHz at a 16 MHz master clock
        and 120 kHz to 61.44 MHz at a 61.44 MHz one, against a master clock range of 220 kHz to
        61.44 MHz. Taking the stated set would cap that radio at whichever clock the probe
        happened to catch.
    verified-by: controls.uhd-rate-ladder-from-the-device
*/
inline std::vector<double> sampleRatesFor(const DeviceTruth& t, std::string_view wireFormat = "sc16") {
    std::vector<double> out;
    if (t.mclkMaxHz > t.mclkMinHz) {
        const double bottom = t.mclkMinHz / static_cast<double>(kMaxDecimation);
        const double top    = t.linkBytesPerS > 0.0 ? std::min(t.mclkMaxHz, linkCeilingHz(wireFormat, t.linkBytesPerS)) : t.mclkMaxHz;
        for (const double r : programmableClockRates()) {
            if (r >= bottom && r <= top) {
                out.push_back(r);
            }
        }
        // Each clock a radio states separately is a rate it reaches at decimation one.
        for (const double clock : t.mclkPoints) {
            if (clock <= top + 1.0 && std::ranges::none_of(out, [clock](double r) { return std::abs(r - clock) <= 1.0; })) {
                out.push_back(clock);
            }
        }
        std::ranges::sort(out);
        // Past the last pick by more than a hertz: a stated end carries the device's rounding.
        if (out.empty() || out.back() < top - 1.0) {
            out.push_back(top);
        }
        return out;
    }
    const auto reachable = reachableRatesFor(t, wireFormat);
    if (reachable.empty()) {
        return programmableClockRates();
    }
    for (const double pick : sampleRates("type=usrp2")) {
        // The device's own entry rather than the pick, and matched within a hertz: a stated
        // rate is a divisor of the clock and carries the device's rounding.
        if (const auto it = std::ranges::find_if(reachable, [pick](double r) { return std::abs(r - pick) <= 1.0; }); it != reachable.end()) {
            out.push_back(*it);
        }
    }
    /*| frame: above the last pick, the entries at a decimation of a power of two, up to the
            top of the ladder: 50, 100 and 200 MS/s on a 200 MHz clock that UHD states no link
            for.
    */
    const double lastPick = out.empty() ? 0.0 : out.back();
    for (const double r : reachable) {
        const double decimation = std::round(t.mclkMinHz / r);
        if (r > lastPick + 1.0 && decimation >= 1.0 && std::exp2(std::round(std::log2(decimation))) == decimation) {
            out.push_back(r);
        }
    }
    if (out.empty() || out.back() < reachable.back()) {
        out.push_back(reachable.back());
    }
    return out;
}

/*| contract: the SAMPLE_RATE and FREQUENCY descriptors for the radio a truth describes, at
        the master clock in force, zero where UHD chooses, and in the wire format named, empty
        for sc16. The ladder is the picks of sampleRatesFor. A device with a programmable clock
        takes every rate from the slowest clock over the deepest decimation to the fastest
        clock or the link ceiling, and the list is anyInRange over that span. The
        coverage is the tuner's in force. A truth that states no coverage adds no FREQUENCY.
    verified-by: controls.uhd-states-its-rate-and-frequency-from-the-truth
*/
inline void appendLadderAndCoverage(std::vector<ControlDesc>& out, const DeviceTruth& truth, double clockInForceHz, std::string_view wireFormat, double defaultRateHz, double defaultFrequencyHz) {
    const DeviceTruth                         t      = ladderTruthFor(truth, clockInForceHz);
    const std::string_view                    format = wireFormat.empty() ? std::string_view("sc16") : wireFormat;
    std::optional<std::pair<double, double>> span;
    if (t.mclkMaxHz > t.mclkMinHz) {
        const double top = t.linkBytesPerS > 0.0 ? std::min(t.mclkMaxHz, linkCeilingHz(format, t.linkBytesPerS)) : t.mclkMaxHz;
        span             = std::pair{t.mclkMinHz / static_cast<double>(kMaxDecimation), top};
    }
    appendRateAndFrequency(out, sampleRatesFor(t, format), defaultRateHz, span, t.freqRanges, defaultFrequencyHz);
}

/*| contract: the master clock the ladder is cut from once a device is open: the clock the device
        read at the open where the clock is fixed, and zero, which leaves the clock to UHD, where
        it is not. The device's own auto_tick_rate node says which where the truth carries it;
        for a device without the node, the address it was opened with says, a master_clock_rate
        key fixing the clock.
    why: the key fixes a B2xx's clock for as long as the device stays open. UHD 4.9's B200 turns
        its automatic clock off wherever the open address carries the key, and the device still
        states its whole clock range, so a ladder read from that range treats the clock as
        programmable and hands the device the request as it stands. UHD then divides the fixed
        clock by the nearest whole number: 2 MS/s at 30.72 MHz is a factor of 15, odd, on the CIC
        alone, in either direction. The key reaches the address from the setting, from a
        control's pin or from the device string a caller typed.
    trap: the address alone is wrong for a second open in one process. UHD hands that open the
        device already open with the first open's keys, so a sink opened with the key beside a
        source opened without it runs on UHD's automatic clock, and a sink opened without it
        beside a source that fixed the clock runs on a fixed one.
    verified-by: sink.uhd-an-address-that-pins-the-clock-fixes-the-ladder
    verified-by: sink.uhd-the-device-says-whether-its-clock-is-fixed
*/
inline double clockInForceAtOpen(const ::uhd::device_addr_t& openAddress, const DeviceTruth& truth) {
    if (truth.autoTickRate.has_value()) {
        return *truth.autoTickRate ? 0.0 : truth.mclkHz;
    }
    return openAddress.has_key("master_clock_rate") ? truth.mclkHz : 0.0;
}

/*| contract: the rate to hand the device for a request. On a radio whose clock is programmable
        it is the request, UHD choosing a clock that reaches it, and the link ceiling UHD states
        in the wire format named where the request passes it; on a fixed clock, or one a write
        pinned (clockInForceHz above zero), it is the nearest entry of the ladder that clock and
        the link allow in the wire format named. A snap is named on standard error under
        blockTag.
    why: a fixed-clock radio reaches only its own divisors, and a request between two of them
        lands on whichever UHD likes, which can be an odd decimation running the CIC alone.
        Choosing the entry here means the rate the block states is one it picked.
    measured: on an N210 an 8 MS/s request lands on 7.692 MS/s, decimation 13, and a
        30.72 MS/s request on 33.33 MS/s, decimation 3, which the link cannot carry.
    verified-by: controls.uhd-rate-ladder-from-the-device
*/
inline double rateToWrite(double requestHz, const DeviceTruth& truth, double clockInForceHz, std::string_view wireFormat, Direction dir, std::string_view blockTag) {
    const DeviceTruth t = ladderTruthFor(truth, clockInForceHz);
    if (!offersOneClockRate(t) || t.rates.empty()) {
        const double ceiling = t.linkBytesPerS > 0.0 ? linkCeilingHz(wireFormat, t.linkBytesPerS) : 0.0;
        if (ceiling > 0.0 && requestHz > ceiling) {
            std::fprintf(stderr, "[%s] rate %.0f snapped to %.0f, the link ceiling UHD states for %.0f bytes per second in %s\n", std::string(blockTag).c_str(), requestHz, ceiling, t.linkBytesPerS,
                         std::string(wireFormat).c_str());
            return ceiling;
        }
        return requestHz;
    }
    // The whole ladder rather than the shorter list a caller is offered: a rate stated
    // outright is snapped to the nearest rate the radio reaches.
    const auto   ladder = reachableRatesFor(t, wireFormat);
    const double picked = snapToLadder(requestHz, ladder);
    if (std::abs(picked - requestHz) > 0.5) {
        std::fprintf(stderr, "[%s] rate %.0f snapped to %.0f, %s %.0f, the nearest this radio's clock and link reach\n", std::string(blockTag).c_str(), requestHz, picked, dir == Direction::Receive ? "decimation" : "interpolation",
                     picked > 0.0 ? t.mclkMinHz / picked : 0.0);
    }
    return picked;
}

// What a rate write left on the device: whether the write went through, and the rate the
// device answers afterwards, zero where it could not be asked either.
struct RateWrite {
    bool   landed = false;
    double heldHz = 0.0;
};

/*| contract: write a rate through write and read what the device holds through readBack, each
        under its own guard. A write the device refuses is named on standard error under
        blockTag, and the rate is read back in either case; a read-back that fails is named in
        words of its own and states no rate.
    why: a refused write leaves the device on the rate it held, and the rate a block states has
        to be the rate the device runs. A receive stream counted at a rate the radio never took
        reads the difference as shed samples on every block, and a transmit queue sized for it
        holds the wrong length of time.
    verified-by: sink.uhd-a-refused-rate-write-reads-the-rate-back
*/
template <typename Write, typename ReadBack>
RateWrite writeRateWith(double hz, Write&& write, ReadBack&& readBack, Direction dir, std::string_view blockTag) {
    RateWrite out;
    try {
        write(hz);
        out.landed = true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[%s] set_%s_rate(%.0f) failed: %s (the device keeps the rate it holds)\n", std::string(blockTag).c_str(), dir == Direction::Receive ? "rx" : "tx", hz, e.what());
    }
    try {
        out.heldHz = readBack();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[%s] get_%s_rate after the write failed: %s (no rate is read back)\n", std::string(blockTag).c_str(), dir == Direction::Receive ? "rx" : "tx", e.what());
    }
    return out;
}

/*| contract: whether a rate write settled the rate: the write landed and the device answered the
        rate it holds. A block records the rate as written only then, so a read-back that failed
        leaves the next replay owing a write, which reads the device again.
    why: a rate recorded as written with no rate read back is never asked for again: every later
        replay finds no write owed, and the block runs on with no rate in force, its queue at the
        floor, every play-out bound at the floor's half second and sample_rate on the request.
    verified-by: sink.uhd-a-rate-nobody-read-back-is-written-again
*/
constexpr bool rateWriteSettled(const RateWrite& written) { return written.landed && written.heldHz > 0.0; }

/*| contract: whether the analog filter follows a rate write: where the write settled the rate
        and the filter's range has room in it.
    why: the filter is derived from the rate the device holds, so a rate nobody could read back
        leaves nothing to derive it from, and a fixed filter takes no write at all.
    verified-by: sink.uhd-a-rate-read-back-that-fails-states-no-rate
*/
constexpr bool filterFollowsRate(const RateWrite& written, double bwMinHz, double bwMaxHz) { return rateWriteSettled(written) && bwMaxHz > bwMinHz; }

// The same write on one open device, the direction choosing the receive or the transmit call.
template <typename Usrp>
RateWrite writeRate(Usrp& usrp, Direction dir, double hz, std::string_view blockTag) {
    return writeRateWith(
        hz,
        [&usrp, dir](double r) {
            if (dir == Direction::Receive) {
                usrp.set_rx_rate(r, 0);
            } else {
                usrp.set_tx_rate(r, 0);
            }
        },
        [&usrp, dir] { return dir == Direction::Receive ? usrp.get_rx_rate(0) : usrp.get_tx_rate(0); }, dir, blockTag);
}

/*| contract: the entry an enum control's value selects out of a list of count options, and
        nothing where the value names none of them.
    why: a control carrying an index is only as good as the list it was drawn from, and a
        caller's stored index outlives the device it was stored against: the next radio lists
        other sources, or fewer. Bounded in the value's own domain, so a value that is not a
        number fails rather than casting into an index.
    verified-by: controls.uhd-reference-selection-comes-from-the-device-list
*/
inline std::optional<std::size_t> optionIndexFor(double value, std::size_t count) {
    const double at = std::round(value);
    if (!(at >= 0.0) || !(at < static_cast<double>(count))) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(at);
}

/*| contract: the clocks offered where a radio states a span, in MHz and ascending: the clocks
        a B2xx or an E3xx converter runs well at, to 61.44 MHz, and above that the clocks UHD
        4.9's X3xx clock driver has a mode for (120, 184.32 and 200 MHz), the clocks an N300 or
        N310 takes (122.88, 125 and 153.6 MHz) and the clocks an N320 takes (200, 245.76 and
        250 MHz).
    frame: the last three lists are x300_clock_ctrl.cpp's clocking modes,
        magnesium_constants.hpp's MAGNESIUM_RADIO_RATES and rhodium_constants.hpp's
        RHODIUM_RADIO_RATES.
*/
inline std::vector<double> masterClockPicksMHz() { return {5.0, 10.0, 16.0, 20.0, 30.72, 32.0, 40.0, 61.44, 120.0, 122.88, 125.0, 153.6, 184.32, 200.0, 245.76, 250.0}; }

/*| role: the master clocks a USRP takes at the open, by the product name UHD gives the unit,
        for the radios whose clock range UHD states as the one clock in force.
    frame: a product carries one set of clocks per kind of FPGA image, each in MHz and
        ascending. An X410 on a 200 MHz image takes 245.76 and 250 MHz, and one on a 400 MHz
        image takes 491.52 and 500 MHz. The X440 sets are the clocks the manual's table of
        validated clocks lists for the xx_200, xx_400 and xx_1600 images.
    frame: the product names are the ones get_mboard_name returns: "X300", "X310" and
        "NI-2974" from the X3xx driver, and the product the device's MPM daemon states for the
        rest, "n300", "n310", "n320", "x410" and "x440". An N321 states "n320".
    origin: the UHD 4.9.0.1 manual: usrp_x3x0.dox for the X300 and X310, usrp_n3xx.dox for
        the N300, N310 and N320, and usrp_x4xx.dox for the X410 and X440. The NI-2974 takes
        the X3xx set: x300_mboard_type.cpp classes it as an X310 motherboard, which the same
        driver and device arguments run. The N3xx sets match magnesium.py and rhodium.py,
        which check the clock at the open. The X410 sets are the manual's. The X410 clock
        policy of MPM also takes 122.88 and 125 MHz and a 200 MHz legacy clock, which the
        table leaves out.
    why: UHD 4.9 states the clock range of these radios as the clock in force alone, so the
        device cannot be asked which other clocks it takes. An X3xx and an X4xx keep the clock
        they were opened at for the whole session. An N3xx can change its clock while open,
        and the block still pins a clock for the next open, where the rates it offers are
        read.
    trap: the table is a copy of UHD's facts, and it drifts where a later UHD changes them. A
        clock the table offers and the UHD in use refuses fails the next open.
    trap: UHD 4.9.0.1 refuses a 120 MHz X3xx clock at the open although its clock driver has a
        mode for one: the device arguments take 184.32 to 200 MHz, and the daughterboard clock
        is derived only inside that span.
*/
struct ProductClocks {
    std::string_view                 product;
    std::vector<std::vector<double>> imageSetsMHz;
};

inline const std::vector<ProductClocks>& productClockTable() {
    static const std::vector<ProductClocks> table{
        {"X300", {{184.32, 200.0}}},
        {"X310", {{184.32, 200.0}}},
        {"NI-2974", {{184.32, 200.0}}},
        {"n300", {{122.88, 125.0, 153.6}}},
        {"n310", {{122.88, 125.0, 153.6}}},
        {"n320", {{200.0, 245.76, 250.0}}},
        {"x410", {{245.76, 250.0}, {491.52, 500.0}}},
        {"x440", {{125.0}, {125.0, 307.2, 327.68, 360.0, 368.64, 400.0, 500.0}, {125.0, 307.2, 327.68, 360.0, 368.64, 400.0, 500.0, 1000.0, 2000.0}}},
    };
    return table;
}

/*| contract: the clocks the product may take at the next open, in MHz and ascending: every
        clock of the image sets of the product that hold clockHz. Empty where the table does
        not name the product or no set holds the clock.
    why: the clock in force names the image only up to the sets that hold it, and the offer
        keeps every clock of those sets. An X440 at 125 MHz runs any of its three images and
        is offered the clocks of all three. The next open refuses a clock the loaded image
        does not take, and the block then clears the pick, so a pick is never stuck. The
        intersection instead leaves a radio at a clock every set holds with one clock and no
        control, and a pick of that clock could not be taken back.
    verified-by: controls.uhd-a-product-table-offers-the-clocks-of-a-fixed-clock-radio
*/
inline std::vector<double> productClocksMHz(std::string_view product, double clockHz) {
    const auto& table = productClockTable();
    const auto  entry = std::ranges::find(table, product, &ProductClocks::product);
    if (entry == table.end()) {
        return {};
    }
    std::vector<double> out;
    for (const auto& set : entry->imageSetsMHz) {
        // Compared in hertz within one: the clock UHD reads back carries its own rounding.
        if (std::ranges::none_of(set, [clockHz](double mhz) { return std::abs(mhz * 1e6 - clockHz) < 1.0; })) {
            continue;
        }
        for (const double mhz : set) {
            if (std::ranges::find(out, mhz) == out.end()) {
                out.push_back(mhz);
            }
        }
    }
    std::ranges::sort(out);
    return out;
}

/*| contract: whether the frontend the truth names is a TwinRX, whose X3xx takes a 200 MHz
        master clock alone.
    frame: usrp_x3x0.dox states that only 200 MHz is available with a TwinRX. The X3xx
        derives the daughterboard clock as half the master clock, and twinrx_ctrl.cpp refuses
        a daughterboard clock that is not a multiple of 6.25 or 12.5 MHz, so a 184.32 MHz
        open fails. UHD names each TwinRX frontend "TwinRX RX0" or "TwinRX RX1".
    trap: the name is the frontend of the channel the truth read. An X3xx whose TwinRX sits
        behind another channel is not seen.
*/
inline bool frontendIsTwinRx(const DeviceTruth& t) { return t.frontend.starts_with("TwinRX"); }

// The one clock an X3xx with a TwinRX takes, in MHz.
inline constexpr double kTwinRxClockMHz = 200.0;

/*| contract: the master-clock picks to offer for a radio whose clock can be chosen, in MHz and
        ascending, the first entry a zero that means UHD chooses. A radio stating separate
        clocks is offered those clocks. A radio stating a span is offered the entries of
        masterClockPicksMHz inside it. A radio stating one clock is offered the clocks
        productClocksMHz states for its product at that clock, 200 MHz alone behind a TwinRX,
        and nothing where the table states none.
    frame: a span offers picks and not the whole range because a caller drawing a slider over
        it would offer clocks no rate needs.
    frame: the product is the motherboard name the truth read, or the product of the
        information map where no name was read.
    verified-by: controls.uhd-master-clock-ladder
    verified-by: controls.uhd-a-radio-stating-clocks-above-the-converter-picks
    verified-by: controls.uhd-a-product-table-offers-the-clocks-of-a-fixed-clock-radio
*/
inline std::vector<double> masterClockLadderMHz(const DeviceTruth& t) {
    if (t.mclkMaxHz <= t.mclkMinHz) {
        auto clocks = offersOneClockRate(t) ? productClocksMHz(t.model.empty() ? t.product : t.model, t.mclkMinHz) : std::vector<double>{};
        if (frontendIsTwinRx(t)) {
            std::erase_if(clocks, [](double mhz) { return mhz != kTwinRxClockMHz; });
        }
        if (clocks.empty()) {
            return {};
        }
        std::vector<double> out{0.0};
        out.insert(out.end(), clocks.begin(), clocks.end());
        return out;
    }
    std::vector<double> out{0.0};
    if (!t.mclkPoints.empty()) {
        for (const double hz : t.mclkPoints) {
            out.push_back(hz / 1e6);
        }
        return out;
    }
    for (const double mhz : masterClockPicksMHz()) {
        // Compared in MHz, so a pick that is exactly an end of the stated range is inside it:
        // scaling the pick up instead can miss by an ulp and drop the fastest clock.
        if (mhz >= t.mclkMinHz / 1e6 && mhz <= t.mclkMaxHz / 1e6) {
            out.push_back(mhz);
        }
    }
    return out.size() == 1UZ ? std::vector<double>{} : out;
}

// Run one read of the reader below. A read the device refuses leaves its field at the default.
template <typename Read>
void readOptional(Read&& read) {
    try {
        read();
    } catch (const std::exception&) { // a facility this model does not state
    }
}

/*| contract: read one open device's control surface for one direction into truth, replacing
        it. Every read is optional, and every read but the firmware and FPGA version pair is
        guarded alone: a facility a model lacks leaves its field at the default, and a refused
        read costs its own field and no other. A gain element whose range or value is refused
        is left out, and the elements after it are read.
    frame: the one place that says what the truth is, so a probe of a stopped device and a
        running block's own record of the device it opened are the same record, and the receive
        record and the transmit record of one unit differ only where the daughterboard states
        the two separately. The receive block adds the three optional frontend facilities, which
        hang off the receive frontend alone.
    trap: two reads under one guard lose the second when the first is refused. A filter range
        lost that way reads as a fixed filter and hides the BANDWIDTH control, and a gain
        element lost that way takes every element after it.
    verified-by: controls.uhd-a-refused-read-costs-its-own-field
*/
template <typename Usrp>
void readTruthInto(Usrp& usrp, DeviceTruth& truth, Direction dir) {
    const bool rx       = dir == Direction::Receive;
    truth               = {};
    truth.linkBytesPerS = statedLinkBytesPerS(usrp);
    readOptional([&] {
        const auto info   = rx ? usrp.get_usrp_rx_info(0) : usrp.get_usrp_tx_info(0);
        truth.product     = info.get("mboard_id", "USRP");
        /*| trap: channel 0's frontend alone. A TwinRX in the other slot of an X3xx whose
                channel 0 is another board is not seen, and a 184.32 MHz pick there fails one
                start, which then clears the pick. */
        truth.frontend    = info.get(rx ? "rx_subdev_name" : "tx_subdev_name", "");
        truth.serial      = info.get("mboard_serial", "");
        truth.fwVersion   = info.get("mboard_fw_version", "");
        truth.fpgaVersion = info.get("mboard_fpga_version", "");
    });
    readOptional([&] { truth.model = usrp.get_mboard_name(0); });
    /*| frame: the versions the information map leaves out. An N-series carries them on the
            motherboard node; a B2xx states them in the map above. */
    readOptional([&] {
        auto tree = usrp.get_device()->get_tree();
        for (const auto& [key, field] : {std::pair{"fw_version", &truth.fwVersion}, std::pair{"fpga_version", &truth.fpgaVersion}}) {
            const std::string path = std::string("/mboards/0/") + key;
            if (field->empty() && tree->exists(path)) {
                *field = tree->template access<std::string>(path).get();
            }
        }
    });
    std::vector<std::string> gainNames;
    readOptional([&] { gainNames = rx ? usrp.get_rx_gain_names(0) : usrp.get_tx_gain_names(0); });
    for (const std::string& g : gainNames) {
        readOptional([&] {
            const auto   r       = rx ? usrp.get_rx_gain_range(g, 0) : usrp.get_tx_gain_range(g, 0);
            const double current = rx ? usrp.get_rx_gain(g, 0) : usrp.get_tx_gain(g, 0);
            truth.gains.push_back({g, r.start(), r.stop(), r.step() > 0.0 ? r.step() : 1.0, current});
        });
    }
    readOptional([&] {
        const auto overall = rx ? usrp.get_rx_gain_range(0) : usrp.get_tx_gain_range(0);
        truth.gainMinDb    = overall.start();
        truth.gainMaxDb    = overall.stop();
        truth.gainStepDb   = overall.step() > 0.0 ? overall.step() : 1.0;
    });
    readOptional([&] { truth.antennas = rx ? usrp.get_rx_antennas(0) : usrp.get_tx_antennas(0); });
    readOptional([&] { truth.antenna = rx ? usrp.get_rx_antenna(0) : usrp.get_tx_antenna(0); });
    readOptional([&] {
        for (const auto& r : rx ? usrp.get_rx_freq_range(0) : usrp.get_tx_freq_range(0)) {
            truth.freqRanges.emplace_back(r.start(), r.stop());
        }
    });
    readOptional([&] {
        const auto bw = rx ? usrp.get_rx_bandwidth_range(0) : usrp.get_tx_bandwidth_range(0);
        truth.bwMinHz = bw.start();
        truth.bwMaxHz = bw.stop();
    });
    readOptional([&] { truth.bwCurHz = rx ? usrp.get_rx_bandwidth(0) : usrp.get_tx_bandwidth(0); });
    std::optional<std::pair<double, double>> mclkRange;
    std::optional<double>                    mclkHz;
    readOptional([&] {
        const auto mclk      = usrp.get_master_clock_rate_range(0);
        mclkRange            = std::pair{mclk.start(), mclk.stop()};
        truth.mclkContinuous = mclk.size() == 1UZ && mclk.front().start() < mclk.front().stop();
        if (mclk.size() >= 2UZ && std::ranges::all_of(mclk, [](const ::uhd::range_t& r) { return r.start() == r.stop(); })) {
            for (const auto& r : mclk) {
                truth.mclkPoints.push_back(r.start());
            }
            std::ranges::sort(truth.mclkPoints);
        }
    });
    readOptional([&] {
        auto tree = usrp.get_device()->get_tree();
        if (tree->exists("/name")) {
            truth.deviceKind = tree->template access<std::string>("/name").get();
        }
    });
    readOptional([&] { mclkHz = usrp.get_master_clock_rate(0); });
    const auto clock = masterClockFrom(mclkRange, mclkHz);
    truth.mclkMinHz  = clock.minHz;
    truth.mclkMaxHz  = clock.maxHz;
    truth.mclkHz     = clock.hz;
    readOptional([&] {
        auto tree = usrp.get_device()->get_tree();
        if (tree->exists("/mboards/0/auto_tick_rate")) {
            truth.autoTickRate = tree->template access<bool>("/mboards/0/auto_tick_rate").get();
        }
    });
    readOptional([&] {
        for (const auto& r : rx ? usrp.get_rx_rates(0) : usrp.get_tx_rates(0)) {
            truth.rates.emplace_back(r.start(), r.stop());
        }
    });
    readOptional([&] { truth.clockSources = usrp.get_clock_sources(0); });
    readOptional([&] { truth.clockSource = usrp.get_clock_source(0); });
    readOptional([&] { truth.timeSources = usrp.get_time_sources(0); });
    readOptional([&] { truth.timeSource = usrp.get_time_source(0); });
}

/*| role: the USRP handles this process has let go of and whose release has not finished.
    contract: releaseLater() takes the caller's references and drops them on a thread of its
        own, so the caller returns at once. awaitReleases() returns once every release handed
        off before the call has ended, one that another caller is joining included. Every
        open in this process calls awaitReleases() first, because a unit is exclusive and a
        selector need not name a unit the way another block's did. A block's destructor does not
        wait, so a caller that destroys the block at every stop gets the stop back as fast as a
        caller that keeps it.
    frame: a device is released when its last reference goes. A source and a sink sharing one
        unit through the vendor's own cache each hand their references here, and the unit is
        released once, by whichever thread drops the last one.
    measured: the release alone costs 1147 ms on an N210 and 16 ms on a B205mini, three runs
        each of the bench's open-time scenario.
    trap: the registry joins what is left when the process ends, so no release thread is
        destroyed while it runs. The join is registered with std::atexit at every hand-off, so
        it runs before the destructors of the vendor library's own static objects, which the
        open behind the hand-off constructed after this registry. The registry's own
        destructor runs after theirs, so a join left to it alone would run a release against a
        library that has already been torn down.
    verified-by: sink.uhd-the-release-ends-before-the-library-at-exit
    verified-by: sink.uhd-a-second-wait-waits-for-the-release-in-flight
*/
class Releases {
public:
    static Releases& instance() {
        static Releases releases;
        return releases;
    }

    Releases()                           = default;
    Releases(const Releases&)            = delete;
    Releases& operator=(const Releases&) = delete;
    ~Releases() { awaitAll(); }

    template <typename... Handles>
    void releaseLater(Handles&&... handles) {
        {
            std::lock_guard lock(_mutex);
            _waiting.push_back({.number = _handedOff + 1U, .thread = std::thread([... held = std::move(handles)]() mutable { (held.reset(), ...); })});
            _unjoined.insert(++_handedOff);
        }
        std::atexit(+[] { Releases::instance().awaitAll(); });
    }

    /*| contract: join the releases no other caller has taken, then wait until every release
            numbered up to the last hand-off before the call has been joined, by this caller or
            by another.
        why: a caller that took the list and joins it holds releases a second caller cannot
            join. Returning on an empty list would let the second caller open a unit whose
            release is still running.
    */
    void awaitAll() {
        std::unique_lock    lock(_mutex);
        const std::uint64_t upTo  = _handedOff;
        std::vector<Release> taken = std::exchange(_waiting, {});
        lock.unlock();
        for (Release& r : taken) {
            r.thread.join();
        }
        lock.lock();
        for (const Release& r : taken) {
            _unjoined.erase(r.number);
        }
        _joined.notify_all();
        _joined.wait(lock, [this, upTo] { return _unjoined.empty() || *_unjoined.begin() > upTo; });
    }

private:
    struct Release {
        std::uint64_t number = 0U;
        std::thread   thread;
    };
    std::mutex              _mutex;
    std::condition_variable _joined;
    std::vector<Release>    _waiting;       // handed off and taken by no caller yet
    std::set<std::uint64_t> _unjoined;      // the numbers of the releases not yet joined
    std::uint64_t           _handedOff = 0U; // the number of the last hand-off
};

/*| contract: reset the given handles on a releasing thread and return at once. The caller's
        handles are empty afterwards. The thread resets them in the order given, the streamer
        before the device.
    verified-by: sink.uhd-a-release-runs-behind-the-stop
*/
template <typename... Handles>
void releaseLater(Handles&&... handles) {
    Releases::instance().releaseLater(std::move(handles)...);
}

/*| contract: return once every release this process handed off before the call has finished,
        whichever caller joins it. */
inline void awaitReleases() { Releases::instance().awaitAll(); }

/*| contract: put an open device on the rate a block writes for a request above zero, snapped
        by rateToWrite on the sc16 ladder the open address gives, and on the analog filter a
        block's rate applier derives from the rate held; then read the control surface of one
        direction into truth. A rate of zero leaves the device where it came up. A refused
        write is named on standard error under blockTag, and the surface is read where the
        device stands.
    why: UHD reaches a request a fixed clock does not divide by the nearest whole decimation
        and prints its own warning where that decimation is odd: an N210 asked for 8 MS/s
        runs 7.692 MS/s, decimation 13, where a block runs 8.333 MS/s.
    frame: a USRP states its coverage as the tuner's range widened by half the analog filter,
        within the digital converter's reach, and UHD clips a request outside it to the nearest
        edge. The filter therefore sets the coverage: a B205mini at 2 MS/s reaches 49 MHz on the
        2 MHz filter a block writes and 34 MHz on the 56 MHz filter the device comes up with.
    verified-by: controls.uhd-the-probe-reads-the-surface-the-block-runs-on
*/
template <typename Usrp>
void readSurfaceAtRate(Usrp& usrp, double sampleRateHz, DeviceTruth& truth, Direction dir, const std::string& kwargs, std::string_view blockTag) {
    const bool rx = dir == Direction::Receive;
    truth         = {};
    if (sampleRateHz > 0.0) {
        try {
            DeviceTruth atOpen;
            readTruthInto(usrp, atOpen, dir);
            const double want = rateToWrite(sampleRateHz, atOpen, clockInForceAtOpen(::uhd::device_addr_t(kwargs), atOpen), "sc16", dir, blockTag);
            if (rx) {
                usrp.set_rx_rate(want, 0);
            } else {
                usrp.set_tx_rate(want, 0);
            }
            const double held  = rx ? usrp.get_rx_rate(0) : usrp.get_tx_rate(0);
            const auto   range = rx ? usrp.get_rx_bandwidth_range(0) : usrp.get_tx_bandwidth_range(0);
            if (range.stop() > range.start()) {
                const double width = std::clamp(held, range.start(), range.stop());
                if (rx) {
                    usrp.set_rx_bandwidth(width, 0);
                } else {
                    usrp.set_tx_bandwidth(width, 0);
                }
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[%s] probe set_%s_rate(%.0f): %s (the surface is read at the rate the device came up at)\n", std::string(blockTag).c_str(), rx ? "rx" : "tx", sampleRateHz, e.what());
        }
    }
    readTruthInto(usrp, truth, dir);
}

/*| contract: open the device briefly, while the graph is stopped, and read the control surface
        of one direction. False when the device cannot be opened. blockTag names the caller on
        standard error.
    frame: a rate above zero is written before the surface is read, so the tuner coverage and
        the rate ladder are the ones that will hold while the block runs. extra runs on the one
        open handle behind the shared read, for whatever a direction states beyond the shared
        record.
    trap: a USRP's coverage and its stated rate set both move with the master clock, and the
        master clock moves with the rate, so a probe taken at whatever rate the device came up
        at states limits the block will not honor. A B205mini states 42 to 6008 MHz at a 16 MHz
        master clock and 34 to 6016 MHz at a 32 MHz one.
*/
template <typename Extra = decltype([](::uhd::usrp::multi_usrp&, DeviceTruth&) {})>
bool probeTruth(const std::string& kwargs, double sampleRateHz, DeviceTruth& truth, Direction dir, std::string_view blockTag, Extra&& extra = {}) {
    try {
        awaitReleases();
        auto usrp = ::uhd::usrp::multi_usrp::make(::uhd::device_addr_t(kwargs));
        readSurfaceAtRate(*usrp, sampleRateHz, truth, dir, kwargs, blockTag);
        extra(*usrp, truth);
        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[%s] probe(%s): %s\n", std::string(blockTag).c_str(), kwargs.c_str(), e.what());
        return false;
    }
}

/*| contract: write the reference and timing selections a caller asked for and leave the rest
        alone. Called with the block's control mutex held, from start() alone. blockTag names
        the caller on standard error.
    why: UHD states that reconfiguring the clock source affects the FPGA clocking and the
        timekeeping, that there should be no streaming operation while it happens, and that a
        device time set beforehand is most likely lost. Both writes therefore happen before the
        streamer exists and before the device time is set, and a control asking for either
        stages it for the next start.
    trap: the device may move one selection when the other is written, so reading both back is
        the only certain statement of what the radio is running on.
*/
inline void applyReferenceSources(::uhd::usrp::multi_usrp& usrp, const std::string& wantClock, const std::string& wantTime, std::string_view blockTag) {
    for (const auto& [isClock, want] : {std::pair{true, wantClock}, std::pair{false, wantTime}}) {
        if (want.empty()) {
            continue;
        }
        try {
            if (isClock) {
                usrp.set_clock_source(want, 0);
            } else {
                usrp.set_time_source(want, 0);
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[%s] set_%s_source(%s): %s (the device keeps the selection it had)\n", std::string(blockTag).c_str(), isClock ? "clock" : "time", want.c_str(), e.what());
        }
    }
}

// ---- the control surface both directions state ---------------------------------------------

// The gain element names a truth record holds, as one printable list.
inline std::string gainElementList(const DeviceTruth& t) {
    std::string names;
    for (const auto& g : t.gains) {
        names += " " + g.name;
    }
    return names.empty() ? std::string(" none") : names;
}

/*| contract: whether a gain write may name element on a device stating truth: the empty name,
        which is the overall gain UHD distributes across the elements itself, and every element
        the device lists. A name the device does not list is refused and named on standard
        error under blockTag, beside the names the device gave.
    why: the element names are a property of the model and the daughterboard — one PGA on a
        B2xx, an ADC pair and a PGA0 on an N-series with a WBX — so a name written into a
        caller cannot be right for both, and UHD refuses the one that is wrong rather than
        doing something near it.
    verified-by: sink.uhd-an-empty-element-name-is-the-overall-gain
*/
inline bool gainElementNamed(const DeviceTruth& t, const std::string& element, std::string_view blockTag) {
    if (element.empty() || std::ranges::any_of(t.gains, [&element](const DeviceTruth::GainElement& g) { return g.name == element; })) {
        return true;
    }
    std::fprintf(stderr, "[%s] this radio has no gain element \"%s\"; it names%s\n", std::string(blockTag).c_str(), element.c_str(), gainElementList(t).c_str());
    return false;
}

/*| contract: a gain brought inside the range the open device states, and the low end of that
        range for a value that is not a number whatever range is stated. A finite request for a
        device stating no usable range, its ends equal or reversed, is left as it stands.
    frame: the low end is the one the truth record carries, and its default for a frontend that
        states no overall range is 0.0 rather than a figure the device gave. Every USRP frontend
        read here, in either direction, floors at 0 dB, so the two agree on this bench and the
        answer is the record's either way.
    why: UHD coerces a gain past the frontend's range without refusing it, so a request of
        200 dB reaches the device's maximum with only the read-back to say so. The family bounds
        a control at the device everywhere else.
    trap: the test for a value that is not a number comes first. A frontend stating no overall
        range and a gain element whose ends are equal — a fixed stage — both reach the
        unusable-range branch, and a NaN passed through it reaches the device.
    verified-by: sink.uhd-bounds-the-transmit-gain-at-the-device
*/
inline double boundedGainDb(double requestDb, double minDb, double maxDb) {
    if (requestDb != requestDb) {
        return minDb;
    }
    if (!(maxDb > minDb)) {
        return requestDb;
    }
    return std::clamp(requestDb, minDb, maxDb);
}

/*| contract: a gain write bounded by the range truth states for element, and by the overall
        range for the empty name, which is the overall gain. A write the bound moved is named
        on standard error under blockTag. A name the truth does not list is left as it stands:
        gainElementNamed refuses it.
    verified-by: controls.uhd-bounds-every-control-value
*/
inline double boundedGainFor(const DeviceTruth& t, const std::string& element, double dB, std::string_view blockTag) {
    double lo = t.gainMinDb;
    double hi = t.gainMaxDb;
    if (!element.empty()) {
        const auto it = std::ranges::find_if(t.gains, [&element](const DeviceTruth::GainElement& g) { return g.name == element; });
        if (it == t.gains.end()) {
            return dB;
        }
        lo = it->min;
        hi = it->max;
    }
    const double bounded = boundedGainDb(dB, lo, hi);
    if (!(bounded == dB)) {
        std::fprintf(stderr, "[%s] the gain %.1f dB%s%s is outside this radio's %.1f to %.1f dB; %.1f dB is used\n", std::string(blockTag).c_str(), dB, element.empty() ? "" : " on ", element.c_str(), lo, hi, bounded);
    }
    return bounded;
}

/*| contract: the line naming a control value the bound moved, under blockTag, or an empty string
        where the value reaches the device as the caller gave it. A value that is not a number
        has words of its own.
    why: a caller holding a block directly is told of a move as the gain bound tells of one. A
        MASTER_CLOCK of 100 MHz on a B205mini is written as its last pick, 61.44 MHz, and a
        BANDWIDTH of 80 MHz as 56 MHz.
    verified-by: sink.uhd-names-a-control-value-it-moves
*/
inline std::string movedControlLine(std::string_view blockTag, const std::string& id, double asked, double bounded) {
    if (asked == bounded) {
        return {};
    }
    if (asked != asked) {
        return std::format("[{}] {} is not a number; {:g} is used", blockTag, id, bounded);
    }
    return std::format("[{}] {} {:g} is outside what this radio states; {:g} is used", blockTag, id, asked, bounded);
}

/*| contract: value bounded to the domain the control id states in surface, or nothing where
        surface states no such control or value names no entry of an enum's list. A value that
        is not a number comes back as a gain's floor, the quiet end boundedGainDb takes, and as
        any other control's default; every other value of a range, a toggle or a list that
        is anyInRange is brought inside the domain clampToControl gives it.
    why: a caller holding a block directly bounds nothing, and a value a caller stored under one
        device or one rate outlives both. The family bounds every control at the device, as
        ControlDesc states. An index past a device's list names no source and no connector, and
        clamped it would select the last entry, one the caller never asked for.
    contract: a value the bound moved is named on standard error under blockTag, where one is
        given, in the words movedControlLine states.
    verified-by: controls.uhd-bounds-every-control-value
    verified-by: sink.uhd-names-a-control-value-it-moves
*/
inline std::optional<double> boundedControlValue(std::span<const ControlDesc> surface, const std::string& id, double value, std::string_view blockTag = {}) {
    const auto desc = std::ranges::find_if(surface, [&id](const ControlDesc& c) { return c.id == id; });
    if (desc == surface.end()) {
        return std::nullopt;
    }
    if (desc->kind == 3 && value == value && !optionIndexFor(value, desc->options.size()).has_value()) {
        return std::nullopt;
    }
    /*| contract: a list that takes its entries alone, kind 4 without anyInRange, takes a value
            within a millionth of the unit of one of its entries as that entry. It refuses every
            other value and names the entries on standard error under blockTag.
        why: a value stored under one radio is not moved onto an entry of another. A B2xx's
            30.72 MHz clock replayed onto an X310 would otherwise pin a clock nobody picked.
        verified-by: controls.uhd-a-list-takes-its-entries-alone
    */
    if (desc->kind == 4 && !desc->anyInRange && value == value && !desc->listValues.empty()) {
        const auto entry = std::ranges::find_if(desc->listValues, [value](double v) { return std::abs(v - value) <= 1e-6; });
        if (entry == desc->listValues.end()) {
            if (!blockTag.empty()) {
                std::string entries;
                for (const double v : desc->listValues) {
                    entries += std::format("{}{:g}", entries.empty() ? "" : ", ", v);
                }
                std::fprintf(stderr, "[%s] %s %g is not one of its entries (%s); nothing is written\n", std::string(blockTag).c_str(), id.c_str(), value, entries.c_str());
            }
            return std::nullopt;
        }
        return *entry;
    }
    // A gain's NaN takes its floor, one answer for a gain whichever path writes it.
    const double bounded = desc->isGain && value != value ? desc->min : clampToControl(*desc, value);
    if (!blockTag.empty()) {
        if (const std::string line = movedControlLine(blockTag, id, value, bounded); !line.empty()) {
            std::fprintf(stderr, "%s\n", line.c_str());
        }
    }
    return bounded;
}

// One range control per gain element the truth names, each defaulting to what the element
// held when the device was read. The UHD element name doubles as the caller's config key.
inline std::vector<ControlDesc> gainElementControls(const DeviceTruth& t) {
    std::vector<ControlDesc> out;
    for (const auto& g : t.gains) {
        ControlDesc c;
        c.id       = g.name;
        c.label    = g.name;
        c.unit     = "dB";
        c.kind     = 0;
        c.min      = g.min;
        c.max      = g.max;
        c.step     = g.step;
        c.defValue = g.current;
        c.isGain   = true;
        out.push_back(std::move(c));
    }
    return out;
}

/*| contract: the analog filter width to ask for at a rate, inside the range the device states.
        A device stating no range, its ends equal or reversed, leaves the rate as it stands.
    verified-by: sink.uhd-antenna-and-bandwidth
*/
inline double bandwidthFor(double rateHz, double minHz, double maxHz) { return maxHz > minHz ? std::clamp(rateHz, minHz, maxHz) : rateHz; }

/*| contract: the BANDWIDTH control for the analog filter truth states, in MHz, or nothing for
        a fixed filter. heldBandwidthHz above zero carries the width the filter holds on a
        running block and becomes the default there; zero leaves the default at the
        rate-derived value the rate applier programs, the only value a caller drawing a surface
        for a stopped device has.
    frame: the analog filter is a control only where the frontend states a range with room in
        it. A range whose ends are equal is a fixed filter: the descriptor kinds carry no
        read-only form, so stating it would draw a slider a caller can move with nothing behind
        it. The width itself reaches a caller through the truth's bwCurHz and each block's
        analogBandwidthHz().
    measured: a WBX states 40 MHz to 40 MHz and holds 40 MHz whatever is written.
*/
inline std::optional<ControlDesc> bandwidthControl(const DeviceTruth& t, const TruthContext& ctx, double heldBandwidthHz) {
    if (!(t.bwMaxHz > t.bwMinHz)) {
        return std::nullopt;
    }
    ControlDesc c;
    c.id          = "BANDWIDTH";
    c.label       = "Analog BW";
    c.unit        = "MHz";
    c.kind        = 0;
    c.min         = t.bwMinHz / 1e6;
    c.max         = t.bwMaxHz / 1e6;
    c.step        = 0.1;
    c.isGain      = false;
    c.defValue    = (heldBandwidthHz > 0.0 ? std::clamp(heldBandwidthHz, t.bwMinHz, t.bwMaxHz) : bandwidthFor(ctx.sampleRate, t.bwMinHz, t.bwMaxHz)) / 1e6;
    c.rateDerived = true;
    return c;
}

// ---- the appliers both directions run ------------------------------------------------------

/*| contract: write the analog filter through write where hz is not the value already in
        written, and keep what the device settled on, read through readBack, in heldHz. A write
        the device refuses is named on standard error under blockTag and leaves heldHz at zero,
        so the width a block states is none until a write lands, and written where it was.
    why: UHD coerces a bandwidth to the nearest valid value, so on a frontend whose filter
        ladder is stepped the device can settle below the request and a descriptor built from
        the request would state a filter the hardware does not have.
    trap: a throw let out of here reaches a settings handler, and an exception raised there
        becomes a scheduler exception that ends the graph over one refused control write.
    measured: a B205mini's filter is continuous from 200 kHz to 56 MHz. Its receive side takes
        0.5, 2 and 3.7 MHz exactly and its transmit side 200 kHz and 1 MHz, and 80 MHz settles
        at 56 MHz on both.
    verified-by: controls.uhd-a-refused-filter-write-states-no-width
*/
template <typename Write, typename ReadBack>
void applyBandwidthWith(double hz, std::optional<double>& written, double& heldHz, Write&& write, ReadBack&& readBack, Direction dir, std::string_view blockTag) {
    if (!writeNeeded(written, hz)) {
        return;
    }
    try {
        write(hz);
        written = hz;
        heldHz  = readBack();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[%s] set_%s_bandwidth(%.0f): %s (continuing)\n", std::string(blockTag).c_str(), dir == Direction::Receive ? "rx" : "tx", hz, e.what());
        heldHz = 0.0;
    }
}

// The same write on one open device, the direction choosing the receive or the transmit call.
template <typename Usrp>
void applyBandwidth(Usrp& usrp, Direction dir, double hz, std::optional<double>& written, double& heldHz, std::string_view blockTag) {
    applyBandwidthWith(
        hz, written, heldHz,
        [&usrp, dir](double w) {
            if (dir == Direction::Receive) {
                usrp.set_rx_bandwidth(w, 0);
            } else {
                usrp.set_tx_bandwidth(w, 0);
            }
        },
        [&usrp, dir] { return dir == Direction::Receive ? usrp.get_rx_bandwidth(0) : usrp.get_tx_bandwidth(0); }, dir, blockTag);
}

/*| contract: replace ranges with the coverage the device states now for one direction, and keep
        the last statement when the device cannot be asked or states none.
    invariant: it runs on every rate change, because a USRP's coverage moves with the rate and a
        request legal at one rate is outside the radio at another.
*/
template <typename Usrp>
void refreshFreqRanges(Usrp& usrp, Direction dir, std::vector<std::pair<double, double>>& ranges, std::string_view blockTag) {
    try {
        std::vector<std::pair<double, double>> stated;
        for (const auto& r : dir == Direction::Receive ? usrp.get_rx_freq_range(0) : usrp.get_tx_freq_range(0)) {
            stated.emplace_back(r.start(), r.stop());
        }
        if (!stated.empty()) {
            ranges = std::move(stated);
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[%s] get_%s_freq_range: %s (the coverage last stated is kept)\n", std::string(blockTag).c_str(), dir == Direction::Receive ? "rx" : "tx", e.what());
    }
}

/*| contract: whether a stop arrived while a start was opening the device: the block has left
        its active states and is not merely initialized. A start that reads true releases the
        device it opened and returns before it claims a streamer, so the teardown behind it
        stays trivial.
    frame: multi_usrp::make takes seconds, a B2xx loading its firmware and FPGA image, which is
        the window a stop request lands in.
*/
inline bool stopArrivedDuringOpen(gr::lifecycle::State state) { return !gr::lifecycle::isActive(state) && state != gr::lifecycle::State::INITIALISED; }

// ---- the reference -------------------------------------------------------------------------

// How long a start waits for an external reference to come up, in milliseconds.
inline constexpr double kRefLockLimitMs = 1500.0;

// A live predicate that never falls, for a wait nothing ends early.
struct AlwaysLive {
    bool operator()() const { return true; }
};

/*| contract: poll locked() until it answers true, the limit passes or live() answers false, and
        answer whether it locked and how long the wait took in milliseconds. live is read before
        every poll, so a stop ends the wait inside one poll.
    why: a stream started before the reference has come up is disciplined by nothing for its
        first samples, and the radio gives no other sign. The wait is bounded because a
        reference that is not there never arrives, and a stream refused for that is worse than
        one started unlocked and said so.
    measured: a B205mini told to run on an external reference with nothing attached never locks,
        so the wait costs its whole limit.
    verified-by: controls.uhd-reference-lock-wait-is-bounded
*/
template <typename Locked, typename Live = AlwaysLive>
std::pair<bool, double> waitForRefLock(Locked&& locked, double limitMs, double pollMs = 25.0, Live&& live = Live{}) {
    const auto start  = std::chrono::steady_clock::now();
    const auto waited = [&start] { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count(); };
    for (;;) {
        if (!live()) {
            return {false, waited()};
        }
        if (locked()) {
            return {true, waited()};
        }
        if (waited() >= limitMs) {
            return {false, waited()};
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(pollMs));
    }
}

// What a start's wait for the reference found: whether it waited at all, whether the reference
// locked, whether a stop ended the wait, and how long the wait took in milliseconds.
struct ReferenceWait {
    bool   waited   = false;
    bool   locked   = false;
    bool   stopped  = false;
    double waitedMs = 0.0;
};

/*| contract: on a clock source other than internal, poll the device's ref_locked sensor for at
        most limitMs and say what happened on standard error under blockTag; on an internal
        clock, or a radio that states no clock source, do not wait at all. A live() that answers
        false ends the wait at the next poll, and the line says a stop ended it. Called from a
        start before the stream begins, off the graph thread, so the wait costs no sample.
    why: a transmitter started on an external reference that has not locked radiates off
        frequency for up to the lock time, with nothing said, as a receiver hears off
        frequency.
    verified-by: sink.uhd-waits-for-the-reference
    why: an internal clock is the radio's own and is locked by construction, and a reference
        sensor that reads unlocked there is reporting the absence of an external reference
        rather than a fault.
    measured: a B205mini on external with nothing attached spends the whole 1500 ms and streams
        anyway.
*/
template <typename Usrp, typename Live = AlwaysLive>
ReferenceWait waitForReference(Usrp& usrp, std::string_view blockTag, double limitMs = kRefLockLimitMs, double pollMs = 25.0, Live&& live = Live{}) {
    try {
        if (usrp.get_clock_source(0) == "internal") {
            return {};
        }
    } catch (const std::exception&) {
        return {}; // a radio that states no clock source has no reference to wait for
    }
    const auto [locked, waitedMs] = waitForRefLock(
        [&usrp]() -> bool {
            try {
                return usrp.get_mboard_sensor("ref_locked", 0).to_bool();
            } catch (const std::exception&) {
                return false;
            }
        },
        limitMs, pollMs, live);
    if (!locked && !live()) {
        std::fprintf(stderr, "[%s] a stop ended the wait for the reference after %.0f ms\n", std::string(blockTag).c_str(), waitedMs);
        return {.waited = true, .locked = false, .stopped = true, .waitedMs = waitedMs};
    }
    std::fprintf(stderr, "[%s] the reference %s after %.0f ms%s\n", std::string(blockTag).c_str(), locked ? "locked" : "did not lock", waitedMs, locked ? "" : "; the stream starts anyway");
    return {.waited = true, .locked = locked, .waitedMs = waitedMs};
}

// ---- the device's sensors ------------------------------------------------------------------

/*| contract: a person's name for a sensor UHD names tersely, by the sensor's id, and empty for
        every other id. A sensor this does not name keeps the name the device gives it.
*/
inline std::string_view operatorLabel(std::string_view id) {
    if (id == "ref_locked") {
        return "Reference lock";
    }
    if (id == "lo_locked") {
        return "LO lock";
    }
    if (id == "mimo_locked") {
        return "MIMO lock";
    }
    if (id == "gps_locked") {
        return "GPS lock";
    }
    if (id == "temp") {
        return "Temperature";
    }
    return {};
}

// One UHD sensor value as the generic reading the sensor seam publishes.
inline SensorReading toReading(const std::string& id, const ::uhd::sensor_value_t& sv) {
    SensorReading r;
    r.id                        = id;
    const std::string_view name = operatorLabel(id);
    r.label                     = name.empty() ? sv.name : std::string(name);
    // A boolean sensor carries its state word in unit ("locked"/"unlocked") and "true"/"false"
    // in value; every other type carries the quantity in value and a real unit beside it.
    if (sv.type == ::uhd::sensor_value_t::BOOLEAN) {
        r.value = sv.unit;
        r.good  = sv.to_bool();
    } else {
        r.value = sv.unit.empty() ? sv.value : sv.value + " " + sv.unit;
    }
    // A free-form string sensor is a multi-field record (NMEA, servo status).
    r.brief = sv.type != ::uhd::sensor_value_t::STRING;
    return r;
}

/*| contract: append one reading per name, the motherboard set first, calling read(name,
        isMboard) for each and live() before each. A reading whose id is already in out is
        dropped. Returns whether the sweep ran to completion.
    invariant: the motherboard set goes first and so wins an id collision with a frontend
        sensor of the same id. One id always denotes one reading.
    trap: a read that throws is skipped rather than failing the sweep: a GPSDO that has just
        been unplugged should not blank the frontend's lock indicator. live() answering false
        abandons the sweep and empties out, because a reading set missing half its entries
        reads as sensors that have disappeared rather than as a sweep that was cut short.
    why: rechecking between every read, rather than once at the top, bounds a waiting caller's
        delay to one read instead of a whole sweep.
*/
template <typename ReadFn, typename LiveFn>
bool sweepSensors(std::vector<SensorReading>& out, const std::vector<std::string>& mboardNames, const std::vector<std::string>& frontendNames, ReadFn&& read, LiveFn&& live) {
    for (const auto* names : {&mboardNames, &frontendNames}) {
        const bool mboard = names == &mboardNames;
        for (const std::string& n : *names) {
            if (!live()) {
                out.clear();
                return false;
            }
            try {
                SensorReading r = read(n, mboard);
                if (std::ranges::none_of(out, [&r](const SensorReading& had) { return had.id == r.id; })) {
                    out.push_back(std::move(r));
                }
            } catch (const std::exception&) { // sensor gone or unreadable this tick
            }
        }
    }
    return true;
}

/*| frame: how long one sensor read may take and still belong in a sweep a caller repeats every
        second or two. Thirteen readings at this bound are a fifth of one poll.
*/
inline constexpr double kBriefSensorLimitMs = 5.0;

/*| frame: which sensors a sweep reads. Brief leaves out the ones the start measured as slow and
        reads the rest; Full reads every sensor the device publishes.
    why: a caller polling for a lock indicator wants a cheap answer several times a second, and
        a caller drawing a GPSDO's own records wants all of them and can afford to ask less
        often. One sweep cannot be both.
*/
enum class SensorSweep { Full, Brief };

// The sensor names one open device publishes, the motherboard's and one direction's frontend's,
// the ids among them one poll can afford, and whether a GPSDO the unit is fitted with is missing
// from them. A USRP's sensor set is fixed while it stays open.
struct SensorNames {
    std::vector<std::string> mboard;
    std::vector<std::string> frontend;
    std::vector<std::string> brief;
    bool                     gpsdoUndetected = false;
};

/*| contract: whether the unit's EEPROM names a GPSDO, "internal" or "onboard", while the open
        device publishes no motherboard sensor whose name starts with gps_.
    why: UHD looks for an N-series GPSDO at the open only while the unit's firmware does not
        hold UHD's do-not-look mark. UHD writes that mark itself, after an open that found no
        GPSDO and after an open that could not identify the unit, and the mark stays until the
        unit is power-cycled. Every open until then comes up with no GPS sensor, with gpsdo
        among neither the clock nor the time sources, and with no line saying so.
    trap: the predicate holds in three states it cannot tell apart: UHD skipped the search,
        UHD searched and the GPSDO did not answer, or the EEPROM names a GPSDO the unit does
        not carry. UHD logs its GPSDO detection at an open that searches, and a power cycle
        makes it search again.
    trap: an open that could not identify the unit writes the mark as well. A GPSDO UHD found
        at every earlier open is then missing from every later one until the power cycle.
    measured: a read of sensorNamesOf and of the property tree through the source's own handle,
        on an N210 whose EEPROM names an internal GPSDO and at an open where UHD logged no GPSDO
        detection, listed mimo_locked and ref_locked alone as motherboard sensors, and
        internal, external and mimo as the clock sources.
    verified-by: sink.uhd-names-a-gpsdo-the-open-did-not-find
*/
inline bool gpsdoUndetected(std::string_view eepromGpsdo, std::span<const std::string> mboardSensors) {
    const bool fitted = eepromGpsdo == "internal" || eepromGpsdo == "onboard";
    return fitted && std::ranges::none_of(mboardSensors, [](const std::string& n) { return n.starts_with("gps_"); });
}

// The GPSDO the unit's EEPROM names, and empty where the device keeps no such field or no tree.
template <typename Usrp>
std::string eepromGpsdo(Usrp& usrp) {
    if constexpr (requires { usrp.get_device(); }) {
        try {
            const auto eeprom = usrp.get_device()->get_tree()->template access<::uhd::usrp::mboard_eeprom_t>("/mboards/0/eeprom").get();
            return eeprom.has_key("gpsdo") ? eeprom["gpsdo"] : std::string{};
        } catch (const std::exception&) { // no tree, or no EEPROM node on this model
        }
    }
    return {};
}

// One frontend sensor of the direction named.
template <typename Usrp>
::uhd::sensor_value_t frontendSensor(Usrp& usrp, Direction dir, const std::string& name) {
    return dir == Direction::Receive ? usrp.get_rx_sensor(name, 0) : usrp.get_tx_sensor(name, 0);
}

/*| contract: the sensor names the open device publishes for one direction, with the ids one read
        of which fits a repeating poll, found by reading every sensor once and timing it against
        limitMs. A live() that answers false ends the timing, and the names found so far come
        back. An enumeration the device refuses is named on standard error under blockTag and
        leaves the names empty, and states nothing about a GPSDO. Where an enumeration succeeds
        and gpsdoUndetected holds, standard error names the GPSDO the EEPROM states under
        blockTag, with the power cycle that makes UHD search for it again.
    why: a sweep a caller repeats every second or two must cost a small part of that, and which
        sensors are slow is a property of the unit rather than of the model: a GPSDO answers
        gps_time from the next time sentence of the oscillator's 1 Hz serial link, while its
        other GPS records and every sensor of a radio without one answer in microseconds.
        Measuring beats a list of names, which would be wrong for the next radio.
    measured: on an N210 with a GPSDO, gps_time cost 999.6 ms a read, the other four GPS
        sensors 0.02 ms or less and the rest under 0.2 ms; a full sweep took 984 ms and a brief
        one 0.8 ms. On a B205mini the whole sweep is 4 ms.
*/
template <typename Usrp, typename Live = AlwaysLive>
SensorNames sensorNamesOf(Usrp& usrp, Direction dir, std::string_view blockTag, double limitMs = kBriefSensorLimitMs, Live&& live = Live{}) {
    SensorNames names;
    try {
        names.mboard   = usrp.get_mboard_sensor_names(0);
        names.frontend = dir == Direction::Receive ? usrp.get_rx_sensor_names(0) : usrp.get_tx_sensor_names(0);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[%s] sensor enumeration: %s (no sensors reported)\n", std::string(blockTag).c_str(), e.what());
        return names;
    }
    if (const std::string fitted = eepromGpsdo(usrp); gpsdoUndetected(fitted, names.mboard)) {
        names.gpsdoUndetected = true;
        std::fprintf(stderr, "[%s] the EEPROM names an %s GPSDO and the open published no GPS sensor; where UHD logged no GPSDO detection at this open, a power cycle of the unit makes it search again\n", std::string(blockTag).c_str(), fitted.c_str());
    }
    for (const auto* list : {&names.mboard, &names.frontend}) {
        const bool mboard = list == &names.mboard;
        for (const std::string& n : *list) {
            if (!live()) {
                return names;
            }
            const auto t0 = std::chrono::steady_clock::now();
            try {
                std::ignore = mboard ? usrp.get_mboard_sensor(n, 0) : frontendSensor(usrp, dir, n);
            } catch (const std::exception&) {
                continue; // unreadable now, and so not one a poll can rely on
            }
            if (std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() <= limitMs) {
                names.brief.push_back(n);
            }
        }
    }
    return names;
}

/*| contract: whether a reading's pass or fail bears on the selection in force: ref_locked on a
        clock source other than internal, mimo_locked where the clock or the time source is
        mimo, and every other reading always. An empty clock source is a radio that states none.
    why: an internal clock is the radio's own, and its reference sensor reads unlocked for want
        of an external reference; a unit with no MIMO cable reads its MIMO lock unlocked. Marked
        by value, both show a fault on a healthy radio.
    verified-by: sink.uhd-marks-a-lock-only-where-it-applies
*/
inline bool lockReadingApplies(std::string_view id, std::string_view clockSource, std::string_view timeSource) {
    if (id == "ref_locked") {
        return !clockSource.empty() && clockSource != "internal";
    }
    if (id == "mimo_locked") {
        return clockSource == "mimo" || timeSource == "mimo";
    }
    return true;
}

/*| contract: append the sensors of one open device that names lists to out, the
        motherboard's and the direction's frontend's, and then the reference and timing state that is not a sensor
        but belongs beside them: clock_source, time_source and time_last_pps, all three brief
        false, the controls stating the two selections. Brief reads only the ids names.brief
        holds. False, with out emptied, where live() answers false before the sweep is through.
        A lock reading the selection in force does not use is published, marked good and kept
        off the glance rows, by lockReadingApplies. Where names.gpsdoUndetected holds, a gpsdo
        reading marked failed says the EEPROM names a GPSDO and the open published no GPS
        sensor, in both sweeps.
    frame: a transmit frontend's sensors carry the tx_ prefix on their ids and TX on their
        labels, because a receive frontend of the same unit names its own the same way: both
        state an lo_locked, and they are two synthesizers. The motherboard's carry the ids the
        unit gives them. A label is operatorLabel's where it names the id, and the unit's own
        otherwise.
    frame: these are control-plane reads over the same transport the settings appliers use, so
        they are safe beside a running stream.
    verified-by: sink.uhd-reads-the-transmit-sensors
*/
template <typename Usrp, typename LiveFn>
bool readDeviceSensors(std::vector<SensorReading>& out, Usrp& usrp, const SensorNames& names, Direction dir, SensorSweep kind, LiveFn&& live) {
    const auto kept = [&names, kind](const std::vector<std::string>& list) -> std::vector<std::string> {
        if (kind == SensorSweep::Full) {
            return list;
        }
        std::vector<std::string> cheap;
        std::ranges::copy_if(list, std::back_inserter(cheap), [&names](const std::string& n) { return std::ranges::find(names.brief, n) != names.brief.end(); });
        return cheap;
    };
    const auto read = [&usrp, dir](const std::string& n, bool mboard) {
        SensorReading r = toReading(n, mboard ? usrp.get_mboard_sensor(n, 0) : frontendSensor(usrp, dir, n));
        if (!mboard && dir == Direction::Transmit) {
            r.id    = "tx_" + n;
            r.label = "TX " + r.label;
        }
        return r;
    };
    if (!sweepSensors(out, kept(names.mboard), kept(names.frontend), read, live)) {
        return false;
    }
    if (!live()) {
        out.clear();
        return false;
    }
    // The reference and the timing edge the radio runs on, which no sensor reports, and the
    // device clock latched at the last PPS edge.
    const auto appendPlain = [&out](const char* id, const char* label, std::string value, bool brief) {
        out.push_back({.id = id, .label = label, .value = std::move(value), .good = true, .brief = brief});
    };
    std::string clockSource;
    std::string timeSource;
    try {
        clockSource = usrp.get_clock_source(0);
        appendPlain("clock_source", "Clock source", clockSource, false);
        timeSource = usrp.get_time_source(0);
        appendPlain("time_source", "Time source", timeSource, false);
        appendPlain("time_last_pps", "Device time at last PPS", std::format("{:.6f} s", usrp.get_time_last_pps(0).get_real_secs()), false);
    } catch (const std::exception&) {
    }
    if (names.gpsdoUndetected) {
        out.push_back({.id = "gpsdo", .label = "GPSDO", .value = "EEPROM names one, no GPS sensor; power-cycle if UHD did not search", .good = false, .brief = true});
    }
    for (SensorReading& r : out) {
        if (!lockReadingApplies(r.id, clockSource, timeSource)) {
            r.good  = true;
            r.brief = false;
        }
    }
    return true;
}

} // namespace capture::uhd
