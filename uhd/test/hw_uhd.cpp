/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the USRP source's guarantees, measured against an attached radio.
    contract: one case per guarantee, selected by name on the command line like every other
        binary here, and every case prints the numbers it measured. A case opens the device
        itself and releases it before it returns, so one process runs one case.
    frame: the graph is the smallest one the source can live in, the source and a counting
        sink on the same scheduler a receiver uses, so the whole path -- start, the consumer
        thread, the publish, the tags, stop, the teardown guard -- runs as a receiver runs it.
    trap: these cases open a radio and are registered under the label hardware, built only
        under CAPTURE_HW_UHD, so the hardware-free suite never reaches one. The device
        selector comes from CAPTURE_HW_UHD_ARGS and names one unit by serial otherwise.
*/
#include <capture/uhd/Sink.hpp>
#include <capture/uhd/Source.hpp>


#include <gnuradio-4.0/Graph.hpp>

#include "support/Cases.hpp"
#include "support/ScheduledGraph.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using CF32      = std::complex<float>;
using Clock     = std::chrono::steady_clock;
using Scheduled = capture::test::ScheduledGraph<gr::scheduler::ExecutionPolicy::singleThreadedBlocking>;

// ---- what the sink reports -----------------------------------------------------------

std::atomic<std::uint64_t> g_samples{0};
std::atomic<std::uint64_t> g_overflowTags{0};
std::atomic<std::uint64_t> g_eosTags{0};
std::atomic<std::uint64_t> g_timingTags{0};
std::atomic<std::uint64_t> g_firstTriggerNs{0};
std::atomic<std::uint64_t> g_firstSampleWallNs{0};
std::atomic<std::int64_t>  g_stallMs{0};
std::atomic<bool>          g_stallTaken{false};
// The rate the block forwarded downstream, which is how a caller's span, decimator and
// recorder hear that the radio moved.
std::atomic<std::uint64_t> g_rateTags{0};
std::atomic<double>        g_lastRateTagHz{0.0};

/*| role: what the arriving samples add up to, over whatever window a case resets around.
    frame: three quantities from one pass — the mean power, the mean sample (the DC
        component, which is where an LO leakage spike sits), and the stream correlated
        against one frequency (the bin a transmitted tone lands in). The bin's phase index
        runs from the reset, because the phase of a received tone is arbitrary and only the
        magnitude of the correlation is read.
    trap: a mutex rather than atomics, since a complex accumulator is two numbers that have
        to move together.
*/
struct Accumulator {
    std::mutex           lock;
    double               power = 0.0;
    std::complex<double> dc{};
    std::complex<double> bin{};
    std::uint64_t        n       = 0;
    std::uint64_t        phase   = 0;
    double               binFreq = 0.0; // cycles per sample

    void reset() {
        std::lock_guard g(lock);
        power = 0.0;
        dc    = {};
        bin   = {};
        n     = 0;
        phase = 0;
    }
    void setBinFreq(double cyclesPerSample) {
        std::lock_guard g(lock);
        binFreq = cyclesPerSample;
    }
    void add(std::span<const CF32> v) {
        std::lock_guard g(lock);
        for (std::size_t i = 0; i < v.size(); ++i) {
            const std::complex<double> s(v[i].real(), v[i].imag());
            power += s.real() * s.real() + s.imag() * s.imag();
            dc += s;
            if (binFreq != 0.0) {
                const double ph = -2.0 * std::numbers::pi * binFreq * static_cast<double>(phase + i);
                bin += s * std::complex<double>(std::cos(ph), std::sin(ph));
            }
        }
        phase += v.size();
        n += v.size();
    }
};
Accumulator g_accum;

/*| frame: where the timing tags fell — the time each carried, the stream index it sat at,
        and the host clock as the sink took delivery of it. A tag's time is judged against
        the third, because the host clock at the first sample of the whole stream is
        a different moment from the one any later tag names.
*/
struct TagRecord {
    std::uint64_t timeNs     = 0;
    std::size_t   index      = 0;
    std::uint64_t seenWallNs = 0;
};
std::mutex               g_tagLock;
std::vector<TagRecord>   g_tagTimes;
std::vector<std::size_t> g_markerAt; // the stream index of every gap marker

void resetCounters() {
    g_samples.store(0);
    g_overflowTags.store(0);
    g_eosTags.store(0);
    g_timingTags.store(0);
    g_firstTriggerNs.store(0);
    g_firstSampleWallNs.store(0);
    g_stallMs.store(0);
    g_stallTaken.store(false);
    g_rateTags.store(0);
    g_lastRateTagHz.store(0.0);
    g_accum.reset();
    g_accum.setBinFreq(0.0);
    std::lock_guard g(g_tagLock);
    g_tagTimes.clear();
    g_markerAt.clear();
}

void resetPower() { g_accum.reset(); }

// Mean power of what has arrived since the last reset, in dB below full scale.
double meanDbfs() {
    std::lock_guard g(g_accum.lock);
    return g_accum.n == 0UL ? -300.0 : 10.0 * std::log10(g_accum.power / static_cast<double>(g_accum.n) + 1e-30);
}

// The DC component of what has arrived, which is where an LO leakage spike sits.
double dcDbfs() {
    std::lock_guard g(g_accum.lock);
    return g_accum.n == 0UL ? -300.0 : 20.0 * std::log10(std::abs(g_accum.dc) / static_cast<double>(g_accum.n) + 1e-30);
}

// The one bin a transmitted tone lands in. Reading that bin alone makes a weak tone visible
// in a noise floor a whole-band mean cannot separate it from.
double binDbfs() {
    std::lock_guard g(g_accum.lock);
    return g_accum.n == 0UL ? -300.0 : 20.0 * std::log10(std::abs(g_accum.bin) / static_cast<double>(g_accum.n) + 1e-30);
}

void countTag(const gr::property_map& map, std::size_t index) {
    for (const auto& [key, value] : map) {
        const std::string_view k(key.data(), key.size());
        if (k == "rx_overflow" || k == "gr:rx_overflow") {
            g_overflowTags.fetch_add(1, std::memory_order_relaxed);
            std::lock_guard g(g_tagLock);
            g_markerAt.push_back(index);
        } else if (k == "end_of_stream" || k == "gr:end_of_stream") {
            g_eosTags.fetch_add(1, std::memory_order_relaxed);
        } else if (k == "trigger_time" || k == "gr:trigger_time") {
            if (const std::uint64_t* t = value.get_if<std::uint64_t>(); t != nullptr) {
                std::uint64_t none = 0UL;
                g_firstTriggerNs.compare_exchange_strong(none, *t);
                std::lock_guard g(g_tagLock);
                g_tagTimes.push_back({.timeNs = *t, .index = index, .seenWallNs = capture::wallClockNs()});
            }
            g_timingTags.fetch_add(1, std::memory_order_relaxed);
        } else if (k == "sample_rate" || k == "gr:sample_rate") {
            g_rateTags.fetch_add(1, std::memory_order_relaxed);
            if (const double* r = value.get_if<double>(); r != nullptr) {
                g_lastRateTagHz.store(*r, std::memory_order_relaxed);
            } else if (const float* f = value.get_if<float>(); f != nullptr) {
                g_lastRateTagHz.store(static_cast<double>(*f), std::memory_order_relaxed);
            }
        }
    }
}

/*| role: the host, as a block: it takes everything and counts what it took.
    frame: the power accumulator runs per span rather than per sample, so a case that asks
        for a mean level pays one atomic add per block.
    trap: the one-shot stall sleeps on the scheduler thread, which is the thread the source's
        work() runs on, so the source's publish blocks for exactly as long as a host that has
        stopped draining would block it.
*/
struct [[maybe_unused]] hw_sink : gr::Block<hw_sink> {
    gr::PortIn<CF32> in;
    GR_MAKE_REFLECTABLE(hw_sink, in);

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            countTag(tag.map, tag.index);
        }
        const std::size_t n = inSpan.size();
        if (n == 0UZ) {
            return gr::work::Status::OK;
        }
        if (g_samples.load(std::memory_order_relaxed) == 0UL) {
            g_firstSampleWallNs.store(capture::wallClockNs(), std::memory_order_relaxed);
        }
        g_accum.add(std::span<const CF32>(inSpan.data(), n));
        g_samples.fetch_add(n, std::memory_order_relaxed);

        if (const std::int64_t ms = g_stallMs.load(std::memory_order_relaxed); ms > 0 && !g_stallTaken.exchange(true)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
        return gr::work::Status::OK;
    }
};

// ---- the rig -------------------------------------------------------------------------

const std::string& deviceArgs() {
    static const std::string args = [] {
        const char* v = std::getenv("CAPTURE_HW_UHD_ARGS");
        return v != nullptr && *v != '\0' ? std::string(v) : std::string("serial=30D3AB9");
    }();
    return args;
}

/*| contract: the connector an antenna is actually wired to on this bench, or empty where
        none is. CAPTURE_HW_UHD_ANTENNA names it.
    why: no device call says which of its ports has an antenna on it, and a case that
        compares received levels between ports is asserting a fact about the bench rather
        than about the radio. Unnamed, such a comparison is printed and not asserted.
*/
const std::string& benchAntenna() {
    static const std::string port = [] {
        const char* v = std::getenv("CAPTURE_HW_UHD_ANTENNA");
        return v != nullptr ? std::string(v) : std::string();
    }();
    return port;
}

// The port a case receives on: the bench's antenna where it named one, and the device's own
// current selection otherwise.
std::string receivePort(const capture::uhd::DeviceTruth& truth) {
    if (!benchAntenna().empty()) {
        return benchAntenna();
    }
    return truth.antenna.empty() ? std::string("RX2") : truth.antenna;
}

// The device's own answers, read once per case while nothing holds the radio.
capture::uhd::DeviceTruth probeAt(double rateHz) {
    capture::uhd::DeviceTruth truth;
    if (!capture::uhd::Source::probeTruth(deviceArgs(), rateHz, truth)) {
        std::fprintf(stderr, "the device did not probe\n");
    }
    return truth;
}

/*| contract: at most n entries of a ladder, its first and last among them and the rest
        spread evenly between.
    why: a fixed-clock radio states hundreds of reachable rates and a case that opened the
        device once per entry would run for twenty minutes. The ends are where the
        interesting failures are and the middle is sampled.
*/
std::vector<double> spreadOver(const std::vector<double>& ladder, std::size_t n) {
    if (ladder.size() <= n || n == 0UZ) {
        return ladder;
    }
    std::vector<double> out;
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(ladder[i * (ladder.size() - 1UZ) / (n - 1UZ)]);
    }
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

// Whether the radio's converter clock can be moved, which decides what a rate ladder means.
bool clockIsProgrammable(const capture::uhd::DeviceTruth& t) { return t.mclkMaxHz > t.mclkMinHz; }

/*| contract: the device's own answers at 2 MS/s, probed once for the whole process. One
        process runs one case, so this is one open per case, made before the graph starts.
*/
const capture::uhd::DeviceTruth& benchTruth() {
    static const capture::uhd::DeviceTruth truth = probeAt(2.0e6);
    return truth;
}

// The port every case receives on, from the bench and the device.
std::string sourcePort() { return receivePort(benchTruth()); }

/*| contract: a frequency inside the coverage the device stated, given as a fraction of the
        way across its widest range. 0 is the bottom edge and 1 the top.
    why: a case that names a frequency picks one the radio it was written for reaches; the
        next radio's tuner stops somewhere else. Asking for a place inside the stated
        coverage reaches both.
*/
double insideCoverage(const capture::uhd::DeviceTruth& t, double fraction) {
    double lo = 0.0;
    double hi = 0.0;
    for (const auto& [a, b] : t.freqRanges) {
        if (b - a > hi - lo) {
            lo = a;
            hi = b;
        }
    }
    return lo + fraction * (hi - lo);
}

/*| contract: a frequency inside the coverage this radio stated, preferring the one given
        where the coverage holds it. Cases that only need somewhere legal to sit take it.
*/
double insideOrElse(double preferredHz) {
    for (const auto& [lo, hi] : benchTruth().freqRanges) {
        if (preferredHz >= lo && preferredHz <= hi) {
            return preferredHz;
        }
    }
    return insideCoverage(benchTruth(), 0.25);
}

// Where the cases that only need a legal tune sit, and where the ones that want a band
// both radios also transmit in sit.
double defaultFreq() { return insideOrElse(100.0e6); }
double bandFreq() { return insideOrElse(915.0e6); }

/*| contract: a frequency below everything the device stated and one above it, each by a
        stated margin, and nothing at all where the device stated no coverage.
    why: a fraction of the bottom is not below a coverage whose bottom is at or below zero,
        which a basic or low-frequency daughterboard has once the digital down-converter
        widens its range by half the rate: half of a negative number is above it, and the
        case then asserts that a tune it expected to be refused left the tuner alone.
*/
constexpr double kCoverageMarginHz = 10.0e6;

std::optional<std::pair<double, double>> outsideCoverage(const capture::uhd::DeviceTruth& t) {
    if (t.freqRanges.empty()) {
        return std::nullopt;
    }
    double lo = std::numeric_limits<double>::max();
    double hi = std::numeric_limits<double>::lowest();
    for (const auto& [a, b] : t.freqRanges) {
        lo = std::min(lo, a);
        hi = std::max(hi, b);
    }
    return std::pair{lo - kCoverageMarginHz, hi + kCoverageMarginHz};
}

gr::property_map sourceSettings(double rateHz, double freqHz, double gainDb, const std::string& antenna) {
    return {
        {"name", std::string("src")},
        {"device", std::string("uhd")},
        {"device_parameter", deviceArgs()},
        {"sample_rate", rateHz},
        {"frequency", std::vector<double>{freqHz}},
        {"rx_gains", std::vector<double>{gainDb}},
        {"rx_antennae", std::vector<std::string>{antenna}},
        {"tag_interval", 0.5f},
    };
}

/*| contract: read one value off the open device through the block's own handle, under the
        block's own control mutex, so the read cannot race a teardown that resets the handle.
    why: what the device holds and what the block was asked for are different numbers, and a
        guarantee about the first is only testable against the first.
*/
template <typename R, typename Fn>
R deviceRead(capture::uhd::Source& src, R fallback, Fn&& fn) {
    std::lock_guard lock(src._ctrlMutex);
    if (src._usrp == nullptr) {
        return fallback;
    }
    try {
        return fn(*src._usrp);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "device read failed: %s\n", e.what());
        return fallback;
    }
}

bool waitForSamples(std::chrono::milliseconds limit) {
    const auto deadline = Clock::now() + limit;
    while (Clock::now() < deadline) {
        if (g_samples.load(std::memory_order_relaxed) > 0UL) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

/*| contract: true once the sink states its device up, false when the limit passes first.
    measured: a transmit start takes 1.46 to 1.52 s on a B205mini and 1.97 to 2.02 s on an N210.
*/
bool waitForDeviceUp(const capture::uhd::Sink& sink, std::chrono::milliseconds limit) {
    const auto deadline = Clock::now() + limit;
    while (Clock::now() < deadline) {
        if (sink.deviceUp()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

void settle(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Run body(source) against a started graph, then stop it and release the device.
template <typename Body>
bool withGraph(gr::property_map settings, Body&& body) {
    resetCounters();
    gr::Graph flow;
    auto&     src  = flow.emplaceBlock<capture::uhd::Source>(std::move(settings));
    auto&     sink = flow.emplaceBlock<hw_sink>({{"name", std::string("host")}});
    if (!flow.connect<"out", "in">(src, sink).has_value()) {
        std::fprintf(stderr, "the source does not wire to the sink\n");
        return false;
    }
    Scheduled scheduled;
    if (const auto adopted = scheduled.adopt(std::move(flow)); !adopted.has_value()) {
        std::fprintf(stderr, "the graph did not build: %s\n", adopted.error().message.c_str());
        return false;
    }
    scheduled.start();
    body(src, scheduled);
    scheduled.stop();
    /*| trap: a consumer never sees the end-of-stream marker through its input span. The
            marker sits at the index the last sample ended on, so the scheduler finds zero
            samples ahead of it and ends the block without one more call. The consumer's own
            tag ring is where the marker can be counted.
    */
    auto tags   = sink.in.tagReader().get();
    std::ignore = tags.consume(0UZ);
    for (const gr::Tag& tag : tags) {
        for (const auto& [key, value] : tag.map) {
            const std::string_view k(key.data(), key.size());
            if ((k == "end_of_stream" || k == "gr:end_of_stream") && value.get_if<bool>() != nullptr) {
                g_eosTags.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
    return true;
}

// True when the device can be opened, which after a stop means the block released it.
bool deviceOpens() {
    try {
        auto usrp = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
        return usrp != nullptr;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "reopen failed: %s\n", e.what());
        return false;
    }
}

std::vector<std::string> ownThreadNames() {
    std::vector<std::string> names;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) {
        std::ifstream comm(entry.path() / "comm");
        std::string   name;
        if (std::getline(comm, name)) {
            names.push_back(name);
        }
    }
    return names;
}

double dbfsOver(double seconds) {
    resetPower();
    settle(static_cast<int>(seconds * 1000.0));
    return meanDbfs();
}

/*| role: the graph end of a transmit case: a complex tone at a fixed offset from the center.
    frame: -20 dBFS is an amplitude of 0.1, which is where these cases transmit. The tone sits
        off the center bin so a receiver can tell it from the oscillator's leakage there.
*/
struct [[maybe_unused]] hw_tone : gr::Block<hw_tone> {
    gr::PortOut<CF32> out;

    gr::Annotated<double, "cycles_per_sample", gr::Doc<"tone offset in cycles per sample">> cycles_per_sample = 0.125;
    gr::Annotated<float, "amplitude", gr::Doc<"peak amplitude of each part">>               amplitude         = 0.1f;

    GR_MAKE_REFLECTABLE(hw_tone, out, cycles_per_sample, amplitude);

    std::uint64_t _n = 0;

    CF32 processOne() {
        const double phase = 2.0 * std::numbers::pi * cycles_per_sample * static_cast<double>(_n++);
        return {static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase))};
    }
};

/*| contract: the settings a transmit case gives the sink: this bench's own unit, the center it
        asks for, the connector the bench names, and 0 dB.
    invariant: 0 dB and nothing else. The port is terminated and the gain is never raised.
*/
gr::property_map sinkSettings(double rateHz, double freqHz, const std::string& antenna) {
    return {
        {"name", std::string("tx")},
        {"device", std::string("uhd")},
        {"device_parameter", deviceArgs()},
        {"sample_rate", rateHz},
        {"frequency", std::vector<double>{freqHz}},
        {"tx_gains", std::vector<double>{0.0}},
        {"tx_antennae", std::vector<std::string>{antenna}},
    };
}

/*| role: the graph end of the tagged-burst case: the -20 dBFS tone of hw_tone in bursts of
        burst_samples each, tx_sob on each burst's first sample and tx_eob on its last, and
        nothing after the last burst.
    frame: the second burst's first sample also carries tx_time: the device time deviceNowNs
        answers when the sample is made, plus leadNs. The lead covers the transmit queue and
        the graph's buffer, which hold under 0.3 s at 2 MS/s, so the time is still ahead when
        the send reaches the device.
*/
struct [[maybe_unused]] hw_burst_tone : gr::Block<hw_burst_tone> {
    gr::PortOut<CF32> out;
    GR_MAKE_REFLECTABLE(hw_burst_tone, out);

    std::size_t                                  burstSamples = 200000UZ;
    std::size_t                                  bursts       = 3UZ;
    std::int64_t                                 leadNs       = 500'000'000;
    std::function<std::optional<std::int64_t>()> deviceNowNs;
    std::optional<std::int64_t>                  timedAtNs; // the time the second burst carried
    std::uint64_t                                _n = 0;

    gr::work::Status processBulk(gr::OutputSpanLike auto& output) {
        const std::uint64_t total = static_cast<std::uint64_t>(burstSamples) * bursts;
        if (_n >= total) {
            output.publish(0UZ);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return gr::work::Status::OK;
        }
        const std::size_t count = static_cast<std::size_t>(std::min<std::uint64_t>(output.size(), total - _n));
        for (std::size_t i = 0UZ; i < count; ++i) {
            const std::uint64_t index = _n + i;
            const double        phase = 2.0 * std::numbers::pi * 0.125 * static_cast<double>(index);
            output[i]                 = CF32{static_cast<float>(0.1 * std::cos(phase)), static_cast<float>(0.1 * std::sin(phase))};
            const std::uint64_t at    = index % burstSamples;
            gr::property_map    tag;
            if (at == 0U) {
                tag["tx_sob"] = true;
                if (index / burstSamples == 1U && deviceNowNs) {
                    if (const auto now = deviceNowNs(); now.has_value()) {
                        timedAtNs      = *now + leadNs;
                        tag["tx_time"] = *timedAtNs;
                    }
                }
            }
            if (at + 1U == burstSamples) {
                tag["tx_eob"] = true;
            }
            if (!tag.empty()) {
                output.publishTag(tag, i);
            }
        }
        output.publish(count);
        _n += count;
        return gr::work::Status::OK;
    }
};

// Run body(sink) against a started transmit graph, then stop it and release the device.
template <typename Body>
bool withTxGraph(gr::property_map settings, Body&& body) {
    gr::Graph flow;
    auto&     tone = flow.emplaceBlock<hw_tone>({{"name", std::string("tone")}});
    auto&     sink = flow.emplaceBlock<capture::uhd::Sink>(std::move(settings));
    if (!flow.connect<"out", "in">(tone, sink).has_value()) {
        std::fprintf(stderr, "the tone does not wire to the sink\n");
        return false;
    }
    Scheduled scheduled;
    if (const auto adopted = scheduled.adopt(std::move(flow)); !adopted.has_value()) {
        std::fprintf(stderr, "the transmit graph did not build: %s\n", adopted.error().message.c_str());
        return false;
    }
    scheduled.start();
    body(sink, scheduled);
    scheduled.stop();
    return true;
}

// What the source counts as shed, where the accessor exists at all.
template <typename Src>
long long droppedOf(const Src& s) {
    if constexpr (requires { s.droppedSamples(); }) {
        return static_cast<long long>(s.droppedSamples());
    } else {
        return -1;
    }
}

} // namespace

int main(int argc, char** argv) {
    using namespace boost::ut;
    capture::test::Cases cases(argc, argv);

    cases("uhd.open-stream-close", [] {
        constexpr double kRate = 2.0e6;
        std::uint64_t    count = 0;
        double           rate  = 0.0;
        std::uint64_t    firstTrigger = 0;
        std::uint64_t    firstSample  = 0;

        const bool ran = withGraph(sourceSettings(kRate, defaultFreq(), 38.0, sourcePort()), [&](capture::uhd::Source&, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            const std::uint64_t at0 = g_samples.load();
            const auto          t0  = Clock::now();
            settle(2000);
            const std::uint64_t at1 = g_samples.load();
            const auto          t1  = Clock::now();
            count                   = at1 - at0;
            rate                    = static_cast<double>(count) / std::chrono::duration<double>(t1 - t0).count();
            firstTrigger            = g_firstTriggerNs.load();
            firstSample             = g_firstSampleWallNs.load();
        });
        expect(ran) << "the graph ran";

        const double offMs = (static_cast<double>(firstTrigger) - static_cast<double>(firstSample)) / 1e6;
        std::printf("open-stream-close: %llu samples in 2 s, %.1f S/s achieved (%.3f %% off %.0f), timing tags %llu, "
                    "first tag %.1f ms from the wall clock at the first sample, eos tags %llu\n",
                    static_cast<unsigned long long>(count), rate, 100.0 * (rate - kRate) / kRate, kRate,
                    static_cast<unsigned long long>(g_timingTags.load()), offMs, static_cast<unsigned long long>(g_eosTags.load()));

        expect(std::abs(rate - kRate) <= 0.02 * kRate) << "the delivered rate is within 2 percent of the requested one";
        expect(g_timingTags.load() >= 1UL) << "at least one timing tag";
        expect(std::abs(offMs) < 100.0) << "the tag's time is within 100 ms of the wall clock at the first sample";
        expect(g_eosTags.load() == 1UL) << "exactly one end-of-stream tag after stop";
        expect(deviceOpens()) << "the device was released";
    });

    cases("uhd.rate-ladder", [] {
        /*| frame: the ladder the device itself states, not a list written for one model. A
                fixed-clock radio states hundreds of entries, so the ones opened are spread
                across it with both ends among them.
        */
        const auto& truth = benchTruth();
        const auto  ladder = capture::uhd::Source::sampleRatesFor(truth);
        const auto  rates  = spreadOver(ladder, 10UZ);
        bool        everyRateStreamed = true;
        bool        everyRateLanded   = true;

        for (const double want : rates) {
            double        achieved = 0.0;
            std::uint64_t got      = 0;
            std::uint64_t overflow = 0;
            double        seconds  = 0.0;

            const bool ran = withGraph(sourceSettings(want, defaultFreq(), 38.0, sourcePort()), [&](capture::uhd::Source& src, auto&) {
                if (!waitForSamples(std::chrono::milliseconds(6000))) {
                    return;
                }
                achieved                 = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
                const std::uint64_t at0  = g_samples.load();
                const std::uint64_t ov0  = g_overflowTags.load();
                const auto          t0   = Clock::now();
                settle(1000);
                got      = g_samples.load() - at0;
                overflow = g_overflowTags.load() - ov0;
                seconds  = std::chrono::duration<double>(Clock::now() - t0).count();
            });

            const double delivered = seconds > 0.0 ? static_cast<double>(got) / seconds : 0.0;
            std::printf("rate-ladder: requested %10.0f  get_rx_rate %10.0f  delivered %10.0f S/s  overflow tags %4llu\n", want, achieved, delivered, static_cast<unsigned long long>(overflow));
            if (!ran || got == 0UL) {
                everyRateStreamed = false;
            }
            if (achieved <= 0.0 || std::abs(achieved - want) > 0.01 * want) {
                everyRateLanded = false;
            }
        }
        expect(everyRateStreamed) << "every rate the ladder names delivered samples";
        expect(everyRateLanded) << "and UHD landed on each within one percent, refusing none";

        double statedTop = 0.0;
        for (const auto& [lo, hi] : truth.rates) {
            statedTop = std::max(statedTop, hi);
        }
        std::printf("rate-ladder: the device states %zu rate sub-ranges topping out at %.0f S/s, a master clock range of %.0f to %.0f Hz and a clock of %.0f Hz\n", truth.rates.size(), statedTop,
                    truth.mclkMinHz, truth.mclkMaxHz, truth.mclkHz);
        std::printf("rate-ladder: the ladder offered has %zu entries, %.0f to %.0f S/s; the %zu opened were", ladder.size(), ladder.empty() ? 0.0 : ladder.front(), ladder.empty() ? 0.0 : ladder.back(),
                    rates.size());
        for (const double r : rates) {
            std::printf(" %.0f", r);
        }
        std::printf("\n");
        expect(!ladder.empty()) << "the device states a ladder";

        if (clockIsProgrammable(truth)) {
            /*| frame: a radio whose clock UHD picks per rate. The stated set is the ladder
                    of whichever clock the probe caught and moves with the rate, so the picks
                    stand and the clock range bounds them.
            */
            std::printf("rate-ladder: this radio's clock is programmable, so the ladder is the picks\n");
            expect(ladder == capture::uhd::Source::sampleRates("type=b200")) << "the ladder is the picks and not the stated set";
            for (const double r : ladder) {
                expect(r >= truth.mclkMinHz / capture::uhd::kMaxDecimation && r <= truth.mclkMaxHz) << "every entry is inside what the master clock reaches";
            }
            expect(statedTop < ladder.back()) << "the stated set stops below the ladder's top, which is why it is not the ladder";
        } else {
            /*| frame: a fixed clock. Every reachable rate is that clock over a decimation,
                    and an odd one runs the CIC alone, so the ladder is the even divisors the
                    link carries at the rate UHD states for it, or all of them where UHD
                    states none.
            */
            const bool   stated  = truth.linkBytesPerS > 0.0;
            const double ceiling = stated ? capture::uhd::linkCeilingHz("sc16", truth.linkBytesPerS) : std::numeric_limits<double>::max();
            std::printf("rate-ladder: this radio's clock is fixed at %.0f Hz, so the ladder is its even divisors at or below %.0f S/s, the link ceiling UHD states (0: none)\n", truth.mclkMinHz, stated ? ceiling : 0.0);
            for (const double r : ladder) {
                const double decimation = truth.mclkMinHz / r;
                expect(std::abs(decimation - std::round(decimation)) < 1e-6) << "every entry is a whole divisor of the clock";
                expect(std::fmod(std::round(decimation), 2.0) == 0.0) << "an even one, so the halfband filter is in the chain";
                expect(r <= ceiling) << "and one the link carries";
            }
            if (stated) {
                expect(ladder.back() <= ceiling && ladder.back() > ceiling / 2.0) << "the top entry is the highest the link carries";
                expect(statedTop > ladder.back()) << "the stated set reaches past the link, which is why it is not the ladder";
            } else {
                expect(ladder.back() == truth.mclkMinHz) << "with no link rate stated the top entry is the clock itself";
            }
        }
    });

    cases("uhd.tuning-and-range", [] {
        /*| frame: four places inside the coverage this radio stated, and one below it and
                one above. A named frequency would be inside one radio's tuner and outside
                the next one's, and the case would then be measuring the bench.
        */
        const auto&               truth   = benchTruth();
        const auto                outside = outsideCoverage(truth);
        expect(outside.has_value()) << "the device states a coverage to be outside of";
        if (!outside.has_value()) {
            std::printf("tuning: this radio stated no coverage, so there is no frequency outside it to refuse\n");
            return;
        }
        const double              belowHz = outside->first;
        const double              aboveHz = outside->second;
        const std::vector<double> wanted{insideCoverage(truth, 0.05), insideCoverage(truth, 0.25), insideCoverage(truth, 0.6), insideCoverage(truth, 0.95)};
        bool                      everyTuneLanded    = true;
        bool                      everyForwardHeld   = true;
        double                    worstHz            = 0.0;
        double                    worstForwardHz     = 0.0;
        double                    beforeRefused      = 0.0;
        double                    afterLow           = 0.0;
        double                    afterHigh          = 0.0;
        double                    forwardedLow       = 0.0;

        const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), 38.0, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            for (const double f : wanted) {
                std::ignore = src.settings().setStaged({{"frequency", std::vector<double>{f}}});
                settle(400);
                const double held      = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
                const double forwarded = src.frequency->empty() ? 0.0 : src.frequency->front();
                std::printf("tuning: requested %12.0f Hz  get_rx_freq %14.3f Hz  forwarded %14.3f Hz  off by %.3f Hz\n", f, held, forwarded, held - f);
                worstHz        = std::max(worstHz, std::abs(held - f));
                worstForwardHz = std::max(worstForwardHz, std::abs(forwarded - held));
                if (std::abs(held - f) > 1.0) {
                    everyTuneLanded = false;
                }
                if (std::abs(forwarded - held) > 1.0) {
                    everyForwardHeld = false;
                }
            }

            beforeRefused = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
            std::ignore   = src.settings().setStaged({{"frequency", std::vector<double>{belowHz}}});
            settle(400);
            afterLow     = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
            forwardedLow = src.frequency->empty() ? 0.0 : src.frequency->front();

            std::ignore = src.settings().setStaged({{"frequency", std::vector<double>{aboveHz}}});
            settle(400);
            afterHigh = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
        });
        expect(ran) << "the graph ran";

        std::printf("tuning: the stated coverage runs %.0f to %.0f Hz; worst in-range error %.3f Hz, worst gap between what is forwarded and what the tuner holds %.3f Hz\n", insideCoverage(truth, 0.0),
                    insideCoverage(truth, 1.0), worstHz, worstForwardHz);
        std::printf("tuning: before a %.0f Hz request the tuner held %.0f Hz, after it %.0f Hz (forwarded %.0f Hz); after a %.0f Hz request %.0f Hz\n", belowHz, beforeRefused, afterLow, forwardedLow,
                    aboveHz, afterHigh);

        expect(everyForwardHeld) << "what is forwarded is what the tuner holds, at every frequency";
        expect(everyTuneLanded) << "every in-range request lands within 1 Hz";
        expect(std::abs(afterLow - beforeRefused) < 1.0) << "a request below the coverage leaves the tuner where it was";
        expect(std::abs(afterHigh - beforeRefused) < 1.0) << "and so does one above it";
        expect(std::abs(forwardedLow - beforeRefused) < 1.0) << "what is forwarded is what the tuner holds";
    });

    cases("uhd.frequency-correction-sign", [] {
        const double     kFreq = insideOrElse(1.0e9);
        constexpr double kPpm  = 100.0;
        double           plain = 0.0;
        double           bent  = 0.0;

        const bool ran = withGraph(sourceSettings(2.0e6, kFreq, 38.0, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            plain       = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
            std::ignore = src.settings().setStaged({{"frequency_correction", kPpm}});
            settle(400);
            bent = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
        });
        expect(ran) << "the graph ran";

        /*| frame: the request divided by one plus the fraction, which is the form both USRP
                blocks apply. The first-order product sits 10.0 Hz from it at this test point
                and 60 Hz at 6 GHz, so the window here is the tuner's own resolution, the one
                uhd.tuning holds an in-range request to: a window of 100 Hz passes under either
                form and tells the two apart nowhere. */
        const double expected = kFreq / (1.0 + kPpm / 1e6);
        std::printf("correction: %.0f ppm at %.0f Hz tunes %.1f Hz (expected %.1f Hz, off by %.1f Hz; the product form would be %.1f Hz); uncorrected %.1f Hz\n", kPpm, kFreq, bent, expected,
                    bent - expected, kFreq * (1.0 - kPpm / 1e6), plain);
        expect(std::abs(bent - expected) < 1.0) << "a positive ppm tunes low, by the exact scale rather than its first-order product";
        expect(bent < plain) << "and below the uncorrected tune";
    });

    cases("uhd.gain", [] {
        /*| frame: the element names and the ranges are the device's own. One radio calls its
                one element PGA over 0 to 76 dB and another calls three of them ADC-digital,
                ADC-fine and PGA0 with an overall range of 0 to 38 dB, so a case naming an
                element or a ceiling measures the radio it was written for and nothing else.
        */
        const auto& truth = benchTruth();
        expect(!truth.gains.empty()) << "the device names its gain elements";
        if (truth.gains.empty()) {
            return;
        }
        // The widest element, which is the one that carries the received level.
        const auto& widest = *std::ranges::max_element(truth.gains, {}, [](const auto& g) { return g.max - g.min; });
        std::printf("gain: the overall range is %.1f to %.1f dB step %.1f, over %zu element(s)\n", truth.gainMinDb, truth.gainMaxDb, truth.gainStepDb, truth.gains.size());
        for (const auto& g : truth.gains) {
            std::printf("gain: element \"%s\" %.1f to %.1f dB step %.1f, now %.1f dB\n", g.name.c_str(), g.min, g.max, g.step, g.current);
        }

        const std::vector<double> steps{truth.gainMinDb, (truth.gainMinDb + truth.gainMaxDb) / 2.0, truth.gainMaxDb};
        const std::vector<double> elementSteps{widest.min, (widest.min + widest.max) / 2.0, widest.max};
        std::vector<double>       readBackFromSetting;
        std::vector<double>       readBackFromElement;
        std::vector<double>       powerDbfs;
        bool                      everyElementWritten = true;
        bool                      unknownRefused      = true;
        double                    readBackUs          = 0.0;

        const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            for (const double g : steps) {
                std::ignore = src.settings().setStaged({{"rx_gains", std::vector<double>{g}}});
                settle(400);
                readBackFromSetting.push_back(src.elementGain({})); // the empty name is the overall gain
                powerDbfs.push_back(dbfsOver(0.5));
            }
            for (const double g : elementSteps) {
                if (!src.setElementGain(widest.name, g)) {
                    everyElementWritten = false;
                }
                settle(200);
                readBackFromElement.push_back(src.elementGain(widest.name));
            }
            // A name no device lists is refused rather than written somewhere near.
            unknownRefused = !src.setElementGain("NO_SUCH_ELEMENT", 0.0);
            /*| frame: the read-back goes to the device under the control mutex, and a
                    receiver's interface polls it, so its cost sits between every tune and
                    every gain write that lands in the same window.
            */
            const auto t0 = Clock::now();
            for (int i = 0; i < 200; ++i) {
                std::ignore = src.elementGain(widest.name);
            }
            readBackUs = std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / 200.0;
        });
        expect(ran) << "the graph ran";

        for (std::size_t i = 0; i < steps.size(); ++i) {
            std::printf("gain: overall asked %5.1f dB  reads %5.1f dB  mean power %7.2f dBFS\n", steps[i], readBackFromSetting[i], powerDbfs[i]);
        }
        for (std::size_t i = 0; i < elementSteps.size(); ++i) {
            std::printf("gain: element \"%s\" asked %5.1f dB  reads %5.1f dB\n", widest.name.c_str(), elementSteps[i], readBackFromElement[i]);
        }
        std::printf("gain: one elementGain read-back costs %.1f us against a streaming radio\n", readBackUs);

        expect(everyElementWritten) << "every write of an element the device named is accepted";
        expect(unknownRefused) << "and a name it did not name is refused";
        for (std::size_t i = 0; i < steps.size(); ++i) {
            expect(std::abs(readBackFromSetting[i] - steps[i]) < truth.gainStepDb + 0.01) << "the overall gain setting reads back within a step";
        }
        for (std::size_t i = 0; i < elementSteps.size(); ++i) {
            expect(std::abs(readBackFromElement[i] - elementSteps[i]) < widest.step + 0.01) << "and so does the element write";
        }
        expect(powerDbfs[1] > powerDbfs[0] + 1.0) << "the received level rises with the gain";
        expect(powerDbfs[2] > powerDbfs[1] + 1.0) << "and again at the top of the range";

        // The control surface the probe states, read while nothing holds the device.
        std::printf("probe: product \"%s\", antenna \"%s\", bandwidth %.0f to %.0f Hz (now %.0f)\n", truth.product.c_str(), truth.antenna.c_str(), truth.bwMinHz, truth.bwMaxHz, truth.bwCurHz);
        for (const auto& [lo, hi] : truth.freqRanges) {
            std::printf("probe: frequency range %.0f to %.0f Hz\n", lo, hi);
        }
        for (const auto& a : truth.antennas) {
            std::printf("probe: antenna \"%s\"\n", a.c_str());
        }
        for (const auto& [label, devstr] : capture::uhd::Source::enumerateDevices()) {
            std::printf("probe: enumerated \"%s\" as \"%s\"\n", label.c_str(), devstr.c_str());
        }
        const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = 2.0e6, .centerFreq = defaultFreq()};
        const auto                  controls = capture::uhd::Source::describeControls(truth, ctx);

        std::size_t gainElements = 0;
        for (const auto& c : controls) {
            std::printf("control: %-12s %-38s kind %d  %.3f to %.3f step %.3f  gain %s  rateDerived %s\n", c.id.c_str(), c.label.c_str(), c.kind, c.min, c.max, c.step, c.isGain ? "yes" : "no",
                        c.rateDerived ? "yes" : "no");
            if (!c.isGain) {
                continue;
            }
            ++gainElements;
            const auto element = std::ranges::find_if(truth.gains, [&c](const auto& g) { return g.name == c.id; });
            expect(element != truth.gains.end()) << "every gain control is an element the device named";
            if (element != truth.gains.end()) {
                expect(c.min == element->min && c.max == element->max && c.step == element->step) << "over the range the device stated for it";
            }
        }
        expect(gainElements == truth.gains.size()) << "one gain control per element the device names";

        const auto named = [&controls](const char* id) {
            return std::ranges::find_if(controls, [id](const auto& c) { return c.id == id; });
        };
        const auto bw = named("BANDWIDTH");
        if (truth.bwMaxHz > truth.bwMinHz) {
            expect(bw != controls.end()) << "a filter with room in it is a control";
            expect(std::abs(bw->min - truth.bwMinHz / 1e6) < 0.01) << "over the range the device stated";
            expect(std::abs(bw->max - truth.bwMaxHz / 1e6) < 0.01);
            expect(bw->rateDerived) << "and it re-derives from the rate";
        } else {
            expect(bw == controls.end()) << "a filter with one width is not a control a caller can move";
        }
        expect((named("DC_CORR") != controls.end()) == truth.hasDcOffsetCorr) << "the DC offset toggle is offered where the frontend carries one";
        expect((named("IQ_CORR") != controls.end()) == truth.hasIqBalanceCorr) << "and so is the IQ balance toggle";
    });

    cases("uhd.hardware-information", [] {
        /*| frame: what the unit says it is, read two ways: through the block's probe, and
                through a direct open of the same radio. A name written into the case would
                be the bench and not the radio.
        */
        const auto& truth = benchTruth();
        std::string directModel;
        std::string directSerial;
        std::string directFw;
        std::string directFpga;
        try {
            auto       usrp = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            const auto info = usrp->get_usrp_rx_info(0);
            directModel     = usrp->get_mboard_name(0);
            directSerial    = info.get("mboard_serial", "");
            directFw        = info.get("mboard_fw_version", "");
            directFpga      = info.get("mboard_fpga_version", "");
            if (auto tree = usrp->get_device()->get_tree(); directFw.empty() && tree->exists("/mboards/0/fw_version")) {
                directFw   = tree->access<std::string>("/mboards/0/fw_version").get();
                directFpga = tree->exists("/mboards/0/fpga_version") ? tree->access<std::string>("/mboards/0/fpga_version").get() : directFpga;
            }
        } catch (const std::exception& e) {
            std::printf("hardware information: the direct open failed: %s\n", e.what());
        }

        std::printf("hardware information: the probe says model \"%s\", serial \"%s\", firmware \"%s\", FPGA \"%s\"\n", truth.model.c_str(), truth.serial.c_str(), truth.fwVersion.c_str(),
                    truth.fpgaVersion.c_str());
        std::printf("hardware information: a direct open says \"%s\", \"%s\", \"%s\", \"%s\"\n", directModel.c_str(), directSerial.c_str(), directFw.c_str(), directFpga.c_str());
        std::printf("hardware information: the unit describes itself as \"%s\"\n", truth.describe().c_str());

        expect(!directModel.empty()) << "the radio names its motherboard";
        expect(truth.model == directModel) << "and the probe states the same name";
        expect(truth.serial == directSerial) << "and the same serial";
        expect(truth.fwVersion == directFw) << "and the same firmware version";
        expect(truth.fpgaVersion == directFpga) << "and the same FPGA version";
        expect(!truth.serial.empty()) << "the serial is not empty";
        expect(truth.describe().find(truth.model) != std::string::npos) << "the one-line description names the model";
        expect(truth.describe().find(truth.serial) != std::string::npos) << "and the serial";

        /*| frame: the same strings reach a caller through the sensor sweep, which is where a
                receiver shows what device it is talking to. */
        std::string reading;
        const bool  ran = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            std::vector<capture::SensorReading> readings;
            if (src.readSensors(readings)) {
                const auto it = std::ranges::find_if(readings, [](const auto& r) { return r.id == "device_model"; });
                reading       = it == readings.end() ? std::string{} : it->value;
            }
        });
        expect(ran) << "the graph ran";
        std::printf("hardware information: the sweep publishes device_model as \"%s\"\n", reading.c_str());
        expect(reading == truth.describe()) << "the running block publishes what the probe read";
    });

    cases("uhd.agc", [] {
        /*| trap: get_rx_gain answers with the gain last commanded and not with the gain the
                automatic loop is running at, so the read-back cannot say whether the device
                took the mode. The received level can: with zero commanded, a loop that has
                the gain path lifts it.
        */
        /*| frame: whether the frontend carries an automatic gain control at all is read from
                its property tree, so the two branches below are the two kinds of radio and
                not two benches.
        */
        const auto& truth = benchTruth();
        expect(!truth.gains.empty()) << "the device names its gain elements";
        if (truth.gains.empty()) {
            return;
        }
        const auto&  widest = *std::ranges::max_element(truth.gains, {}, [](const auto& g) { return g.max - g.min; });
        const double high   = widest.min + 0.8 * (widest.max - widest.min);
        const double low    = widest.min + 0.2 * (widest.max - widest.min);

        double atStart     = 0.0;
        double underAgc    = 0.0;
        double afterFirst  = 0.0;
        double afterSecond = 0.0;
        double afterAgcOff = 0.0;
        double manualDbfs  = 0.0;
        double agcDbfs     = 0.0;

        const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            /*| trap: the frontend's automatic DC offset correction is a loop that takes a
                    second or two to pull the offset down, and the offset is part of the mean
                    power. A level taken the moment the stream starts is therefore above the
                    level the settled device sits at, by more than the AGC's own lift on a
                    model that has no AGC.
            */
            settle(1500);
            atStart     = src.elementGain(widest.name);
            manualDbfs  = dbfsOver(0.5);
            std::ignore = src.settings().setStaged({{"gain_mode", true}});
            settle(1000);
            underAgc = src.elementGain(widest.name);
            agcDbfs  = dbfsOver(0.5);

            expect(src.setElementGain(widest.name, high)) << "a gain write under the AGC is accepted";
            settle(200);
            afterFirst = src.elementGain(widest.name);
            expect(src.setElementGain(widest.name, low)) << "and so is a second one";
            settle(200);
            afterSecond = src.elementGain(widest.name);

            std::ignore = src.settings().setStaged({{"gain_mode", false}});
            settle(500);
            afterAgcOff = src.elementGain(widest.name);
        });
        expect(ran) << "the graph ran";

        std::printf("agc: this radio's frontend %s\n", truth.hasAgc ? "carries an automatic gain control" : "carries no automatic gain control");
        std::printf("agc: element \"%s\" reads %.1f dB at start, %.1f dB under the AGC, %.1f dB after asking for %.1f dB, %.1f dB after asking for %.1f dB, %.1f dB after the AGC; "
                    "mean power %.2f dBFS manual, %.2f dBFS under the AGC\n",
                    widest.name.c_str(), atStart, underAgc, afterFirst, high, afterSecond, low, afterAgcOff, manualDbfs, agcDbfs);

        const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = 2.0e6, .centerFreq = defaultFreq()};
        const auto                  controls = capture::uhd::Source::describeControls(truth, ctx);
        expect(std::ranges::find_if(controls, [](const auto& c) { return c.id == "AGC"; }) == controls.end()) << "the automatic mode is the gain_mode setting and never a descriptor";

        expect(std::abs(atStart - truth.gainMinDb) < widest.step + 0.01) << "the manual gain starts where the settings put it";
        if (truth.hasAgc) {
            expect(agcDbfs > manualDbfs + 3.0) << "the loop lifts the level above what the lowest gain gives, so this model takes the mode";
            expect(std::abs(afterFirst - high) > widest.step) << "a gain asked for under the AGC is not written";
            expect(std::abs(afterSecond - low) > widest.step) << "nor is the next one";
            expect(std::abs(afterAgcOff - low) < widest.step + 0.01) << "and leaving the AGC writes the value the block held";
        } else {
            /*| frame: a model with no loop. The mode is logged once and nothing is written,
                    so the level does not move and a gain asked for reaches the device at
                    once rather than being held for a loop that does not exist.
            */
            expect(std::abs(agcDbfs - manualDbfs) < 3.0) << "the level does not move, because the mode was not written";
            expect(std::abs(afterFirst - high) < widest.step + 0.01) << "a gain asked for reaches the device at once";
            expect(std::abs(afterSecond - low) < widest.step + 0.01) << "and so does the next one";
            expect(std::abs(afterAgcOff - low) < widest.step + 0.01) << "and leaving a mode that was never entered changes nothing";
        }
    });

    cases("uhd.antenna", [] {
        /*| frame: every connector the device lists, in turn and then back to the first, so
                the write and the read-back are measured on each. Which of them has an
                antenna on it is a fact about the bench that no device call answers, so the
                level comparison runs only where CAPTURE_HW_UHD_ANTENNA names one.
        */
        const auto& truth = benchTruth();
        expect(!truth.antennas.empty()) << "the device lists its connectors";
        if (truth.antennas.empty()) {
            return;
        }
        std::vector<std::string> order(truth.antennas);
        order.push_back(truth.antennas.front()); // and back to where it started
        std::vector<std::string> readBack;
        std::vector<double>      powerDbfs;
        std::vector<double>      stated; // the ANTENNA default the surface states after each write, -1 where it states none

        const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMaxDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            for (const std::string& a : order) {
                std::ignore = src.settings().setStaged({{"rx_antennae", std::vector<std::string>{a}}});
                settle(400);
                readBack.push_back(deviceRead(src, std::string{}, [](uhd::usrp::multi_usrp& u) { return u.get_rx_antenna(0); }));
                powerDbfs.push_back(dbfsOver(0.5));
                const auto surface = src.controlSurface();
                const auto control = std::ranges::find_if(surface, [](const capture::ControlDesc& c) { return c.id == "ANTENNA"; });
                stated.push_back(control == surface.end() ? -1.0 : control->defValue);
            }
        });
        expect(ran) << "the graph ran";

        for (std::size_t i = 0; i < order.size(); ++i) {
            std::printf("antenna: asked %-8s  get_rx_antenna %-8s  mean power %7.2f dBFS\n", order[i].c_str(), readBack[i].c_str(), powerDbfs[i]);
        }
        for (std::size_t i = 0; i < order.size(); ++i) {
            expect(readBack[i] == order[i]) << "the connector the device holds is the one asked for";
        }
        for (std::size_t i = 0; i < stated.size(); ++i) {
            const auto   at    = std::ranges::find(truth.antennas, order[i]);
            const double index = static_cast<double>(std::distance(truth.antennas.begin(), at));
            expect(stated[i] == (truth.antennas.size() > 1UZ ? index : -1.0)) << "the surface states the connector the write landed on, and no control for a single connector";
        }

        if (benchAntenna().empty()) {
            std::printf("antenna: no connector on this bench carries an antenna (CAPTURE_HW_UHD_ANTENNA names one), so the levels above are printed and not compared\n");
            return;
        }
        const auto wired = std::ranges::find(order, benchAntenna());
        expect(wired != order.end()) << "the bench names a connector this device lists";
        if (wired == order.end()) {
            return;
        }
        const std::size_t at = static_cast<std::size_t>(std::distance(order.begin(), wired));
        for (std::size_t i = 0; i < order.size(); ++i) {
            if (i == at || order[i] == benchAntenna()) {
                continue;
            }
            expect(powerDbfs[at] > powerDbfs[i]) << "the connector with the antenna on it hears more than one with nothing on it";
        }
    });

    cases("uhd.analog-bandwidth", [] {
        /*| frame: a frontend states its filter as a range. A range with room in it is a
                filter the block programs from the rate; a range whose ends are equal is a
                filter fixed in the hardware, which takes no write at all and which the block
                holds at the width the device came up with.
        */
        const auto&  truth   = benchTruth();
        const bool   movable = truth.bwMaxHz > truth.bwMinHz;
        const double firstRate  = 2.0e6;
        const double secondRate = 4.0e6;
        // Inside the stated range at each end, so the writes are ones the device can take.
        const double narrowMHz = movable ? std::clamp(1.0e6, truth.bwMinHz, truth.bwMaxHz) / 1e6 : 1.0;
        const double wideMHz   = movable ? truth.bwMaxHz / 1e6 * 1.5 : 1.0;

        double atRate      = 0.0;
        double afterWrite  = 0.0;
        double afterRate   = 0.0;
        double overCeiling = 0.0;
        bool   narrowTaken = false;
        bool   wideTaken   = false;
        // What the block says it holds, beside what the device answers, at each step.
        std::vector<std::pair<double, double>> heldAgainstDevice;
        double                                 descriptorDefaultMHz = -1.0;
        bool                                   descriptorExists     = false;

        const bool ran = withGraph(sourceSettings(firstRate, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            const auto readBw = [&src] { return deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_bandwidth(0); }); };
            const auto record = [&] { heldAgainstDevice.emplace_back(src.analogBandwidthHz(), readBw()); };
            atRate = readBw();
            record();
            narrowTaken = src.setControl("BANDWIDTH", narrowMHz);
            settle(200);
            afterWrite = readBw();
            record();
            std::ignore = src.settings().setStaged({{"sample_rate", secondRate}});
            settle(600);
            afterRate = readBw();
            record();
            /*| frame: a request past the frontend's ceiling, which UHD coerces down. The
                    read-back is the only way to know where it landed. */
            wideTaken = src.setControl("BANDWIDTH", wideMHz);
            settle(200);
            overCeiling = readBw();
            record();

            const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = secondRate, .centerFreq = defaultFreq()};
            for (const auto& c : capture::uhd::Source::describeControls(truth, ctx, src.analogBandwidthHz())) {
                if (c.id == "BANDWIDTH") {
                    descriptorExists     = true;
                    descriptorDefaultMHz = c.defValue;
                }
            }
        });
        expect(ran) << "the graph ran";

        std::printf("bandwidth: the device states %.0f to %.0f Hz, which is %s\n", truth.bwMinHz, truth.bwMaxHz, movable ? "a filter the block programs" : "a filter fixed in the hardware");
        std::printf("bandwidth: %.0f Hz at %.0f S/s, %.0f Hz after asking for %.3f MHz, %.0f Hz after the rate moved to %.0f S/s, %.0f Hz after asking for %.3f MHz\n", atRate, firstRate, afterWrite,
                    narrowMHz, afterRate, secondRate, overCeiling, wideMHz);
        for (const auto& [held, device] : heldAgainstDevice) {
            std::printf("bandwidth: the block holds %.0f Hz, the device answers %.0f Hz\n", held, device);
        }
        std::printf("bandwidth: the descriptor %s, default %.3f MHz\n", descriptorExists ? "exists" : "is not offered", descriptorDefaultMHz);

        for (const auto& [held, device] : heldAgainstDevice) {
            expect(std::abs(held - device) < 1.0) << "what the block holds is what the device answered, at every step";
        }
        if (movable) {
            expect(narrowTaken && wideTaken) << "a filter with room in it takes a write";
            expect(std::abs(atRate - firstRate) < 0.05e6) << "the filter follows the rate";
            expect(std::abs(afterWrite - narrowMHz * 1e6) < 0.05e6) << "a manual write lands";
            expect(std::abs(afterRate - secondRate) < 0.1e6) << "and the next rate change re-derives it";
            expect(std::abs(overCeiling - truth.bwMaxHz) < 0.1e6) << "a request past the ceiling settles at the ceiling";
            expect(descriptorExists) << "and the filter is a control";
            expect(std::abs(descriptorDefaultMHz - truth.bwMaxHz / 1e6) < 0.1) << "whose default is the filter in force, not the rate-derived guess";
        } else {
            expect(!narrowTaken && !wideTaken) << "a fixed filter refuses a write rather than pretending to take one";
            expect(std::abs(atRate - truth.bwMinHz) < 1.0) << "the width the device came up with is the one it states";
            expect(std::abs(afterWrite - atRate) < 1.0) << "a write leaves it alone";
            expect(std::abs(afterRate - atRate) < 1.0) << "and so does a rate change";
            expect(std::abs(overCeiling - atRate) < 1.0);
            expect(heldAgainstDevice.front().first == truth.bwCurHz) << "and the block holds it from the first read rather than from the first write";
            expect(!descriptorExists) << "a filter with one width is not a control a caller can move";
        }
    });

    cases("uhd.sensors", [] {
        std::vector<capture::SensorReading> readings;
        std::vector<capture::SensorReading> brief;
        std::vector<std::pair<std::string, double>> perSensorMs;
        std::vector<std::string>            mboardNames;
        std::vector<std::string>            rxNames;
        double                              sweepMs      = 0.0;
        double                              briefMs      = 0.0;
        bool                                sweptTrue    = false;
        bool                                briefSwept   = false;
        bool                                falseIsEmpty = true;
        bool                                sawFalse     = false;

        const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), benchTruth().gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto& scheduled) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            const auto t0 = Clock::now();
            sweptTrue     = src.readSensors(readings);
            sweepMs       = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

            /*| frame: the names the device itself lists, read through the block's own
                    handle so nothing else opens the radio. */
            mboardNames = deviceRead(src, std::vector<std::string>{}, [](uhd::usrp::multi_usrp& u) { return u.get_mboard_sensor_names(0); });
            rxNames     = deviceRead(src, std::vector<std::string>{}, [](uhd::usrp::multi_usrp& u) { return u.get_rx_sensor_names(0); });

            // The cost of one read of each sensor, which decides whether a poll can afford it.
            for (const auto* pair : {&mboardNames, &rxNames}) {
                const bool mboard = pair == &mboardNames;
                for (const std::string& n : *pair) {
                    const auto   at = Clock::now();
                    const double ms = deviceRead(src, -1.0, [&n, mboard, &at](uhd::usrp::multi_usrp& u) {
                        std::ignore = mboard ? u.get_mboard_sensor(n, 0) : u.get_rx_sensor(n, 0);
                        return std::chrono::duration<double, std::milli>(Clock::now() - at).count();
                    });
                    perSensorMs.emplace_back(n, ms);
                }
            }
            const auto t1 = Clock::now();
            briefSwept    = src.readSensors(brief, capture::uhd::Source::SensorSweep::Brief);
            briefMs       = std::chrono::duration<double, std::milli>(Clock::now() - t1).count();

            // A sweep that is running while the device goes down must publish nothing at all.
            std::atomic<bool> sweeping{true};
            std::thread       racer([&src, &sweeping, &falseIsEmpty, &sawFalse] {
                std::vector<capture::SensorReading> out;
                while (sweeping.load(std::memory_order_acquire)) {
                    if (!src.readSensors(out)) {
                        sawFalse = true;
                        if (!out.empty()) {
                            falseIsEmpty = false;
                        }
                    }
                }
            });
            settle(200);
            scheduled.stop();
            settle(200);
            sweeping.store(false, std::memory_order_release);
            racer.join();
        });
        expect(ran) << "the graph ran";

        std::printf("sensors: a full sweep is %zu readings in %.1f ms; a brief one %zu in %.1f ms\n", readings.size(), sweepMs, brief.size(), briefMs);
        for (const auto& r : readings) {
            std::printf("sensor: %-20s %-26s %-28s good %-3s brief %s\n", r.id.c_str(), r.label.c_str(), r.value.c_str(), r.good ? "yes" : "no", r.brief ? "yes" : "no");
        }
        for (const auto& [id, ms] : perSensorMs) {
            std::printf("sensor cost: %-20s %8.3f ms\n", id.c_str(), ms);
        }

        const auto find = [&readings](const std::string& id) {
            return std::ranges::find_if(readings, [&id](const auto& r) { return r.id == id; });
        };
        expect(sweptTrue) << "the sweep ran to completion while streaming";
        /*| frame: the ids the device itself lists, which differ per model: a B2xx publishes
                a temperature and an RSSI that an N-series does not, and an N-series with a
                GPSDO publishes five GPS records that a B2xx does not.
        */
        expect(!mboardNames.empty()) << "the device lists its motherboard sensors";
        for (const std::string& n : mboardNames) {
            expect(find(n) != readings.end()) << "every motherboard sensor the device lists is in the sweep";
        }
        for (const std::string& n : rxNames) {
            expect(find(n) != readings.end()) << "and every receive-chain one";
        }
        /*| frame: the readings the block publishes beside them: the reference and timing
                selections and the device clock, which are not sensors, what the stream has
                lost, which no device sensor reports, and the unit's name.
        */
        for (const char* id : {"clock_source", "time_source", "time_last_pps", "rx_overflows", "rx_sequence_errors", "rx_dropped_samples", "device_model"}) {
            expect(find(id) != readings.end()) << "the block's own readings are in the sweep";
        }
        for (const char* id : {"rx_overflows", "rx_sequence_errors", "rx_dropped_samples"}) {
            const auto it = find(id);
            if (it != readings.end()) {
                expect(it->good) << "a count is not a pass or a fail, so it reads good";
                expect(it->brief == (std::string_view(id) != "rx_sequence_errors")) << "and the link's losses alone are a diagnostic";
            }
        }
        if (const auto dropped = find("rx_dropped_samples"); dropped != readings.end()) {
            expect(dropped->label == "Dropped samples") << "the shed count names what it counts";
        }
        /*| frame: ref_locked reports the lock to an external reference and mimo_locked the lock
                to a MIMO cable, so a radio on its own internal clock with no cable reports both
                unlocked and that is the truth. A two-state sensor's verdict is its own state word
                where the selection in force uses the sensor, and good where it does not.
        */
        const auto selection = [&find, &readings](const char* id) { const auto it = find(id); return it == readings.end() ? std::string{} : it->value; };
        for (const char* id : {"ref_locked", "lo_locked", "gps_locked", "mimo_locked"}) {
            const auto it = find(id);
            if (it != readings.end()) {
                const bool applies = capture::uhd::lockReadingApplies(id, selection("clock_source"), selection("time_source"));
                expect(it->good == (!applies || it->value == std::string("locked"))) << "a two-state sensor's verdict is the state it reports where the selection uses it";
            }
        }
        const auto lo = find("lo_locked");
        if (lo != readings.end()) {
            expect(lo->good) << "the tuner is locked while it streams";
        }
        expect(find("time_last_pps") != readings.end() && !find("time_last_pps")->brief) << "the PPS time is a wide reading";

        /*| frame: a receiver polls the brief sweep, so it has to fit inside the poll. A
                caller drawing a GPSDO's own records asks for the full one, which on a radio
                with such an oscillator costs the better part of a second.
        */
        expect(briefSwept) << "the brief sweep ran to completion";
        expect(briefMs < 100.0) << "and costs less than a tenth of the two-second poll a receiver runs it on";
        expect(brief.size() <= readings.size()) << "it is the full sweep or a part of it";
        for (const auto& r : brief) {
            expect(find(r.id) != readings.end()) << "and every reading in it is in the full one";
        }
        expect(sawFalse) << "a sweep against a device going down answers false";
        expect(falseIsEmpty) << "and publishes nothing rather than a partial set";
    });

    cases("uhd.overflow-under-a-stalled-consumer", [] {
        /*| frame: the stall runs at the top of the radio's own ladder, and it has to be
                long enough that the samples arriving during it exceed whatever the device
                and the transport hold for the host. The assumption stated: the buffer on
                the host side is at most the kernel's own socket receive limit, 50 MB on
                this box, which at four bytes a sample is 12.5 M samples, so a stall of one
                second at any rate above 12.5 MS/s fills it. The stall lasts a second at
                every rate, and the case prints the counts it read.
        trap: the expected count comes from the rate the device is running at, not from the
                rate asked for. A fixed-clock radio lands on its own divisor, and judging a
                7.692 MS/s stream against an 8 MS/s request reads as a 3.7 percent loss that
                never happened.
        */
        const auto&      truth = benchTruth();
        const auto       ladder = capture::uhd::Source::sampleRatesFor(truth);
        const double     kRate  = ladder.empty() ? 8.0e6 : ladder.back();
        constexpr int    kStallMs = 1000;
        double           rateInForce = 0.0;
        std::uint64_t    got      = 0;
        std::uint64_t    overflow = 0;
        long long        dropped  = -1;
        double           seconds  = 0.0;

        std::string   overflowReading;
        std::string   droppedReading;
        std::string   sequenceReading;
        std::uint64_t overflowsAtRead = 0;
        std::uint64_t droppedAtRead   = 0;
        std::uint64_t sequenceAtRead  = 0;
        std::size_t   frameSamples    = 0UZ;

        const bool ran = withGraph(sourceSettings(kRate, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(8000))) << "the stream starts";
            settle(500);
            rateInForce  = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
            frameSamples = src.recvFrameSamples();
            const std::uint64_t at0   = g_samples.load();
            const std::uint64_t ov0   = g_overflowTags.load();
            const long long     drop0 = droppedOf(src);
            const auto          t0    = Clock::now();
            g_stallMs.store(kStallMs);
            settle(3000);
            got      = g_samples.load() - at0;
            overflow = g_overflowTags.load() - ov0;
            dropped  = droppedOf(src) < 0 ? -1 : droppedOf(src) - drop0;
            seconds  = std::chrono::duration<double>(Clock::now() - t0).count();

            /*| frame: the same counts read the way a receiver reads them, through the
                    sensor sweep, against the accessors they come from. */
            std::vector<capture::SensorReading> readings;
            if (src.readSensors(readings)) {
                overflowsAtRead = src.overflowEvents();
                droppedAtRead   = src.droppedSamples();
                sequenceAtRead  = src.sequenceErrors();
                for (const auto& r : readings) {
                    if (r.id == "rx_overflows") {
                        overflowReading = r.value;
                    } else if (r.id == "rx_dropped_samples") {
                        droppedReading = r.value;
                    } else if (r.id == "rx_sequence_errors") {
                        sequenceReading = r.value;
                    }
                }
            }
        });
        expect(ran) << "the graph ran";

        const double expected = rateInForce * seconds;
        const double closure  = dropped < 0 || expected <= 0.0 ? -1.0 : 100.0 * (static_cast<double>(got) + static_cast<double>(dropped) - expected) / expected;
        std::printf("stall: %.3f s at %.0f S/s in force (the ladder's top %.0f was asked for) across a %d ms stall: sink counted %llu, source shed %lld, expected %.0f, closure %.3f %%, gap tags %llu\n",
                    seconds, rateInForce, kRate, kStallMs, static_cast<unsigned long long>(got), dropped, expected, closure, static_cast<unsigned long long>(overflow));
        std::printf("stall: the receive frame holds %zu samples; the sweep reads rx_overflows \"%s\" against %llu counted, rx_sequence_errors \"%s\" against %llu, and rx_dropped_samples \"%s\" "
                    "against %llu\n",
                    frameSamples, overflowReading.c_str(), static_cast<unsigned long long>(overflowsAtRead), sequenceReading.c_str(), static_cast<unsigned long long>(sequenceAtRead),
                    droppedReading.c_str(), static_cast<unsigned long long>(droppedAtRead));
        std::printf("stall: this radio answers a stall with %llu overflow event(s) and %llu sequence error(s)\n", static_cast<unsigned long long>(overflowsAtRead),
                    static_cast<unsigned long long>(sequenceAtRead));

        expect(overflow >= 1UL) << "the stall put a gap in the stream and the gap was marked";
        expect(dropped >= 0) << "the source states how many samples the device shed";
        expect(dropped > 0) << "and the stall shed some";
        expect(std::abs(closure) < 1.0) << "what the sink counted plus what the device shed is the whole stream, so the publish dropped nothing";
        expect(overflowReading == std::format("{}", overflowsAtRead)) << "the reading carries the overflow count the accessor does";
        expect(sequenceReading == std::format("{}", sequenceAtRead)) << "and the sequence-error count";
        expect(droppedReading == std::format("{}", droppedAtRead)) << "and the shed count";
        expect(droppedAtRead > 0UL) << "the stall moved the shed count off zero";
        expect(overflowsAtRead + sequenceAtRead > 0UL) << "and one of the two gap counters with it";
    });

    cases("uhd.teardown-and-restart", [] {
        /*| frame: the scheduler's own worker pool outlives a graph by design, so the count of
                threads says nothing. The source promises one thing here: its consumer
                thread, the one it names, is joined before stop() returns.
        */
        const auto streamingThreads = [] {
            const auto names = ownThreadNames();
            return static_cast<std::size_t>(std::ranges::count_if(names, [](const std::string& n) { return n.starts_with("uhd:"); }));
        };
        std::vector<std::string>   namesWhileRunning;
        std::vector<std::uint64_t> perRun;

        for (int run = 0; run < 3; ++run) {
            const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), 38.0, sourcePort()), [&](capture::uhd::Source&, auto&) {
                expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
                settle(300);
                if (namesWhileRunning.empty()) {
                    namesWhileRunning = ownThreadNames();
                }
            });
            expect(ran) << "the graph ran";
            perRun.push_back(g_samples.load());
            expect(deviceOpens()) << "the device is released between runs";
        }

        // A stop that arrives inside the seconds-long device open.
        resetCounters();
        gr::Graph flow;
        auto&     src  = flow.emplaceBlock<capture::uhd::Source>(sourceSettings(2.0e6, defaultFreq(), 38.0, sourcePort()));
        auto&     sink = flow.emplaceBlock<hw_sink>({{"name", std::string("host")}});
        expect(flow.connect<"out", "in">(src, sink).has_value()) << "the source wires to the sink";
        Scheduled scheduled;
        expect(scheduled.adopt(std::move(flow)).has_value()) << "the graph builds";
        scheduled.start();
        settle(200);
        scheduled.stop();
        const bool        releasedAfterEarlyStop = deviceOpens();
        const std::size_t streamingAfter         = streamingThreads();

        const bool named = std::ranges::any_of(namesWhileRunning, [](const std::string& n) { return n == "uhd:src"; });
        std::printf("teardown: samples per run %llu / %llu / %llu; the consumer thread is named %s; %zu threads named uhd: remain after the last stop\n",
                    static_cast<unsigned long long>(perRun[0]), static_cast<unsigned long long>(perRun[1]), static_cast<unsigned long long>(perRun[2]), named ? "uhd:src" : "not uhd:src", streamingAfter);

        for (const std::uint64_t n : perRun) {
            expect(n > 0UL) << "every run streamed";
        }
        expect(named) << "the consumer thread is named uhd: plus the block's name";
        expect(releasedAfterEarlyStop) << "a stop inside the device open still releases the device";
        expect(streamingAfter == 0UZ) << "and no consumer thread outlives its block";
    });

    cases("uhd.corrections", [] {
        /*| frame: a correction is taken only where the frontend's property tree carries its
                enable. A model without one takes the UHD call, logs a warning and changes
                nothing, so the block refuses it rather than reporting a write that did not
                happen.
        */
        const auto& truth = benchTruth();
        bool        dcOff = false, iqOff = false, dcOn = false, iqOn = false;

        const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            dcOff = src.setControl("DC_CORR", 0.0);
            iqOff = src.setControl("IQ_CORR", 0.0);
            settle(200);
            dcOn = src.setControl("DC_CORR", 1.0);
            iqOn = src.setControl("IQ_CORR", 1.0);
        });
        expect(ran) << "the graph ran";

        std::printf("corrections: the frontend carries a DC offset enable %s and an IQ balance enable %s\n", truth.hasDcOffsetCorr ? "yes" : "no", truth.hasIqBalanceCorr ? "yes" : "no");
        std::printf("corrections: DC off %s, IQ off %s, DC on %s, IQ on %s\n", dcOff ? "accepted" : "refused", iqOff ? "accepted" : "refused", dcOn ? "accepted" : "refused", iqOn ? "accepted" : "refused");
        expect(dcOff == truth.hasDcOffsetCorr && dcOn == truth.hasDcOffsetCorr) << "the DC offset correction takes both states where the frontend carries it, and is refused where it does not";
        expect(iqOff == truth.hasIqBalanceCorr && iqOn == truth.hasIqBalanceCorr) << "and the IQ balance correction likewise";
    });

    cases("uhd.correction-defaults", [] {
        /*| frame: the state the two automatic corrections come up in, which the descriptor's
                default has to state. Read two ways: the device's own property
                tree before anything writes it, and the DC bin of a received frame against
                the same frame with the correction written off.
        */
        const auto& truth        = benchTruth();
        bool        treeSaysDcOn = false;
        bool        treeSaysIqOn = false;
        bool        treeHasDc    = false;
        bool        treeHasIq    = false;
        std::string frontendPath;
        double      dcAsItCame   = 0.0;
        double      dcWithOff    = 0.0;
        double      dcWithOn     = 0.0;

        const bool ran = withGraph(sourceSettings(2.0e6, bandFreq(), truth.gainMaxDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            /*| trap: the daughterboard slot and the frontend name differ per model, so the
                    path is asked of the device rather than written out. A B2xx answers A:A
                    and an N-series with a WBX answers A:0.
            */
            std::ignore = deviceRead(src, false, [&](uhd::usrp::multi_usrp& u) {
                const std::string dc = capture::uhd::Source::rxFacilityPath(u, "dc_offset/enable");
                const std::string iq = capture::uhd::Source::rxFacilityPath(u, "iq_balance/enable");
                frontendPath         = dc.empty() ? iq : dc;
                auto tree            = u.get_device()->get_tree();
                treeHasDc            = !dc.empty();
                treeHasIq            = !iq.empty();
                treeSaysDcOn         = treeHasDc && tree->access<bool>(dc).get();
                treeSaysIqOn         = treeHasIq && tree->access<bool>(iq).get();
                return true;
            });
            /*| trap: the automatic correction is a loop, not a switch, and it needs a second
                    or two to pull the offset down. A window taken the moment the stream
                    starts catches it mid-convergence and reads a level the settled device
                    never sits at.
            */
            settle(1500);
            resetPower();
            settle(500);
            dcAsItCame = dcDbfs();
            if (treeHasDc) {
                expect(src.setControl("DC_CORR", 0.0)) << "the correction the frontend carries is written off";
            }
            std::ignore = src.setControl("IQ_CORR", 0.0);
            settle(400);
            resetPower();
            settle(500);
            dcWithOff = dcDbfs();
            if (treeHasDc) {
                expect(src.setControl("DC_CORR", 1.0)) << "and on again";
            }
            std::ignore = src.setControl("IQ_CORR", 1.0);
            settle(600);
            resetPower();
            settle(500);
            dcWithOn = dcDbfs();
        });
        expect(ran) << "the graph ran";

        std::printf("correction defaults: the enables sit at \"%s\"; the frontend carries dc_offset/enable %s and iq_balance/enable %s\n", frontendPath.c_str(), treeHasDc ? "yes" : "no",
                    treeHasIq ? "yes" : "no");
        std::printf("correction defaults: as the device came up dc_offset/enable reads %s and iq_balance/enable %s\n", treeSaysDcOn ? "true" : "false", treeSaysIqOn ? "true" : "false");
        std::printf("correction defaults: the DC bin reads %.2f dBFS as the device came up, %.2f dBFS with the corrections written off, %.2f dBFS with them written on\n", dcAsItCame, dcWithOff,
                    dcWithOn);

        const auto named = [](const std::vector<capture::ControlDesc>& cs, const char* id) {
            return std::ranges::find_if(cs, [id](const auto& c) { return c.id == id; });
        };
        const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = 2.0e6, .centerFreq = bandFreq()};
        const auto                  controls = capture::uhd::Source::describeControls(truth, ctx);
        const auto                  dc       = named(controls, "DC_CORR");
        const auto                  iq       = named(controls, "IQ_CORR");

        expect(truth.hasDcOffsetCorr == treeHasDc) << "the probe and the running device agree on whether the DC offset enable is there";
        expect(truth.hasIqBalanceCorr == treeHasIq) << "and on the IQ balance enable";
        expect((dc != controls.end()) == treeHasDc) << "the descriptor exists exactly where the tree carries the enable";
        expect((iq != controls.end()) == treeHasIq);
        if (dc != controls.end()) {
            expect(dc->defValue == 1.0) << "and its default is on, which is the state the device comes up in";
            expect(treeSaysDcOn) << "as the tree itself says";
        }
        if (iq != controls.end()) {
            expect(iq->defValue == 1.0);
            expect(treeSaysIqOn);
        }
        if (treeHasDc) {
            expect(dcAsItCame < dcWithOff - 10.0) << "the DC bin as the device came up sits far below the DC bin with the correction off, which is the same answer over the air";
            expect(dcWithOn < dcWithOff - 10.0) << "and writing it on again puts it back there";
        } else {
            std::printf("correction defaults: this frontend carries no DC offset enable, so the three levels above are printed and not compared\n");
        }
    });

    cases("uhd.device-string-keys", [] {
        /*| frame: a device string can open with driver=<name>, a key UHD's own device_addr
                documents nothing about. This case measures whether
                multi_usrp::make takes it, ignores it or refuses it on this radio.
        */
        /*| trap: the selector names the radio by serial over USB and by address over
                Ethernet, so the enumeration is matched on whichever of those two keys the
                selector carries. Matching on an empty serial matches every entry.
        */
        std::string ours;
        for (const char* key : {"serial", "addr"}) {
            if (const std::string value = capture::kwargsValue(deviceArgs(), key); !value.empty()) {
                ours = std::string(key) + "=" + value;
                break;
            }
        }
        expect(!ours.empty()) << "the selector names this radio by serial or by address";

        std::string enumerated;
        for (const auto& [label, devstr] : capture::uhd::Source::enumerateDevices()) {
            std::printf("device string: enumerated \"%s\" as \"%s\"\n", label.c_str(), devstr.c_str());
            if (devstr.find(ours) != std::string::npos) {
                enumerated = devstr;
            }
        }
        expect(!enumerated.empty()) << "the radio these cases own is in the enumeration";
        expect(enumerated.starts_with("driver=uhd")) << "and its string carries the workspace's driver key";

        capture::uhd::DeviceTruth plainTruth;
        capture::uhd::DeviceTruth keyedTruth;
        const bool              plainOpens = capture::uhd::Source::probeTruth(deviceArgs(), plainTruth);
        const bool              keyedOpens = capture::uhd::Source::probeTruth(enumerated, keyedTruth);

        // The motherboard name a direct open reads, with no probe between.
        std::string directName;
        try {
            auto usrp  = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            directName = usrp->get_mboard_name(0);
        } catch (const std::exception& e) {
            std::printf("device string: the direct open failed: %s\n", e.what());
        }

        std::printf("device string: \"%s\" %s; \"%s\" %s\n", deviceArgs().c_str(), plainOpens ? "opens" : "is refused", enumerated.c_str(), keyedOpens ? "opens" : "is refused");
        std::printf("device string: the two probes name the models \"%s\" and \"%s\"; a direct open names \"%s\"\n", plainTruth.model.c_str(), keyedTruth.model.c_str(), directName.c_str());

        expect(plainOpens) << "the plain selector opens the radio";
        expect(keyedOpens) << "and so does the enumerated one, so UHD ignores the extra key rather than refusing it";
        expect(plainTruth.model == keyedTruth.model) << "and both reach the same radio";
        expect(!directName.empty()) << "the radio names its motherboard";
        expect(plainTruth.model == directName) << "and the probe states that name rather than one written into this case";
    });

    cases("uhd.clock-and-time-source", [] {
        /*| frame: which reference a radio comes up on is its own business: a B2xx comes up
                on internal and an N-series with a GPSDO comes up on gpsdo, so the case reads
                the selection in force rather than assuming one, and restores it at the end.
        */
        const auto& truth = benchTruth();
        std::printf("clock: the device offers %zu clock sources and %zu time sources, running on \"%s\" and \"%s\"\n", truth.clockSources.size(), truth.timeSources.size(), truth.clockSource.c_str(), truth.timeSource.c_str());
        for (const auto& s : truth.clockSources) {
            std::printf("clock: clock source \"%s\"\n", s.c_str());
        }
        for (const auto& s : truth.timeSources) {
            std::printf("clock: time source \"%s\"\n", s.c_str());
        }

        const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = 2.0e6, .centerFreq = bandFreq()};
        const auto                  controls = capture::uhd::Source::describeControls(truth, ctx);
        const auto                  named    = [&controls](const char* id) {
            return std::ranges::find_if(controls, [id](const auto& c) { return c.id == id; });
        };
        const auto clockDesc = named("CLOCK_SOURCE");
        const auto timeDesc  = named("TIME_SOURCE");
        expect(clockDesc != controls.end() && timeDesc != controls.end()) << "both selections are descriptors";
        if (clockDesc == controls.end() || timeDesc == controls.end()) {
            return;
        }
        expect(clockDesc->kind == 3 && timeDesc->kind == 3) << "each is an enum over the device's own list";
        expect(clockDesc->options == truth.clockSources) << "whose options are what the device said";
        expect(timeDesc->options == truth.timeSources);
        expect(!clockDesc->isGain && !timeDesc->isGain);
        const auto indexOf = [](const std::vector<std::string>& options, const std::string& want) {
            const auto it = std::ranges::find(options, want);
            return it == options.end() ? -1.0 : static_cast<double>(std::distance(options.begin(), it));
        };
        expect(clockDesc->defValue == indexOf(truth.clockSources, truth.clockSource)) << "and whose default is what the device is running on";

        const double externalClock = indexOf(truth.clockSources, "external");
        const double cameUpOn      = indexOf(truth.clockSources, truth.clockSource);
        expect(externalClock >= 0.0) << "this radio offers an external reference";
        expect(cameUpOn >= 0.0) << "and the reference it came up on is in its own list";

        /*| frame: neither selection is written to a streaming device. UHD states that
                reconfiguring the clock source affects the FPGA clocking and the timekeeping,
                that there should be no streaming operation while it happens, and that a
                device time set beforehand is most likely lost, so a control stages the
                choice and the next start writes it before anything else. This measures all
                three halves: the write is accepted, the streaming device does not move, and
                a fresh start comes up on what was asked for.
        */
        std::string whileRunning, timeWhileRunning, readingWhileRunning;
        bool        staged = false, refusedOutOfRange = true, everyTimeSourceStaged = true;

        const bool ran = withGraph(sourceSettings(2.0e6, bandFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            const auto readback = [&src](const char* which) {
                return deviceRead(src, std::string{}, [which](uhd::usrp::multi_usrp& u) { return std::string(which) == "clock" ? u.get_clock_source(0) : u.get_time_source(0); });
            };
            for (std::size_t i = 0; i < truth.timeSources.size(); ++i) {
                if (!src.setControl("TIME_SOURCE", static_cast<double>(i))) {
                    everyTimeSourceStaged = false;
                }
            }
            staged            = src.setControl("CLOCK_SOURCE", externalClock);
            refusedOutOfRange = !src.setControl("CLOCK_SOURCE", static_cast<double>(truth.clockSources.size()));
            settle(400);
            whileRunning     = readback("clock");
            timeWhileRunning = readback("time");
            std::vector<capture::SensorReading> readings;
            if (src.readSensors(readings)) {
                const auto it       = std::ranges::find_if(readings, [](const auto& r) { return r.id == "clock_source"; });
                readingWhileRunning = it == readings.end() ? std::string{} : it->value;
            }
        });
        expect(ran) << "the graph ran";

        // A fresh start, which is where a staged selection reaches the device.
        std::string      onExternal, timeOnExternal, refOnExternal, readingOnExternal;
        gr::property_map external      = sourceSettings(2.0e6, bandFreq(), truth.gainMinDb, sourcePort());
        external["clock_source"]       = std::string("external");
        const bool startedOnExternal = withGraph(std::move(external), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts on the external reference";
            onExternal     = deviceRead(src, std::string{}, [](uhd::usrp::multi_usrp& u) { return u.get_clock_source(0); });
            timeOnExternal = deviceRead(src, std::string{}, [](uhd::usrp::multi_usrp& u) { return u.get_time_source(0); });
            /*| trap: a boolean UHD sensor carries "true" or "false" in value and its state
                    word in unit, which is the way round toReading() publishes them. */
            refOnExternal = deviceRead(src, std::string("unread"), [](uhd::usrp::multi_usrp& u) { return u.get_mboard_sensor("ref_locked", 0).to_bool() ? std::string("locked") : std::string("unlocked"); });
            std::vector<capture::SensorReading> readings;
            if (src.readSensors(readings)) {
                const auto it     = std::ranges::find_if(readings, [](const auto& r) { return r.id == "clock_source"; });
                readingOnExternal = it == readings.end() ? std::string{} : it->value;
            }
        });
        expect(startedOnExternal) << "the second graph ran";

        // And back to the reference the radio came up on, which is where it is left.
        std::string      onRestored;
        gr::property_map restore = sourceSettings(2.0e6, bandFreq(), truth.gainMinDb, sourcePort());
        restore["clock_source"]  = truth.clockSource;
        restore["time_source"]   = truth.timeSource;
        const bool restored      = withGraph(std::move(restore), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts on the reference the radio came up on";
            onRestored = deviceRead(src, std::string{}, [](uhd::usrp::multi_usrp& u) { return u.get_clock_source(0); });
        });
        expect(restored) << "the third graph ran";

        std::printf("clock: with a staged external reference the streaming device still reads clock \"%s\", time \"%s\"; the reading says \"%s\"\n", whileRunning.c_str(), timeWhileRunning.c_str(),
                    readingWhileRunning.c_str());
        std::printf("clock: a start on the staged selection reads clock \"%s\", time \"%s\", ref_locked \"%s\"; the reading says \"%s\"\n", onExternal.c_str(), timeOnExternal.c_str(),
                    refOnExternal.c_str(), readingOnExternal.c_str());
        std::printf("clock: the radio is left on \"%s\", which reads back as \"%s\"\n", truth.clockSource.c_str(), onRestored.c_str());

        expect(staged) << "a reference the device listed is accepted";
        expect(everyTimeSourceStaged) << "and so is every timing source it listed";
        expect(refusedOutOfRange) << "an index past the device's own list is refused";
        expect(whileRunning == truth.clockSource) << "the streaming device stays on the reference it came up on, the write being staged rather than made";
        expect(timeWhileRunning == truth.timeSource) << "and on its timing source";
        expect(readingWhileRunning == whileRunning) << "and the reading says what the device holds rather than what was asked for";
        expect(onExternal == std::string("external")) << "the next start takes the external reference";
        expect(readingOnExternal == std::string("external")) << "and the reading follows it";
        expect(onRestored == truth.clockSource) << "and the reference the radio came up on is restored";
        /*| frame: whether an external reference locks with nothing on the connector is the
                radio's own answer, not a fault either way. A B2xx never locks there. An
                N-series whose GPSDO drives the same internal reference path reads locked,
                because the oscillator it is disciplined by is inside the box.
        */
        std::printf("clock: with the clock source external the reference reads \"%s\"; this radio's own reference is %s\n", refOnExternal.c_str(),
                    refOnExternal == "locked" ? "driven from inside the box, so external finds a reference there" : "not driven, so external finds nothing");
        expect(refOnExternal == std::string("locked") || refOnExternal == std::string("unlocked")) << "the reference sensor answers one of its two states";
    });

    cases("uhd.reference-lock-wait", [] {
        /*| frame: the wait is skipped on an internal clock, which is locked by construction,
                and spent on every other. A radio picks the clock it comes up on for itself:
                a B2xx comes up internal and waits nothing, an N-series with a GPSDO comes up
                on gpsdo and waits until the oscillator answers locked.
        */
        const auto& truth = benchTruth();
        double      firstWaitMs = -1.0;
        double      freshWaitMs = -1.0;
        std::string firstClock;
        std::string freshClock;
        double      measuredWaitMs = 0.0;
        bool        measuredLock   = false;
        bool        externalTried  = false;
        std::uint64_t samplesOnExternal = 0;

        const bool first = withGraph(sourceSettings(2.0e6, bandFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            firstWaitMs = src.refLockWaitMs();
            firstClock  = deviceRead(src, std::string{}, [](uhd::usrp::multi_usrp& u) { return u.get_clock_source(0); });
        });
        expect(first) << "the first graph ran";

        /*| frame: the same bounded wait against the external reference this radio offers,
                which a start makes for itself. The selection is a setting rather than a
                control write, because the reference is written before the streamer exists.
        */
        if (std::ranges::find(truth.clockSources, "external") != truth.clockSources.end()) {
            externalTried                = true;
            gr::property_map onExternal  = sourceSettings(2.0e6, bandFreq(), truth.gainMinDb, sourcePort());
            onExternal["clock_source"]   = std::string("external");
            const bool ranOnExternal     = withGraph(std::move(onExternal), [&](capture::uhd::Source& src, auto&) {
                expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts on the external reference";
                measuredWaitMs    = src.refLockWaitMs();
                measuredLock      = measuredWaitMs < capture::uhd::kRefLockLimitMs;
                samplesOnExternal = g_samples.load();
            });
            expect(ranOnExternal) << "the graph on the external reference ran";
        }

        // A fresh start on the reference the radio came up on, which is where it is left.
        gr::property_map back = sourceSettings(2.0e6, bandFreq(), truth.gainMinDb, sourcePort());
        back["clock_source"]  = truth.clockSource;
        const bool second     = withGraph(std::move(back), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts again";
            freshWaitMs = src.refLockWaitMs();
            freshClock  = deviceRead(src, std::string{}, [](uhd::usrp::multi_usrp& u) { return u.get_clock_source(0); });
        });
        expect(second) << "the second graph ran";

        std::printf("reference: the radio came up on the \"%s\" clock; the first start waited %.0f ms and the second, on \"%s\", waited %.0f ms\n", firstClock.c_str(), firstWaitMs, freshClock.c_str(),
                    freshWaitMs);
        if (externalTried) {
            std::printf("reference: a start on the external reference waited %.0f ms, %s, and delivered %llu samples\n", measuredWaitMs, measuredLock ? "which means it locked" : "the whole bound",
                        static_cast<unsigned long long>(samplesOnExternal));
        }

        /*| frame: this asserts the wait's bound and not its outcome. A reference that is
                there costs one poll; one that is not costs the whole limit; either way the
                stream starts. This bench gives one or the other, depending on the connector
                and on whether the radio drives its own reference path.
        */
        for (const auto& [clock, waited] : {std::pair{firstClock, firstWaitMs}, std::pair{freshClock, freshWaitMs}}) {
            if (clock == "internal") {
                expect(waited == 0.0) << "an internal clock is locked by construction and is not waited for";
            } else {
                expect(waited >= 0.0 && waited <= 2.0 * capture::uhd::kRefLockLimitMs) << "every other clock is waited for, and for no longer than the bound";
            }
        }
        if (externalTried) {
            expect(measuredWaitMs <= 2.0 * capture::uhd::kRefLockLimitMs) << "the bounded wait costs at most its limit";
            expect(samplesOnExternal > 0UL) << "and the stream starts on that reference whether it locked or not";
        }
        expect(g_samples.load() > 0UL) << "and the stream runs either way";
    });

    cases("uhd.lo-offset", [] {
        const auto&      truth   = benchTruth();
        const double     kCenter = bandFreq();
        constexpr double kRate   = 2.0e6;
        constexpr double kOffset = 1.0e6;
        double           centerNoOffset = 0.0, rfNoOffset = 0.0, dcNoOffset = 0.0;
        double           centerOffset = 0.0, rfOffset = 0.0, dcOffset = 0.0;
        double           descMinMHz = 0.0, descMaxMHz = 0.0;
        bool             descRateDerived = false;
        double           rfAtTop = 0.0, rfAfterRefusal = 0.0, rfBounded = 0.0;
        bool             offsetRefused = false, boundedAccepted = false;

        const bool ran = withGraph(sourceSettings(kRate, kCenter, truth.gainMaxDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            // The automatic DC correction would remove the very spur this moves, so it is
            // written off for the measurement where the frontend carries one.
            std::ignore = src.setControl("DC_CORR", 0.0);
            settle(400);
            resetPower();
            settle(600);
            centerNoOffset = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
            rfNoOffset     = src.tunedRfHz();
            dcNoOffset     = dcDbfs();

            expect(src.setControl("LO_OFFSET", kOffset / 1e6)) << "a one megahertz offset is accepted";
            settle(500);
            resetPower();
            settle(600);
            centerOffset = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
            rfOffset     = src.tunedRfHz();
            dcOffset     = dcDbfs();

            // An offset beyond the descriptor's own bound is held at that bound — half the
            // rate in force — rather than written as asked.
            boundedAccepted = src.setControl("LO_OFFSET", 10.0 * kRate / 1e6);
            settle(500);
            rfBounded = src.tunedRfHz();

            /*| frame: the frontend sits at the center plus the offset, so an offset that
                    carries it past the end of the stated coverage is refused rather than
                    coerced there quietly. The offset asked for is the descriptor's own
                    bound, at a center near the top of the coverage.
            */
            std::ignore = src.setControl("LO_OFFSET", 0.0);
            std::ignore = src.settings().setStaged({{"frequency", std::vector<double>{insideCoverage(truth, 1.0) - 10.0e3}}});
            settle(500);
            rfAtTop        = src.tunedRfHz();
            offsetRefused  = !src.setControl("LO_OFFSET", kRate / 2.0 / 1e6);
            settle(300);
            rfAfterRefusal = src.tunedRfHz();
        });
        expect(ran) << "the graph ran";

        const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = kRate, .centerFreq = kCenter};
        for (const auto& c : capture::uhd::Source::describeControls(truth, ctx)) {
            if (c.id == "LO_OFFSET") {
                descMinMHz      = c.min;
                descMaxMHz      = c.max;
                descRateDerived = c.rateDerived;
            }
        }

        /*| frame: a synthesizer moves in steps, so the RF frontend lands near the offset
                rather than on it, and no UHD call on a radio without a controllable LO
                stage states that step. The tolerance is 10 kHz, which is the order a WBX's
                stepped LO misses by: a 1 MHz offset moved its RF by 1.001221 MHz. A B2xx's
                continuous synthesizer lands inside a hertz.
        */
        constexpr double stepHz = 10.0e3;
        std::printf("lo offset: with none, get_rx_freq %.3f Hz, the RF frontend %.3f Hz, the DC bin %.2f dBFS\n", centerNoOffset, rfNoOffset, dcNoOffset);
        std::printf("lo offset: with %.0f Hz, get_rx_freq %.3f Hz, the RF frontend %.3f Hz, the DC bin %.2f dBFS\n", kOffset, centerOffset, rfOffset, dcOffset);
        std::printf("lo offset: the RF frontend moved %.3f Hz for an offset of %.0f Hz, %.3f Hz off, against a tolerance of %.0f Hz\n", rfOffset - rfNoOffset, kOffset,
                    (rfOffset - rfNoOffset) - kOffset, stepHz);
        std::printf("lo offset: the descriptor runs %.3f to %.3f MHz, rate-derived %s\n", descMinMHz, descMaxMHz, descRateDerived ? "yes" : "no");

        expect(std::abs(centerOffset - kCenter) < 1.0) << "the center frequency is still the one asked for";
        expect(std::abs(centerNoOffset - kCenter) < 1.0) << "as it was without the offset";
        expect(std::abs((rfOffset - rfNoOffset) - kOffset) < stepHz) << "while the RF frontend moved by the offset, inside the step its synthesizer takes";
        expect(dcOffset < dcNoOffset - 10.0) << "and the DC spur is no longer in the center bin";
        expect(std::abs(descMaxMHz - kRate / 2.0 / 1e6) < 1e-9) << "the descriptor is bounded by half the rate in force";
        expect(std::abs(descMinMHz + kRate / 2.0 / 1e6) < 1e-9) << "at each end";
        expect(descRateDerived) << "and re-derives when the rate moves";

        std::printf("lo offset: an offset of %.0f Hz, ten times the rate, moved the frontend to %.3f Hz, which is %.3f Hz from where it sat with none — half the rate is %.0f Hz\n", 10.0 * kRate,
                    rfBounded, rfBounded - rfNoOffset, kRate / 2.0);
        std::printf("lo offset: at a center 10 kHz below the top of the coverage the frontend held %.3f Hz; an offset of half the rate was %s and the frontend then held %.3f Hz\n", rfAtTop,
                    offsetRefused ? "refused" : "taken", rfAfterRefusal);
        expect(boundedAccepted) << "an offset beyond the descriptor's bound is held at the bound rather than refused";
        expect(std::abs((rfBounded - rfNoOffset) - kRate / 2.0) < stepHz) << "and the frontend moves by half the rate, which is that bound";
        expect(offsetRefused) << "an offset that would carry the frontend past the stated coverage is refused rather than coerced";
        expect(std::abs(rfAfterRefusal - rfAtTop) < 1.0) << "and the tuner stays where it was, as a refused frequency leaves it";
    });

    cases("uhd.device-time", [] {
        constexpr double       kRate = 2.0e6;
        const auto&            truth = benchTruth();
        double                 hostOffMs     = 0.0;
        double                 worstHostMs   = 0.0;
        double                 worstDeviceMs = 0.0;
        std::vector<TagRecord> deviceTags;
        std::uint64_t          overflowsSeen = 0;

        // The host clock as the tag source, the source a toggle left off gives.
        const bool first = withGraph(sourceSettings(kRate, bandFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source&, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(1500);
        });
        expect(first) << "the first graph ran";
        hostOffMs = (static_cast<double>(g_firstTriggerNs.load()) - static_cast<double>(g_firstSampleWallNs.load())) / 1e6;
        {
            std::lock_guard g(g_tagLock);
            for (const TagRecord& r : g_tagTimes) {
                worstHostMs = std::max(worstHostMs, std::abs((static_cast<double>(r.timeNs) - static_cast<double>(r.seenWallNs)) / 1e6));
            }
        }

        // The device clock as the tag source.
        const bool second = withGraph(sourceSettings(kRate, bandFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            expect(src.setControl("DEVICE_TIME", 1.0)) << "the device-time toggle is accepted";
            /*| trap: on a radio with a pulse-per-second time source the device clock is set
                    on the next edge, up to a second away, so the tags either side of that
                    edge are counted against two different clocks and the pair spanning it
                    reads as a jump. The wait covers the edge.
            */
            settle(2500);
            {
                std::lock_guard g(g_tagLock);
                g_tagTimes.clear();
            }
            const std::uint64_t before = g_overflowTags.load();
            settle(2500);
            overflowsSeen = g_overflowTags.load() - before;
            std::lock_guard g(g_tagLock);
            deviceTags = g_tagTimes;
        });
        expect(second) << "the second graph ran";
        for (const TagRecord& r : deviceTags) {
            worstDeviceMs = std::max(worstDeviceMs, std::abs((static_cast<double>(r.timeNs) - static_cast<double>(r.seenWallNs)) / 1e6));
        }

        std::printf("device time: with the toggle off the first tag sat %.1f ms from the wall clock at the first sample, and every tag within %.1f ms of the host clock as it arrived\n", hostOffMs,
                    worstHostMs);
        std::printf("device time: with it on, %zu tags, %llu overflow tags, every tag within %.1f ms of the host clock as it arrived\n", deviceTags.size(),
                    static_cast<unsigned long long>(overflowsSeen), worstDeviceMs);

        double worstResidualUs = 0.0;
        for (std::size_t i = 1; i < deviceTags.size(); ++i) {
            const double byTime    = (static_cast<double>(deviceTags[i].timeNs) - static_cast<double>(deviceTags[i - 1].timeNs)) / 1e9;
            const double bySamples = static_cast<double>(deviceTags[i].index - deviceTags[i - 1].index) / kRate;
            const double residual  = (byTime - bySamples) * 1e6;
            std::printf("device time: tag %zu advanced %.9f s over %zu samples, which is %.9f s at the rate, residual %.3f us\n", i, byTime, deviceTags[i].index - deviceTags[i - 1].index, bySamples,
                        residual);
            worstResidualUs = std::max(worstResidualUs, std::abs(residual));
        }
        std::printf("device time: the worst residual is %.3f us\n", worstResidualUs);

        expect(deviceTags.size() >= 3UZ) << "the stream carried several timing tags";
        expect(overflowsSeen == 0UL) << "with no gap to break the count of samples between them";
        expect(worstDeviceMs < 100.0) << "the device clock was set from the host, so its tags read as wall-clock times";
        expect(worstResidualUs < 2.0) << "and the tags advance by exactly the samples between them at the rate";
    });

    cases("uhd.device-time-set-midstream", [] {
        /*| frame: setting the device clock moves it from the seconds since the device came
                up to a wall-clock time, about 1.79e9 seconds further on. Two things have to
                survive that move: the count of what the radio shed, which is measured from
                the stream's own ticks, and the timing tags, which must never name 1970.
            trap: on a pulse-per-second time source the move lands on the next edge, up to a
                second after the write, so this measures the window either side of it.
        */
        constexpr double       kRate = 2.0e6;
        const auto&            truth = benchTruth();
        long long              droppedBefore = -1, droppedAfter = -1;
        bool                   toggled     = false;
        std::uint64_t          toggledAtNs = 0;
        std::vector<TagRecord> tags;

        const bool ran = withGraph(sourceSettings(kRate, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(2000);
            droppedBefore = droppedOf(src);
            {
                std::lock_guard g(g_tagLock);
                g_tagTimes.clear();
            }
            toggledAtNs = capture::wallClockNs();
            toggled     = src.setControl("DEVICE_TIME", 1.0);
            settle(3000);
            droppedAfter = droppedOf(src);
            std::lock_guard g(g_tagLock);
            tags = g_tagTimes;
        });
        expect(ran) << "the graph ran";

        std::uint64_t earliest = std::numeric_limits<std::uint64_t>::max();
        double        worstMs  = 0.0;
        for (const TagRecord& r : tags) {
            earliest = std::min(earliest, r.timeNs);
            worstMs  = std::max(worstMs, std::abs((static_cast<double>(r.timeNs) - static_cast<double>(r.seenWallNs)) / 1e6));
        }
        std::printf("clock set: the time source is \"%s\", which %s a pulse per second; the shed count read %lld before the set and %lld after it, a rise of %lld\n", truth.timeSource.c_str(),
                    capture::uhd::Source::timeSourceHasPps(truth.timeSource) ? "carries" : "does not carry", droppedBefore, droppedAfter, droppedAfter - droppedBefore);
        std::printf("clock set: %zu tags after the set; the earliest reads %llu ns against a host clock of %llu ns at the write, and the worst tag sat %.1f ms from the host clock as the sink took "
                    "delivery\n",
                    tags.size(), static_cast<unsigned long long>(tags.empty() ? 0UL : earliest), static_cast<unsigned long long>(toggledAtNs), worstMs);

        expect(toggled) << "the device-time toggle is accepted";
        expect(droppedBefore >= 0 && droppedAfter >= 0) << "the source states what the radio shed";
        expect(droppedAfter - droppedBefore < static_cast<long long>(kRate)) << "and setting the device clock sheds nothing: the tick grid moved, the stream did not";
        expect(!tags.empty()) << "the stream carried timing tags across the set";
        expect(earliest > 1'000'000'000'000'000'000UL) << "none of them names 1970, which a device clock read before the edge latched would give";
        expect(worstMs < 500.0) << "and every one of them reads as a host wall-clock time";
    });

    cases("uhd.tag-time-after-a-stall", [] {
        /*| frame: what a timing tag names once the host has stopped draining for a while.
                The device stamps the first sample of every packet, so tags stay the samples
                between them apart however long the transport held them; a tag taken from the
                host clock at the drain jumps forward by the whole backlog instead.
            trap: the residual is measured between consecutive tags against the samples
                between them. The host clock at the moment the sink took delivery is late by
                that backlog by construction, so it measures the delivery and not the tag.
        */
        const auto&            truth  = benchTruth();
        const auto             ladder = capture::uhd::Source::sampleRatesFor(truth);
        const double           kRate  = capture::uhd::snapToLadder(8.0e6, ladder);
        constexpr int          kStallMs = 300;
        std::vector<TagRecord> tags;
        long long              shed            = -1;
        double                 worstDeliveryMs = 0.0;

        gr::property_map settings = sourceSettings(kRate, defaultFreq(), truth.gainMinDb, sourcePort());
        settings["tag_interval"]  = 0.25f;
        const bool ran            = withGraph(std::move(settings), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(8000))) << "the stream starts";
            settle(1000);
            const long long shed0 = droppedOf(src);
            {
                std::lock_guard g(g_tagLock);
                g_tagTimes.clear();
            }
            g_stallMs.store(kStallMs);
            settle(2000);
            shed = droppedOf(src) < 0 ? -1 : droppedOf(src) - shed0;
            std::lock_guard g(g_tagLock);
            tags = g_tagTimes;
        });
        expect(ran) << "the graph ran";

        double worstResidualUs = 0.0;
        for (std::size_t i = 1; i < tags.size(); ++i) {
            const double byTime    = (static_cast<double>(tags[i].timeNs) - static_cast<double>(tags[i - 1].timeNs)) / 1e9;
            const double bySamples = static_cast<double>(tags[i].index - tags[i - 1].index) / kRate;
            worstResidualUs        = std::max(worstResidualUs, std::abs((byTime - bySamples) * 1e6));
        }
        for (const TagRecord& r : tags) {
            worstDeliveryMs = std::max(worstDeliveryMs, std::abs((static_cast<double>(r.timeNs) - static_cast<double>(r.seenWallNs)) / 1e6));
        }
        std::printf("stall timing: %zu tags at %.0f S/s across a %d ms stall; the radio shed %lld samples; the worst residual between two tags is %.1f us and the worst gap to the host clock as the "
                    "sink took delivery is %.1f ms\n",
                    tags.size(), kRate, kStallMs, shed, worstResidualUs, worstDeliveryMs);

        expect(tags.size() >= 3UZ) << "the stream carried tags either side of the stall";
        if (shed > 0) {
            std::printf("stall timing: the stall cost samples at this rate, so the samples between two tags no longer measure the time between them; the residual is printed and not asserted\n");
            return;
        }
        expect(worstResidualUs < 2000.0) << "the tags stay the samples between them apart, so each names when its own block was taken rather than when the host drained it";
    });

    cases("uhd.tx-burst", [] {
        /*| frame: a -20 dBFS tone into the terminated transmit port at 0 dB gain, carried to
                the block by a graph. Nothing here raises the transmit gain.
            frame: the pause and the resume are the block's own lifecycle hooks, driven here the
                way the scheduler drives them.
        */
        constexpr double kRate = 2.0e6;
        const double     kFreq = bandFreq();
        std::string      line;
        bool             deviceWasUp = false;
        bool             heldFlag    = false;
        bool             resumedFlag = true;
        long long        queued      = -1;
        std::uint64_t    events      = 0;

        const bool ran = withTxGraph(sinkSettings(kRate, kFreq, {}), [&](capture::uhd::Sink& sink, auto&) {
            deviceWasUp = waitForDeviceUp(sink, std::chrono::seconds(10));
            settle(100);
            line        = sink.describe();
            queued      = static_cast<long long>(sink.queuedSamples());
            events      = sink.underrunEvents();
            sink.pause();
            settle(300);
            heldFlag = sink.carrierHeld();
            sink.resume();
            settle(300);
            resumedFlag = sink.carrierHeld();
        });
        expect(ran) << "the transmit graph ran";

        /*| frame: the connector and the filter the device itself gave the sink, read from a
                direct open so the status line is judged against the radio rather than against
                a name. A B2xx's filter follows the rate; a WBX's is fixed at 40 MHz and the
                line says that width.
        */
        std::string txAntenna;
        double      txBandwidth = 0.0;
        try {
            auto       usrp    = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            const auto offered = usrp->get_tx_antennas(0);
            txAntenna          = capture::uhd::Sink::chooseAntenna(offered, {});
            usrp->set_tx_rate(kRate, 0);
            const auto range = usrp->get_tx_bandwidth_range(0);
            usrp->set_tx_bandwidth(capture::uhd::bandwidthFor(usrp->get_tx_rate(0), range.start(), range.stop()), 0);
            txBandwidth = usrp->get_tx_bandwidth(0);
        } catch (const std::exception& e) {
            std::printf("tx burst: the direct open failed: %s\n", e.what());
        }

        std::printf("tx burst: the sink says \"%s\"; %lld samples were queued mid-burst and %llu transmit gaps were reported\n", line.c_str(), queued,
                    static_cast<unsigned long long>(events));
        std::printf("tx burst: a direct open gives the connector \"%s\" and the filter %.3f MHz\n", txAntenna.c_str(), txBandwidth / 1e6);
        std::printf("tx burst: a pause left the carrier %s and the resume left it %s\n", heldFlag ? "held" : "running", resumedFlag ? "held" : "running");

        expect(deviceWasUp) << "the device opens for transmit";
        expect(!txAntenna.empty()) << "the device offers a transmit connector";
        expect(line.find(txAntenna) != std::string::npos) << "the status line names the connector the device gave it";
        expect(line.find(std::format("{:.6f} MHz", kFreq / 1e6)) != std::string::npos) << "and the frequency it reached";
        expect(line.find("gain 0.0 dB") != std::string::npos) << "at 0 dB gain";
        expect(line.find(std::format("filter {:.3f} MHz", txBandwidth / 1e6)) != std::string::npos) << "and the filter the device holds";
        expect(heldFlag) << "a pause ends the burst and holds the carrier down";
        expect(!resumedFlag) << "and the resumed stream sends again";
        expect(deviceOpens()) << "and the device is released when the graph stops";
    });

    cases("uhd.tx-tagged-bursts", [] {
        /*| frame: three bursts of 0.1 s of a -20 dBFS tone into the terminated transmit port at
                0 dB gain, each opened by tx_sob and ended by tx_eob, the second timed by tx_time
                half a second ahead of the device clock. Nothing here raises the transmit gain.
            frame: the case waits for the device's burst acknowledgments before it stops the
                graph, since the stop finds no burst open and waits for none.
        */
        constexpr double kRate  = 2.0e6;
        const double     kFreq  = bandFreq();
        bool             upFlag = false;
        std::uint64_t    acks = 0, ends = 0, gaps = 0, lost = 0, late = 0;
        std::optional<std::int64_t> timedAt;
        capture::uhd::Sink::BurstEnd stopSaid = capture::uhd::Sink::BurstEnd::Unconfirmed;

        gr::Graph flow;
        auto&     tone = flow.emplaceBlock<hw_burst_tone>({{"name", std::string("bursts")}});
        auto&     sink = flow.emplaceBlock<capture::uhd::Sink>(sinkSettings(kRate, kFreq, {}));
        tone.deviceNowNs = [&sink]() -> std::optional<std::int64_t> {
            std::lock_guard lock(sink._ctrlMutex);
            if (sink._usrp == nullptr) {
                return std::nullopt;
            }
            return sink._usrp->get_time_now(0).to_ticks(1e9);
        };
        expect(fatal(flow.connect<"out", "in">(tone, sink).has_value())) << "the burst tone wires to the sink";
        Scheduled scheduled;
        expect(fatal(scheduled.adopt(std::move(flow)).has_value())) << "the transmit graph builds";
        scheduled.start();
        upFlag              = waitForDeviceUp(sink, std::chrono::seconds(10));
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        while (sink.burstAcks() < 3U && Clock::now() < deadline) {
            settle(10);
        }
        acks    = sink.burstAcks();
        ends    = sink._endsPlaced.load();
        gaps    = sink.underrunEvents();
        lost    = sink.sequenceErrors();
        late    = sink.latePackets();
        timedAt = tone.timedAtNs;
        scheduled.stop();
        stopSaid = sink.stopBurstEnd();

        std::printf("tx tagged bursts: %llu ends placed, %llu burst acknowledgments, %llu transmit gaps, %llu packets lost, %llu packets late\n", static_cast<unsigned long long>(ends),
                    static_cast<unsigned long long>(acks), static_cast<unsigned long long>(gaps), static_cast<unsigned long long>(lost), static_cast<unsigned long long>(late));
        std::printf("tx tagged bursts: the second burst carried the device time %lld ns; the stop found %s\n", static_cast<long long>(timedAt.value_or(-1)),
                    stopSaid == capture::uhd::Sink::BurstEnd::NoneOpen ? "no burst open" : "a burst open");

        expect(upFlag) << "the device opens for transmit";
        expect(timedAt.has_value()) << "the second burst carried a time";
        expect(ends == 3U) << "each tx_eob placed one end of burst";
        expect(acks == 3U) << "and the device acknowledged each";
        expect(late == 0U) << "the timed burst reached the device ahead of its time";
        expect(stopSaid == capture::uhd::Sink::BurstEnd::NoneOpen) << "the last tx_eob left no burst for the stop to end";
        expect(deviceOpens()) << "and the device is released when the graph stops";
    });

    cases("uhd.tx-loopback", [] {
        /*| frame: the radio receives on RX2 while it transmits into the terminated TX/RX port
                at 0 dB gain. Only leakage crosses between the two ports, far below the noise
                floor across the whole band, so the tone is measured in the one bin it lands in
                rather than as a mean power.
        */
        constexpr double kRate = 2.0e6;
        constexpr double kBin  = 0.125;
        double           floorBin = 0.0, burstBin = 0.0, afterBin = 0.0;
        double           floorBand = 0.0, burstBand = 0.0;
        std::string      sinkLine;
        bool             sinkRan = false;
        bool             sinkUp  = false;

        double rateBefore = 0.0, rateAfter = 0.0, clockBefore = 0.0, clockAfter = 0.0;

        const bool ran = withGraph(sourceSettings(kRate, bandFreq(), benchTruth().gainMaxDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the receive stream starts";
            g_accum.setBinFreq(kBin);
            resetPower();
            settle(700);
            floorBin    = binDbfs();
            floorBand   = meanDbfs();
            rateBefore  = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
            clockBefore = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_master_clock_rate(0); });

            sinkRan = withTxGraph(sinkSettings(kRate, bandFreq(), {}), [&](capture::uhd::Sink& sink, auto&) {
                /*| frame: the sink opens its device on the scheduler's start, which takes about
                        1.5 s on a B205mini and 2 s on an N210, so the case waits for the open
                        before it reads the sink or measures the burst.
                */
                sinkUp = waitForDeviceUp(sink, std::chrono::seconds(10));
                settle(400);
                sinkLine = sink.describe();
                /*| trap: the two chains share one device, so a transmit rate that moves the
                        master clock moves the receive rate under a stream that is already
                        running. These two reads say whether that happened. */
                rateAfter  = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
                clockAfter = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_master_clock_rate(0); });
                resetPower();
                settle(700);
                burstBin  = binDbfs();
                burstBand = meanDbfs();
            });

            settle(400);
            resetPower();
            settle(700);
            afterBin = binDbfs();
        });
        expect(ran) << "the receive graph ran";

        if (!sinkUp) {
            std::printf("tx loopback: the radio would not run both streamers\n");
        }
        std::printf("tx loopback: the sink says \"%s\"\n", sinkLine.c_str());
        std::printf("tx loopback: the tone bin reads %.2f dBFS with nothing transmitting, %.2f dBFS during the burst and %.2f dBFS after it\n", floorBin, burstBin, afterBin);
        std::printf("tx loopback: the whole band reads %.2f dBFS and %.2f dBFS over the same two windows, a rise of %.2f dB against the bin's %.2f dB\n", floorBand, burstBand, burstBand - floorBand,
                    burstBin - floorBin);
        std::printf("tx loopback: the receive rate read %.3f S/s on a %.0f Hz master clock before the sink opened and %.3f S/s on %.0f Hz after\n", rateBefore, clockBefore, rateAfter, clockAfter);

        expect(sinkRan) << "the transmit graph built and ran";
        expect(sinkUp) << "the radio runs a receive and a transmit streamer at once";
        expect(burstBin > floorBin + 8.0) << "the burst lifts the bin it lands in clear of the floor";
        expect(burstBin > afterBin + 8.0) << "and the bin falls back when the burst ends";
        /*| frame: how much leakage crosses from the transmit port to the receive one is a
                property of the radio and of what is on its connectors: a B205mini's bin rose
                22.9 dB over a band that moved 0.3 dB, and an N210 with a WBX rose 87.5 dB over
                a band that moved 30.0 dB. Both radios lift the bin the tone lands in further
                than they lift the whole band.
        */
        expect(burstBin - floorBin > burstBand - floorBand) << "and the bin it lands in rises more than the whole band, which is why the tone is measured in its own bin";
    });

    cases("uhd.tx-underrun", [] {
        /*| frame: a burst the graph stops feeding part way through, which a pause does: the
                block ends the burst and the device keys down. A USRP transmits nothing for a
                gap, so what it reports is an event and not a count of samples.
        */
        constexpr double kRate = 2.0e6;
        std::uint64_t    before = 0, after = 0, silence = 0;
        bool             deviceWasUp = false;

        const bool ran = withTxGraph(sinkSettings(kRate, bandFreq(), {}), [&](capture::uhd::Sink& sink, auto&) {
            deviceWasUp = waitForDeviceUp(sink, std::chrono::seconds(10));
            settle(100);
            before      = sink.underrunEvents();
            // The gap: the block stops sending for longer than the device's own transmit
            // buffer holds, and the radio reports the hole it left.
            sink.pause();
            settle(600);
            sink.resume();
            settle(600);
            after   = sink.underrunEvents();
            silence = sink.underrunSamples();
        });
        expect(ran) << "the transmit graph ran";

        std::printf("tx underrun: %llu events before a 600 ms gap mid-burst and %llu after it; the sink reports %llu samples of silence\n", static_cast<unsigned long long>(before),
                    static_cast<unsigned long long>(after), static_cast<unsigned long long>(silence));

        expect(deviceWasUp) << "the device opens for transmit";
        expect(after >= before) << "the transmit gap count only rises";
        expect(silence == 0UL) << "a gap is an event and not silence, because a USRP transmits nothing for one rather than filling it";
        expect(deviceOpens()) << "the device is released";
    });

    cases("uhd.device-time-source", [] {
        /*| frame: a radio with a disciplined oscillator knows UTC; the host does not. With
                the device-time toggle on and a pulse-per-second source in force the block
                latches the oscillator's own second on the next edge, and the timestamps the
                samples carry are then UTC rather than the host's idea of it.
            trap: the edge is up to a second away, so the device clock is still the host's
                until it arrives. This waits it out.
        */
        const auto& truth = benchTruth();
        const bool  hasPps = capture::uhd::Source::timeSourceHasPps(truth.timeSource);
        std::string gpsLocked;
        double      gpsSeconds = 0.0;
        double      lastPps    = 0.0;
        double      timeNow    = 0.0;
        double      hostNow    = 0.0;
        std::vector<TagRecord> tags;
        constexpr double kRate = 2.0e6;

        const bool ran = withGraph(sourceSettings(kRate, bandFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            expect(src.setControl("DEVICE_TIME", 1.0)) << "the device-time toggle is accepted";
            settle(2500); // the next pulse-per-second edge, and then some
            gpsLocked = deviceRead(src, std::string("absent"), [](uhd::usrp::multi_usrp& u) { return u.get_mboard_sensor("gps_locked", 0).to_bool() ? std::string("locked") : std::string("unlocked"); });
            // Through the reading's own text, which is the path the block takes: the
            // accessor beside it is 32 bits wide and 2038 passes that.
            gpsSeconds = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) {
                const auto reading = u.get_mboard_sensor("gps_time", 0);
                return static_cast<double>(capture::uhd::Source::wholeSecondsFrom(reading.value).value_or(static_cast<std::int64_t>(reading.to_real())));
            });
            lastPps    = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_time_last_pps(0).get_real_secs(); });
            timeNow    = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_time_now(0).get_real_secs(); });
            hostNow    = static_cast<double>(capture::wallClockNs()) / 1e9;
            {
                std::lock_guard g(g_tagLock);
                g_tagTimes.clear();
            }
            settle(2500);
            std::lock_guard g(g_tagLock);
            tags = g_tagTimes;
        });
        expect(ran) << "the graph ran";

        std::printf("device time source: the radio's time source is \"%s\", which %s a pulse per second; gps_locked reads \"%s\" and gps_time %.0f\n", truth.timeSource.c_str(),
                    hasPps ? "carries" : "does not carry", gpsLocked.c_str(), gpsSeconds);
        std::printf("device time source: get_time_last_pps reads %.6f s, get_time_now %.6f s, the host clock %.6f s; the device is %.3f s from the host\n", lastPps, timeNow, hostNow, timeNow - hostNow);

        double worstResidualUs = 0.0;
        for (std::size_t i = 1; i < tags.size(); ++i) {
            const double byTime    = (static_cast<double>(tags[i].timeNs) - static_cast<double>(tags[i - 1].timeNs)) / 1e9;
            const double bySamples = static_cast<double>(tags[i].index - tags[i - 1].index) / kRate;
            worstResidualUs        = std::max(worstResidualUs, std::abs((byTime - bySamples) * 1e6));
        }
        std::printf("device time source: %zu tags, worst residual between two of them %.3f us at %.0f S/s\n", tags.size(), worstResidualUs, kRate);

        expect(tags.size() >= 2UZ) << "the stream carried timing tags";
        expect(worstResidualUs < 2.0) << "which advance by exactly the samples between them at the rate";
        expect(std::abs(timeNow - hostNow) < 2.0) << "and the device clock reads as a wall-clock time";
        if (hasPps && gpsLocked == "locked") {
            /*| frame: the oscillator has a fix, so the second latched on the edge is the
                    GPS second. get_time_last_pps is then that second exactly, give or take
                    the seconds that have passed since the block set it. */
            std::printf("device time source: the oscillator has a fix, so the device clock is the GPS one\n");
            expect(std::abs(lastPps - gpsSeconds) < 5.0) << "the device clock at the last pulse is the GPS time to within the seconds since it was set";
            expect(std::abs(std::round(lastPps) - lastPps) < 0.01) << "and it is a whole second, latched on an edge";
        } else {
            std::printf("device time source: no disciplined second was available, so the device clock came from the host\n");
        }
    });

    cases("uhd.wire-format", [] {
        /*| frame: the packing the samples take on the link. sc8 halves the bytes, which on a
                link-bound radio doubles the rate that arrives, and costs the lower eight
                bits of every sample. The rate tried is the top of the sc8 ladder, which is
                above the sc16 ladder's top exactly where the link is the limit.
        */
        const auto&  truth    = benchTruth();
        const auto   wide     = capture::uhd::Source::sampleRatesFor(truth, "sc16");
        const auto   narrow   = capture::uhd::Source::sampleRatesFor(truth, "sc8");
        const double sc16Top  = wide.empty() ? 0.0 : wide.back();
        const double sc8Top   = narrow.empty() ? 0.0 : narrow.back();
        std::printf("wire format: the sc16 ladder tops out at %.0f S/s and the sc8 one at %.0f S/s\n", sc16Top, sc8Top);

        const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = 2.0e6, .centerFreq = defaultFreq()};
        const auto                  controls = capture::uhd::Source::describeControls(truth, ctx);
        const auto                  wire     = std::ranges::find_if(controls, [](const auto& c) { return c.id == "WIRE_FORMAT"; });
        expect(wire != controls.end()) << "the wire format is a control";
        if (wire == controls.end()) {
            return;
        }
        expect(wire->options == std::vector<std::string>{"sc16", "sc8"}) << "over the formats a streamer is built for";

        // Each format opened at the top of its own ladder, and the delivered rate measured.
        for (const auto& [format, rate] : {std::pair{std::string("sc16"), sc16Top}, std::pair{std::string("sc8"), sc8Top}}) {
            if (rate <= 0.0) {
                continue;
            }
            gr::property_map settings = sourceSettings(rate, defaultFreq(), truth.gainMinDb, sourcePort());
            settings["wire_format"]   = format;
            double        achieved    = 0.0;
            std::uint64_t got         = 0;
            double        seconds     = 0.0;
            const bool    ran         = withGraph(std::move(settings), [&](capture::uhd::Source& src, auto&) {
                if (!waitForSamples(std::chrono::milliseconds(8000))) {
                    return;
                }
                achieved                = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
                const std::uint64_t at0 = g_samples.load();
                const auto          t0  = Clock::now();
                settle(1500);
                got     = g_samples.load() - at0;
                seconds = std::chrono::duration<double>(Clock::now() - t0).count();
            });
            const double delivered = seconds > 0.0 ? static_cast<double>(got) / seconds : 0.0;
            std::printf("wire format: %s at %.0f S/s: get_rx_rate %.0f, delivered %.0f S/s, %.1f %% of the rate in force\n", format.c_str(), rate, achieved, delivered,
                        achieved > 0.0 ? 100.0 * delivered / achieved : 0.0);
            expect(ran) << "the graph ran";
            expect(got > 0UL) << "the top of a format's own ladder delivers samples in that format";
        }

        if (!clockIsProgrammable(truth)) {
            expect(sc8Top > sc16Top) << "on a link-bound radio the narrow format reaches a rate the wide one cannot";
        } else {
            std::printf("wire format: this radio's clock is programmable, so its ladder is the picks and the format does not move it\n");
            expect(sc8Top == sc16Top) << "and the ladder is the same either way";
        }

        /*| frame: a format written to a running block. The streamer was built for the format
                in force and keeps it, so the rate ladder must keep it too: a rate reached in
                the narrow format, asked for while the wide one is on the wire, would deliver
                nothing at all and read as a dead stream.
        */
        std::string formatInForce;
        double      rateAfterWrite = 0.0;
        std::uint64_t arrivedAfter = 0;
        const bool    stillRuns    = withGraph(sourceSettings(sc16Top, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(8000))) << "the stream starts at the top of the sc16 ladder";
            expect(src.setControl("WIRE_FORMAT", 1.0)) << "the narrow format is accepted";
            formatInForce = src.wireFormatInForce();
            std::ignore   = src.settings().setStaged({{"sample_rate", sc8Top}});
            settle(1500);
            const std::uint64_t at0 = g_samples.load();
            settle(2000);
            arrivedAfter   = g_samples.load() - at0;
            rateAfterWrite = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
        });
        expect(stillRuns) << "the graph ran";
        std::printf("wire format: after asking for sc8 and then for %.0f S/s mid-stream, the format in force is still \"%s\", the device runs %.0f S/s and %llu samples arrived over two seconds\n",
                    sc8Top, formatInForce.c_str(), rateAfterWrite, static_cast<unsigned long long>(arrivedAfter));
        expect(formatInForce == std::string("sc16")) << "the running streamer keeps the format it was built with";
        expect(rateAfterWrite <= sc16Top + 1.0) << "so the rate asked for is snapped inside what that format's link carries";
        expect(arrivedAfter > 0UL) << "and the stream keeps delivering rather than going dead";

        /*| frame: what the narrow format costs. The descriptor's label says sc8 costs the
                lower eight bits of every sample, which is a claim about UHD's own converter:
                a converter with another scalar would move every level reading by tens of
                decibels with nothing to say so. Reading both formats at the same rate, the
                same gain and the same port settles it.
            trap: at eight bits a part the quantization floor sits about 6 dB above a
                terminated radio's own noise floor, so the two means agreeing within a few
                decibels is the answer and agreeing within one says the signal is above that
                floor.
        */
        const double kLevelRate = capture::uhd::snapToLadder(8.0e6, wide);
        double       levelSc16 = 0.0, levelSc8 = 0.0;
        for (const auto& [format, level] : {std::pair{std::string("sc16"), &levelSc16}, std::pair{std::string("sc8"), &levelSc8}}) {
            gr::property_map settings = sourceSettings(kLevelRate, defaultFreq(), truth.gainMaxDb, sourcePort());
            settings["wire_format"]   = format;
            const bool measured       = withGraph(std::move(settings), [&](capture::uhd::Source&, auto&) {
                expect(waitForSamples(std::chrono::milliseconds(8000))) << "the stream starts";
                settle(2000); // the frontend's own correction loop settles first
                *level = dbfsOver(2.0);
            });
            expect(measured) << "the level graph ran";
        }
        std::printf("wire format: at %.0f S/s and %.1f dB gain the mean level reads %.2f dBFS in sc16 and %.2f dBFS in sc8, a difference of %.2f dB\n", kLevelRate, truth.gainMaxDb, levelSc16, levelSc8,
                    levelSc8 - levelSc16);
        std::printf("wire format: %s\n", std::abs(levelSc8 - levelSc16) < 1.0 ? "the two formats carry the same full scale, and the signal sits above the narrow format's quantization floor"
                                                                             : "the difference is the narrow format's quantization floor rather than another converter scale");
        expect(std::abs(levelSc8 - levelSc16) < 12.0) << "both formats carry the same full scale: a converter with another scalar would move the reading by tens of decibels";
    });

    cases("uhd.stop-latency", [] {
        /*| frame: what a caller waits for when it stops the graph. A receiver reaches stop()
                from the thread that serves its interface, so this wait is the interface's:
                the consumer's receive in flight and then the bounded drain of the transport.
            frame: measured twice, at half a second into the stream and at fifty
                milliseconds, which lands inside the first receive — the window a B2xx spends
                spinning its stream path up.
        */
        const auto& truth = benchTruth();
        const auto  latencyOf = [&truth](int afterMs) {
            resetCounters();
            gr::Graph flow;
            auto&     src  = flow.emplaceBlock<capture::uhd::Source>(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()));
            auto&     sink = flow.emplaceBlock<hw_sink>({{"name", std::string("host")}});
            expect(flow.connect<"out", "in">(src, sink).has_value()) << "the source wires to the sink";
            Scheduled scheduled;
            expect(scheduled.adopt(std::move(flow)).has_value()) << "the graph builds";
            scheduled.start();
            settle(afterMs);
            const auto t0 = Clock::now();
            scheduled.stop();
            return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        };

        const double lateMs   = latencyOf(500);
        const bool   released = deviceOpens();
        const double earlyMs  = latencyOf(50);

        /*| frame: the block's own teardown, timed on its own thread: the consumer's receive
                in flight, the bounded drain, and the device close. A graph's stop() waits
                for this and then for every other block and the scheduler's own threads, so
                this is the part the source owns.
        */
        double teardownMs = 0.0;
        const bool tornDown = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(500);
            const auto t0 = Clock::now();
            std::ignore   = src.hardwareTeardown();
            teardownMs    = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        });
        expect(tornDown) << "the teardown graph ran";

        /*| frame: what the vendor's own open and close cost on this radio, measured with no
                graph at all. A B2xx uploads firmware and tears a USB transport down; an
                N-series opens a socket. The block's own share of a stop is the remainder
                after that close, and the block bounds that remainder.
        */
        double openMs = 0.0, closeMs = 0.0;
        try {
            const auto t0   = Clock::now();
            auto       usrp = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            openMs          = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            const auto t1   = Clock::now();
            usrp.reset();
            closeMs = std::chrono::duration<double, std::milli>(Clock::now() - t1).count();
        } catch (const std::exception& e) {
            std::printf("stop latency: the direct open failed: %s\n", e.what());
        }

        std::printf("stop latency: a graph stopped half a second into the stream took %.0f ms; one stopped fifty milliseconds in, inside the device's own open, took %.0f ms\n", lateMs, earlyMs);
        std::printf("stop latency: the vendor's own open costs %.0f ms on this radio and its close %.0f ms; the block's whole teardown costs %.0f ms, which is %.0f ms beyond that close\n", openMs,
                    closeMs, teardownMs, teardownMs - closeMs);
        expect(teardownMs - closeMs < 1500.0) << "the block's own share of a stop is the receive in flight and the bounded drain, not seconds";
        expect(lateMs < openMs + closeMs + 4000.0) << "a graph's stop costs the block's teardown and the scheduler's own, not an unbounded wait";
        expect(earlyMs < openMs + closeMs + 4000.0) << "and a stop inside the device's own open waits that open out and adds no more";
        expect(released) << "the device is released either way";
        expect(deviceOpens()) << "after every stop";
    });

    cases("uhd.accessors-after-a-stop", [] {
        /*| frame: what the four accessors answer once the block has let the radio go. Three
                of them describe a device this block has released and say so: an empty
                record, no frame, no tuned frequency. The analog filter width is kept on
                purpose, because a caller drawing the control surface of a stopped radio has
                nowhere else to read it, and its own contract says that.
        */
        const auto& truth = benchTruth();
        std::string describedWhileUp, describedAfter;
        std::size_t frameWhileUp = 0UZ, frameAfter = 0UZ;
        double      tunedWhileUp = 0.0, tunedAfter = 0.0;
        double      bwWhileUp = 0.0, bwAfter = 0.0;

        /*| trap: the four readings after the stop are taken inside the graph's own scope,
                the scheduler stopped by hand first. The block is destroyed with the graph, so
                there is no afterwards outside it. */
        bool released = false;
        const bool ran = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto& scheduled) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(300);
            describedWhileUp = src.deviceTruth().describe();
            frameWhileUp     = src.recvFrameSamples();
            tunedWhileUp     = src.tunedRfHz();
            bwWhileUp        = src.analogBandwidthHz();

            scheduled.stop();
            settle(200);
            describedAfter = src.deviceTruth().describe();
            frameAfter     = src.recvFrameSamples();
            tunedAfter     = src.tunedRfHz();
            bwAfter        = src.analogBandwidthHz();
            released       = deviceOpens();
        });
        expect(ran) << "the graph ran";
        expect(released) << "the stop released the device";

        std::printf("after a stop: the record says \"%s\" while it runs and \"%s\" after; the frame holds %zu samples and then %zu; the tuner reached %.0f Hz and then %.0f; the filter reads %.0f Hz "
                    "and then %.0f\n",
                    describedWhileUp.c_str(), describedAfter.c_str(), frameWhileUp, frameAfter, tunedWhileUp, tunedAfter, bwWhileUp, bwAfter);

        expect(!describedWhileUp.empty()) << "a running block states what it opened";
        expect(frameWhileUp > 0UZ) << "and the frame the streamer chose";
        expect(tunedWhileUp > 0.0) << "and the frequency the tuner reached";
        expect(describedAfter.empty()) << "a stopped block states an empty record, as its contract says";
        expect(frameAfter == 0UZ) << "no frame, the streamer having gone";
        expect(tunedAfter == 0.0) << "and no tuned frequency";
        expect(bwAfter == bwWhileUp) << "while the filter width is kept, which is the one contract that says it survives a stop";
        expect(bwAfter > 0.0) << "and it is the width the device settled on";
    });

    cases("uhd.rate-change-midstream", [] {
        /*| frame: a rate written to a streaming radio. The samples already in the transport
                were taken at the old rate, so the block stops the stream, empties the
                transport, writes the rate, starts it again and marks the discontinuity. A
                reader then sees a marked hole and samples at the new rate after it, rather
                than old-rate samples behind the new rate's tag.
            trap: the tag residual is measured only over the tags after the change, at the
                rate in force there. A pair spanning the change measures two rates.
        */
        const auto&  truth  = benchTruth();
        const auto   ladder = capture::uhd::reachableRatesFor(truth).empty() ? capture::uhd::Source::sampleRatesFor(truth) : capture::uhd::reachableRatesFor(truth);
        const double first  = capture::uhd::snapToLadder(2.0e6, ladder);
        const double second = capture::uhd::snapToLadder(8.0e6, ladder);
        double       deviceRate = 0.0, settingRate = 0.0, tagRate = 0.0, refusedSetting = 0.0, refusedDevice = 0.0;
        std::uint64_t gapMarkers = 0, rateTags = 0, arrived = 0;
        double        seconds = 0.0;
        std::vector<TagRecord> after;

        gr::property_map settings = sourceSettings(first, defaultFreq(), truth.gainMinDb, sourcePort());
        settings["tag_interval"]  = 0.25f;
        const bool ran            = withGraph(std::move(settings), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(8000))) << "the stream starts";
            settle(1000);
            const std::uint64_t gaps0 = g_overflowTags.load();
            const std::uint64_t tags0 = g_rateTags.load();
            {
                std::lock_guard g(g_tagLock);
                g_tagTimes.clear();
            }
            std::ignore = src.settings().setStaged({{"sample_rate", second}});
            settle(500);
            gapMarkers = g_overflowTags.load() - gaps0;
            rateTags   = g_rateTags.load() - tags0;
            tagRate    = g_lastRateTagHz.load();
            {
                // Only the tags taken after the change, which is where the new rate holds.
                std::lock_guard g(g_tagLock);
                g_tagTimes.clear();
            }
            const std::uint64_t at0 = g_samples.load();
            const auto          t0  = Clock::now();
            settle(2000);
            arrived     = g_samples.load() - at0;
            seconds     = std::chrono::duration<double>(Clock::now() - t0).count();
            deviceRate  = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
            settingRate = src.sample_rate.value;
            {
                std::lock_guard g(g_tagLock);
                after = g_tagTimes;
            }

            /*| frame: a rate no radio reaches. Whether the vendor library refuses it or
                    coerces it, the setting the block states has to be the rate the device
                    holds: the stream's ticks are counted at the stated rate, and a setting
                    left on a request the radio never took reads as shed samples on every
                    block.
            */
            std::ignore = src.settings().setStaged({{"sample_rate", 1.0}});
            settle(800);
            refusedDevice  = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
            refusedSetting = src.sample_rate.value;
        });
        expect(ran) << "the graph ran";

        double worstResidualUs = 0.0;
        for (std::size_t i = 1; i < after.size(); ++i) {
            const double byTime    = (static_cast<double>(after[i].timeNs) - static_cast<double>(after[i - 1].timeNs)) / 1e9;
            const double bySamples = static_cast<double>(after[i].index - after[i - 1].index) / deviceRate;
            worstResidualUs        = std::max(worstResidualUs, std::abs((byTime - bySamples) * 1e6));
        }
        const double delivered = seconds > 0.0 ? static_cast<double>(arrived) / seconds : 0.0;
        std::printf("rate change: %.0f S/s to %.0f S/s mid-stream: the device runs %.0f, the block states %.0f, the forwarded tag carries %.0f, %llu gap marker(s) and %llu rate tag(s) went out\n",
                    first, second, deviceRate, settingRate, tagRate, static_cast<unsigned long long>(gapMarkers), static_cast<unsigned long long>(rateTags));
        std::printf("rate change: %.0f S/s delivered over %.2f s after the change, %zu tags there with a worst residual of %.1f us\n", delivered, seconds, after.size(), worstResidualUs);
        std::printf("rate change: after asking for 1 S/s the device holds %.0f S/s and the block states %.0f S/s\n", refusedDevice, refusedSetting);

        expect(std::abs(deviceRate - second) <= 0.01 * second) << "the device runs the rate that was asked for";
        expect(std::abs(settingRate - deviceRate) < 1.0) << "and the block states the rate the device runs";
        expect(rateTags >= 1UL && std::abs(tagRate - deviceRate) < 1.0) << "the new rate is forwarded downstream";
        expect(gapMarkers >= 1UL) << "the transport was emptied across the change and the hole is marked";
        expect(std::abs(delivered - deviceRate) <= 0.05 * deviceRate) << "the stream runs at the new rate afterwards";
        if (after.size() >= 2UZ) {
            expect(worstResidualUs < 2000.0) << "and the tags after the change are the samples between them apart, so no old-rate samples were published behind the new rate's tag";
        }
        expect(std::abs(refusedSetting - refusedDevice) <= 0.01 * std::max(refusedDevice, 1.0)) << "a rate the radio cannot take leaves the setting on the rate it does hold";
    });

    cases("uhd.a-drain-at-the-top-rate-ends-by-duration", [] {
        /*| frame: the radio streams at the top of the rate offer its truth states, and the rate
                then moves to the nearest rate under half of it. The rate change stops the
                stream and drains it, and so does the teardown. Each drain states its samples,
                its time and what ended it.
            contract: a drain of a radio that stopped ends on the end-of-burst marker or on a
                receive that finds nothing, within its duration bound, at the top rate too.
        */
        using Source      = capture::uhd::Source;
        const auto& truth = benchTruth();
        const auto  offer = Source::sampleRatesFor(truth);
        const auto  ladder = capture::uhd::reachableRatesFor(truth).empty() ? offer : capture::uhd::reachableRatesFor(truth);
        const double top   = offer.back();
        const double half  = capture::uhd::snapToLadder(top / 2.0, ladder);
        Source::DrainResult atChange, atTeardown;
        std::string         depthAtTop, depthAtHalf;
        std::uint64_t       overflows = 0;
        const auto          depthOf = [](Source& src) {
            std::vector<capture::SensorReading> readings;
            std::ignore = src.readSensors(readings, Source::SensorSweep::Brief);
            for (const auto& r : readings) {
                if (r.id == "rx_transport_depth") {
                    return r.value;
                }
            }
            return std::string();
        };
        const bool ran = withGraph(sourceSettings(top, defaultFreq(), truth.gainMinDb, sourcePort()), [&](Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(8000))) << "the stream starts";
            settle(2000);
            depthAtTop  = depthOf(src);
            std::ignore = src.settings().setStaged({{"sample_rate", half}});
            settle(1000);
            atChange    = src.lastDrain();
            depthAtHalf = depthOf(src);
            overflows   = src.overflowEvents();
            std::ignore = src.hardwareTeardown();
            atTeardown  = src.lastDrain();
        });
        expect(ran) << "the graph ran";
        const auto endName = [](Source::DrainEnd e) {
            switch (e) {
            case Source::DrainEnd::EndOfBurst: return "the end-of-burst marker";
            case Source::DrainEnd::Empty: return "a receive that found nothing";
            case Source::DrainEnd::Deadline: return "the duration bound";
            default: return "a failed receive";
            }
        };
        std::printf("drain: %.0f S/s to %.0f S/s: the rate change drained %zu samples in %.2f ms, ended by %s\n", top, half, atChange.samples, atChange.seconds * 1e3, endName(atChange.end));
        std::printf("drain: the teardown at %.0f S/s drained %zu samples in %.2f ms, ended by %s; %llu overflow event(s) in the run\n", half, atTeardown.samples, atTeardown.seconds * 1e3,
                    endName(atTeardown.end), static_cast<unsigned long long>(overflows));
        std::printf("drain: the receive transport at %.0f S/s: %s; at %.0f S/s: %s\n", top, depthAtTop.c_str(), half, depthAtHalf.c_str());
        for (const auto& d : {atChange, atTeardown}) {
            expect(d.end == Source::DrainEnd::EndOfBurst || d.end == Source::DrainEnd::Empty) << "a stopped radio empties before the bound";
            expect(d.seconds < Source::kDrainLimitS) << "within the duration bound";
        }
        expect(!depthAtTop.empty() && depthAtTop != depthAtHalf) << "the depth reading states the time at the rate in force";
    });

    cases("uhd.coverage-past-the-stated-range", [] {
        /*| frame: two statements about the same thing. This block refuses a frequency
                outside the coverage the device states, on the reading that a USRP's stated
                range already carries the digital down-converter's widening; the vendor
                header states the opposite, that the range "does not include the baseband
                bandwidth" and a caller should assume the reach is half the rate wider at
                each end. On a model that behaves the second way, the block refuses up to
                half a rate of band the radio can hear.
            frame: measured against the radio rather than argued: the block refuses, and then
                a direct open asks the same device for the same frequency and says where its
                tuner landed.
        */
        const auto&  truth  = benchTruth();
        const auto   ladder = capture::uhd::Source::sampleRatesFor(truth);
        const double kRate  = capture::uhd::snapToLadder(8.0e6, ladder);
        double       statedBottom = 0.0;
        double       blockHeld = 0.0, blockForwarded = 0.0, blockBefore = 0.0;
        double       directTarget = 0.0, directActual = 0.0, directRead = 0.0;
        bool         directOpened = false;

        try {
            auto usrp = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            usrp->set_rx_rate(kRate, 0);
            double widest = 0.0;
            for (const auto& r : usrp->get_rx_freq_range(0)) {
                if (r.stop() - r.start() > widest) {
                    widest       = r.stop() - r.start();
                    statedBottom = r.start();
                }
            }
        } catch (const std::exception& e) {
            std::printf("coverage: the first direct open failed: %s\n", e.what());
        }
        expect(statedBottom > 0.0) << "the device states a coverage at this rate";
        if (statedBottom <= 0.0) {
            return;
        }
        const double belowHz = statedBottom - 1.0e6;

        const bool ran = withGraph(sourceSettings(kRate, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(8000))) << "the stream starts";
            blockBefore = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
            std::ignore = src.settings().setStaged({{"frequency", std::vector<double>{belowHz}}});
            settle(500);
            blockHeld      = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_freq(0); });
            blockForwarded = src.frequency->empty() ? 0.0 : src.frequency->front();
        });
        expect(ran) << "the graph ran";

        try {
            auto usrp = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            directOpened = true;
            usrp->set_rx_rate(kRate, 0);
            const auto result = usrp->set_rx_freq(uhd::tune_request_t(belowHz), 0);
            directTarget      = result.target_rf_freq;
            directActual      = result.actual_rf_freq;
            directRead        = usrp->get_rx_freq(0);
        } catch (const std::exception& e) {
            std::printf("coverage: the second direct open failed: %s\n", e.what());
        }

        std::printf("coverage: at %.0f S/s the device states a bottom of %.0f Hz; the block refused %.0f Hz and left the tuner at %.0f Hz, forwarding %.0f Hz\n", kRate, statedBottom, belowHz,
                    blockHeld, blockForwarded);
        std::printf("coverage: a direct open asked the same device for %.0f Hz: the tune targeted %.0f Hz, reached %.0f Hz, and get_rx_freq then read %.0f Hz\n", belowHz, directTarget, directActual,
                    directRead);
        std::printf("coverage: %s\n", directOpened && std::abs(directRead - belowHz) < 1.0e3
                                          ? "the radio reaches past the range it states, so the stated range does not carry the down-converter's widening and this block's refusal is tighter than "
                                            "the radio"
                                          : "the radio does not reach past the range it states, so the stated range already carries the whole reach");

        expect(directOpened) << "the device opens directly";
        expect(std::abs(blockHeld - blockBefore) < 1.0) << "the block's refusal left the tuner where it was";
        expect(std::abs(blockForwarded - blockBefore) < 1.0) << "and what it forwards is what the tuner holds";
    });

    cases("uhd.vendor-agc-claim", [] {
        /*| frame: what UHD does with set_rx_agc on a model whose frontend has no automatic
                gain control. The vendor header states that an exception is thrown; this
                driver's comments said it warns and changes nothing. Neither statement
                decides what the block does — the automatic loop is detected from the
                property tree, which is a reading rather than a trial write — but one of the
                two statements about the vendor is wrong and the code carries both.
        */
        const auto& truth = benchTruth();
        bool        threw = false, opened = false, treeHasAgc = false;
        std::string what;
        std::string agcPath;

        try {
            auto usrp  = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            opened     = true;
            agcPath    = capture::uhd::Source::rxFacilityPath(*usrp, "gain/agc/enable");
            treeHasAgc = !agcPath.empty();
            try {
                usrp->set_rx_agc(true, 0);
            } catch (const std::exception& e) {
                threw = true;
                what  = e.what();
            }
            try {
                usrp->set_rx_agc(false, 0);
            } catch (const std::exception&) { // the same answer on the way back
            }
        } catch (const std::exception& e) {
            std::printf("agc claim: the direct open failed: %s\n", e.what());
        }

        std::printf("agc claim: the frontend's property tree %s an automatic gain control (\"%s\")\n", treeHasAgc ? "carries" : "does not carry", agcPath.c_str());
        std::printf("agc claim: set_rx_agc(true) %s on this model%s%s\n", threw ? "threw" : "returned without throwing", threw ? ": " : "", what.c_str());
        std::printf("agc claim: so on UHD 4.9.0.1 the vendor header's \"an exception will be thrown\" %s what this radio does\n", threw ? "is" : "is not");

        expect(opened) << "the device opens directly";
        expect(truth.hasAgc == treeHasAgc) << "the block's own detection is the property tree's answer, which is a reading and not a trial write";
    });

    cases("uhd.property-tree-link-rate", [] {
        /*| frame: the block cuts a fixed-clock radio's ladder at the link rate UHD states in
                the motherboard's property tree, read as a double as UHD's own check reads it,
                and cuts nothing where UHD states none. This walks the tree and says what the
                running device states.
        */
        std::vector<std::string> nodes;
        std::vector<std::string> ratish;
        double                   stated = 0.0;
        bool                     opened = false;
        try {
            auto usrp = uhd::usrp::multi_usrp::make(uhd::device_addr_t(deviceArgs()));
            opened    = true;
            auto tree = usrp->get_device()->get_tree();
            std::vector<std::pair<std::string, int>> pending{{"/mboards/0", 0}};
            while (!pending.empty() && nodes.size() < 2000UZ) {
                const std::string path  = pending.back().first;
                const int         depth = pending.back().second;
                pending.pop_back();
                nodes.push_back(path);
                if (path.find("rate") != std::string::npos || path.find("link") != std::string::npos) {
                    ratish.push_back(path);
                }
                if (depth >= 4) {
                    continue;
                }
                try {
                    for (const std::string& child : tree->list(path)) {
                        pending.emplace_back(path + "/" + child, depth + 1);
                    }
                } catch (const std::exception&) { // a leaf, or a node that will not list
                }
            }
            stated = capture::uhd::statedLinkBytesPerS(*usrp);
        } catch (const std::exception& e) {
            std::printf("link rate: the direct open failed: %s\n", e.what());
        }

        std::printf("link rate: the tree under /mboards/0 carries %zu nodes; %zu of them name a rate or a link\n", nodes.size(), ratish.size());
        for (const std::string& path : ratish) {
            std::printf("link rate: node \"%s\"\n", path.c_str());
        }
        std::printf("link rate: UHD states %.0f bytes per second for the link of \"%s\"; the truth carries %.0f\n", stated, deviceArgs().c_str(), benchTruth().linkBytesPerS);
        std::printf("link rate: at that rate the sc16 ceiling is %.0f S/s and the sc8 one %.0f S/s\n",
                    benchTruth().linkBytesPerS > 0.0 ? capture::uhd::linkCeilingHz("sc16", benchTruth().linkBytesPerS) : 0.0,
                    benchTruth().linkBytesPerS > 0.0 ? capture::uhd::linkCeilingHz("sc8", benchTruth().linkBytesPerS) : 0.0);

        expect(opened) << "the device opens";
        expect(!nodes.empty()) << "and states a property tree";
        expect(benchTruth().linkBytesPerS == stated) << "the truth carries the figure UHD states, with nothing taken off";
        if (stated <= 0.0) {
            std::printf("link rate: UHD states no link rate for this radio, so nothing cuts the ladder and the delivered count measures the link\n");
        }
    });

    cases("uhd.transport-sizing", [] {
        /*| frame: the three transport sizes UHD reads when it opens the device. A socket
                buffer cut to two megabytes holds a quarter of a second at 8 MS/s and four
                bytes a sample, so a stall that the default absorbs loses samples with it.
            trap: the sizes reach UHD only through the device address, so they are settings
                and not controls; a write while the device is open reaches nothing.
        */
        const auto&      truth  = benchTruth();
        const auto       ladder = capture::uhd::Source::sampleRatesFor(truth);
        const double     kRate  = ladder.empty() ? 8.0e6 : ladder.back();
        constexpr int    kStallMs = 300;
        constexpr double kSmallBuffer = 2.0 * 1024.0 * 1024.0;

        const auto stallOnce = [&](double buffSize, std::size_t& frameSamples, long long& dropped, std::uint64_t& gaps) {
            gr::property_map settings = sourceSettings(kRate, defaultFreq(), truth.gainMinDb, sourcePort());
            if (buffSize > 0.0) {
                settings["recv_buff_size"] = buffSize;
            }
            return withGraph(std::move(settings), [&](capture::uhd::Source& src, auto&) {
                if (!waitForSamples(std::chrono::milliseconds(8000))) {
                    return;
                }
                settle(500);
                frameSamples              = src.recvFrameSamples();
                const long long     drop0 = droppedOf(src);
                const std::uint64_t gap0  = g_overflowTags.load();
                g_stallMs.store(kStallMs);
                settle(2000);
                dropped = droppedOf(src) < 0 ? -1 : droppedOf(src) - drop0;
                gaps    = g_overflowTags.load() - gap0;
            });
        };

        std::size_t   defaultFrame = 0UZ, smallFrame = 0UZ;
        long long     defaultDropped = -1, smallDropped = -1;
        std::uint64_t defaultGaps = 0, smallGaps = 0;
        expect(stallOnce(0.0, defaultFrame, defaultDropped, defaultGaps)) << "the graph ran with UHD's own transport sizing";
        expect(stallOnce(kSmallBuffer, smallFrame, smallDropped, smallGaps)) << "and with the socket buffer cut to two megabytes";

        std::printf("transport sizing: at %.0f S/s across a %d ms stall, UHD's own sizing gives a %zu-sample frame and sheds %lld samples over %llu gaps\n", kRate, kStallMs, defaultFrame,
                    defaultDropped, static_cast<unsigned long long>(defaultGaps));
        std::printf("transport sizing: a %.0f byte receive buffer gives a %zu-sample frame and sheds %lld samples over %llu gaps\n", kSmallBuffer, smallFrame, smallDropped,
                    static_cast<unsigned long long>(smallGaps));

        expect(defaultFrame > 0UZ) << "the streamer states the frame UHD chose";
        expect(smallFrame > 0UZ);
        /*| frame: every receive asks for whole frames. With a timing tag on every block, each
                receive that delivered samples carries one tag at its first sample, so the
                distance from one tag to the next is one receive's count, and the two tags'
                times are the device's stamps of the two receives' first samples. A pair with
                a gap marker between its tags spans a gap and is left out.
        */
        double frameRate = 8.0e6;
        if (!ladder.empty()) {
            frameRate = *std::ranges::min_element(ladder, {}, [](double r) { return std::abs(r - 8.0e6); });
        }
        std::size_t            frame = 0UZ;
        double                 rateInForce = 0.0;
        std::vector<TagRecord> stamps;
        std::vector<std::size_t> markers;
        gr::property_map       everyBlock = sourceSettings(frameRate, defaultFreq(), truth.gainMinDb, sourcePort());
        everyBlock["tag_interval"]        = 0.0f;
        const bool framesRan              = withGraph(std::move(everyBlock), [&](capture::uhd::Source& src, auto&) {
            if (!waitForSamples(std::chrono::milliseconds(8000))) {
                return;
            }
            settle(300);
            {
                std::lock_guard g(g_tagLock);
                g_tagTimes.clear();
                g_markerAt.clear();
            }
            settle(1000);
            frame       = src.recvFrameSamples();
            rateInForce = static_cast<double>(src.sample_rate.value);
            std::lock_guard g(g_tagLock);
            stamps  = g_tagTimes;
            markers = g_markerAt;
        });
        expect(framesRan) << "the graph ran with a timing tag on every block";

        std::size_t pairs = 0UZ, whole = 0UZ, apart = 0UZ, fewest = std::numeric_limits<std::size_t>::max(), most = 0UZ;
        for (std::size_t i = 1UZ; i < stamps.size(); ++i) {
            const std::size_t from = stamps[i - 1UZ].index;
            const std::size_t to   = stamps[i].index;
            if (std::ranges::any_of(markers, [&](std::size_t m) { return m > from && m <= to; })) {
                continue;
            }
            const std::size_t n = to - from;
            ++pairs;
            fewest = std::min(fewest, n);
            most   = std::max(most, n);
            whole += frame > 0UZ && n % frame == 0UZ ? 1UZ : 0UZ;
            // Subtracted as integers: a double holds a stamp near 1.8e18 ns only to 256 ns.
            const auto dtNs = static_cast<double>(static_cast<std::int64_t>(stamps[i].timeNs - stamps[i - 1UZ].timeNs));
            apart += std::llround(dtNs * rateInForce / 1e9) == static_cast<long long>(n) ? 1UZ : 0UZ;
        }
        std::printf("transport sizing: %zu receives at %.0f S/s held %zu to %zu samples against a %zu-sample frame; %zu were whole frames and %zu were stamped their own counts apart\n", pairs,
                    rateInForce, pairs > 0UZ ? fewest : 0UZ, most, frame, whole, apart);
        expect(pairs >= 20UZ) << "a second of stream carries receives enough to judge";
        expect(whole == pairs) << "every receive returns a whole number of frames";
        expect(apart == pairs) << "and consecutive receives are stamped their own counts apart";
        if (defaultDropped == 0) {
            expect(smallDropped > 0) << "a receive buffer cut to two megabytes loses samples across a stall the default absorbs whole";
        } else {
            /*| frame: a radio whose link is already the limit at the top of its ladder sheds
                    with either sizing, and the two counts are then a measure of the link
                    rather than of the buffer. Printed, not compared.
            */
            std::printf("transport sizing: the default sizing already sheds %lld samples at this rate, so the two counts are printed and not compared\n", defaultDropped);
        }
        expect(capture::kwargsValue(capture::uhd::withDeviceArgs(deviceArgs(), 0.0, capture::uhd::Direction::Receive, 0.0, 0.0, kSmallBuffer), "recv_buff_size") == std::format("{:.0f}", kSmallBuffer))
            << "and the size reaches UHD as a device-address key";
    });

    cases("uhd.master-clock", [] {
        /*| frame: the converter clock on a radio that lets one be picked. Every reachable
                rate is that clock over a decimation, so a clock chosen by hand fixes the
                whole ladder. A radio stating one clock whose product the table names is
                offered the table's clocks, and a pick is pinned for the next open. Any other
                radio whose clock is fixed offers no such control, and the case says so and
                stops.
        */
        const auto& truth  = benchTruth();
        const auto  ladder = capture::uhd::masterClockLadderMHz(truth);
        std::printf("master clock: the device states %.0f to %.0f Hz and is running at %.0f Hz\n", truth.mclkMinHz, truth.mclkMaxHz, truth.mclkHz);
        if (capture::uhd::offersOneClockRate(truth) && !ladder.empty()) {
            /*| frame: the pick is the first table clock other than the one in force, or the
                    clock in force where the table offers it alone. The case reads the pin,
                    the open address and the clock the device still runs, and does not open
                    the radio at the picked clock.
            */
            const auto   other      = std::ranges::find_if(ladder, [&truth](double mhz) { return mhz > 0.0 && std::abs(mhz * 1e6 - truth.mclkMinHz) >= 1.0; });
            const double pickMHz    = other != ladder.end() ? *other : ladder.back();
            double       pinnedHz   = 0.0;
            double       clockAfter = 0.0;
            std::string  addressKey;
            const bool   ran = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
                expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
                expect(src.setControl("MASTER_CLOCK", pickMHz)) << "a table clock is accepted";
                {
                    std::lock_guard lock(src._ctrlMutex);
                    pinnedHz = src._masterClockPinnedHz.value_or(0.0);
                }
                addressKey = src.openAddress().get("master_clock_rate", "");
                clockAfter = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_master_clock_rate(0); });
            });
            expect(ran) << "the graph ran";
            std::printf("master clock: picked %.2f MHz from the table; pinned %.0f Hz, the open address carries %s, the device still runs %.0f Hz\n", pickMHz, pinnedHz, addressKey.c_str(), clockAfter);
            expect(std::abs(pinnedHz - pickMHz * 1e6) < 1.0) << "the pick is pinned";
            expect(addressKey == std::format("{:.0f}", pickMHz * 1e6)) << "the next open carries it";
            expect(std::abs(clockAfter - truth.mclkMinHz) < 1.0) << "and the running device keeps its clock";
            return;
        }
        if (!clockIsProgrammable(truth) || ladder.empty()) {
            std::printf("master clock: this radio's clock is fixed, so there is no control to offer\n");
            const capture::TruthContext ctx{.deviceParams = deviceArgs(), .sampleRate = 2.0e6, .centerFreq = defaultFreq()};
            const auto                  controls = capture::uhd::Source::describeControls(truth, ctx);
            expect(std::ranges::find_if(controls, [](const auto& c) { return c.id == "MASTER_CLOCK"; }) == controls.end()) << "and none is offered";
            return;
        }

        // 30.72 MHz, which 7.68 MS/s divides exactly, where the device reaches it.
        const double pickMHz  = std::ranges::find(ladder, 30.72) != ladder.end() ? 30.72 : ladder.back();
        const double wantRate = pickMHz * 1e6 / 4.0;
        double       clockAfterWrite = 0.0, rateAfterWrite = 0.0;
        double       clockAutomatic  = 0.0, rateAutomatic  = 0.0;
        double       clockNextStart  = 0.0;
        // The pin alone, before any rate was asked for: the rate the block moved the radio
        // to, the setting it states, and the tag it forwarded for it.
        double        rateAfterPin = 0.0, settingAfterPin = 0.0, tagAfterPin = 0.0;
        std::uint64_t rateTagsForPin = 0;

        const bool first = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            const std::uint64_t tagsBefore = g_rateTags.load();
            expect(src.setControl("MASTER_CLOCK", pickMHz)) << "a clock the device states is accepted";
            settle(1200);
            rateAfterPin    = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
            settingAfterPin = src.sample_rate.value;
            rateTagsForPin  = g_rateTags.load() - tagsBefore;
            tagAfterPin     = g_lastRateTagHz.load();

            std::ignore = src.settings().setStaged({{"sample_rate", wantRate}});
            settle(600);
            clockAfterWrite = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_master_clock_rate(0); });
            rateAfterWrite  = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });

            /*| frame: whether UHD's own choice comes back while the device stays open. An
                    explicit write clears the automatic flag inside the driver, so a rate
                    that needs another clock is reached by decimation instead. */
            expect(src.setControl("MASTER_CLOCK", 0.0)) << "the automatic entry is accepted";
            std::ignore = src.settings().setStaged({{"sample_rate", 2.0e6}});
            settle(600);
            clockAutomatic = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_master_clock_rate(0); });
            rateAutomatic  = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_rx_rate(0); });
        });
        expect(first) << "the first graph ran";

        const bool second = withGraph(sourceSettings(2.0e6, defaultFreq(), truth.gainMinDb, sourcePort()), [&](capture::uhd::Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts again";
            clockNextStart = deviceRead(src, 0.0, [](uhd::usrp::multi_usrp& u) { return u.get_master_clock_rate(0); });
        });
        expect(second) << "the second graph ran";

        const double pinDecimation = rateAfterPin > 0.0 ? std::round(pickMHz * 1e6 / rateAfterPin) : 0.0;
        std::printf("master clock: the pin alone moved the radio to %.0f S/s, decimation %.0f of the pinned clock; the block states %.0f S/s and forwarded %llu rate tag(s), the last carrying %.0f\n",
                    rateAfterPin, pinDecimation, settingAfterPin, static_cast<unsigned long long>(rateTagsForPin), tagAfterPin);
        std::printf("master clock: written %.2f MHz, the device reads %.0f Hz and the rate %.0f S/s for a request of %.0f\n", pickMHz, clockAfterWrite, rateAfterWrite, wantRate);
        std::printf("master clock: with the automatic entry and a 2 MS/s request the device reads %.0f Hz and %.0f S/s, still open\n", clockAutomatic, rateAutomatic);
        std::printf("master clock: the next start reads %.0f Hz, which is what restores UHD's own choice\n", clockNextStart);

        /*| frame: a clock written by hand does not move again while the device stays open,
                so the radio is a fixed-clock radio from that write on: the rate it is put on
                is one of that clock's even decimations, and the block states and forwards
                it, because a caller's span, decimator and recorder hear a rate change only
                as a forwarded setting.
        */
        expect(pinDecimation > 0.0 && std::fmod(pinDecimation, 2.0) == 0.0) << "the pin lands the radio on an even decimation of the pinned clock, not on the odd one UHD would pick";
        expect(std::abs(settingAfterPin - rateAfterPin) < 1.0) << "the block states the rate the radio is running";
        expect(rateTagsForPin >= 1UL) << "and forwards it downstream, which a write from a caller's own thread cannot do for itself";
        expect(std::abs(tagAfterPin - rateAfterPin) < 1.0) << "the tag carries that same rate";
        expect(std::abs(clockAfterWrite - pickMHz * 1e6) < 1.0) << "the device runs on the clock that was written";
        expect(std::abs(rateAfterWrite - wantRate) < 1.0) << "and reaches a rate that clock divides exactly";
        /*| measured: the automatic entry does not move the clock while the device stays
                open. An explicit write clears UHD's own choice inside the driver, and only a
                fresh open sets it again, so a later rate is reached by decimating the clock
                that was written. The block knows that, and snaps such a request to an even
                decimation of the clock in force rather than letting UHD land it on an odd
                one: 2 MS/s on a pinned 30.72 MHz clock reaches 1.92 MS/s, decimation 16,
                where UHD alone would give 2.048 MS/s, decimation 15.
        */
        const double automaticDecimation = rateAutomatic > 0.0 ? std::round(clockAutomatic / rateAutomatic) : 0.0;
        expect(std::abs(clockAutomatic - pickMHz * 1e6) < 1.0) << "the automatic entry leaves the clock where the write put it, for as long as the device stays open";
        expect(std::abs(rateAutomatic - 2.0e6) / 2.0e6 < 0.05) << "and a later rate is reached by decimating that clock, near the request rather than on it";
        expect(automaticDecimation > 0.0 && std::fmod(automaticDecimation, 2.0) == 0.0) << "by an even decimation, so the halfband filter is in the chain";
        expect(std::abs(clockNextStart - truth.mclkHz) < 1.0) << "the next start comes up on the clock UHD chooses for itself, which is what restores the automatic choice";
    });

    const int rc     = cases.report();
    const bool failed = boost::ut::cfg<boost::ut::override>.run({.report_errors = true});
    std::fflush(stdout);
    std::fflush(stderr);
    /*| trap: a binary that links GNU Radio 4 leaves its static teardown to the operating
            system, because the framework's own global destructors run after the scheduler's
            threads are gone. The verdict is taken first so a failure still reaches ctest.
    */
    std::_Exit(failed ? 1 : rc);
}
