/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the USRP driver's decisions, where a stored value meets a device.
    why: a USRP states its control surface rather than carrying a table, so what can be
        settled without hardware is the arithmetic the driver applies to what the device
        said. None of this opens anything.
    frame: the ranges and gain figures here are a B205mini's, which keeps the numbers
        readable; the decisions hold for any USRP that states its own.
*/
#include <capture/uhd/Sink.hpp>
#include <capture/uhd/Source.hpp>

#include "support/Cases.hpp"
#include "support/Surface.hpp"

#include "UnparsedAddress.hpp"

#include <uhd/stream.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/time_spec.hpp>

#include <dlfcn.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <format>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace {

/*| role: a stand-in for the reads the shared truth reader makes, with any of them refused on
        request, the way a model that lacks a facility refuses it. It answers as an N210 with a
        WBX would in both directions, three gain elements named. No device is opened.
*/
struct TruthStandIn {
    std::set<std::string> refuse; // the reads that throw, by the names below

    void check(const std::string& what) const {
        if (refuse.contains(what)) {
            throw std::runtime_error(what + " refused");
        }
    }
    static ::uhd::dict<std::string, std::string> info() {
        ::uhd::dict<std::string, std::string> d;
        d["mboard_id"] = "N210r4";
        return d;
    }
    std::vector<std::string> gainNames() const {
        check("gain_names");
        return {"PGA0", "BAD", "ADC-pga"};
    }
    ::uhd::gain_range_t gainRange(const std::string& g) const {
        check("gain_range:" + g);
        return ::uhd::gain_range_t(0.0, g == "ADC-pga" ? 6.0 : 31.5, 0.5);
    }
    std::vector<std::string> antennas() const {
        check("antennas");
        return {"TX/RX", "RX2"};
    }
    ::uhd::meta_range_t rates() const {
        ::uhd::meta_range_t out;
        for (int n = 1; n <= 512; ++n) {
            out.push_back(::uhd::range_t(100.0e6 / static_cast<double>(n)));
        }
        return out;
    }
    double bandwidth() const {
        check("bandwidth");
        return 2.0e6;
    }

    ::uhd::dict<std::string, std::string> get_usrp_rx_info(std::size_t) const { return info(); }
    ::uhd::dict<std::string, std::string> get_usrp_tx_info(std::size_t) const { return info(); }
    std::string                           get_mboard_name(std::size_t) const { return "N210r4"; }
    ::uhd::device::sptr                   get_device() const { throw std::runtime_error("no property tree"); }
    std::vector<std::string>              get_rx_gain_names(std::size_t) const { return gainNames(); }
    std::vector<std::string>              get_tx_gain_names(std::size_t) const { return gainNames(); }
    ::uhd::gain_range_t                   get_rx_gain_range(const std::string& g, std::size_t) const { return gainRange(g); }
    ::uhd::gain_range_t                   get_tx_gain_range(const std::string& g, std::size_t) const { return gainRange(g); }
    double                                get_rx_gain(const std::string&, std::size_t) const { return 3.0; }
    double                                get_tx_gain(const std::string&, std::size_t) const { return 3.0; }
    ::uhd::gain_range_t                   get_rx_gain_range(std::size_t) const { return ::uhd::gain_range_t(0.0, 38.0, 0.5); }
    ::uhd::gain_range_t                   get_tx_gain_range(std::size_t) const { return ::uhd::gain_range_t(0.0, 38.0, 0.5); }
    std::vector<std::string>              get_rx_antennas(std::size_t) const { return antennas(); }
    std::vector<std::string>              get_tx_antennas(std::size_t) const { return antennas(); }
    std::string                           get_rx_antenna(std::size_t) const { return "TX/RX"; }
    std::string                           get_tx_antenna(std::size_t) const { return "TX/RX"; }
    ::uhd::freq_range_t                   get_rx_freq_range(std::size_t) const { return ::uhd::freq_range_t(48.75e6, 2201.25e6, 0.0); }
    ::uhd::freq_range_t                   get_tx_freq_range(std::size_t) const { return ::uhd::freq_range_t(48.75e6, 2201.25e6, 0.0); }
    ::uhd::meta_range_t                   get_rx_bandwidth_range(std::size_t) const { return ::uhd::meta_range_t(0.2e6, 56.0e6, 0.0); }
    ::uhd::meta_range_t                   get_tx_bandwidth_range(std::size_t) const { return ::uhd::meta_range_t(0.2e6, 56.0e6, 0.0); }
    double                                get_rx_bandwidth(std::size_t) const { return bandwidth(); }
    double                                get_tx_bandwidth(std::size_t) const { return bandwidth(); }
    ::uhd::meta_range_t                   get_master_clock_rate_range(std::size_t) const { return ::uhd::meta_range_t(100.0e6, 100.0e6, 0.0); }
    double                                get_master_clock_rate(std::size_t) const { return 100.0e6; }
    ::uhd::meta_range_t                   get_rx_rates(std::size_t) const { return rates(); }
    ::uhd::meta_range_t                   get_tx_rates(std::size_t) const { return rates(); }
    std::vector<std::string>              get_clock_sources(std::size_t) const {
        check("clock_sources");
        return {"internal", "external", "gpsdo"};
    }
    std::string get_clock_source(std::size_t) const {
        check("clock_source");
        return "internal";
    }
    std::vector<std::string> get_time_sources(std::size_t) const {
        check("time_sources");
        return {"none", "external"};
    }
    std::string get_time_source(std::size_t) const {
        check("time_source");
        return "none";
    }
};

/*| role: the truth stand-in with a property tree the link reads reach, empty until a case
        creates a node, and a master clock the case sets: the range the device states and the
        clock in force, whose divisors are the rates it states. No device is opened.
*/
struct TreeStandIn : TruthStandIn {
    struct Device {
        ::uhd::property_tree::sptr tree = ::uhd::property_tree::make();
        ::uhd::property_tree::sptr get_tree() const { return tree; }
    };
    std::shared_ptr<Device> device     = std::make_shared<Device>();
    ::uhd::meta_range_t     clockRange = ::uhd::meta_range_t(100.0e6, 100.0e6, 0.0);
    double                  clockHz    = 100.0e6;
    std::string             mboardName = "N210r4"; // what get_mboard_name answers
    std::string             mboardId   = "N210r4"; // the information map's mboard_id
    std::string             frontend;              // its rx_subdev_name and tx_subdev_name, none where empty

    ::uhd::dict<std::string, std::string> infoFor(const char* frontendKey) const {
        ::uhd::dict<std::string, std::string> d;
        d["mboard_id"] = mboardId;
        if (!frontend.empty()) {
            d[frontendKey] = frontend;
        }
        return d;
    }
    ::uhd::dict<std::string, std::string> get_usrp_rx_info(std::size_t) const { return infoFor("rx_subdev_name"); }
    ::uhd::dict<std::string, std::string> get_usrp_tx_info(std::size_t) const { return infoFor("tx_subdev_name"); }
    std::string                           get_mboard_name(std::size_t) const { return mboardName; }
    std::shared_ptr<Device>               get_device() const { return device; }
    ::uhd::meta_range_t     get_master_clock_rate_range(std::size_t) const { return clockRange; }
    double                  get_master_clock_rate(std::size_t) const { return clockHz; }
    ::uhd::meta_range_t     clockRates() const {
        ::uhd::meta_range_t out;
        for (int n = 512; n >= 1; --n) {
            out.push_back(::uhd::range_t(clockHz / static_cast<double>(n)));
        }
        return out;
    }
    ::uhd::meta_range_t get_rx_rates(std::size_t) const { return clockRates(); }
    ::uhd::meta_range_t get_tx_rates(std::size_t) const { return clockRates(); }
};

/*| frame: the thread starts this binary lets through before it refuses one, as the system
        refuses a start when it has no thread to give; -1 lets every start through. The refusal
        runs onRefusal first and sets the count back to -1.
*/
std::atomic<int>      threadStartsBeforeRefusal{-1};
std::function<void()> onRefusal;

/*| role: a stand-in for a receive streamer that hands the source's consumer loop the blocks it
        was given, each stamped from whole master-clock ticks as UHD stamps a packet. A receive
        that asks for less than the rest of a block takes what it asked for, and the next one
        takes on from there, stamped at its own first sample as UHD stamps the rest of a frame.
        A block at a rate other than the one before it moves the source's rate first and counts
        a rate break, as the source's own rate change does. After the last block it asks the
        loop to stop. No device is opened.
*/
struct StampedStream : ::uhd::rx_streamer {
    struct Block {
        double      rateHz  = 0.0;
        long long   ticks   = 0; // the master-clock tick of the block's first sample
        std::size_t samples = 0UZ;
    };
    capture::uhd::Source& src;
    double                clockHz;
    std::vector<Block>    blocks;
    std::size_t           next  = 0UZ;
    std::size_t           taken = 0UZ; // the samples of blocks[next] already handed over

    StampedStream(capture::uhd::Source& source, double clock, std::vector<Block> given) : src(source), clockHz(clock), blocks(std::move(given)) {}

    std::size_t get_num_channels() const override { return 1UZ; }
    std::size_t get_max_num_samps() const override { return 2040UZ; }
    std::size_t recv(const buffs_type& buffs, const std::size_t nsamps, ::uhd::rx_metadata_t& md, const double, const bool) override {
        md.reset();
        if (next == blocks.size()) {
            src._consumer.requestStop();
            md.error_code = ::uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
            return 0UZ;
        }
        const Block& block = blocks[next];
        if (taken == 0UZ && block.rateHz != static_cast<double>(src.sample_rate.value)) {
            src.sample_rate = block.rateHz;
            src._rateBreaks.fetch_add(1, std::memory_order_relaxed);
            src._lastGapCause.store(capture::uhd::Source::kGapRateChange, std::memory_order_relaxed);
        }
        const std::size_t n = std::min(block.samples - taken, nsamps);
        std::fill_n(static_cast<std::complex<float>*>(buffs[0]), n, std::complex<float>{});
        md.has_time_spec = true;
        md.time_spec     = ::uhd::time_spec_t::from_ticks(block.ticks + std::llround(static_cast<double>(taken) * clockHz / block.rateHz), clockHz);
        taken += n;
        if (taken == block.samples) {
            ++next;
            taken = 0UZ;
        }
        return n;
    }
    void issue_stream_cmd(const ::uhd::stream_cmd_t&) override {}
    void post_input_action(const std::shared_ptr<::uhd::rfnoc::action_info>&, const std::size_t) override {}
};

/*| role: what one run of the source's consumer loop over a stamped stand-in counted: the
        samples it read as shed and the gap markers it placed, by cause.
*/
struct StampedRun {
    std::uint64_t shed        = 0UL;
    std::size_t   missing     = 0UZ;
    std::size_t   rateChanges = 0UZ;
};

StampedRun runStamped(double clockHz, double firstRateHz, std::vector<StampedStream::Block> blocks) {
    using namespace boost::ut;
    capture::uhd::Source            src{};
    gr::PortIn<std::complex<float>> reader;
    expect(src.out.connect(reader).has_value()) << "the markers reach a reader";
    src.sample_rate = firstRateHz;
    src._stream     = std::make_shared<StampedStream>(src, clockHz, std::move(blocks));
    src._consumer.clearStop();
    src.consumerLoop();
    src._stream.reset();

    StampedRun run{.shed = src.droppedSamples()};
    auto       tags = reader.tagReader().get();
    for (const gr::Tag& tag : tags) {
        for (const auto& [key, value] : tag.map) {
            if (std::string_view(key.data(), key.size()) == "rx_overflow_cause") {
                const std::string_view cause = value.value_or(std::string_view{});
                run.missing += cause == std::string_view("missing samples") ? 1UZ : 0UZ;
                run.rateChanges += cause == std::string_view("rate change") ? 1UZ : 0UZ;
            }
        }
    }
    std::ignore = tags.consume(0UZ);
    return run;
}

// What the calls print on standard error, the descriptor taken aside for the duration.
template <typename Calls>
std::string stderrOf(Calls&& calls) {
    std::fflush(stderr);
    std::FILE* capture = std::tmpfile();
    const int  saved   = ::dup(2);
    ::dup2(::fileno(capture), 2);
    calls();
    std::fflush(stderr);
    ::dup2(saved, 2);
    ::close(saved);
    std::rewind(capture);
    std::string text;
    for (int c = std::fgetc(capture); c != EOF; c = std::fgetc(capture)) {
        text.push_back(static_cast<char>(c));
    }
    std::fclose(capture);
    return text;
}

} // namespace

/*| frame: this binary's own pthread_create, which the dynamic linker binds ahead of the C
        library's for every thread start in the process. It hands each start to the library's
        own, apart from the one refusal a case asks for.
*/
extern "C" int pthread_create(pthread_t* __restrict thread, const pthread_attr_t* __restrict attr, void* (*start)(void*), void* __restrict arg) noexcept {
    using Create             = int (*)(pthread_t*, const pthread_attr_t*, void* (*)(void*), void*);
    static const auto create = reinterpret_cast<Create>(dlsym(RTLD_NEXT, "pthread_create"));
    for (int left = threadStartsBeforeRefusal.load(); left >= 0;) {
        if (threadStartsBeforeRefusal.compare_exchange_weak(left, left - 1)) {
            if (left == 0) {
                if (onRefusal) {
                    onRefusal();
                }
                return EAGAIN;
            }
            break;
        }
    }
    return create(thread, attr, start, arg);
}

int main(int argc, char** argv) {
    using namespace boost::ut;
    capture::test::Cases cases(argc, argv);

    cases("controls.uhd-frequency-out-of-range-refused", [] {
        // What a B205mini states at 2 MS/s: the 50 MHz to 6 GHz front end, widened by half
        // the rate at each end by the digital down-converter behind it.
        const std::vector<std::pair<double, double>> coverage{{49.0e6, 6001.0e6}};

        expect(capture::uhd::tuneFrequencyFor(100.0e6, 0.0, coverage).value() == 100.0e6) << "inside the stated coverage";
        expect(capture::uhd::tuneFrequencyFor(49.0e6, 0.0, coverage).has_value()) << "the lower edge is inside";
        expect(capture::uhd::tuneFrequencyFor(6001.0e6, 0.0, coverage).has_value()) << "and so is the upper one";

        expect(!capture::uhd::tuneFrequencyFor(40.0e6, 0.0, coverage).has_value()) << "40 MHz is refused rather than tuned to 49 MHz";
        expect(!capture::uhd::tuneFrequencyFor(6.1e9, 0.0, coverage).has_value()) << "and 6.1 GHz rather than tuned to 6001 MHz";
        expect(!capture::uhd::tuneFrequencyFor(-1.0, 0.0, coverage).has_value()) << "a negative request is refused";

        // The correction is folded in before the test, because the scaled frequency is the
        // one the tuner has to reach. It is the exact scale, the request divided by one plus
        // the fraction.
        expect(std::abs(capture::uhd::tuneFrequencyFor(1.0e9, 100.0, coverage).value() - 1.0e9 / 1.0001) < 1e-3) << "a positive ppm tunes low, the crystal-error convention";
        expect(std::abs(capture::uhd::tuneFrequencyFor(1.0e9, -100.0, coverage).value() - 1.0e9 / 0.9999) < 1e-3) << "and a negative one tunes high";
        expect(!capture::uhd::tuneFrequencyFor(49.0e6, 100.0, coverage).has_value()) << "a request at the edge that the correction carries outside is refused";

        // A radio that stated no coverage reaches nothing, rather than reaching whatever the
        // vendor library coerces a request to.
        expect(!capture::uhd::tuneFrequencyFor(100.0e6, 0.0, {}).has_value()) << "no stated coverage is no coverage";
    });

    cases("controls.uhd-bounds-the-lo-offset", [] {
        // What a B205mini states at 2 MS/s, and the offsets a caller can ask for at 61.44.
        const std::vector<std::pair<double, double>> coverage{{49.0e6, 6001.0e6}};

        expect(capture::uhd::Source::boundedLoOffsetHz(30.0e6, 61.44e6) == 30.0e6) << "an offset inside half the rate is applied as it stands";
        expect(capture::uhd::Source::boundedLoOffsetHz(40.0e6, 61.44e6) == 30.72e6) << "and one beyond it is held at half the rate";
        expect(capture::uhd::Source::boundedLoOffsetHz(-40.0e6, 61.44e6) == -30.72e6) << "at each end";
        expect(capture::uhd::Source::boundedLoOffsetHz(1.0e6, 0.0) == 1.0e6) << "a rate of zero states no bound to apply";
        expect(capture::uhd::Source::boundedLoOffsetHz(std::numeric_limits<double>::quiet_NaN(), 2.0e6) == 0.0) << "and a value that is not a number is no offset at all";

        /*| frame: the request handed to the tuner puts the RF frontend at the center plus
                the offset, so the coverage has to hold that frontend frequency. UHD coerces
                one past the end of that coverage without refusing it. */
        expect(capture::uhd::tuneFrequencyFor(6000.0e6, 0.0, coverage, 0.0).value() == 6000.0e6) << "a center inside the coverage, with no offset";
        expect(!capture::uhd::tuneFrequencyFor(6000.0e6, 0.0, coverage, 30.0e6).has_value()) << "an offset that carries the frontend past the top is refused";
        expect(!capture::uhd::tuneFrequencyFor(60.0e6, 0.0, coverage, -30.0e6).has_value()) << "and so is one that carries it below the bottom";
        expect(capture::uhd::tuneFrequencyFor(100.0e6, 0.0, coverage, 30.0e6).value() == 100.0e6) << "an offset the coverage holds tunes the center that was asked for";
        expect(std::abs(capture::uhd::tuneFrequencyFor(100.0e6, 100.0, coverage, 1.0e6).value() - 100.0e6 / 1.0001) < 1e-6) << "and the correction is still folded into the center first";
    });

    cases("controls.uhd-agc-hands-the-gain-back", [] {
        // The elements a B2xx names, and the ones an N-series with a WBX names beside them.
        const std::vector<capture::uhd::DeviceTruth::GainElement> b2xx{{"PGA", 0.0, 76.0, 1.0, 0.0}, {"ATT", 0.0, 31.5, 0.5, 0.0}};

        // Nothing was asked for while the loop had the gain path, so leaving it writes the
        // overall gain and nothing else.
        const auto plain = capture::uhd::Source::agcReleaseWrites(35.0, {}, b2xx);
        expect(plain.size() == 1UZ) << "one write";
        expect(plain.front().first.empty()) << "an empty element name is the overall gain";
        expect(plain.front().second == 35.0);

        // A caller moved the PGA while the loop held it. The overall gain goes first and
        // the element after it, so the device ends on the value the caller moved it to and
        // not on whatever rx_gains last carried.
        const std::vector<std::pair<std::string, double>> held{{"PGA", 12.0}};
        const auto                                        after = capture::uhd::Source::agcReleaseWrites(35.0, held, b2xx);
        expect(after.size() == 2UZ);
        expect(after.front().second == 35.0) << "the overall gain first";
        expect(after.back().first == std::string("PGA"));
        expect(after.back().second == 12.0) << "and the element the caller moved last, so it wins";

        // Several elements keep the order they were asked in.
        const std::vector<std::pair<std::string, double>> two{{"PGA", 12.0}, {"ATT", 6.0}};
        const auto                                        both = capture::uhd::Source::agcReleaseWrites(0.0, two, b2xx);
        expect(both.size() == 3UZ);
        expect(both[1].first == std::string("PGA") && both[1].second == 12.0);
        expect(both[2].first == std::string("ATT") && both[2].second == 6.0);

        /*| frame: a block restarted against another model. The element names are the
                model's, so a name held from the radio before it is not written to this one:
                UHD refuses it once per element, which is the check a direct write already
                makes. */
        const std::vector<capture::uhd::DeviceTruth::GainElement> wbx{{"PGA0", 0.0, 31.5, 0.5, 0.0}, {"ADC-digital", 0.0, 6.0, 0.5, 0.0}};
        const auto                                             other = capture::uhd::Source::agcReleaseWrites(20.0, two, wbx);
        expect(other.size() == 1UZ) << "the overall gain alone, the held names being another radio's";
        expect(other.front().first.empty() && other.front().second == 20.0);
        expect(capture::uhd::Source::agcReleaseWrites(20.0, held, {}).size() == 1UZ) << "and a radio that names no element takes only the overall gain";
    });

    cases("controls.uhd-counts-what-the-device-shed", [] {
        using capture::uhd::Source;
        using Spec          = ::uhd::time_spec_t;
        constexpr double hz = 8.0e6;
        const Spec       t0 = Spec(7, 0.25);

        // A block that arrived where it was expected cost nothing.
        expect(Source::shedBetween(t0, t0, hz) == 0UL) << "no gap is no loss";

        // A 300 ms gap at 8 MS/s, the cost of a host that stopped draining.
        expect(Source::shedBetween(t0, t0 + Spec(0.3), hz) == 2'400'000UL) << "the gap is the count, in samples";

        // A block that arrived early is a reset and not a negative gap: the expectation was
        // formed at the rate before a rate change.
        expect(Source::shedBetween(t0 + Spec(0.3), t0, hz) == 0UL) << "an early arrival sheds nothing";
        expect(Source::shedBetween(Spec(0.0), Spec(-5.0 / hz), hz) == 0UL) << "and neither does a stamp before the stream";

        /*| frame: a device takes a sample on every decimation-th tick of its master clock and
                stamps a packet from whole ticks, as UHD's own from_ticks builds it. Each row
                is a clock and a decimation: the B205mini's 8 MS/s and 5 MS/s, its master
                clock at a decimation of one, and an N210's rate from 100 MHz by 12, which is
                no whole number of hertz. Every phase of the first sample against the rate's
                grid is taken, the half included. A thousand blocks of 16384 samples in a row
                shed nothing, and one or a thousand samples missing between two of them
                count one or a thousand.
        */
        struct Row {
            double    clockHz;
            long long decim;
        };
        constexpr Row             rows[] = {{32.0e6, 4}, {40.0e6, 8}, {61.44e6, 1}, {100.0e6, 12}};
        constexpr long long       kBlock = 16'384;
        constexpr long long       kFirst = 7LL * 32'000'000LL;
        for (const auto& [clockHz, decim] : rows) {
            const double rateHz = clockHz / static_cast<double>(decim);
            for (long long phase = 0; phase < decim; ++phase) {
                std::uint64_t shed = 0, lostOne = 0, lostMany = 0;
                for (long long k = 1; k < 1000; ++k) {
                    const long long before = kFirst + phase + (k - 1) * kBlock * decim;
                    const Spec      due    = Spec::from_ticks(before, clockHz) + Spec(static_cast<double>(kBlock) / rateHz);
                    shed += Source::shedBetween(due, Spec::from_ticks(before + kBlock * decim, clockHz), rateHz);
                    lostOne += Source::shedBetween(due, Spec::from_ticks(before + (kBlock + 1) * decim, clockHz), rateHz);
                    lostMany += Source::shedBetween(due, Spec::from_ticks(before + (kBlock + 1000) * decim, clockHz), rateHz);
                }
                const std::string where = std::format("{:.0f} S/s, the first sample {} of {} ticks off the grid", rateHz, phase, decim);
                expect(shed == 0UL) << "stamps off the grid shed nothing at" << where;
                expect(lostOne == 999UL) << "a sample lost after each block counts one each at" << where;
                expect(lostMany == 999'000UL) << "a thousand lost after each block count a thousand each at" << where;
            }
        }
    });

    cases("controls.uhd-holds-the-tick-grid-across-a-clock-set", [] {
        using capture::uhd::Source;
        using Spec = ::uhd::time_spec_t;

        // A block that arrived where the expectation put it cost nothing; one that arrived
        // late cost the gap.
        const Spec expected = Spec(0, 16'384.0 / 2.0e6);
        expect(Source::shedForBlock(true, false, 2.0e6, 2.0e6, expected, expected) == 0UL) << "no gap is no loss";
        expect(Source::shedForBlock(true, false, 2.0e6, 2.0e6, expected, expected + Spec(1.2)) == 2'400'000UL) << "the gap is the count, in samples";

        /*| frame: setting the device clock moves it from the seconds since the device came
                up to a wall-clock time. At 2 MS/s the stamp difference across that move is
                about 3.6e15 samples, and the stream lost none of them.
        */
        const Spec jumped = Spec(1'789'900'000, 0.0);
        expect(Source::shedForBlock(true, true, 2.0e6, 2.0e6, expected, jumped) == 0UL) << "a block that arrived while a clock set was in flight is not measured";
        expect(Source::shedForBlock(true, false, 2.0e6, 2.0e6, expected, jumped) > 1'000'000'000'000'000UL) << "and the same block measured against that clock would count the whole move";
        expect(Source::shedForBlock(false, false, 2.0e6, 2.0e6, expected, expected + Spec(1.2)) == 0UL) << "no expectation yet is no measurement";
        expect(Source::shedForBlock(true, false, 4.0e6, 2.0e6, expected, expected + Spec(1.2)) == 0UL) << "and neither is one formed at another rate";

        // The latch is read from the stream's own stamps.
        constexpr std::uint64_t kNow      = 1'789'936'968'000'000'000UL;
        constexpr std::uint64_t kDeadline = kNow + 1'000'000'000UL;
        expect(capture::uhd::Source::clockLatchState(1'789'936'969L, 1'789'936'969L, kNow, kDeadline) == capture::uhd::Source::ClockLatch::Latched) << "a block stamped at the second asked for";
        expect(capture::uhd::Source::clockLatchState(1'789'936'970L, 1'789'936'969L, kNow, kDeadline) == capture::uhd::Source::ClockLatch::Latched) << "or past it";
        expect(capture::uhd::Source::clockLatchState(12L, 1'789'936'969L, kNow, kDeadline) == capture::uhd::Source::ClockLatch::Pending) << "the seconds since the device came up are not that clock";
        expect(capture::uhd::Source::clockLatchState(12L, 1'789'936'969L, kNow + 2'000'000'000UL, kDeadline) == capture::uhd::Source::ClockLatch::Abandoned) << "and an edge that never arrived is given up at the deadline";

        // The tag source: the device's own stamp, carried into the host's epoch unless the
        // caller asked for the device's.
        constexpr std::int64_t kUptimeNs = 12'000'000'000LL;
        constexpr std::int64_t kOffset   = 1'789'936'956'000'000'000LL;
        expect(capture::uhd::Source::deviceStampNs(kUptimeNs, false, kOffset).value() == kUptimeNs + kOffset) << "a stamp taken before the clock was set still names a host time";
        expect(capture::uhd::Source::deviceStampNs(kUptimeNs, true, kOffset).value() == kUptimeNs) << "and the device's own epoch stands where a caller asked for it";
        expect(!capture::uhd::Source::deviceStampNs(std::nullopt, false, kOffset).has_value()) << "a block with no stamp carries no device time";
        expect(!capture::uhd::Source::deviceStampNs(std::optional<std::int64_t>{12LL}, false, -1'000'000'000'000LL).has_value()) << "and an offset that carries a stamp before the epoch is not a time";

        // The tag a block then carries, against the clock read at the drain.
        constexpr std::uint64_t kWallNs = 1'700'000'000'000'000'000UL;
        expect(capture::uhd::Source::blockTagNs(capture::uhd::Source::deviceStampNs(kUptimeNs, false, kOffset), kWallNs, 16384UZ, 2.0e6) == static_cast<std::uint64_t>(kUptimeNs + kOffset)) << "the device stamp names the first sample";
        expect(capture::uhd::Source::blockTagNs(capture::uhd::Source::deviceStampNs(std::nullopt, false, kOffset), kWallNs, 16384UZ, 2.0e6) == kWallNs - 8'192'000UL) << "and without one the drain reading less the block's own duration does";
    });

    cases("controls.uhd-forwards-the-frequency-the-tuner-holds", [] {
        // With no correction the read-back is the answer, whatever UHD coerced it to.
        expect(capture::uhd::tunedFrequencyFrom(914'999'999.404, 0.0) == 914'999'999.404) << "the tuner's own value, unchanged";
        expect(capture::uhd::tunedFrequencyFrom(100.0e6, 0.0) == 100.0e6);

        /*| frame: the tune is the request divided by one plus the fraction, so the value to
                state is the read-back carried back out of that scale. Asking for 1 GHz at
                +100 ppm tunes about 999.9 MHz, and the tuner then reads that back.
        */
        const double tuned = capture::uhd::tuneFrequencyFor(1.0e9, 100.0, std::vector<std::pair<double, double>>{{49.0e6, 6001.0e6}}).value();
        expect(tuned < 1.0e9) << "a fast reference tunes low";
        expect(std::abs(tuned - 1.0e9 / 1.0001) < 1e-3) << "by the exact scale, the request divided by one plus the fraction";
        expect(std::abs(capture::uhd::tunedFrequencyFrom(tuned, 100.0) - 1.0e9) < 1e-3) << "and the frequency stated for it is the one asked for";

        /*| frame: the first-order product differs from the exact form by the request times the
                square of the fraction, which at 1 GHz and 100 ppm is ten hertz — larger than
                the hundredth of a hertz a USRP tunes in.
        */
        expect(std::abs(tuned - 1.0e9 * (1.0 - 100.0 / 1e6)) > 5.0) << "the product form sits about ten hertz away";
        expect(std::abs(tuned - 1.0e9 * (1.0 - 100.0 / 1e6)) < 15.0);

        // Without the scale carried back out, the next write of the stated value would
        // scale it a second time and the tuner would walk.
        expect(capture::uhd::tunedFrequencyFrom(999'900'000.0, 100.0) > 999'900'000.0) << "a positive correction states a frequency above the tuner's";
        expect(capture::uhd::tunedFrequencyFrom(1'000'100'000.0, -100.0) < 1'000'100'000.0) << "and a negative one below it";

        // A correction that scales to zero or turns the sign over has no inverse.
        expect(capture::uhd::tunedFrequencyFrom(100.0e6, -1.0e6) == 100.0e6) << "minus a million ppm states the read-back as it stands";
        expect(capture::uhd::tunedFrequencyFrom(100.0e6, -2.0e6) == 100.0e6) << "and so does anything beyond it";
    });

    cases("controls.uhd-rate-ladder-from-the-device", [] {
        // A B205mini: a programmable master clock, and a stated rate set that is only the
        // ladder of whichever clock the probe caught.
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220'000.0;
        b205.mclkMaxHz = 61'440'000.0;
        for (int n = 1; n <= 256; ++n) {
            const double r = 16.0e6 / static_cast<double>(n);
            b205.rates.emplace_back(r, r);
        }
        const auto ladder = capture::uhd::Source::sampleRatesFor(b205);
        expect(ladder == capture::uhd::Source::sampleRates("type=b200")) << "the picks stand, and the 16 MHz ceiling the device stated does not";
        expect(ladder.size() == 10UZ) << "ten entries";
        expect(ladder.front() == 250e3 && ladder.back() == 61440e3) << "250 kS/s to 61.44 MS/s";
        for (const double r : ladder) {
            expect(r >= b205.mclkMinHz / capture::uhd::kMaxDecimation && r <= b205.mclkMaxHz) << "every entry is inside what the master clock reaches";
        }

        // A clock range that cannot reach the top of the list drops the entries above it.
        capture::uhd::DeviceTruth narrow = b205;
        narrow.mclkMaxHz               = 20.0e6;
        const auto narrowLadder        = capture::uhd::Source::sampleRatesFor(narrow);
        expect(narrowLadder.size() == 8UZ) << "the entries above 20 MS/s are dropped";
        expect(narrowLadder[6] == 16000e3 && narrowLadder.back() == 20.0e6) << "and the fastest clock is offered after the last pick under it";

        /*| frame: a fixed master clock is the other radio: the stated set is its own ladder,
                and an N-series reaches 100 MHz over an even divisor with the FPGA halfband
                in the chain. The link cuts the top of it: UHD states 125 MB/s for an N2xx's
                link, which carries 31.25 MS/s at four bytes a sample.
        */
        capture::uhd::DeviceTruth fixed;
        fixed.mclkMinHz     = 100.0e6;
        fixed.mclkMaxHz     = 100.0e6;
        fixed.linkBytesPerS = 125.0e6;
        for (int n = 1; n <= 512; ++n) {
            const double r = 100.0e6 / static_cast<double>(n);
            fixed.rates.emplace_back(r, r);
        }
        const auto fixedLadder = capture::uhd::reachableRatesFor(fixed);
        expect(std::ranges::find(fixedLadder, 100.0e6 / 3.0) == fixedLadder.end()) << "an odd divisor runs the CIC alone and droops, so the radio does not reach it";
        expect(std::ranges::find(fixedLadder, 100.0e6 / 7.0) == fixedLadder.end());
        expect(std::ranges::find(fixedLadder, 100.0e6) == fixedLadder.end()) << "the undivided clock is four times what the link carries";
        expect(std::ranges::find(fixedLadder, 50.0e6) == fixedLadder.end()) << "and half of it is twice";
        expect(fixedLadder.back() == 25.0e6) << "25 MS/s is the highest even divisor under the link's 31.25 MS/s";
        expect(std::ranges::find(fixedLadder, 12.5e6) != fixedLadder.end()) << "every even divisor below it is reachable";
        expect(fixedLadder.size() > 100UZ) << "which on a 100 MHz clock is hundreds of entries";
        expect(std::ranges::is_sorted(fixedLadder)) << "ascending";

        /*| frame: what a caller is offered is shorter than that. The entries below about
                1 MS/s sit less than a percent apart, and a list drawn over all of them
                cannot be aimed by hand, so the offer is the static picks for the kind that
                lie on the ladder plus the ladder's own top.
        */
        const auto offered = capture::uhd::Source::sampleRatesFor(fixed);
        expect(offered.size() == capture::uhd::Source::sampleRates("type=usrp2").size()) << "the offer is the static picks, every one of which this clock reaches";
        expect(offered.front() == 250.0e3 && offered.back() == 25.0e6) << "250 kS/s to the link's ceiling";
        for (const double r : offered) {
            expect(std::ranges::find(fixedLadder, r) != fixedLadder.end()) << "and every entry offered is one the radio reaches";
        }
        expect(offered.size() < fixedLadder.size()) << "the offer is a part of the ladder and not the whole of it";

        // sc8 is two bytes a sample instead of four, so the same link carries twice the rate.
        expect(capture::uhd::wireBytesPerSample("sc16") == 4UZ);
        expect(capture::uhd::wireBytesPerSample("sc8") == 2UZ);
        expect(capture::uhd::linkCeilingHz("sc8", fixed.linkBytesPerS) == 2.0 * capture::uhd::linkCeilingHz("sc16", fixed.linkBytesPerS)) << "half the bytes is twice the rate";
        const auto narrowWire = capture::uhd::reachableRatesFor(fixed, "sc8");
        expect(narrowWire.back() == 50.0e6) << "so sc8 reaches the divisor sc16 could not carry";
        expect(std::ranges::find(narrowWire, 100.0e6) == narrowWire.end()) << "and still not the undivided clock";
        expect(capture::uhd::Source::sampleRatesFor(fixed, "sc8").back() == 50.0e6) << "and the top of the ladder is offered whatever the picks reach";

        // A link UHD states no rate for cuts nothing.
        capture::uhd::DeviceTruth unstated = fixed;
        unstated.linkBytesPerS           = 0.0;
        expect(capture::uhd::reachableRatesFor(unstated).back() == 100.0e6) << "with no link rate the ladder is the clock's own reach";

        /*| frame: a request between two entries goes to the nearer entry of the whole
                ladder, so the rate the block states is one it chose rather than whatever the
                device coerced it to. */
        expect(capture::uhd::snapToLadder(8.0e6, fixedLadder) == 100.0e6 / 12.0) << "8 MS/s is nearest decimation 12";
        expect(capture::uhd::snapToLadder(30.72e6, fixedLadder) == 25.0e6) << "and a request above the ceiling lands on the ceiling";
        expect(capture::uhd::snapToLadder(12.5e6, fixedLadder) == 12.5e6) << "an entry is its own nearest";
        expect(capture::uhd::snapToLadder(8.0e6, {}) == 8.0e6) << "an empty ladder leaves the request alone";

        // A probe that described nothing falls back to the picks rather than to an empty
        // list, because a caller with no rates to offer cannot start.
        expect(capture::uhd::Source::sampleRatesFor({}) == capture::uhd::Source::sampleRates("type=b200")) << "no truth is the static answer";
        expect(capture::uhd::reachableRatesFor({}).empty()) << "and a radio that stated nothing reaches nothing this can name";
    });

    cases("controls.uhd-the-source-rate-answers-for-two-radios", [] {
        /*| frame: an N210, a fixed 100 MHz clock reached over a 1 Gbit link, and a B205mini, whose
                clock is programmable. The receive block's answers are the shared ladder's.
        */
        capture::uhd::DeviceTruth n210;
        n210.mclkMinHz     = 100.0e6;
        n210.mclkMaxHz     = 100.0e6;
        n210.linkBytesPerS = 125.0e6;
        for (int n = 1; n <= 512; ++n) {
            const double r = 100.0e6 / static_cast<double>(n);
            n210.rates.emplace_back(r, r);
        }
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220'000.0;
        b205.mclkMaxHz = 61'440'000.0;
        for (int n = 1; n <= 256; ++n) {
            const double r = 16.0e6 / static_cast<double>(n);
            b205.rates.emplace_back(r, r);
        }

        capture::uhd::Source src;
        src._openTruth = n210;
        expect(src.rateToWrite(8.0e6) == 100.0e6 / 12.0) << "8 MS/s on an N210 is decimation 12";
        expect(src.rateToWrite(30.72e6) == 25.0e6) << "and 30.72 MS/s is the top of the ladder the link allows";
        expect(capture::uhd::Source::sampleRatesFor(n210) == std::vector<double>{250e3, 500e3, 1000e3, 2000e3, 2500e3, 5000e3, 6250e3, 10000e3, 12500e3, 25000e3}) << "an N210 is offered the static picks";
        src._openTruth = b205;
        expect(src.rateToWrite(8.0e6) == 8.0e6) << "a B205mini reaches 8 MS/s by choosing its clock";
        expect(src.rateToWrite(30.72e6) == 30.72e6) << "and 30.72 MS/s the same way";
        expect(capture::uhd::Source::sampleRatesFor(b205) == capture::uhd::programmableClockRates()) << "and is offered the programmable picks";
        for (const auto* t : {&n210, &b205}) {
            src._openTruth = *t;
            for (const double request : {8.0e6, 30.72e6}) {
                expect(src.rateToWrite(request) == capture::uhd::rateToWrite(request, *t, 0.0, "sc16", capture::uhd::Direction::Receive, "case")) << "the block answers what the shared ladder answers";
            }
        }
    });

    cases("controls.uhd-an-mpm-device-is-cut-at-its-own-rate", [] {
        using capture::uhd::Direction;
        /*| frame: a device an MPM daemon runs carries mpm_version beside a link-rate node that
                holds 125000000 bytes per second whatever its link, kept as a size_t. This one
                runs a fixed 2 GHz clock and is reached by address, over a 100 Gbit link that
                no node states.
        */
        TreeStandIn mpm;
        mpm.clockRange = ::uhd::meta_range_t(2.0e9, 2.0e9, 0.0);
        mpm.clockHz    = 2.0e9;
        mpm.device->tree->create<std::size_t>("/mboards/0/link_max_rate").set(125000000UZ);
        mpm.device->tree->create<std::string>("/mboards/0/mpm_version").set("5.3");
        capture::uhd::DeviceTruth truth;
        capture::uhd::readTruthInto(mpm, truth, Direction::Receive);
        expect(capture::uhd::statedLinkBytesPerS(mpm) == 0.0) << "a size_t node is one UHD's check does not read, so it states nothing";
        expect(truth.linkBytesPerS == 0.0) << "and the truth carries no link";
        expect(capture::uhd::offersOneClockRate(truth)) << "one master-clock rate is a fixed clock";
        expect(capture::uhd::reachableRatesFor(truth).back() == 2.0e9) << "the ladder reaches the radio's own clock";
        expect(capture::uhd::rateToWrite(2.0e9, truth, 0.0, "sc16", Direction::Receive, "case") == 2.0e9) << "a 2 GS/s request is written as it stands";
        expect(capture::uhd::Source::sampleRatesFor(truth).back() == 2.0e9) << "and the top of the offer is the radio's own rate";
        expect(truth.describe().find("link rate unknown, ceiling 2000 MS/s from the radio") != std::string::npos) << "the device line names the ceiling and where it comes from";

        /*| frame: the same radio with the node as a double, the type UHD's check reads, holding
                the 125 MB/s an N2xx states. The figure then cuts the ladder at 31.25 MS/s in
                sc16, which is the 64th divisor of the clock. */
        TreeStandIn stated = mpm;
        stated.device      = std::make_shared<TreeStandIn::Device>();
        stated.device->tree->create<double>("/mboards/0/link_max_rate").set(125.0e6);
        capture::uhd::DeviceTruth statedTruth;
        capture::uhd::readTruthInto(stated, statedTruth, Direction::Receive);
        expect(statedTruth.linkBytesPerS == 125.0e6) << "a double node states the link, nothing taken off";
        expect(capture::uhd::rateToWrite(2.0e9, statedTruth, 0.0, "sc16", Direction::Receive, "case") == 31.25e6) << "and cuts a 2 GS/s request to 31.25 MS/s";
        expect(statedTruth.describe().find("link 125 MB/s as UHD states it") != std::string::npos) << "the device line states UHD's figure";
        expect(statedTruth.describe().find("unknown") == std::string::npos) << "and says nothing is unknown";

        /*| frame: an X4xx at 245.76 MHz states that one clock rate. The request is its own
                clock, decimation 1, where the placeholder read as a stated link would snap it
                to 30.72 MS/s. */
        TreeStandIn x4xx = mpm;
        x4xx.clockRange  = ::uhd::meta_range_t(245.76e6, 245.76e6, 0.0);
        x4xx.clockHz     = 245.76e6;
        capture::uhd::DeviceTruth x4xxTruth;
        capture::uhd::readTruthInto(x4xx, x4xxTruth, Direction::Receive);
        expect(capture::uhd::rateToWrite(245.76e6, x4xxTruth, 0.0, "sc16", Direction::Receive, "case") == 245.76e6) << "245.76 MS/s is written as it stands";
        x4xxTruth.linkBytesPerS = 125.0e6;
        expect(capture::uhd::rateToWrite(245.76e6, x4xxTruth, 0.0, "sc16", Direction::Receive, "case") == 30.72e6) << "the placeholder taken as a stated link snaps it to decimation 8";
    });

    cases("controls.uhd-the-link-ceiling-is-the-rate-uhd-states", [] {
        using capture::uhd::Direction;
        const auto truthOf = [](TreeStandIn& device) {
            capture::uhd::DeviceTruth t;
            capture::uhd::readTruthInto(device, t, Direction::Receive);
            return t;
        };
        const auto write = [](double hz, const capture::uhd::DeviceTruth& t, double clockHz, std::string_view wire) { return capture::uhd::rateToWrite(hz, t, clockHz, wire, Direction::Receive, "case"); };

        // The ceiling is the stated bytes per second over the bytes a sample takes, exactly.
        expect(capture::uhd::linkCeilingHz("sc16", 125.0e6) == 31.25e6) << "UHD's N2xx figure carries 31.25 MS/s in sc16";
        expect(capture::uhd::linkCeilingHz("sc8", 125.0e6) == 62.5e6) << "and 62.5 MS/s in sc8";
        expect(capture::uhd::linkCeilingHz("sc16", 500.0e6) == 125.0e6) << "a B2xx on USB 3 carries 125 MS/s";
        expect(capture::uhd::linkCeilingHz("sc16", 53.248e6) == 13.312e6) << "and on USB 2 13.312 MS/s";

        /*| frame: an X3xx: a fixed 200 MHz clock, reached by address over a link UHD states no
                rate for, as UHD creates no link-rate node for the kind. The reader takes no
                selector, so the address has no say. */
        TreeStandIn x3xx;
        x3xx.clockRange             = ::uhd::meta_range_t(200.0e6, 200.0e6, 0.0);
        x3xx.clockHz                = 200.0e6;
        const auto x3xxTruth        = truthOf(x3xx);
        expect(x3xxTruth.linkBytesPerS == 0.0) << "no node, no link rate";
        expect(capture::uhd::reachableRatesFor(x3xxTruth).back() == 200.0e6) << "the ladder reaches the radio's own 200 MS/s";
        expect(write(200.0e6, x3xxTruth, 0.0, "sc16") == 200.0e6) << "and a 200 MS/s request is written as it stands";
        expect(write(30.72e6, x3xxTruth, 0.0, "sc16") == 200.0e6 / 6.0) << "a request between two divisors goes to the nearer even one";
        expect(x3xxTruth.describe().find("link rate unknown, ceiling 200 MS/s from the radio") != std::string::npos) << "the device line says the link rate is unknown";

        /*| frame: an N2xx: a fixed 100 MHz clock, and the 125 MB/s double UHD's usrp2 driver
                states for its Gigabit Ethernet link. sc16 carries 31.25 MS/s, whose highest
                even divisor is 25 MS/s, and sc8 carries 62.5 MS/s, which reaches 50 MS/s. */
        TreeStandIn n2xx;
        n2xx.device->tree->create<double>("/mboards/0/link_max_rate").set(125.0e6);
        const auto n2xxTruth = truthOf(n2xx);
        expect(n2xxTruth.linkBytesPerS == 125.0e6) << "the truth carries UHD's figure, nothing taken off";
        expect(capture::uhd::reachableRatesFor(n2xxTruth).back() == 25.0e6) << "the sc16 ladder tops at 25 MS/s";
        expect(write(50.0e6, n2xxTruth, 0.0, "sc16") == 25.0e6) << "a 50 MS/s request is snapped to it";
        expect(write(50.0e6, n2xxTruth, 0.0, "sc8") == 50.0e6) << "and written as it stands in sc8";
        expect(n2xxTruth.describe() == std::string("N210r4, link 125 MB/s as UHD states it")) << "the device line states UHD's figure";

        /*| frame: a B2xx: a programmable clock to 61.44 MHz, and the double UHD's b200 driver
                states for its bus: 500 MB/s on USB 3, 53.248 MB/s on USB 2. A clock a control
                pinned makes the radio fixed, and the ladder of that clock is cut at the
                ceiling. */
        const auto b2xxOn = [&truthOf](double bytesPerS) {
            TreeStandIn b2xx;
            b2xx.clockRange = ::uhd::meta_range_t(220.0e3, 61.44e6, 0.0);
            b2xx.clockHz    = 30.72e6;
            if (bytesPerS > 0.0) {
                b2xx.device->tree->create<double>("/mboards/0/link_max_rate").set(bytesPerS);
            }
            return truthOf(b2xx);
        };
        for (const auto& [bytesPerS, pinnedTop] : {std::pair{500.0e6, 30.72e6}, std::pair{53.248e6, 7.68e6}}) {
            const auto b2xxTruth = b2xxOn(bytesPerS);
            expect(b2xxTruth.linkBytesPerS == bytesPerS) << "the truth carries UHD's figure for the bus";
            expect(write(30.72e6, b2xxTruth, 30.72e6, "sc16") == pinnedTop) << "a pinned 30.72 MHz clock tops at its highest even divisor the bus carries";
        }

        // On USB 3 the ceiling, 125 MS/s in sc16, lies past the converter, so it cuts nothing.
        const auto usb3 = b2xxOn(500.0e6);
        expect(write(61.44e6, usb3, 0.0, "sc16") == 61.44e6) << "a programmable clock writes 61.44 MS/s as it stands";
        expect(capture::uhd::Source::sampleRatesFor(usb3) == capture::uhd::programmableClockRates()) << "and offers every pick";

        /*| frame: on USB 2 the ceiling is 13.312 MS/s in sc16 and 26.624 MS/s in sc8. The offer
                stops there and ends with the ceiling itself, and a request past it is written
                at the ceiling, as a fixed clock's request is written at the top of its
                ladder. */
        const auto usb2 = b2xxOn(53.248e6);
        const auto usb2Sc16 = capture::uhd::Source::sampleRatesFor(usb2, "sc16");
        expect(usb2Sc16.back() == 13.312e6) << "the sc16 offer tops at UHD's ceiling";
        expect(usb2Sc16[usb2Sc16.size() - 2UZ] == 8.0e6) << "after the last pick under it";
        const auto usb2Sc8 = capture::uhd::Source::sampleRatesFor(usb2, "sc8");
        expect(usb2Sc8.back() == 26.624e6 && usb2Sc8[usb2Sc8.size() - 2UZ] == 16.0e6) << "and the sc8 offer at twice that";
        expect(write(30.72e6, usb2, 0.0, "sc16") == 13.312e6) << "30.72 MS/s is written at the sc16 ceiling";
        expect(write(30.72e6, usb2, 0.0, "sc8") == 26.624e6) << "and at the sc8 ceiling";
        expect(write(8.0e6, usb2, 0.0, "sc16") == 8.0e6) << "a request under the ceiling stands";

        // A programmable clock UHD states no link for is offered its fastest clock and takes any request.
        const auto unstatedB2xx = b2xxOn(0.0);
        expect(unstatedB2xx.linkBytesPerS == 0.0) << "no node, no link rate";
        expect(capture::uhd::Source::sampleRatesFor(unstatedB2xx).back() == 61.44e6) << "the offer reaches the fastest clock";
        expect(write(61.44e6, unstatedB2xx, 0.0, "sc16") == 61.44e6) << "and 61.44 MS/s is written as it stands";
    });

    cases("controls.uhd-several-clock-rates-are-a-variable-clock", [] {
        using capture::uhd::Direction;
        /*| frame: a radio that states two master-clock rates, 245.76 and 250 MHz, and the rate
                set of the one in force. UHD chooses the clock for a request, so the request
                is written as it stands.
        */
        TreeStandIn two;
        two.clockRange = ::uhd::meta_range_t();
        two.clockRange.push_back(::uhd::range_t(245.76e6));
        two.clockRange.push_back(::uhd::range_t(250.0e6));
        two.clockHz = 245.76e6;
        two.device->tree->create<std::string>("/mboards/0/mpm_version").set("5.3");
        capture::uhd::DeviceTruth truth;
        capture::uhd::readTruthInto(two, truth, Direction::Receive);
        expect(truth.mclkMinHz == 245.76e6 && truth.mclkMaxHz == 250.0e6) << "the truth spans both rates";
        expect(!capture::uhd::offersOneClockRate(truth)) << "two rates are a variable clock";
        expect(capture::uhd::reachableRatesFor(truth).empty()) << "which states no ladder of its own";
        expect(capture::uhd::rateToWrite(100.0e6, truth, 0.0, "sc16", Direction::Receive, "case") == 100.0e6) << "and takes a request as it stands";
        const auto offer = capture::uhd::Source::sampleRatesFor(truth);
        expect(offer.back() == 250.0e6) << "the offer reaches the fastest clock, far past the last pick";
        expect(offer[offer.size() - 2UZ] == 245.76e6 && offer[offer.size() - 3UZ] == 61.44e6) << "which follows the picks and the other stated clock";

        // The same radio stating only the rate in force is a fixed clock, and the request snaps.
        TreeStandIn one = two;
        one.clockRange  = ::uhd::meta_range_t(245.76e6, 245.76e6, 0.0);
        capture::uhd::DeviceTruth oneTruth;
        capture::uhd::readTruthInto(one, oneTruth, Direction::Receive);
        expect(capture::uhd::offersOneClockRate(oneTruth)) << "one rate is a fixed clock";
        expect(capture::uhd::rateToWrite(100.0e6, oneTruth, 0.0, "sc16", Direction::Receive, "case") == 122.88e6) << "whose nearest even decimation to 100 MS/s is 2";
        expect(!capture::uhd::offersOneClockRate(capture::uhd::DeviceTruth{})) << "a truth stating no clock is neither";
    });

    cases("controls.uhd-a-refused-filter-write-states-no-width", [] {
        /*| frame: the filter applier both blocks run, driven through the two callables that
                stand for the device. A write the device refuses throws, as UHD does.
        */
        for (const auto dir : {capture::uhd::Direction::Receive, capture::uhd::Direction::Transmit}) {
            std::optional<double> written;
            double                held   = 1.5e6; // a width an earlier write left
            int                   writes = 0;
            capture::uhd::applyBandwidthWith(
                2.0e6, written, held,
                [&writes](double) {
                    ++writes;
                    throw std::runtime_error("refused");
                },
                [] { return 2.0e6; }, dir, "case");
            expect(writes == 1) << "the write was made";
            expect(held == 0.0) << "a refused write leaves the block stating no width";
            expect(!written.has_value()) << "and nothing recorded as written, so the next request writes again";

            capture::uhd::applyBandwidthWith(2.0e6, written, held, [](double) {}, [] { return 1.92e6; }, dir, "case");
            expect(written == std::optional<double>{2.0e6}) << "a write that lands is recorded";
            expect(held == 1.92e6) << "and the width stated is the one the device settled on";

            capture::uhd::applyBandwidthWith(2.0e6, written, held, [&writes](double) { ++writes; }, [] { return 0.0; }, dir, "case");
            expect(writes == 1 && held == 1.92e6) << "a width already in force is not written again";
        }
    });

    cases("controls.uhd-a-refused-read-costs-its-own-field", [] {
        using capture::uhd::Direction;
        const auto read = [](std::set<std::string> refused, Direction dir) {
            TruthStandIn device;
            device.refuse = std::move(refused);
            capture::uhd::DeviceTruth truth;
            capture::uhd::readTruthInto(device, truth, dir);
            return truth;
        };
        const auto names = [](const capture::uhd::DeviceTruth& t) {
            std::vector<std::string> out;
            for (const auto& g : t.gains) {
                out.push_back(g.name);
            }
            return out;
        };
        for (const auto dir : {Direction::Receive, Direction::Transmit}) {
            const auto whole = read({}, dir);
            expect(names(whole) == std::vector<std::string>{"PGA0", "BAD", "ADC-pga"}) << "nothing refused, every element";
            expect(whole.product == "N210r4" && whole.clockSources.size() == 3UZ && whole.timeSources.size() == 2UZ && whole.bwCurHz == 2.0e6);
            expect(whole.linkBytesPerS == 0.0) << "a device stating no link rate carries none";
            expect(whole.mclkHz == 100.0e6 && !capture::uhd::reachableRatesFor(whole).empty()) << "and the rate ladder is read";

            expect(names(read({"gain_range:BAD"}, dir)) == std::vector<std::string>{"PGA0", "ADC-pga"}) << "a refused element range drops that element alone";

            const auto noWidth = read({"bandwidth"}, dir);
            expect(noWidth.bwMinHz == 0.2e6 && noWidth.bwMaxHz == 56.0e6) << "a refused filter width keeps the range";
            expect(noWidth.bwCurHz == 0.0) << "and states no width";

            const auto noTimeList = read({"time_sources"}, dir);
            expect(noTimeList.clockSources.size() == 3UZ && noTimeList.clockSource == "internal") << "a refused time-source list keeps the clock-source list";
            expect(noTimeList.timeSources.empty() && noTimeList.timeSource == "none") << "and the time source in force";

            const auto noClockList = read({"clock_sources"}, dir);
            expect(noClockList.clockSources.empty() && noClockList.clockSource == "internal") << "a refused clock-source list keeps the clock in force";
            expect(noClockList.timeSources.size() == 2UZ && noClockList.timeSource == "none") << "and the time-source list";

            const auto noClock = read({"clock_source"}, dir);
            expect(noClock.clockSources.size() == 3UZ && noClock.timeSources.size() == 2UZ) << "a refused clock-source read keeps both lists";

            const auto noPorts = read({"antennas"}, dir);
            expect(noPorts.antennas.empty() && noPorts.antenna == "TX/RX") << "a refused connector list keeps the connector in force";
        }
    });

    cases("controls.uhd-bounds-every-control-value", [] {
        /*| frame: a B205mini's receive surface at 2 MS/s: one gain element, the filter, both
                corrections, the reference and timing lists, the LO offset, the device-time
                toggle, the wire format and the master clock.
        */
        capture::uhd::DeviceTruth t;
        t.gains.push_back({"PGA", 0.0, 76.0, 1.0, 30.0});
        t.gainMinDb        = 0.0;
        t.gainMaxDb        = 76.0;
        t.bwMinHz          = 0.2e6;
        t.bwMaxHz          = 56.0e6;
        t.mclkMinHz        = 220'000.0;
        t.mclkMaxHz        = 61'440'000.0;
        t.hasDcOffsetCorr  = true;
        t.hasIqBalanceCorr = true;
        t.clockSources     = {"internal", "external", "gpsdo"};
        t.clockSource      = "external";
        t.timeSources      = {"none", "external"};
        const double                nan = std::numeric_limits<double>::quiet_NaN();
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};
        const auto                  surface = capture::uhd::Source::describeControls(t, ctx, 0.0);
        const auto                  bound   = [&surface](const std::string& id, double v) { return capture::uhd::boundedControlValue(surface, id, v); };

        expect(bound("BANDWIDTH", nan) == std::optional<double>{2.0}) << "a filter width that is not a number is the rate-derived default";
        expect(bound("BANDWIDTH", 80.0) == std::optional<double>{56.0}) << "and one past the range is its top";
        expect(bound("MASTER_CLOCK", nan) == std::optional<double>{0.0}) << "a clock that is not a number is the automatic entry";
        expect(bound("DC_CORR", nan) == std::optional<double>{1.0}) << "a correction that is not a number stays on";
        expect(bound("LO_OFFSET", nan) == std::optional<double>{0.0}) << "an offset that is not a number is none";
        expect(bound("LO_OFFSET", 5.0) == std::optional<double>{1.0}) << "and one past half the rate is half the rate";
        expect(bound("DEVICE_TIME", nan) == std::optional<double>{0.0}) << "the device-time toggle that is not a number is off";
        expect(bound("CLOCK_SOURCE", nan) == std::optional<double>{1.0}) << "a source index that is not a number is the source in force";
        expect(!bound("CLOCK_SOURCE", 3.0).has_value()) << "and an index past the list names no source";
        expect(!bound("WIRE_FORMAT", -1.0).has_value()) << "nor a format";
        expect(bound("WIRE_FORMAT", 1.0) == std::optional<double>{1.0}) << "an index on the list stands";
        expect(!bound("ANTENNA", 0.0).has_value()) << "an id the surface does not state is refused";

        capture::uhd::DeviceTruth fixed = t;
        fixed.mclkMinHz                 = 100.0e6;
        fixed.mclkMaxHz                 = 100.0e6;
        expect(!capture::uhd::boundedControlValue(capture::uhd::Source::describeControls(fixed, ctx, 0.0), "MASTER_CLOCK", 100.0).has_value()) << "a fixed clock offers no clock control to write";

        expect(capture::uhd::boundedGainFor(t, "", nan, "case") == 0.0) << "an overall gain that is not a number is the bottom of the range";
        expect(capture::uhd::boundedGainFor(t, "", 200.0, "case") == 76.0) << "and one past the range is its top";
        expect(capture::uhd::boundedGainFor(t, "PGA", -3.0, "case") == 0.0) << "an element is bounded by its own range";
        expect(capture::uhd::boundedGainFor(t, "NOPE", 5.0, "case") == 5.0) << "a name the device does not list is left to the name check";

        capture::uhd::DeviceTruth tx;
        tx.antennas = {"TX/RX", "RX2"};
        tx.antenna  = "TX/RX";
        const auto sinkSurface = capture::uhd::Sink::describeControls(tx, ctx, 0.0);
        expect(!capture::uhd::boundedControlValue(sinkSurface, "ANTENNA", 2.0).has_value()) << "the transmit connector index past the list is refused as well";
        expect(capture::uhd::boundedControlValue(sinkSurface, "ANTENNA", nan) == std::optional<double>{0.0}) << "and one that is not a number is the connector the block defaults to";
    });

    cases("controls.uhd-a-pinned-clock-is-a-fixed-clock", [] {
        /*| frame: a B205mini once a control has written its converter clock. The device
                still states the range it can reach, but UHD's own choice is cleared inside
                the driver until the next open, so every rate from here is that one clock
                over a decimation.
        */
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220'000.0;
        b205.mclkMaxHz = 61'440'000.0;
        b205.mclkHz    = 30.72e6;
        for (int n = 1; n <= 256; ++n) {
            const double r = 30.72e6 / static_cast<double>(n); // the ladder the device states once the clock is pinned
            b205.rates.emplace_back(r, r);
        }

        // Left on the stated range the radio reads as programmable, and nothing is snapped.
        expect(capture::uhd::reachableRatesFor(b205).empty()) << "a clock that can move states no ladder of its own";
        expect(capture::uhd::Source::sampleRatesFor(b205) == capture::uhd::Source::sampleRates("type=b200")) << "so the picks are what a caller is offered";

        const auto pinned = capture::uhd::ladderTruthFor(b205, 30.72e6);
        expect(pinned.mclkMinHz == pinned.mclkMaxHz) << "a clock in force is a fixed clock";
        expect(pinned.mclkMinHz == 30.72e6);
        const auto ladder = capture::uhd::reachableRatesFor(pinned);
        expect(!ladder.empty()) << "which states its own ladder";
        expect(std::ranges::find(ladder, 30.72e6 / 15.0) == ladder.end()) << "decimation 15 is odd, so the radio is not offered it";
        expect(std::ranges::find(ladder, 30.72e6 / 16.0) != ladder.end()) << "decimation 16 is even";
        expect(capture::uhd::snapToLadder(2.0e6, ladder) == 30.72e6 / 16.0) << "so a 2 MS/s request lands on 1.92 MS/s rather than on the 2.048 MS/s UHD reaches by an odd decimation";
        expect(capture::uhd::snapToLadder(7.68e6, ladder) == 7.68e6) << "and a rate the pinned clock divides exactly is its own answer";

        // Nothing pinned leaves the truth alone, whatever the device stated.
        expect(capture::uhd::ladderTruthFor(b205, 0.0).mclkMaxHz == b205.mclkMaxHz) << "no clock in force changes nothing";
    });

    cases("controls.uhd-a-silent-receive-counts-whatever-it-says", [] {
        /*| frame: UHD names several ways a receive can fail — a timeout, a bad packet, a
                failed alignment, a broken chain — and a device stuck on any of them
                delivers nothing. One deadline counts them all, because what the liveness
                check is about is samples and not error codes.
        */
        int consecutive = 0;
        for (int i = 0; i < 10; ++i) {
            consecutive = capture::uhd::Source::livenessAfterReceive(consecutive, 0UZ);
        }
        expect(consecutive == 10) << "ten receives with no samples are ten against the deadline";
        expect(capture::uhd::Source::livenessAfterReceive(consecutive, 2040UZ) == 0) << "and one that delivered samples starts the count again";
        expect(capture::uhd::Source::livenessAfterReceive(capture::uhd::Source::kDeadReceives - 1, 0UZ) == capture::uhd::Source::kDeadReceives) << "the last silent receive reaches the deadline";
        expect(capture::uhd::Source::kDeadReceives * capture::uhd::Source::kRecvTimeoutS >= 5.0) << "which is at least five seconds of silence";

        // One line per error kind per stream, rather than one per iteration.
        std::vector<int> seen;
        expect(capture::uhd::Source::firstSightOf(seen, 8)) << "the first of a kind is said";
        expect(!capture::uhd::Source::firstSightOf(seen, 8)) << "and the next thousand are not";
        expect(capture::uhd::Source::firstSightOf(seen, 12)) << "another kind is its own first";
        expect(seen.size() == 2UZ) << "so the list is one entry per kind";
    });

    cases("controls.uhd-reports-a-rate-the-host-cannot-carry", [] {
        /*| frame: what the host took against what the rate in force should have delivered.
                No header states what a link carries, so this is the one statement about it
                that holds whatever the link is.
        */
        expect(capture::uhd::Source::deliveryShortfall(100'000'000UL, 10.0, 10.0e6).value() == 0.0) << "a stream that kept up is short of nothing";
        expect(std::abs(capture::uhd::Source::deliveryShortfall(508'000'000UL, 10.0, 61.44e6).value() - 0.173) < 0.002) << "a B205mini asked for 61.44 MS/s delivers about 83 percent of it";
        expect(capture::uhd::Source::deliveryShortfall(508'000'000UL, 10.0, 61.44e6).value() > capture::uhd::Source::kShortfallFraction) << "which is past the fraction that is worth a line";
        expect(capture::uhd::Source::deliveryShortfall(101'000'000UL, 10.0, 10.0e6).value() < 0.0) << "a count past the rate is a negative shortfall rather than a fault";

        expect(!capture::uhd::Source::deliveryShortfall(0UL, 0.0, 10.0e6).has_value()) << "no window says nothing";
        expect(!capture::uhd::Source::deliveryShortfall(0UL, 10.0, 0.0).has_value()) << "and neither does no rate";

        // The clock is read once the stream has delivered what the rest of the window holds.
        using Src = capture::uhd::Source;
        expect(Src::shortfallNextCheck(0UL, 0.0, 10.0e6) == static_cast<std::uint64_t>(Src::kShortfallWindowS * 10.0e6)) << "from the start, the whole window at the rate";
        expect(Src::shortfallNextCheck(40'000'000UL, Src::kShortfallWindowS / 2.0, 10.0e6) == 40'000'000UL + static_cast<std::uint64_t>(Src::kShortfallWindowS / 2.0 * 10.0e6)) << "part way, the rest of it";
        expect(Src::shortfallNextCheck(7UL, Src::kShortfallWindowS, 10.0e6) == 7UL) << "a closed window reads the clock at the next receive";
        expect(Src::shortfallNextCheck(7UL, Src::kShortfallWindowS - 1.0e-9, 10.0e6) == 7UL) << "and so does one closing inside a sample";
        expect(Src::shortfallNextCheck(7UL, 0.0, 0.0) == 7UL) << "as does a rate of zero";
        expect(Src::shortfallNextCheck(7UL, 0.0, std::numeric_limits<double>::quiet_NaN()) == 7UL) << "or a rate that is not a number";
        expect(Src::shortfallNextCheck(7UL, 0.0, std::numeric_limits<double>::infinity()) > (1ULL << 61U)) << "an unbounded rate stays inside the type";
    });

    cases("controls.uhd-offers-only-the-controls-the-device-has", [] {
        const auto named = [](const std::vector<capture::ControlDesc>& cs, const char* id) { return std::ranges::find_if(cs, [id](const auto& c) { return c.id == id; }) != cs.end(); };
        const capture::TruthContext ctx{.deviceParams = "serial=X", .sampleRate = 2.0e6, .centerFreq = 100.0e6};

        // A B2xx: a filter with room in it, both frontend corrections, a programmable clock.
        capture::uhd::DeviceTruth rich;
        rich.bwMinHz          = 0.2e6;
        rich.bwMaxHz          = 56.0e6;
        rich.mclkMinHz        = 220.0e3;
        rich.mclkMaxHz        = 61.44e6;
        rich.hasAgc           = true;
        rich.hasDcOffsetCorr  = true;
        rich.hasIqBalanceCorr = true;
        const auto richControls = capture::uhd::Source::describeControls(rich, ctx);
        expect(named(richControls, "BANDWIDTH")) << "a filter with room in it is a control";
        expect(named(richControls, "DC_CORR")) << "and so is each correction the frontend carries";
        expect(named(richControls, "IQ_CORR"));
        expect(named(richControls, "MASTER_CLOCK")) << "a programmable clock is a control";
        expect(named(richControls, "WIRE_FORMAT")) << "the wire format is offered on every model";

        /*| frame: an N-series with a WBX: a fixed 40 MHz filter, a DC offset correction and
                no IQ balance one, and a clock that cannot be moved. */
        capture::uhd::DeviceTruth plain;
        plain.bwMinHz         = 40.0e6;
        plain.bwMaxHz         = 40.0e6;
        plain.bwCurHz         = 40.0e6;
        plain.mclkMinHz       = 100.0e6;
        plain.mclkMaxHz       = 100.0e6;
        plain.hasDcOffsetCorr = true;
        const auto plainControls = capture::uhd::Source::describeControls(plain, ctx);
        expect(!named(plainControls, "BANDWIDTH")) << "a filter with one width is not a control a caller can move";
        expect(named(plainControls, "DC_CORR")) << "the correction the tree carries is offered";
        expect(!named(plainControls, "IQ_CORR")) << "and the one it does not carry is not";
        expect(!named(plainControls, "MASTER_CLOCK")) << "a fixed clock is not a control";
        expect(named(plainControls, "WIRE_FORMAT"));

        // The wire format's options are the formats a streamer is built for, sc16 first.
        const auto wire = std::ranges::find_if(plainControls, [](const auto& c) { return c.id == "WIRE_FORMAT"; });
        expect(wire->kind == 3) << "an enum over the format list";
        expect(wire->options == std::vector<std::string>{"sc16", "sc8"});
        expect(wire->defValue == 0.0) << "sc16 by default, which keeps every sample's sixteen bits";
        expect(wire->label.find("dynamic range") != std::string::npos) << "and the label says what the narrow format costs";
    });

    cases("controls.uhd-states-when-each-control-applies", [] {
        /*| frame: a radio with every control the source can state: two gain elements, a
                filter with room in it, both corrections, a programmable clock, and the
                reference and timing lists of a unit carrying a GPSDO. */
        capture::uhd::DeviceTruth t;
        t.gains.push_back({"PGA", 0.0, 76.0, 1.0, 30.0});
        t.gains.push_back({"ADC-pga", 0.0, 6.0, 0.5, 3.0});
        t.bwMinHz          = 0.2e6;
        t.bwMaxHz          = 56.0e6;
        t.mclkMinHz        = 220.0e3;
        t.mclkMaxHz        = 61.44e6;
        t.hasDcOffsetCorr  = true;
        t.hasIqBalanceCorr = true;
        t.clockSources     = {"internal", "external", "mimo", "gpsdo"};
        t.clockSource      = "gpsdo";
        t.timeSources      = {"none", "external", "mimo", "gpsdo"};
        t.timeSource       = "gpsdo";
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 100.0e6};
        const auto                  surface = capture::uhd::Source::describeControls(t, ctx, 2.0e6);

        /*| frame: the four settings the start alone reads that a control reaches: the
                reference and timing selections are written before the stream is configured,
                the wire format when the streamer is built, and the converter clock as a key of
                the address the device is opened with. */
        const std::set<std::string> atStart{"CLOCK_SOURCE", "TIME_SOURCE", "WIRE_FORMAT", "MASTER_CLOCK"};
        std::set<std::string>       stated;
        for (const auto& c : surface) {
            stated.insert(c.id);
            const bool start = c.appliesAt == capture::AppliesAt::Start;
            expect(start == atStart.contains(c.id)) << c.id << " states the start exactly where the start alone reads its setting";
        }
        for (const std::string& id : atStart) {
            expect(stated.contains(id)) << "the surface offers " << id;
        }
        for (const char* id : {"PGA", "ADC-pga", "BANDWIDTH", "DC_CORR", "IQ_CORR", "LO_OFFSET", "DEVICE_TIME"}) {
            expect(stated.contains(id)) << "and " << id << ", which a running stream takes";
        }

        // The transmit surface holds no control that waits for a start.
        capture::uhd::DeviceTruth tx = t;
        tx.antennas                  = {"TX/RX", "CAL"};
        tx.antenna                   = "TX/RX";
        for (const auto& c : capture::uhd::Sink::describeControls(tx, ctx, 2.0e6)) {
            expect(c.appliesAt == capture::AppliesAt::Runtime) << c.id << " on the sink takes its write in the running stream";
        }
    });

    cases("controls.uhd-reference-selection-comes-from-the-device-list", [] {
        // An N210 with a GPSDO lists four clock sources; a B205mini lists two of them.
        constexpr std::size_t kSources = 4UZ;

        expect(capture::uhd::Source::optionIndexFor(0.0, kSources).value() == 0UZ) << "the value is an index into the device's own list";
        expect(capture::uhd::Source::optionIndexFor(3.0, kSources).value() == 3UZ) << "including its last entry";
        expect(capture::uhd::Source::optionIndexFor(1.4, kSources).value() == 1UZ) << "a value between two indices names the nearer";
        expect(capture::uhd::Source::optionIndexFor(1.6, kSources).value() == 2UZ);

        /*| frame: a stored index outlives the device it was stored against, so an index the
                next radio's list does not reach names nothing rather than its first entry or
                a position off the end of it. */
        expect(!capture::uhd::Source::optionIndexFor(4.0, kSources).has_value()) << "an index past the list names nothing";
        expect(!capture::uhd::Source::optionIndexFor(-1.0, kSources).has_value()) << "and neither does one below it";
        expect(!capture::uhd::Source::optionIndexFor(std::numeric_limits<double>::quiet_NaN(), kSources).has_value()) << "nor a value that is not a number";
        expect(!capture::uhd::Source::optionIndexFor(1.0e18, kSources).has_value()) << "nor one no index type holds";
        expect(!capture::uhd::Source::optionIndexFor(0.0, 0UZ).has_value()) << "a radio that listed no source offers no selection";
    });

    cases("controls.uhd-master-clock-ladder", [] {
        // A B205mini states 220 kHz to 61.44 MHz, which holds every pick.
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz    = 220.0e3;
        b205.mclkMaxHz    = 61.44e6;
        const auto ladder = capture::uhd::masterClockLadderMHz(b205);
        expect(ladder.front() == 0.0) << "the first entry is the choice UHD makes for itself";
        expect(ladder.back() == 61.44) << "and the last is the fastest clock the device states";
        expect(std::ranges::is_sorted(ladder)) << "ascending, in MHz";
        expect(std::ranges::find(ladder, 30.72) != ladder.end()) << "the clock 7.68 MS/s divides exactly";

        // A clock range that reaches none of the picks offers nothing at all rather than a
        // list with only the automatic entry in it.
        capture::uhd::DeviceTruth narrow = b205;
        narrow.mclkMaxHz               = 1.0e6;
        expect(capture::uhd::masterClockLadderMHz(narrow).empty()) << "no pick inside the range is no control";

        // A fixed clock is not a choice.
        capture::uhd::DeviceTruth fixed;
        fixed.mclkMinHz = 100.0e6;
        fixed.mclkMaxHz = 100.0e6;
        expect(capture::uhd::masterClockLadderMHz(fixed).empty()) << "a clock that cannot move offers nothing";
    });

    cases("controls.uhd-a-radio-stating-clocks-above-the-converter-picks", [] {
        using capture::uhd::Direction;
        const auto pointsOf = [](std::initializer_list<double> mhz) {
            TreeStandIn radio;
            radio.clockRange = ::uhd::meta_range_t();
            for (const double m : mhz) {
                radio.clockRange.push_back(::uhd::range_t(m * 1e6));
            }
            radio.clockHz = std::data(mhz)[mhz.size() - 1UZ] * 1e6;
            capture::uhd::DeviceTruth t;
            capture::uhd::readTruthInto(radio, t, Direction::Receive);
            return t;
        };
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 100.0e6};
        const auto clockOf = [&ctx](const capture::uhd::DeviceTruth& t) -> std::optional<capture::ControlDesc> {
            for (const auto& c : capture::uhd::Source::describeControls(t, ctx)) {
                if (c.id == "MASTER_CLOCK") {
                    return c;
                }
            }
            return std::nullopt;
        };

        // A radio stating three separate clocks, the three the X3xx clock driver has modes for.
        const auto x3xx = pointsOf({120.0, 184.32, 200.0});
        expect(x3xx.mclkPoints == std::vector<double>{120.0e6, 184.32e6, 200.0e6}) << "the truth carries the stated clocks";
        expect(capture::uhd::masterClockLadderMHz(x3xx) == std::vector<double>{0.0, 120.0, 184.32, 200.0}) << "the ladder is those clocks after the automatic entry";
        const auto x3xxClock = clockOf(x3xx);
        expect(fatal(x3xxClock.has_value()));
        expect(!x3xxClock->anyInRange && x3xxClock->min == 0.0 && x3xxClock->max == 200.0) << "a list of points, no span between them";
        const auto x3xxRates = capture::uhd::sampleRatesFor(x3xx);
        expect(x3xxRates.size() >= 3UZ && x3xxRates.back() == 200.0e6) << "the offer reaches the fastest clock";
        expect(std::ranges::find(x3xxRates, 120.0e6) != x3xxRates.end() && std::ranges::find(x3xxRates, 184.32e6) != x3xxRates.end()) << "and carries each stated clock";
        expect(std::ranges::is_sorted(x3xxRates));

        // An N3xx-shaped radio of each daughterboard.
        const auto n310 = pointsOf({122.88, 125.0, 153.6});
        expect(capture::uhd::masterClockLadderMHz(n310) == std::vector<double>{0.0, 122.88, 125.0, 153.6}) << "an N310 offers its three clocks";
        expect(!clockOf(n310)->anyInRange);
        const auto n320 = pointsOf({200.0, 245.76, 250.0});
        expect(capture::uhd::masterClockLadderMHz(n320) == std::vector<double>{0.0, 200.0, 245.76, 250.0}) << "an N320 offers its three";
        const auto n320Rates = capture::uhd::sampleRatesFor(n320);
        expect(n320Rates.back() == 250.0e6 && std::ranges::find(n320Rates, 245.76e6) != n320Rates.end()) << "and its rates reach 250 MS/s";

        // A link ceiling UHD states cuts the stated clocks as it cuts the picks.
        auto linked          = x3xx;
        linked.linkBytesPerS = 500.0e6;
        const auto cut       = capture::uhd::sampleRatesFor(linked);
        expect(cut.back() == 125.0e6) << "the offer tops at 500 MB/s over four bytes";
        expect(std::ranges::find(cut, 120.0e6) != cut.end() && std::ranges::find(cut, 184.32e6) == cut.end()) << "120 MHz stays and 184.32 MHz goes";

        // A span that covers the clocks above the converter picks offers them too.
        capture::uhd::DeviceTruth span;
        span.mclkMinHz      = 5.0e6;
        span.mclkMaxHz      = 250.0e6;
        span.mclkContinuous = true;
        const auto spanLadder = capture::uhd::masterClockLadderMHz(span);
        expect(spanLadder.back() == 250.0 && std::ranges::find(spanLadder, 184.32) != spanLadder.end() && std::ranges::find(spanLadder, 61.44) != spanLadder.end()) << "the picks inside the span";

        // A B2xx span stops where its converter stops.
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220.0e3;
        b205.mclkMaxHz = 61.44e6;
        expect(capture::uhd::masterClockLadderMHz(b205).back() == 61.44) << "no pick above 61.44 MHz on a B2xx";

        /*| frame: a radio stating one clock, 200 MHz, and no link rate, under a product name
                the table does not hold. The clock is fixed and offers no control; its rate offer
                climbs past the last static pick by the decimations of a power of two. */
        TreeStandIn fixed;
        fixed.clockRange = ::uhd::meta_range_t(200.0e6, 200.0e6, 0.0);
        fixed.clockHz    = 200.0e6;
        capture::uhd::DeviceTruth fixedTruth;
        capture::uhd::readTruthInto(fixed, fixedTruth, Direction::Receive);
        expect(fixedTruth.mclkPoints.empty() && capture::uhd::masterClockLadderMHz(fixedTruth).empty()) << "one clock is no choice";
        const auto fixedRates = capture::uhd::sampleRatesFor(fixedTruth);
        const auto at25       = std::ranges::find(fixedRates, 25.0e6);
        expect(fatal(at25 != fixedRates.end())) << "the last static pick";
        expect(std::vector<double>(at25, fixedRates.end()) == std::vector<double>{25.0e6, 50.0e6, 100.0e6, 200.0e6}) << "then decimations 4, 2 and 1";
    });

    cases("controls.uhd-a-product-table-offers-the-clocks-of-a-fixed-clock-radio", [] {
        using capture::uhd::Direction;
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 100.0e6};
        // A radio UHD states as one clock, named as UHD names the product, behind a frontend.
        const auto fixedAt = [](std::string_view product, double clockHz, std::string_view frontend = {}) {
            TreeStandIn radio;
            radio.clockRange = ::uhd::meta_range_t(clockHz, clockHz, 0.0);
            radio.clockHz    = clockHz;
            radio.mboardName = std::string(product);
            radio.mboardId   = std::string(product);
            radio.frontend   = std::string(frontend);
            capture::uhd::DeviceTruth t;
            capture::uhd::readTruthInto(radio, t, Direction::Receive);
            return t;
        };
        const auto clockOf = [&ctx](const capture::uhd::DeviceTruth& t) -> std::optional<capture::ControlDesc> {
            for (const auto& c : capture::uhd::Source::describeControls(t, ctx)) {
                if (c.id == "MASTER_CLOCK") {
                    return c;
                }
            }
            return std::nullopt;
        };
        const auto ladderOf = [&clockOf, &fixedAt](std::string_view product, double clockHz) {
            const auto c = clockOf(fixedAt(product, clockHz));
            return c.has_value() ? c->listValues : std::vector<double>{};
        };

        // An X310 stating 200 MHz alone.
        const auto x310 = fixedAt("X310", 200.0e6);
        expect(capture::uhd::offersOneClockRate(x310)) << "the truth is a fixed clock";
        const auto x310Clock = clockOf(x310);
        expect(fatal(x310Clock.has_value())) << "a fixed clock the table knows is a control";
        expect(x310Clock->listValues == std::vector<double>{0.0, 184.32, 200.0}) << "the automatic entry and the X3xx clocks";
        expect(!x310Clock->anyInRange && x310Clock->min == 0.0 && x310Clock->max == 200.0) << "a list of points";
        expect(x310Clock->appliesAt == capture::AppliesAt::Start) << "applied at the next start";
        expect(x310Clock->label == std::string("Master clock (200 MHz in force; a pick applies at the next start)")) << "the label names the clock in force: " << x310Clock->label;
        expect(ladderOf("X310", 184.32e6) == std::vector<double>{0.0, 184.32, 200.0}) << "the same list at the other clock";

        // The rate offer stays the ladder of the clock in force.
        auto unnamed    = x310;
        unnamed.model   = "";
        unnamed.product = "";
        const auto rateOf = [&ctx](const capture::uhd::DeviceTruth& t) {
            for (const auto& c : capture::uhd::Source::describeControls(t, ctx)) {
                if (c.id == "SAMPLE_RATE") {
                    return c;
                }
            }
            return capture::ControlDesc{};
        };
        expect(!clockOf(unnamed).has_value()) << "without a product the fixed clock has no control";
        expect(!rateOf(x310).listValues.empty() && rateOf(x310) == rateOf(unnamed)) << "the table moves no rate";

        // The ladder reads the information map's product where the truth read no motherboard name.
        auto byId  = fixedAt("", 200.0e6);
        byId.product = "X310";
        expect(byId.model.empty() && clockOf(byId).has_value() && clockOf(byId)->listValues == x310Clock->listValues) << "an X310 named by the map alone";

        // Each product of the table at a clock it takes.
        expect(ladderOf("X300", 200.0e6) == std::vector<double>{0.0, 184.32, 200.0});
        expect(ladderOf("NI-2974", 200.0e6) == std::vector<double>{0.0, 184.32, 200.0}) << "an X310 motherboard under its own name";
        expect(ladderOf("n300", 125.0e6) == std::vector<double>{0.0, 122.88, 125.0, 153.6});
        expect(ladderOf("n310", 153.6e6) == std::vector<double>{0.0, 122.88, 125.0, 153.6});
        expect(ladderOf("n320", 245.76e6) == std::vector<double>{0.0, 200.0, 245.76, 250.0});
        expect(ladderOf("x410", 250.0e6) == std::vector<double>{0.0, 245.76, 250.0}) << "an X410 on a 200 MHz image";
        expect(ladderOf("x410", 491.52e6) == std::vector<double>{0.0, 491.52, 500.0}) << "an X410 on a 400 MHz image";
        const std::vector<double> x440All{0.0, 125.0, 307.2, 327.68, 360.0, 368.64, 400.0, 500.0, 1000.0, 2000.0};
        expect(ladderOf("x440", 368.64e6) == x440All) << "an X440 clock two images take offers the clocks of both";
        expect(ladderOf("x440", 125.0e6) == x440All) << "and one every image takes offers the clocks of all three, so a pick of it keeps the control";
        expect(ladderOf("x440", 2000.0e6) == x440All) << "a clock of the xx_1600 image alone offers that image's set";

        // An X3xx behind a TwinRX takes 200 MHz alone.
        const auto twinRx = fixedAt("X310", 200.0e6, "TwinRX RX0");
        expect(twinRx.frontend == std::string("TwinRX RX0")) << "the truth reads the frontend's name";
        const auto twinRxClock = clockOf(twinRx);
        expect(fatal(twinRxClock.has_value())) << "the control stays";
        expect(twinRxClock->listValues == std::vector<double>{0.0, 200.0}) << "and offers 200 MHz alone";
        expect(clockOf(fixedAt("X310", 200.0e6, "UBX RX"))->listValues == x310Clock->listValues) << "another frontend keeps both clocks";

        // Outside the table: another product, or a clock no set of the product holds.
        expect(!clockOf(fixedAt("N210r4", 100.0e6)).has_value()) << "an N210 is still a fixed clock with no control";
        TreeStandIn n210;
        capture::uhd::DeviceTruth n210Truth;
        capture::uhd::readTruthInto(n210, n210Truth, Direction::Receive);
        expect(n210Truth.model == std::string("N210r4") && !clockOf(n210Truth).has_value()) << "as the truth reads it";
        expect(ladderOf("X310", 120.0e6).empty()) << "a clock outside the X3xx set offers nothing";
        expect(ladderOf("x410", 122.88e6).empty()) << "nor an X410 clock of an image the table does not name";
        expect(ladderOf("x310", 200.0e6).empty()) << "the name is matched as UHD spells it";

        // The B2xx path: a span of the converter's picks, labeled as before.
        capture::uhd::DeviceTruth b205;
        b205.model     = "B205mini";
        b205.mclkMinHz = 220.0e3;
        b205.mclkMaxHz = 61.44e6;
        b205.mclkHz    = 32.0e6;
        const auto b205Clock = clockOf(b205);
        expect(fatal(b205Clock.has_value()));
        expect(b205Clock->listValues == std::vector<double>{0.0, 5.0, 10.0, 16.0, 20.0, 30.72, 32.0, 40.0, 61.44}) << "the converter picks";
        expect(b205Clock->label == std::string("Master clock")) << "and the plain label";
    });

    cases("controls.uhd-a-table-clock-applies-at-the-next-open", [] {
        // An X310 opened on UHD's default 200 MHz clock, with no handle behind the block: a
        // device write would reach no handle.
        capture::uhd::Source source{};
        source._openTruth.model     = "X310";
        source._openTruth.mclkMinHz = 200.0e6;
        source._openTruth.mclkMaxHz = 200.0e6;
        source._openTruth.mclkHz    = 200.0e6;
        source.device_parameter     = std::string("type=x300,addr=192.168.40.2");
        source._deviceUp.store(true);
        expect(capture::kwargsValue(source.openAddress().to_string(), "master_clock_rate").empty()) << "the open carries no clock before a pick";

        bool              taken  = false;
        const std::string change = stderrOf([&] { taken = source.setControl("MASTER_CLOCK", 184.32); });
        expect(taken) << "a table clock is taken";
        expect(change.find("the master clock 184.32 MHz takes effect at the next start that opens the device") != std::string::npos) << "and named: " << change;
        expect(change.find("another block of this process holds open") != std::string::npos) << "with the open that does not apply it";
        expect(source._masterClockPinnedHz == 184.32e6) << "pinned";
        expect(source.openAddress().get("master_clock_rate", "") == std::string("184320000")) << "the next open carries it: " << source.openAddress().to_string();
        expect(source.controlValue("MASTER_CLOCK") == std::optional<double>{200.0}) << "the clock in force is still the one the control reads";

        const std::string replay = stderrOf([&] { taken = source.setControl("MASTER_CLOCK", 184.32); });
        expect(taken && replay.empty()) << "a replay of the pick says nothing: " << replay;
        const std::string back = stderrOf([&] { taken = source.setControl("MASTER_CLOCK", 200.0); });
        expect(taken && source._masterClockPinnedHz == 200.0e6 && back.find("200 MHz") != std::string::npos) << "the other entry is taken and named: " << back;
        const std::string automatic = stderrOf([&] { taken = source.setControl("MASTER_CLOCK", 0.0); });
        expect(taken && automatic.find("automatic master clock") != std::string::npos) << "the automatic entry leaves the clock to UHD at the next open";
        expect(capture::kwargsValue(source.openAddress().to_string(), "master_clock_rate").empty()) << "with no clock in the open address";

        // The start compares the clock it reads with the pin.
        using capture::uhd::Source;
        expect(Source::clockPinMissedLine(184.32e6, 184.32e6).empty()) << "a pin the open put in force says nothing";
        expect(Source::clockPinMissedLine(0.0, 200.0e6).empty() && Source::clockPinMissedLine(184.32e6, 0.0).empty()) << "nor does no pin, or no clock read";
        const std::string missed = Source::clockPinMissedLine(184.32e6, 200.0e6);
        expect(missed.find("runs a 200 MHz master clock, not the pinned 184.32 MHz") != std::string::npos && missed.find("second open in one process") != std::string::npos) << missed;

        // A product outside the table takes no clock.
        source._openTruth.model = "N210r4";
        expect(!source.setControl("MASTER_CLOCK", 100.0)) << "an N210 has no clock control";
        source._deviceUp.store(false);
    });

    cases("controls.uhd-a-list-takes-its-entries-alone", [] {
        capture::uhd::DeviceTruth x310;
        x310.model     = "X310";
        x310.mclkMinHz = 200.0e6;
        x310.mclkMaxHz = 200.0e6;
        x310.mclkHz    = 200.0e6;
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 100.0e6};
        const auto                  surface = capture::uhd::Source::describeControls(x310, ctx);
        std::optional<double>       bounded{1.0};
        const std::string           said = stderrOf([&] { bounded = capture::uhd::boundedControlValue(surface, "MASTER_CLOCK", 30.72, "case"); });
        expect(!bounded.has_value()) << "a B2xx's 30.72 MHz clock is not an entry of an X310's list";
        expect(said.find("MASTER_CLOCK 30.72 is not one of its entries (0, 184.32, 200)") != std::string::npos) << "and the refusal names the entries: " << said;
        expect(capture::uhd::boundedControlValue(surface, "MASTER_CLOCK", 184.32) == std::optional<double>{184.32}) << "an entry stands";
        expect(capture::uhd::boundedControlValue(surface, "MASTER_CLOCK", 184.3200000001) == std::optional<double>{184.32}) << "as the entry itself, within a millionth";
        expect(capture::uhd::boundedControlValue(surface, "MASTER_CLOCK", 250.0) == std::nullopt) << "a value past the last entry is refused, not clamped";

        // The same replay onto a running X310 source pins nothing.
        capture::uhd::Source source{};
        source._openTruth = x310;
        source._deviceUp.store(true);
        expect(!source.setControl("MASTER_CLOCK", 30.72)) << "the replay is refused";
        expect(!source._masterClockPinnedHz.has_value()) << "and pins nothing";
        source._deviceUp.store(false);
    });

    cases("controls.uhd-a-failed-open-clears-the-clock-pin", [] {
        using gr::lifecycle::State;
        expect(fatal(capture::uhd::test::uhdRefusesAddress())) << "UHD's parser refuses the address, so no device is asked for";

        /*| frame: a table pick on an X310 fails the open that carries it, as a clock the loaded
                image or a TwinRX refuses does. The failed start stands for that open. */
        capture::uhd::Source table{};
        table.device_parameter = capture::uhd::test::kUnparsedAddress;
        std::ignore            = table.changeStateTo(State::INITIALISED);
        table._openTruth.model     = "X310";
        table._openTruth.mclkMinHz = 200.0e6;
        table._openTruth.mclkMaxHz = 200.0e6;
        table._openTruth.mclkHz    = 200.0e6;
        table._deviceUp.store(true);
        std::ignore = stderrOf([&table] { expect(table.setControl("MASTER_CLOCK", 184.32)) << "the table pick is pinned"; });
        table._deviceUp.store(false);
        const auto tableStart = table.changeStateTo(State::RUNNING);
        expect(fatal(!tableStart.has_value())) << "the start fails";
        expect(tableStart.error().message.contains("the master clock pinned at 184.32 MHz is cleared")) << "and names the pin it clears: " << tableStart.error().message;
        expect(!table._masterClockPinnedHz.has_value()) << "which is gone";
        expect(!table._masterClockPinFromTable) << "and no longer marked as a table pick";

        /*| frame: a clock a live write put on a span radio, a B205mini at 30.72 MHz. The device
                ran it, so a failed open is not its doing and the pin stays. */
        capture::uhd::Source span{};
        span.device_parameter = capture::uhd::test::kUnparsedAddress;
        std::ignore           = span.changeStateTo(State::INITIALISED);
        span._openTruth.model     = "B205mini";
        span._openTruth.mclkMinHz = 220.0e3;
        span._openTruth.mclkMaxHz = 61.44e6;
        span._masterClockPinnedHz = 30.72e6;
        const auto spanStart      = span.changeStateTo(State::RUNNING);
        expect(fatal(!spanStart.has_value())) << "the start fails";
        expect(!spanStart.error().message.contains("is cleared")) << "and names no pin: " << spanStart.error().message;
        expect(span._masterClockPinnedHz == 30.72e6) << "the proven clock stays pinned";

        // A pin of the automatic entry carried no clock.
        capture::uhd::Source automatic{};
        automatic._masterClockPinnedHz     = 0.0;
        automatic._masterClockPinFromTable = true;
        std::string cleared;
        {
            std::lock_guard lock(automatic._ctrlMutex);
            cleared = automatic.clearClockPinAfterFailedOpenLocked();
        }
        expect(cleared.empty() && automatic._masterClockPinnedHz == 0.0) << "and stays";
    });

    cases("controls.uhd-a-settings-write-clears-the-pins", [] {
        capture::uhd::Source source{};
        source.device_parameter = std::string("type=x300,addr=192.168.40.2");
        const auto pinAll = [&source] {
            std::lock_guard lock(source._ctrlMutex);
            source._masterClockPinnedHz = 184.32e6;
            source._wireFormatPinned    = std::string("sc8");
            source._clockSourcePinned   = std::string("external");
            source._timeSourcePinned    = std::string("external");
        };
        const auto apply = [&source](gr::property_map settings) {
            std::ignore = source.settings().setStaged(settings);
            std::ignore = source.settings().applyStagedParameters();
        };

        pinAll();
        apply({{"master_clock_rate", 200.0e6}});
        expect(!source._masterClockPinnedHz.has_value()) << "a write of master_clock_rate clears the clock pin";
        expect(source._wireFormatPinned.has_value() && source._clockSourcePinned.has_value() && source._timeSourcePinned.has_value()) << "and no other";
        expect(source.openAddress().get("master_clock_rate", "") == std::string("200000000")) << "the setting reaches the next open";
        apply({{"wire_format", std::string("sc8")}});
        expect(!source._wireFormatPinned.has_value() && source._clockSourcePinned.has_value()) << "a write of wire_format clears the format pin alone";
        apply({{"clock_source", std::string("internal")}});
        expect(!source._clockSourcePinned.has_value() && source._timeSourcePinned.has_value()) << "one of clock_source the reference pin";
        apply({{"time_source", std::string("internal")}});
        expect(!source._timeSourcePinned.has_value()) << "and one of time_source the timing pin";

        /*| frame: the hook called directly, so the comparison with the device string held is
                what decides. A settings pass through the core never carries an unchanged value.
        */
        pinAll();
        gr::property_map forward;
        const gr::property_map held{{"device_parameter", std::string("type=x300,addr=192.168.40.2")}};
        gr::property_map       carried = held;
        source.settingsChanged(held, carried, forward);
        expect(source._masterClockPinnedHz == 184.32e6 && source._wireFormatPinned.has_value() && source._clockSourcePinned.has_value() && source._timeSourcePinned.has_value()) << "a pass whose device string was already held keeps the pins";
        const gr::property_map before{{"device_parameter", std::string("type=b200")}};
        source.settingsChanged(before, carried, forward);
        expect(!source._masterClockPinnedHz.has_value() && !source._wireFormatPinned.has_value() && !source._clockSourcePinned.has_value() && !source._timeSourcePinned.has_value()) << "one that moved it clears every pin";

        pinAll();
        apply({{"device_parameter", std::string("type=n3xx,addr=192.168.10.2")}});
        expect(!source._masterClockPinnedHz.has_value() && !source._wireFormatPinned.has_value() && !source._clockSourcePinned.has_value() && !source._timeSourcePinned.has_value()) << "another device clears every pin";
    });

    cases("controls.uhd-the-receive-depth-is-stated-in-time", [] {
        using capture::uhd::recvDepthFor;
        expect(capture::uhd::uhdDefaultRecvFrames("B-Series Device") == 16UZ) << "a B2xx over USB";
        expect(capture::uhd::uhdDefaultRecvFrames("USRP2 / N-Series Device") == 32UZ) << "an N2xx over UDP";
        expect(capture::uhd::uhdDefaultRecvFrames("X-Series Device") == 0UZ) << "a kind whose default is not stated";

        // UHD's default on a B2xx at two rates: the samples stay, the time scales with the rate.
        const auto fast = recvDepthFor(0.0, 0.0, "B-Series Device", 2040UZ, 61.44e6, "sc16");
        const auto slow = recvDepthFor(0.0, 0.0, "B-Series Device", 2040UZ, 2.0e6, "sc16");
        expect(fast.frames == 16UZ && fast.framesDefault && fast.bufferBytes == 0.0);
        expect(std::abs(fast.framesSeconds() - 16.0 * 2040.0 / 61.44e6) < 1e-12) << "16 frames of 2040 samples over the rate";
        expect(std::abs(slow.framesSeconds() / fast.framesSeconds() - 61.44e6 / 2.0e6) < 1e-9) << "the time goes as the inverse of the rate";
        expect(fast.describe() == "16 frames of 2040 samples, 0.531 ms at 61.44 MS/s, UHD's default") << fast.describe();

        // The caller's frames replace the default.
        const auto set = recvDepthFor(256.0, 0.0, "B-Series Device", 2040UZ, 61.44e6, "sc16");
        expect(set.frames == 256UZ && !set.framesDefault && set.describe().ends_with(", as set")) << set.describe();

        // An N2xx carries the socket buffer UHD asks for, in time in the wire format in force.
        const auto n210 = recvDepthFor(0.0, 0.0, "USRP2 / N-Series Device", 363UZ, 25.0e6, "sc16");
        expect(n210.frames == 32UZ && n210.bufferDefault);
        expect(n210.bufferBytes == capture::uhd::uhdDefaultRecvBuffBytes("USRP2 / N-Series Device") && n210.bufferBytes > 0.0);
        expect(std::abs(n210.bufferSeconds() - n210.bufferBytes / (25.0e6 * 4.0)) < 1e-12) << "four bytes a sample in sc16";
        expect(std::abs(recvDepthFor(0.0, 0.0, "USRP2 / N-Series Device", 363UZ, 25.0e6, "sc8").bufferSeconds() / n210.bufferSeconds() - 2.0) < 1e-12) << "twice the time in sc8";
        expect(recvDepthFor(0.0, 4.0e6, "USRP2 / N-Series Device", 363UZ, 25.0e6, "sc16").bufferBytes == 4.0e6) << "a caller's buffer stands";
        expect(n210.describe() == "32 frames of 363 samples, 0.465 ms at 25 MS/s, UHD's default; socket buffer 50000000 bytes, 500 ms at 25 MS/s, UHD's request" || n210.bufferBytes != 50e6) << n210.describe();
        expect(recvDepthFor(0.0, 0.0, "USRP2 / N-Series Device", 363UZ, 12.5e6, "sc16").describe().ends_with("socket buffer 50000000 bytes, 1 s at 12.5 MS/s, UHD's request") || n210.bufferBytes != 50e6) << "a second or more is stated in seconds";

        // The reading leaves out a figure it does not know.
        const auto x3xx = recvDepthFor(0.0, 0.0, "X-Series Device", 1996UZ, 200.0e6, "sc16");
        expect(x3xx.frames == 0UZ && x3xx.framesSeconds() == 0.0) << "no default frame count for the kind";
        expect(x3xx.describe() == "frames of 1996 samples, as many as UHD chose") << x3xx.describe();
        expect(recvDepthFor(0.0, 0.0, "B-Series Device", 0UZ, 61.44e6, "sc16").describe().empty()) << "no streamer, no reading";
        expect(recvDepthFor(0.0, 0.0, "B-Series Device", 2040UZ, 0.0, "sc16").framesSeconds() == 0.0) << "no rate, no time";

        // The kind comes from the root of the property tree.
        TreeStandIn named;
        named.device->tree->create<std::string>("/name").set("B-Series Device");
        capture::uhd::DeviceTruth t;
        capture::uhd::readTruthInto(named, t, capture::uhd::Direction::Receive);
        expect(t.deviceKind == "B-Series Device");
        TreeStandIn               unnamed;
        capture::uhd::DeviceTruth none;
        capture::uhd::readTruthInto(unnamed, none, capture::uhd::Direction::Receive);
        expect(none.deviceKind.empty()) << "a tree that names no kind";
    });

    cases("controls.uhd-a-drain-ends-by-duration-at-any-rate", [] {
        using Source = capture::uhd::Source;
        /*| frame: a stopped stream holding held samples, received 16384 at a time, and a clock
                that moves by the time each receive's samples take at the rate. A stream
                that keeps delivering never empties. No device is opened. */
        struct Transport {
            double      rateHz     = 0.0;
            std::size_t held       = 0UZ;
            bool        endOfBurst = false;
            bool        endless    = false;
            std::chrono::steady_clock::time_point clock{};
            std::size_t overflowAt = 0UZ; // a report with no samples once this many are taken
            std::size_t taken      = 0UZ;

            std::size_t recv(::uhd::rx_metadata_t& md) {
                md.reset();
                if (overflowAt != 0UZ && taken >= overflowAt) {
                    overflowAt    = 0UZ;
                    md.error_code = ::uhd::rx_metadata_t::ERROR_CODE_OVERFLOW;
                    return 0UZ;
                }
                const std::size_t n = endless ? 16384UZ : std::min<std::size_t>(16384UZ, held - taken);
                if (n == 0UZ) {
                    clock += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(Source::kDrainRecvS));
                    md.error_code = ::uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
                    return 0UZ;
                }
                taken += n;
                clock += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(static_cast<double>(n) / rateHz));
                md.end_of_burst = endOfBurst && !endless && taken == held;
                return n;
            }
        };
        const auto make = [](double rateHz, std::size_t held, bool endOfBurst, bool endless, std::size_t overflowAt) {
            Transport t;
            t.rateHz     = rateHz;
            t.held       = held;
            t.endOfBurst = endOfBurst;
            t.endless    = endless;
            t.overflowAt = overflowAt;
            return t;
        };
        const auto drain = [](Transport& t) {
            return Source::drainStopped([&t](::uhd::rx_metadata_t& md) { return t.recv(md); }, [&t] { return t.clock; }, Source::kDrainLimitS);
        };

        // A count of 64 receives of this size stops here at every rate.
        constexpr std::size_t kCountBound = 64UZ * 16384UZ;

        // UHD's default depth on a B2xx at 61.44 MS/s, and a transport holding 31 ms at 2 GS/s.
        for (const auto& [rate, held] : {std::pair{61.44e6, 32640UZ}, std::pair{2.0e9, 62'000'000UZ}}) {
            Transport  marked = make(rate, held, true, false, 0UZ);
            const auto m = drain(marked);
            expect(m.end == Source::DrainEnd::EndOfBurst && m.samples == held) << std::format("at {:g} S/s the drain takes all {} samples and ends on the marker", rate, held);
            expect(std::abs(m.seconds - static_cast<double>(held) / rate) < 1e-6) << "in the time those samples take at the rate";
            Transport  unmarked = make(rate, held, false, false, 0UZ);
            const auto u = drain(unmarked);
            expect(u.end == Source::DrainEnd::Empty && u.samples == held) << "without a marker it ends at the first receive that finds nothing";
        }
        expect(62'000'000UZ > kCountBound) << "the fast transport holds more than a count of receives takes";

        // A stream that never ends is cut at the duration, which takes the rate times the bound.
        for (const double rate : {61.44e6, 2.0e9}) {
            Transport  endless = make(rate, 0UZ, false, true, 0UZ);
            const auto e      = drain(endless);
            const double want = rate * Source::kDrainLimitS;
            expect(e.end == Source::DrainEnd::Deadline && e.seconds >= Source::kDrainLimitS) << std::format("at {:g} S/s the bound ends it", rate);
            expect(static_cast<double>(e.samples) >= want && static_cast<double>(e.samples) <= want + 16384.0) << std::format("{} samples, the rate times {} s", e.samples, Source::kDrainLimitS);
        }

        // An overflow report with no samples does not end the drain.
        Transport  reported = make(61.44e6, 32640UZ, true, false, 16384UZ);
        const auto r = drain(reported);
        expect(r.end == Source::DrainEnd::EndOfBurst && r.samples == 32640UZ) << "the packets behind the report are taken";
    });

    cases("controls.uhd-thread-placement-is-the-callers", [] {
        const capture::uhd::Source src{};
        const capture::uhd::Sink   sink{};
        expect(src.thread_priority.value == 0.0 && src.cpu.value == -1.0) << "the source asks for nothing by default";
        expect(sink.thread_priority.value == 0.0 && sink.cpu.value == -1.0) << "and so does the sink";
        expect(!capture::threading::placeOwnThread(src.thread_priority.value, src.cpu.value).asked()) << "the defaults leave the threads as the system gives them";
        expect(!capture::threading::placeOwnThread(sink.thread_priority.value, sink.cpu.value).asked());
    });

    cases("controls.uhd-a-clock-span-takes-any-clock-in-it", [] {
        using capture::uhd::Direction;
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 100.0e6};
        const auto clockOf = [&ctx](const capture::uhd::DeviceTruth& t) -> std::optional<capture::ControlDesc> {
            for (const auto& c : capture::uhd::Source::describeControls(t, ctx)) {
                if (c.id == "MASTER_CLOCK") {
                    return c;
                }
            }
            return std::nullopt;
        };

        // A B2xx states its converter's clock range as one span, 220 kHz to 61.44 MHz.
        TreeStandIn span;
        span.clockRange = ::uhd::meta_range_t(220.0e3, 61.44e6);
        span.clockHz    = 16.0e6;
        capture::uhd::DeviceTruth spanTruth;
        capture::uhd::readTruthInto(span, spanTruth, Direction::Receive);
        expect(spanTruth.mclkContinuous) << "one sub-range with room in it is a continuous clock range";
        const auto spanClock = clockOf(spanTruth);
        expect(fatal(spanClock.has_value())) << "the programmable clock is a control";
        expect(spanClock->kind == 4 && spanClock->listValues.front() == 0.0) << "a list whose first entry is the automatic one";
        expect(spanClock->anyInRange) << "which takes any clock in the span beside its entries";
        expect(spanClock->min == 0.22 && spanClock->max == 61.44) << "min and max state the span in MHz";
        const std::vector<capture::ControlDesc> spanSurface{*spanClock};
        expect(capture::uhd::boundedControlValue(spanSurface, "MASTER_CLOCK", 45.0) == std::optional<double>{45.0}) << "a clock between two entries is written as it stands";
        expect(capture::uhd::boundedControlValue(spanSurface, "MASTER_CLOCK", 0.0) == std::optional<double>{0.0}) << "and the automatic entry stays selectable";
        expect(capture::uhd::boundedControlValue(spanSurface, "MASTER_CLOCK", 70.0) == std::optional<double>{61.44}) << "a clock past the span is bounded to its top";
        expect(capture::controlFromMap(capture::controlToMap(*spanClock)).anyInRange) << "the property map carries the acceptance";
        expect(capture::controlFromMap(capture::controlToMap(*spanClock)) == *spanClock) << "with every other field as it was";

        /*| frame: a span whose top is not an entry. The bound reaches the top of the span and
                not the last entry. */
        capture::uhd::DeviceTruth wide = spanTruth;
        wide.mclkMaxHz                 = 50.0e6;
        const auto wideClock           = clockOf(wide);
        expect(fatal(wideClock.has_value()));
        expect(wideClock->listValues.back() == 40.0 && wideClock->max == 50.0) << "the span reaches past the last entry";
        const std::vector<capture::ControlDesc> wideSurface{*wideClock};
        expect(capture::uhd::boundedControlValue(wideSurface, "MASTER_CLOCK", 45.0) == std::optional<double>{45.0}) << "and a clock above that entry is written as it stands";

        // A radio stating separate clocks takes those alone.
        TreeStandIn points;
        points.clockRange = ::uhd::meta_range_t();
        points.clockRange.push_back(::uhd::range_t(30.72e6));
        points.clockRange.push_back(::uhd::range_t(61.44e6));
        points.clockHz = 30.72e6;
        capture::uhd::DeviceTruth pointsTruth;
        capture::uhd::readTruthInto(points, pointsTruth, Direction::Receive);
        expect(!pointsTruth.mclkContinuous) << "separate clocks are not a continuous range";
        const auto pointsClock = clockOf(pointsTruth);
        expect(fatal(pointsClock.has_value())) << "two clocks with entries between them are still a control";
        expect(!pointsClock->anyInRange) << "which offers its entries alone";
        expect(pointsClock->min == pointsClock->listValues.front() && pointsClock->max == pointsClock->listValues.back()) << "min and max are the ends of the list";
        const std::vector<capture::ControlDesc> pointsSurface{*pointsClock};
        expect(!capture::uhd::boundedControlValue(pointsSurface, "MASTER_CLOCK", 70.0).has_value()) << "a clock that is not an entry is refused";
        expect(capture::uhd::boundedControlValue(pointsSurface, "MASTER_CLOCK", 61.44) == std::optional<double>{61.44}) << "and an entry stands";

        // A radio stating one clock states no control.
        TreeStandIn one;
        capture::uhd::DeviceTruth oneTruth;
        capture::uhd::readTruthInto(one, oneTruth, Direction::Receive);
        expect(!oneTruth.mclkContinuous) << "one point is not a continuous range";
        expect(!clockOf(oneTruth).has_value()) << "and a single clock is not a control";

        // A clock written by hand makes the span a fixed clock until the next open.
        expect(!capture::uhd::ladderTruthFor(spanTruth, 30.72e6).mclkContinuous) << "a pinned clock is not a span";
        expect(capture::uhd::ladderTruthFor(spanTruth, 0.0).mclkContinuous) << "and no clock in force changes nothing";
    });

    cases("controls.uhd-transport-sizing", [] {
        // Nothing asked for is the caller's own selector, unchanged.
        expect(capture::uhd::withDeviceArgs("addr=192.168.10.3", 0.0, capture::uhd::Direction::Receive, 0.0, 0.0, 0.0) == std::string("addr=192.168.10.3")) << "zero means UHD's own choice";

        const std::string sized = capture::uhd::withDeviceArgs("addr=192.168.10.3", 0.0, capture::uhd::Direction::Receive, 8000.0, 512.0, 2097152.0);
        expect(sized == std::string("addr=192.168.10.3,recv_frame_size=8000,num_recv_frames=512,recv_buff_size=2097152")) << "each size above zero becomes a device-address key";
        expect(capture::kwargsValue(sized, "addr") == std::string("192.168.10.3")) << "and the selector still selects";

        // A key the caller spelled is theirs: a device string a person typed says what they
        // meant, and a setting left at its default must not overwrite it.
        expect(capture::uhd::withDeviceArgs("addr=1.2.3.4,recv_buff_size=99", 0.0, capture::uhd::Direction::Receive, 0.0, 0.0, 2097152.0) == std::string("addr=1.2.3.4,recv_buff_size=99")) << "a spelled key stands";

        // The master clock rate takes the same route, UHD reading it when it opens.
        expect(capture::kwargsValue(capture::uhd::withDeviceArgs("serial=X", 30.72e6, capture::uhd::Direction::Receive, 0.0, 0.0, 0.0), "master_clock_rate") == std::string("30720000"));

        // An empty selector is still an address once a size is asked for.
        expect(capture::uhd::withDeviceArgs("", 0.0, capture::uhd::Direction::Receive, 1472.0, 0.0, 0.0) == std::string("recv_frame_size=1472"));
    });

    cases("controls.uhd-device-time-from-the-pps", [] {
        // Which sources carry an edge a whole second can be latched on.
        expect(capture::uhd::Source::timeSourceHasPps("external"));
        expect(capture::uhd::Source::timeSourceHasPps("_external_")) << "UHD's own spelling of a reference it drives";
        expect(capture::uhd::Source::timeSourceHasPps("gpsdo")) << "the oscillator's own pulse";
        expect(capture::uhd::Source::timeSourceHasPps("mimo")) << "and the radio upstream's";
        expect(!capture::uhd::Source::timeSourceHasPps("internal")) << "a free-running clock has no edge to wait for";
        expect(!capture::uhd::Source::timeSourceHasPps("none"));

        constexpr std::uint64_t kHostNs = 1'789'936'968'500'000'000UL; // half a second past 1789936968

        // With a fix, the second after the one the oscillator reported, so the device clock
        // reads UTC rather than the host's idea of it.
        expect(capture::uhd::Source::ppsWholeSecond(true, 1'789'936'968L, kHostNs) == 1'789'936'969L) << "the GPS second plus one is latched on the next edge";

        // Without a fix the host is all there is, and the same edge takes its next second.
        expect(capture::uhd::Source::ppsWholeSecond(false, 1'789'936'968L, kHostNs) == 1'789'936'969L) << "an unlocked oscillator falls back to the host clock";
        expect(capture::uhd::Source::ppsWholeSecond(true, 0L, kHostNs) == 1'789'936'969L) << "and so does a GPS time of zero";
        expect(capture::uhd::Source::ppsWholeSecond(true, -1L, kHostNs) == 1'789'936'969L);

        // A GPS second far from the host's is still the one that is latched, because the
        // oscillator is the disciplined clock and the host is not.
        expect(capture::uhd::Source::ppsWholeSecond(true, 1'000'000'000L, kHostNs) == 1'000'000'001L) << "the oscillator wins where it has a fix";
    });

    cases("controls.uhd-reads-a-disciplined-second", [] {
        /*| frame: a GPSDO states its second as text. The accessor beside it is 32 bits
                wide, so a second past 2038-01-19 read through that accessor comes back
                negative or throws, and the block falls back to the host clock while the
                oscillator holds a disciplined second.
        */
        expect(capture::uhd::Source::wholeSecondsFrom("1789942087").value() == 1'789'942'087L) << "the second the oscillator reports today";
        expect(capture::uhd::Source::wholeSecondsFrom("2147483648").value() == 2'147'483'648L) << "one past the width of a 32-bit second, which 2038 reaches";
        expect(capture::uhd::Source::wholeSecondsFrom("4102444800").value() == 4'102'444'800L) << "and one in 2100";
        expect(capture::uhd::Source::wholeSecondsFrom(" 1789942087").value() == 1'789'942'087L) << "space around the value is not part of it";
        expect(capture::uhd::Source::wholeSecondsFrom("1789942087 s").value() == 1'789'942'087L) << "and neither is a unit after it";

        expect(!capture::uhd::Source::wholeSecondsFrom("").has_value()) << "an empty reading is no second";
        expect(!capture::uhd::Source::wholeSecondsFrom("unlocked").has_value()) << "and neither is a word";

        // The whole second latched on the next edge is taken from that reading.
        constexpr std::uint64_t kHostNs = 1'789'936'968'500'000'000UL;
        expect(capture::uhd::Source::ppsWholeSecond(true, capture::uhd::Source::wholeSecondsFrom("2147483648").value(), kHostNs) == 2'147'483'649L) << "the oscillator's own second, whatever its width";
    });

    cases("controls.uhd-device-listing-names-one-unit-each", [] {
        /*| frame: a selector is key=value pairs joined by commas, so a value carrying either
                separator parses back as something else: the reader finds a key where the
                value continues, and the selector then names another radio or none.
        */
        expect(capture::uhd::Source::deviceStringSafe("30D3AB9")) << "an ordinary serial";
        expect(capture::uhd::Source::deviceStringSafe("192.168.10.3")) << "and an address";
        expect(!capture::uhd::Source::deviceStringSafe("A,B")) << "a serial carrying the separator between two pairs is not spellable";
        expect(!capture::uhd::Source::deviceStringSafe("A=B")) << "and neither is one carrying the separator inside a pair";

        // A reader of the string makes this of such a value, which is why it is not offered.
        expect(capture::kwargsValue("driver=uhd,serial=A,B,type=b200", "type") == std::string("b200")) << "the reader takes the pairs it can";
        expect(capture::kwargsValue("driver=uhd,serial=A,B,type=b200", "serial") == std::string("A")) << "and a serial split by a comma names half a unit";
    });

    cases("controls.uhd-a-listing-asks-for-dpdk-where-asked", [] {
        const char* variable = capture::uhd::kUseDpdkVariable;
        ::unsetenv(variable);
        expect(capture::uhd::listingHint().size() == 0UZ) << "with nothing asked the hint is empty";
        const auto asked = capture::uhd::listingHint(true);
        expect(asked.size() == 1UZ && asked.has_key("use_dpdk") && asked["use_dpdk"] == "1") << "a caller that asks for DPDK adds use_dpdk=1 and nothing else";

        ::setenv(variable, "1", 1);
        const auto fromEnvironment = capture::uhd::listingHint();
        expect(fromEnvironment.size() == 1UZ && fromEnvironment["use_dpdk"] == "1") << "the switch on in the environment adds the same key";
        ::setenv(variable, "0", 1);
        expect(capture::uhd::listingHint().size() == 0UZ) << "and the switch set to 0 adds nothing";
        ::unsetenv(variable);
    });

    cases("controls.uhd-describes-the-unit", [] {
        capture::uhd::DeviceTruth n210;
        n210.product     = "N210r4";
        n210.model       = "N210r4";
        n210.serial      = "E2R18S9UP";
        n210.fwVersion   = "12.4";
        n210.fpgaVersion = "11.1";
        n210.linkBytesPerS = 125.0e6;
        expect(n210.describe() == std::string("N210r4 E2R18S9UP, firmware 12.4, FPGA 11.1, link 125 MB/s as UHD states it")) << "the model, the serial, both versions and UHD's link figure";

        // A model that states no versions says what it can.
        capture::uhd::DeviceTruth b205;
        b205.product = "B205mini";
        b205.model   = "B205mini";
        b205.serial  = "30D3AB9";
        b205.linkBytesPerS = 500.0e6;
        expect(b205.describe() == std::string("B205mini 30D3AB9, link 500 MB/s as UHD states it")) << "the model, the serial and the link alone";

        // The motherboard name where there is one, and the identifier otherwise.
        capture::uhd::DeviceTruth bare;
        bare.product = "USRP";
        expect(bare.describe() == std::string("USRP, link rate unknown")) << "the identifier stands in for a missing name, and a link UHD states nothing for is unknown";
        expect(capture::uhd::DeviceTruth{}.describe().empty()) << "a record of nothing describes nothing";
    });

    cases("controls.uhd-skips-an-unchanged-write", [] {
        // A value never written reaches the device; the same value again does not.
        std::optional<double> written;
        expect(capture::uhd::writeNeeded(written, 2.0e6)) << "the first write always goes";
        written = 2.0e6;
        expect(!capture::uhd::writeNeeded(written, 2.0e6)) << "and the replay of it does not";
        expect(capture::uhd::writeNeeded(written, 4.0e6)) << "a different value does";

        // The same rule for the modes and the names the AGC and the antenna appliers cache.
        std::optional<bool> agc;
        expect(capture::uhd::writeNeeded(agc, false)) << "a mode never written is written, even when it is the mode a device comes up in";
        agc = false;
        expect(!capture::uhd::writeNeeded(agc, false));
        expect(capture::uhd::writeNeeded(agc, true));

        std::optional<std::string> antenna;
        expect(capture::uhd::writeNeeded(antenna, std::string("RX2")));
        antenna = std::string("RX2");
        expect(!capture::uhd::writeNeeded(antenna, std::string("RX2")));
        expect(capture::uhd::writeNeeded(antenna, std::string("TX/RX")));
    });

    cases("controls.uhd-reference-lock-wait-is-bounded", [] {
        // A reference that is there costs one poll.
        const auto [lockedAtOnce, quickMs] = capture::uhd::waitForRefLock([] { return true; }, 1500.0, 5.0);
        expect(lockedAtOnce) << "a reference already locked is not waited for";
        expect(quickMs < 100.0) << "and costs nothing";

        // The wait for a reference that locks after a few polls ends at the poll that sees it.
        int        polls = 0;
        const auto [lockedLate, lateMs] = capture::uhd::waitForRefLock([&polls] { return ++polls >= 4; }, 1500.0, 5.0);
        expect(lockedLate) << "a reference that comes up is caught";
        expect(polls == 4) << "on the poll it came up on";
        expect(lateMs < 500.0) << "without waiting out the limit";

        // A reference that never comes up costs the limit and no more, and the answer says
        // it did not lock, on which the caller starts the stream anyway.
        const auto [neverLocked, waitedMs] = capture::uhd::waitForRefLock([] { return false; }, 120.0, 5.0);
        expect(!neverLocked) << "a reference that is not there never locks";
        expect(waitedMs >= 120.0) << "the whole limit is spent";
        expect(waitedMs < 600.0) << "and not much beyond it";
    });

    cases("controls.uhd-tag-time-comes-from-the-device", [] {
        constexpr std::uint64_t kWallNs = 1'700'000'000'000'000'000UL;
        constexpr double        kRate   = 2.0e6;

        // With no device timestamp the wall clock is read at the drain and the block's own
        // duration comes off it: 16384 samples at 2 MS/s is 8.192 ms.
        const std::uint64_t host = capture::uhd::Source::blockTagNs(std::nullopt, kWallNs, 16384UZ, kRate);
        expect(host == kWallNs - 8'192'000UL) << "the block's duration comes off the reading";

        // A device timestamp names the first sample outright, so nothing comes off it.
        expect(capture::uhd::Source::blockTagNs(std::optional<std::int64_t>{1'789'903'354'000'835'419L}, kWallNs, 16384UZ, kRate) == 1'789'903'354'000'835'419UL) << "the device's own timestamp stands";

        // Two blocks a known distance apart in the stream are that distance apart in time.
        const std::uint64_t first  = capture::uhd::Source::blockTagNs(std::optional<std::int64_t>{1'000'000'000'000L}, kWallNs, 16384UZ, kRate);
        const std::uint64_t second = capture::uhd::Source::blockTagNs(std::optional<std::int64_t>{1'000'000'000'000L + 8'192'000L}, kWallNs, 16384UZ, kRate);
        expect(second - first == 8'192'000UL) << "16384 samples at 2 MS/s is 8.192 ms of device time";

        // A device whose clock was never set answers zero or below, and there the wall
        // clock is the only honest reading.
        expect(capture::uhd::Source::blockTagNs(std::optional<std::int64_t>{0L}, kWallNs, 16384UZ, kRate) == host) << "an unset device clock falls back";
        expect(capture::uhd::Source::blockTagNs(std::optional<std::int64_t>{-1L}, kWallNs, 16384UZ, kRate) == host) << "and so does one before its epoch";
    });

    cases("controls.uhd-a-gap-marker-names-its-cause", [] {
        using capture::uhd::Source;
        // Three things make a hole in this stream and a reader has to tell them apart: a
        // buffer the radio ran over, a packet the link lost, and a rate change this block
        // emptied the transport across. The marker carries the name of the one that made it.
        expect(std::string_view(Source::gapCauseName(Source::kGapLostPacket)) == std::string_view("lost packet")) << "a packet the link lost is named as one";
        expect(std::string_view(Source::gapCauseName(Source::kGapRateChange)) == std::string_view("rate change")) << "and a rate change as one";
        expect(std::string_view(Source::gapCauseName(Source::kGapBufferOverrun)) == std::string_view("overflow")) << "and the radio's own overrun as one";

        const std::string_view names[] = {Source::gapCauseName(Source::kGapBufferOverrun), Source::gapCauseName(Source::kGapLostPacket), Source::gapCauseName(Source::kGapRateChange)};
        expect(names[0] != names[1] && names[1] != names[2] && names[0] != names[2]) << "the three names are three, so a reader can tell them apart";
        expect(std::string_view(Source::gapCauseName(99)) == names[0]) << "a value naming none of the three reads as the radio's own overrun";
    });

    cases("controls.uhd-a-retune-forwards-nothing-ahead-of-its-samples", [] {
        /*| frame: the block reads as streaming and holds no device, so the drain reaches the
                retune path and no library call is made.
        */
        capture::uhd::Source src{};
        src._deviceUp.store(true, std::memory_order_release);
        gr::property_map changed{{"frequency", std::vector<double>{433.0e6}}};
        gr::property_map forward{{"frequency", std::vector<double>{433.0e6}}};
        src.settingsChanged({}, changed, forward);
        expect(!forward.contains("frequency")) << "the drain forwards no frequency ahead of the first sample the tune reaches";
        src._deviceUp.store(false, std::memory_order_release);
    });

    cases("controls.uhd-a-retune-tag-lands-on-the-first-sample-it-reached", [] {
        using capture::uhd::Source;
        using Spec           = ::uhd::time_spec_t;
        constexpr double hz  = 2.0e6;
        const Spec       t0  = Spec(10, 0.5);
        const Spec       mid = t0 + Spec(1000.4 / hz);
        expect(Source::tuneTagOffset(mid, t0, hz, 2040UZ) == std::optional<std::size_t>(1001UZ)) << "the first sample stamped at or after the reading, never one before it";
        expect(Source::tuneTagOffset(t0 + Spec(1000.0 / hz), t0, hz, 2040UZ) == std::optional<std::size_t>(1000UZ)) << "a sample stamped at the reading carries it";
        expect(Source::tuneTagOffset(t0 - Spec(0.01), t0, hz, 2040UZ) == std::optional<std::size_t>(0UZ)) << "a block taken after the reading carries it on its first sample";
        expect(!Source::tuneTagOffset(t0 + Spec(2040.0 / hz), t0, hz, 2040UZ).has_value()) << "a block taken wholly before the reading carries nothing";
        expect(Source::tuneTagOffset(std::nullopt, t0, hz, 2040UZ) == std::optional<std::size_t>(0UZ)) << "a tune with no reading tags the next block";
        expect(Source::tuneTagOffset(mid, std::nullopt, hz, 2040UZ) == std::optional<std::size_t>(0UZ)) << "and so does a block with no stamp";
        expect(Source::tuneTagOffset(t0 + Spec(Source::kTuneTagLimitS + 1.0), t0, hz, 2040UZ) == std::optional<std::size_t>(0UZ)) << "a reading past the limit belongs to a clock that moved";
    });

    cases("controls.uhd-an-overflow-during-a-retune-is-named-for-it", [] {
        using capture::uhd::Source;
        expect(Source::overflowCause(false, true) == Source::kGapRetune) << "an overflow while a tune is open is named for the tune";
        expect(std::string_view(Source::gapCauseName(Source::kGapRetune)) == std::string_view("retune")) << "under the word retune";
        expect(Source::overflowCause(false, false) == Source::kGapBufferOverrun) << "without one it is the radio's own overrun";
        expect(Source::overflowCause(true, true) == Source::kGapLostPacket) << "and a packet the link lost stays a lost packet";
    });

    cases("controls.uhd-a-shed-no-counter-explains-is-marked", [] {
        using capture::uhd::Source;
        expect(Source::gapMarkerFor(false, Source::kGapBufferOverrun, 1068UL, false) == std::optional<int>(Source::kGapMissingSamples)) << "a shed no counter recorded is marked";
        expect(std::string_view(Source::gapCauseName(Source::kGapMissingSamples)) == std::string_view("missing samples")) << "under the words missing samples";
        expect(Source::gapMarkerFor(false, Source::kGapBufferOverrun, 1068UL, true) == std::optional<int>(Source::kGapRetune)) << "and under retune where a tune is open";
        expect(Source::gapMarkerFor(true, Source::kGapLostPacket, 1068UL, false) == std::optional<int>(Source::kGapLostPacket)) << "a gap a counter recorded names its counted cause";
        expect(Source::gapMarkerFor(true, Source::kGapRateChange, 0UL, false) == std::optional<int>(Source::kGapRateChange)) << "whether or not the stamps measure a shed";
        expect(!Source::gapMarkerFor(false, Source::kGapBufferOverrun, 0UL, true).has_value()) << "a block that follows the last without a gap carries no marker";
    });

    cases("controls.uhd-stamps-off-the-sample-grid-count-only-lost-samples", [] {
        using Block = StampedStream::Block;
        using Spec  = ::uhd::time_spec_t;
        /*| frame: the stamps a B205mini handed over across a live change from 1 MS/s to 8 MS/s
                with its master clock at 32 MHz. At 1 MS/s the first sample of each block sat
                14 of 32 ticks off that rate's grid, and at 8 MS/s 2 of 4 ticks, half a sample.
                Each block holds 16384 samples and follows the one before it with none lost.
        */
        constexpr double    kClock = 32.0e6;
        constexpr long long kSlow  = 176'993'358LL;
        constexpr long long kFast  = 226'929'306LL;
        constexpr long long kStep  = 16'384LL * 4;
        const auto          upTo   = [](long long fastBlocks) {
            std::vector<Block> blocks;
            for (long long k = 0; k < 4; ++k) {
                blocks.push_back({1.0e6, kSlow + k * 16'384LL * 32, 16'384UZ});
            }
            for (long long k = 0; k < fastBlocks; ++k) {
                blocks.push_back({8.0e6, kFast + k * kStep, 16'384UZ});
            }
            return blocks;
        };
        const auto ticksAt = [](long long masterTicks) { return Spec::from_ticks(masterTicks, kClock).to_ticks(8.0e6); };

        // UHD rounds each stamp to the 8 MS/s grid on its own, and the half goes both ways:
        // blocks 19 and 20 of the run, one straight after the other, round 16385 samples apart.
        expect(ticksAt(kFast + 20 * kStep) - ticksAt(kFast + 19 * kStep) == 16'385LL) << "the fixture's stamps round apart";
        const auto clean = runStamped(kClock, 1.0e6, upTo(40));
        expect(clean.shed == 0UL) << "forty blocks with no sample between them shed none, not" << std::to_string(clean.shed);
        expect(clean.missing == 0UZ) << "and carry no missing-samples marker";
        expect(clean.rateChanges == 1UZ) << "the one marker names the rate change";

        // One sample lost after block 16, whose stamp UHD rounds up, before a block it rounds
        // down; and one lost after block 19, rounded down, before a block rounded up.
        const long long upThenDown = kFast + 16 * kStep + (16'384LL + 1) * 4;
        const long long downThenUp = kFast + 19 * kStep + (16'384LL + 1) * 4;
        expect(ticksAt(upThenDown) - ticksAt(kFast + 16 * kStep) == 16'384LL) << "rounded one by one, the first lost sample reads as none";
        expect(ticksAt(downThenUp) - ticksAt(kFast + 19 * kStep) == 16'386LL) << "and the second as two";
        auto early = upTo(17);
        early.push_back({8.0e6, upThenDown, 16'384UZ});
        const auto first = runStamped(kClock, 1.0e6, std::move(early));
        expect(first.shed == 1UL) << "a sample lost between a stamp rounded up and one rounded down counts one, not" << std::to_string(first.shed);
        expect(first.missing == 1UZ) << "and carries one missing-samples marker";
        auto late = upTo(20);
        late.push_back({8.0e6, downThenUp, 16'384UZ});
        const auto second = runStamped(kClock, 1.0e6, std::move(late));
        expect(second.shed == 1UL) << "a sample lost between a stamp rounded down and one rounded up counts one, not" << std::to_string(second.shed);
        expect(second.missing == 1UZ) << "and carries one missing-samples marker";
    });

    cases("controls.uhd-a-receive-asks-for-whole-frames", [] {
        using capture::uhd::Source;
        constexpr std::size_t kB205 = 2040UZ; // the frames a B205mini and an N210 carry
        constexpr std::size_t kN210 = 363UZ;
        constexpr std::size_t kWide = 1UZ << 22; // an edge far longer than any window here
        const auto capOf = [](double rateHz, std::size_t frame) { return static_cast<std::size_t>(rateHz * Source::kRecvWindowS) / frame * frame; };

        // The window binds where the edge has more room than it.
        expect(Source::recvRequest(kWide, kWide, kB205, 1.0e6) == capOf(1.0e6, kB205)) << "a roomy edge takes the window's whole frames at 1 MS/s";
        expect(Source::recvRequest(kWide, kWide, kN210, 1.0e6) == capOf(1.0e6, kN210)) << "and at an N210's frame";
        expect(capOf(1.0e6, kB205) <= static_cast<std::size_t>(1.0e6 * Source::kRecvWindowS)) << "the window rounds down, never up";

        // A share of the edge binds where the window is longer, so one receive lands while the
        // reader takes the others.
        constexpr std::size_t kEdge  = 65'536UZ;
        constexpr std::size_t kShare = kEdge / Source::kRecvEdgeShares;
        expect(Source::kRecvEdgeShares == 4UZ) << "a receive takes at most a quarter of the edge";
        expect(capOf(8.0e6, kB205) > kShare) << "20 ms at 8 MS/s is longer than a quarter of a 64 Ki edge";
        expect(Source::recvRequest(kEdge, kEdge, kB205, 8.0e6) == kShare / kB205 * kB205) << "so an empty edge asks for a quarter of itself in whole frames";
        expect(Source::recvRequest(kEdge, kEdge, kB205, 30.0e6) == Source::recvRequest(kEdge, kEdge, kB205, 8.0e6)) << "at any rate whose window is longer";

        // The room binds where it holds less than both.
        constexpr std::size_t kRoom = 10UZ * kB205 + kB205 - 1UZ;
        expect(Source::recvRequest(kRoom, kWide, kB205, 30.0e6) == 10UZ * kB205) << "the room's whole frames are the request, a part frame adding nothing";

        // Below one frame of room nothing is asked for, and the loop waits.
        expect(Source::recvRequest(kB205 - 1UZ, kEdge, kB205, 8.0e6) == 0UZ) << "room for less than one frame asks for nothing";
        expect(Source::recvRequest(0UZ, kEdge, kB205, 8.0e6) == 0UZ) << "and a full edge asks for nothing";
        expect(Source::recvRequest(kB205, kEdge, kB205, 8.0e6) == kB205) << "room for exactly one frame asks for that frame";

        // A window or a share of the edge shorter than one frame still asks for one.
        constexpr double kSlow = 50.0e3;
        expect(kSlow * Source::kRecvWindowS < static_cast<double>(kB205)) << "20 ms at 50 kS/s is shorter than a frame";
        expect(Source::recvRequest(kEdge, kEdge, kB205, kSlow) == kB205) << "so the request is one frame";
        expect(Source::recvRequest(kEdge, kEdge, kB205, 0.0) == kB205) << "and so it is for a rate of zero";
        expect(Source::recvRequest(kEdge, kEdge, kB205, std::numeric_limits<double>::quiet_NaN()) == kB205) << "or a rate that is not a number";
        expect(Source::recvRequest(4UZ * kB205, 4UZ * kB205 + 1UZ, kB205, 8.0e6) == kB205) << "and for an edge whose quarter holds one frame";
        expect(Source::recvRequest(kB205, kB205, kB205, 8.0e6) == kB205) << "or less than one";
        expect(Source::recvRequest(kEdge, kEdge, kB205, std::numeric_limits<double>::infinity()) == Source::recvRequest(kEdge, kEdge, kB205, 8.0e6)) << "an unbounded rate is bounded by the edge";

        // A streamer that states no frame asks for nothing.
        expect(Source::recvRequest(kEdge, kEdge, 0UZ, 8.0e6) == 0UZ) << "a frame of zero asks for nothing";

        // Every answer is whole frames within the room, and within both caps or one frame.
        for (const std::size_t frame : {kN210, kB205, 4088UZ}) {
            for (const std::size_t edge : {4'096UZ, kEdge, kWide}) {
                for (const double rate : {0.2e6, 1.0e6, 8.0e6, 30.72e6, 61.44e6}) {
                    for (std::size_t room = 0UZ; room <= edge; room += edge / 61UZ + 1UZ) {
                        const std::size_t n = Source::recvRequest(room, edge, frame, rate);
                        expect(n % frame == 0UZ && n <= room && n <= std::max(std::min(capOf(rate, frame), edge / Source::kRecvEdgeShares), frame)) << "frame" << frame << "edge" << edge << "rate" << rate << "room" << room;
                        expect((n == 0UZ) == (room < frame)) << "nothing is asked for only below one frame of room";
                    }
                }
            }
        }
    });

    cases("controls.uhd-an-unconnected-output-ends-the-block", [] {
        // Pins this source's answer to an unconnected output and its count of refused markers.
        capture::uhd::Source src{};
        expect(!src.out.isConnected()) << "a block standing on its own reaches nothing";
        expect(capture::stopWhenOutputIsUnconnected(src)) << "so the work preamble answers that this call cannot go on";
        expect(src.state() == gr::lifecycle::State::REQUESTED_STOP) << "and the block is asked to stop rather than spinning";
        expect(src.droppedTags() == 0UZ) << "and the block reports the markers its port refused, none so far";
    });

    cases("controls.uhd-a-failed-start-throws", [] {
        capture::uhd::Source blk{};
        /*| frame: the parser refuses a pair with two separators before any device is asked for,
                so the start fails with no device opened and no enumeration made.
        */
        expect(fatal(capture::uhd::test::uhdRefusesAddress())) << "UHD's parser refuses the address, so no device is asked for";
        blk.device_parameter = std::string("a=b=c");
        std::ignore        = blk.changeStateTo(gr::lifecycle::State::INITIALISED);
        const auto started = blk.changeStateTo(gr::lifecycle::State::RUNNING);
        expect(!started.has_value()) << "the start throws, and the transition answers the error";
        expect(!started.has_value() && started.error().message.contains("throws: uhd make(a=b=c): ") && !started.error().message.contains(".hpp:")) << "carrying the reason and no source location";
        expect(blk.state() == gr::lifecycle::State::ERROR) << "and the block is left in ERROR";
        expect(blk._usrp == nullptr && blk._stream == nullptr && !blk.deviceUp()) << "holding no device and no streamer";
        expect(!blk.hardwareTeardown()) << "with nothing left for a teardown to release";
    });

    cases("controls.uhd-a-thread-that-does-not-start-ends-the-start", [] {
        /*| frame: the block holds a stand-in for an open device, a shared pointer whose deleter
                waits for the case's word and records the thread it ran on. The control thread
                starts, and the system refuses the consumer thread. The refusal first gives the
                control thread a piece of work that runs for 100 ms. No device is opened.
        */
        using Clock   = std::chrono::steady_clock;
        auto released = std::make_shared<std::atomic<bool>>(false);
        auto freed    = std::make_shared<std::atomic<bool>>(false);
        auto releaser = std::make_shared<std::thread::id>();
        auto device   = std::shared_ptr<int>(new int(0), [released, freed, releaser](int* p) {
            const auto deadline = Clock::now() + std::chrono::seconds(2);
            while (!freed->load() && Clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            *releaser = std::this_thread::get_id();
            released->store(true);
            delete p;
        });
        capture::uhd::Source blk{};
        blk._usrp = ::uhd::usrp::multi_usrp::sptr(device, reinterpret_cast<::uhd::usrp::multi_usrp*>(device.get()));
        device.reset();
        blk._deviceUp.store(true);

        std::atomic<bool> begun{false};
        std::atomic<bool> ended{false};
        onRefusal = [&blk, &begun, &ended] {
            blk._control.post(0, [&begun, &ended] {
                begun.store(true);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ended.store(true);
            });
            const auto deadline = Clock::now() + std::chrono::seconds(2);
            while (!begun.load() && Clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        };
        threadStartsBeforeRefusal.store(1);
        bool        typed = false;
        std::string reason;
        try {
            blk.startThreads();
        } catch (const capture::StartFailure& e) {
            typed  = true;
            reason = e.what();
        } catch (const std::exception& e) {
            reason = e.what();
        }
        const bool endedAtThrow    = ended.load();
        const bool releasedAtThrow = released->load();
        onRefusal                  = nullptr;
        threadStartsBeforeRefusal.store(-1);
        freed->store(true);

        expect(begun.load()) << "the control thread started and took the piece of work";
        expect(typed) << "a thread the system refuses ends the start with a StartFailure";
        expect(reason.starts_with("uhd could not start a thread: ")) << "carrying the reason, read " << reason;
        expect(endedAtThrow) << "the control thread's running piece ended before the throw";
        expect(blk._usrp == nullptr && blk._stream == nullptr && !blk.deviceUp()) << "the block holds no device and no streamer";
        expect(!releasedAtThrow) << "and the start returned before the release ended";
        capture::uhd::awaitReleases();
        expect(released->load() && *releaser != std::this_thread::get_id()) << "the release ran on the releasing thread";
        expect(!blk.hardwareTeardown()) << "with nothing left for a teardown to release";
    });

    cases("controls.uhd-a-refused-clock-range-keeps-the-clock", [] {
        using capture::uhd::masterClockFrom;
        // The range and the rate are two device calls and either raises on a model that does
        // not carry it. The rate names the clock a stated ladder was read at, so a refused
        // range may not take it with it.
        const std::optional<std::pair<double, double>> range   = std::pair{220.0e3, 61.44e6};
        const std::optional<std::pair<double, double>> noRangeRead;
        const std::optional<double>                    rate = 16.0e6;
        const std::optional<double>                    noRateRead;

        const capture::uhd::MasterClock both = masterClockFrom(range, rate);
        expect(both.minHz == 220.0e3 && both.maxHz == 61.44e6 && both.hz == 16.0e6) << "a device that states both states both";

        const capture::uhd::MasterClock noRange = masterClockFrom(noRangeRead, rate);
        expect(noRange.hz == 16.0e6) << "a device that states no range still states the clock it runs";
        expect(noRange.minHz == 0.0 && noRange.maxHz == 0.0) << "with no range behind it";

        const capture::uhd::MasterClock noRate = masterClockFrom(range, noRateRead);
        expect(noRate.minHz == 220.0e3 && noRate.maxHz == 61.44e6) << "and a device that refuses the rate still states the range";
        expect(noRate.hz == 0.0) << "with no rate behind it";
    });

    cases("controls.uhd-the-transport-keys-follow-the-direction", [] {
        using capture::uhd::Direction;
        using capture::uhd::withDeviceArgs;
        // One body writes the address keys for both halves of the radio, so the receive block
        // and the transmit block spell the master clock alike and their three transport keys
        // apart. The four are read once when the device opens and never again.
        const std::string rx = withDeviceArgs("type=b200", 16.0e6, Direction::Receive, 8000.0, 64.0, 2.0e6);
        expect(rx == std::string("type=b200,master_clock_rate=16000000,recv_frame_size=8000,num_recv_frames=64,recv_buff_size=2000000")) << "the receive half spells recv";

        const std::string tx = withDeviceArgs("type=b200", 16.0e6, Direction::Transmit, 8000.0, 64.0, 2.0e6);
        expect(tx == std::string("type=b200,master_clock_rate=16000000,send_frame_size=8000,num_send_frames=64,send_buff_size=2000000")) << "and the transmit half spells send";

        expect(withDeviceArgs("recv_frame_size=1472", 0.0, Direction::Receive, 8000.0, 0.0, 0.0) == std::string("recv_frame_size=1472")) << "a key the caller typed is left alone";
        expect(withDeviceArgs("", 0.0, Direction::Transmit, 0.0, 0.0, 0.0).empty()) << "and nothing above zero adds nothing";
    });

    cases("controls.uhd-the-overall-gain-writes-through-its-setting", [] {
        // A source whose device is up with no handle behind it: the settings hook's appliers
        // return where they hold no handle, so a drain moves the members and writes no device.
        capture::uhd::Source source{};
        source._openTruth.gainMinDb = 0.0;
        source._openTruth.gainMaxDb = 76.0;
        source._openTruth.gains.push_back({"PGA", 0.0, 76.0, 1.0, 10.0});
        source._deviceUp.store(true);
        expect(source.setElementGain("", 20.0)) << "the overall gain is staged";
        std::ignore = source.settings().applyStagedParameters();
        expect(source.rx_gains->size() == 1UZ && source.rx_gains->front() == 20.0) << "through rx_gains, so a replay writes the gain a caller chose";
        expect(source.setElementGain("", 200.0));
        std::ignore = source.settings().applyStagedParameters();
        expect(source.rx_gains->front() == 76.0) << "bounded by the range the device states";
        expect(!source.setElementGain("NOPE", 1.0)) << "an element the device does not name is refused";
        source._deviceUp.store(false);
        expect(!source.setElementGain("", 1.0)) << "and a block that is down stages nothing";
    });

    cases("controls.uhd-states-the-antennas-the-unit-names", [] {
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};
        const auto                  antenna = [](const std::vector<capture::ControlDesc>& cs) { return std::ranges::find_if(cs, [](const auto& c) { return c.id == "ANTENNA"; }); };

        // The unit's own list, read through the truth reader, with TX/RX in force.
        TruthStandIn              device;
        capture::uhd::DeviceTruth read;
        capture::uhd::readTruthInto(device, read, capture::uhd::Direction::Receive);
        const auto surface = capture::uhd::Source::describeControls(read, ctx);
        const auto stated  = antenna(surface);
        expect(stated != surface.end()) << "the receive surface states the connector";
        expect(stated != surface.end() && stated->kind == 3 && !stated->isGain) << "as an enum";
        expect(stated != surface.end() && stated->options == std::vector<std::string>{"TX/RX", "RX2"}) << "one option per name the unit gives, in its order";
        expect(stated != surface.end() && stated->min == 0.0 && stated->max == 1.0 && stated->defValue == 0.0) << "defaulting to the connector in force";
        expect(stated != surface.end() && stated->appliesAt == capture::AppliesAt::Runtime) << "and a running stream takes it";

        capture::uhd::DeviceTruth onRx2 = read;
        onRx2.antenna                   = "RX2";
        const auto second               = capture::uhd::Source::describeControls(onRx2, ctx);
        expect(antenna(second) != second.end() && antenna(second)->defValue == 1.0) << "a unit on RX2 defaults to RX2";

        capture::uhd::DeviceTruth one = read;
        one.antennas                  = {"RX2"};
        one.antenna                   = "RX2";
        const auto single             = capture::uhd::Source::describeControls(one, ctx);
        expect(antenna(single) == single.end()) << "a single connector states no control, as on the sink";

        TruthStandIn silent;
        silent.refuse = {"antennas"};
        capture::uhd::DeviceTruth none;
        capture::uhd::readTruthInto(silent, none, capture::uhd::Direction::Receive);
        const auto bare = capture::uhd::Source::describeControls(none, ctx);
        expect(none.antennas.empty() && antenna(bare) == bare.end()) << "a unit stating no connector states no control";
    });

    cases("controls.uhd-the-antenna-writes-through-its-setting", [] {
        // A source whose device is up with no handle behind it, as for the overall gain.
        capture::uhd::Source source{};
        source._openTruth.antennas = {"TX/RX", "RX2"};
        source._openTruth.antenna  = "TX/RX";
        source.rx_antennae         = std::vector<std::string>{"TX/RX"};
        source._deviceUp.store(true);
        expect(capture::uhd::Source::settingForControl(source._openTruth, "ANTENNA", 1.0).contains("rx_antennae")) << "the connector control names its setting";
        expect(capture::uhd::Source::settingForControl(source._openTruth, "BANDWIDTH", 1.0).empty()) << "and another control has none";
        expect(source.setControl("ANTENNA", 1.0)) << "the control is staged";
        std::ignore = source.settings().applyStagedParameters();
        expect(source.rx_antennae->size() == 1UZ && source.rx_antennae->front() == "RX2") << "and rx_antennae carries the name the index chose";
        expect(source.setControl("ANTENNA", 0.0)) << "index 0 is staged";
        std::ignore = source.settings().applyStagedParameters();
        expect(source.rx_antennae->front() == "TX/RX") << "and names the first connector";
        expect(!source.setControl("ANTENNA", 2.0)) << "an index past the unit's list is refused";
        std::ignore = source.settings().applyStagedParameters();
        expect(source.rx_antennae->front() == "TX/RX") << "and stages nothing, the last connector included";
        source._deviceUp.store(false);
        expect(!source.setControl("ANTENNA", 0.0)) << "a block that is down stages nothing";
    });

    cases("controls.uhd-refuses-an-antenna-the-unit-does-not-offer", [] {
        const std::vector<std::string> offered{"TX/RX", "RX2"};
        expect(!capture::uhd::Source::antennaRefusal(offered, "RX2").has_value()) << "a listed name passes";
        const auto refused = capture::uhd::Source::antennaRefusal(offered, "RX");
        expect(refused.has_value()) << "a name the unit does not list is refused";
        expect(refused.has_value() && refused->find("antenna RX ") != std::string::npos) << "the line names it";
        expect(refused.has_value() && refused->find("(TX/RX, RX2)") != std::string::npos) << "and the options the unit gives";
        expect(!capture::uhd::Source::antennaRefusal(std::vector<std::string>{"RX2"}, "RX2").has_value()) << "the name of a single connector goes through";
        expect(capture::uhd::Source::antennaRefusal(std::vector<std::string>{"RX2"}, "TX/RX").has_value()) << "and a one-name list still refuses another name";
        expect(!capture::uhd::Source::antennaRefusal({}, "RX2").has_value()) << "a unit stating no list refuses nothing, and the name goes to the device";
        expect(!capture::uhd::Source::antennaRefusal({}, "J1").has_value()) << "whatever the name";

        // The applier refuses from the list the open read, ahead of any handle, and leaves the
        // setting as it was given.
        capture::uhd::Source source{};
        source._openTruth.antennas = {"TX/RX", "RX2"};
        source._openTruth.antenna  = "TX/RX";
        source.rx_antennae         = std::vector<std::string>{"RX"};
        expect(!source.applyAntennaOf("RX")) << "the applier refuses a name outside the list";
        expect(source.rx_antennae->front() == "RX" && source._openTruth.antenna == "TX/RX") << "the setting keeps the refused name and the surface its connector";
        expect(source.applyAntennaOf("RX2")) << "a listed name is not refused";
        source._openTruth.antennas.clear();
        expect(source.applyAntennaOf("RX")) << "and a unit stating no list refuses nothing";
    });

    cases("controls.uhd-a-replay-of-the-clock-in-force-writes-nothing", [] {
        // A B205mini whose open address carried master_clock_rate: the clock is in force from
        // the first sample, and the source holds no device, so a write would reach no handle.
        capture::uhd::Source source{};
        source._clockInForceHz = 30.72e6;
        source._rateWrittenHz  = 1.92e6;
        expect(source.applyMasterClockLocked(30.72 * 1e6)) << "a replay of the clock in force answers true";
        expect(source._rateWrittenHz == 1.92e6 && !source._restageRate.load()) << "with no write, the rate cache kept and no restage of the rate";
        expect(source._masterClockPinnedHz == 30.72 * 1e6) << "and the clock pinned";
    });

    cases("controls.uhd-a-bounded-gain-is-written-back", [] {
        // A B205mini's receive and transmit ranges, with no handle behind either block: the
        // bound and its write-back read the range the device stated and nothing else.
        capture::uhd::Source source{};
        source._openTruth.gainMinDb = 0.0;
        source._openTruth.gainMaxDb = 76.0;
        source.rx_gains             = std::vector<double>{96.0};
        source.applyGain();
        expect(source.rx_gains->size() == 1UZ && source.rx_gains->front() == 76.0) << "the source's setting holds the gain the device takes";
        source.rx_gains = std::vector<double>{40.0};
        source.applyGain();
        expect(source.rx_gains->front() == 40.0) << "and a gain inside the range is left as asked";

        capture::uhd::Sink sink{};
        sink._openTruth.gainMinDb = 0.0;
        sink._openTruth.gainMaxDb = 89.75;
        sink.tx_gains             = std::vector<double>{100.0};
        sink.applyGainLocked();
        expect(sink.tx_gains->size() == 1UZ && sink.tx_gains->front() == 89.75) << "the sink's setting holds the ceiling";
        sink.tx_gains = std::vector<double>{std::numeric_limits<double>::quiet_NaN()};
        sink.applyGainLocked();
        expect(sink.tx_gains->front() == 0.0) << "and a gain that is not a number holds the floor";
    });

    cases("controls.uhd-the-drain-writes-the-bounded-gain-back", [] {
        /*| frame: the block reads as streaming and holds no device, and its control thread is
                not running, so a write staged there runs nothing. The setting moves inside the
                drain's own call or not at all.
        */
        capture::uhd::Source src{};
        src._openTruth.gainMinDb = 0.0;
        src._openTruth.gainMaxDb = 76.0;
        src._deviceUp.store(true, std::memory_order_release);
        src.rx_gains = std::vector<double>{96.0};
        gr::property_map changed{{"rx_gains", std::vector<double>{96.0}}};
        gr::property_map forward;
        src.settingsChanged({}, changed, forward);
        expect(src.rx_gains->size() == 1UZ && src.rx_gains->front() == 76.0) << "the drain writes the bounded gain back before it returns";
        src.rx_gains = std::vector<double>{40.0};
        gr::property_map inside{{"rx_gains", std::vector<double>{40.0}}};
        src.settingsChanged({}, inside, forward);
        expect(src.rx_gains->front() == 40.0) << "and leaves a gain inside the range as asked";
        src._deviceUp.store(false, std::memory_order_release);
    });

    cases("controls.uhd-the-stated-coverage-is-the-coverage-in-force", [] {
        // What a B205mini states at the open against what it states at 2 MS/s on a 2 MHz filter,
        // where UHD clips a request below 49 MHz to 49 MHz.
        const std::vector<std::pair<double, double>> atOpen{{42.0e6, 6008.0e6}};
        const std::vector<std::pair<double, double>> inForce{{49.0e6, 6001.0e6}};
        capture::uhd::Source source{};
        source._openTruth.freqRanges = atOpen;
        source._freqRanges           = inForce;
        expect(source.deviceTruth().freqRanges == inForce) << "the source states the coverage a tune is tested against";
        capture::uhd::Sink sink{};
        sink._openTruth.freqRanges = atOpen;
        sink._freqRanges           = inForce;
        sink._deviceUp.store(true);
        expect(sink.deviceTruth().freqRanges == inForce) << "and so does the sink";
        sink._deviceUp.store(false);
    });

    cases("controls.uhd-the-probe-reads-the-surface-the-block-runs-on", [] {
        /*| frame: the stand-in's settable filter states its coverage as UHD does, the tuner's
                50 MHz floor widened by half the filter. */
        struct ProbeStandIn : TruthStandIn {
            double              rateHz = 0.0;
            double              bwHz   = 56.0e6;
            void                set_rx_rate(double r, std::size_t) { rateHz = r; }
            double              get_rx_rate(std::size_t) const { return rateHz; }
            void                set_rx_bandwidth(double b, std::size_t) { bwHz = b; }
            double              get_rx_bandwidth(std::size_t) const { return bwHz; }
            void                set_tx_rate(double r, std::size_t c) { set_rx_rate(r, c); }
            double              get_tx_rate(std::size_t c) const { return get_rx_rate(c); }
            void                set_tx_bandwidth(double b, std::size_t c) { set_rx_bandwidth(b, c); }
            ::uhd::freq_range_t get_rx_freq_range(std::size_t) const { return ::uhd::freq_range_t(50.0e6 - bwHz / 2.0, 6000.0e6 + bwHz / 2.0, 0.0); }
        };
        ProbeStandIn               device;
        capture::uhd::DeviceTruth truth;
        capture::uhd::readSurfaceAtRate(device, 2.0e6, truth, capture::uhd::Direction::Receive, "", "case");
        expect(device.bwHz == 2.0e6) << "the probe writes the filter the rate applier derives from the rate";
        expect(truth.bwCurHz == 2.0e6) << "before the surface is read";
        expect(!truth.freqRanges.empty() && truth.freqRanges.front().first == 49.0e6) << "so the coverage offered is the one a tune at that rate reaches";

        // The stand-in divides a fixed 100 MHz clock, as an N210 does.
        ProbeStandIn n210;
        capture::uhd::readSurfaceAtRate(n210, 8.0e6, truth, capture::uhd::Direction::Receive, "", "case");
        expect(std::abs(n210.rateHz - 100.0e6 / 12.0) < 1.0) << "8 MS/s is written as the block writes it, decimation 12, rather than as UHD would reach it";
    });

    cases("controls.uhd-a-replay-of-the-selection-in-force-says-nothing", [] {
        // A B205mini opened on its internal reference, sc16 and UHD's own clock, with no handle
        // behind the block: the four staged controls touch no device.
        capture::uhd::Source source{};
        source._openTruth.clockSources = {"internal", "external"};
        source._openTruth.clockSource  = "internal";
        source._openTruth.timeSources  = {"internal", "external"};
        source._openTruth.timeSource   = "internal";
        source._openTruth.mclkMinHz    = 220.0e3;
        source._openTruth.mclkMaxHz    = 61.44e6;
        source._openTruth.mclkHz       = 32.0e6;
        source._clockSources           = source._openTruth.clockSources;
        source._timeSources            = source._openTruth.timeSources;
        source._deviceUp.store(true);
        const std::string replay = stderrOf([&source] {
            expect(source.setControl("CLOCK_SOURCE", 0.0) && source.setControl("TIME_SOURCE", 0.0)) << "the reference and the timing selection are staged";
            expect(source.setControl("WIRE_FORMAT", 0.0) && source.setControl("MASTER_CLOCK", 0.0)) << "and so are the format and the automatic clock";
        });
        expect(replay.empty()) << "a replay of what is in force prints nothing: " << replay;
        const std::string change = stderrOf([&source] { std::ignore = source.setControl("CLOCK_SOURCE", 1.0); });
        expect(change.find("the clock source external takes effect at the next start") != std::string::npos) << "a change is named";
        const std::string again = stderrOf([&source] { std::ignore = source.setControl("CLOCK_SOURCE", 1.0); });
        expect(again.empty()) << "once";
        source._masterClockPinnedHz.reset();
        source._clockInForceHz  = 30.72e6; // an open address that fixed the clock
        const std::string clock = stderrOf([&source] { std::ignore = source.setControl("MASTER_CLOCK", 0.0); });
        expect(clock.find("automatic master clock") != std::string::npos) << "and the automatic clock is named where the open fixed one";
        source._deviceUp.store(false);
    });

    cases("controls.uhd-a-settings-pass-never-waits-on-a-tune", [] {
        /*| frame: the block reads as streaming and holds no device, and a controls subscriber is
                registered on the block's own list: a Subscribe to a block that has not started
                answers from a probe, which opens the unit the device string names, any attached
                USRP for an empty one. A rate write then moves the surface, since the
                LO offset is bounded by the rate. The block's control thread runs a stand-in
                tune, which holds the control mutex until the case lets it go, as a retune holds
                that mutex across the device's tune. A settings pass carrying a frequency runs on
                a thread of the case's own, which stands for the consumer thread, and the case
                waits at most a second for it. No device is opened.
        */
        using capture::uhd::Source;
        Source src{};
        src._control.start("uhdctl", "case");
        src._deviceUp.store(true, std::memory_order_release);
        {
            std::lock_guard lock(src._controlProperties.mutex);
            src._controlProperties.controlsClients.insert("watch");
        }
        const auto offersOffset = [](const std::vector<capture::ControlDesc>& surface) { return std::ranges::any_of(surface, [](const capture::ControlDesc& c) { return c.id == "LO_OFFSET"; }); };
        expect(!offersOffset(src._controlProperties.lastSurface)) << "a block with no rate in force states no LO offset";
        {
            std::lock_guard lock(src._ctrlMutex);
            src.applyRateWith([](double hz) { return capture::uhd::RateWrite{.landed = true, .heldHz = hz}; });
        }

        std::promise<void>       tuning;
        std::promise<void>       letGo;
        std::shared_future<void> release = letGo.get_future().share();
        src._control.post(99, [&src, &tuning, release] {
            std::lock_guard lock(src._ctrlMutex);
            tuning.set_value();
            std::ignore = release.wait_for(std::chrono::seconds(5));
        });
        expect(fatal(tuning.get_future().wait_for(std::chrono::seconds(2)) == std::future_status::ready)) << "the stand-in tune holds the control mutex";

        std::promise<void> passed;
        std::future<void>  pass = passed.get_future();
        std::thread        consumer([&src, &passed] {
            gr::property_map changed{{"frequency", std::vector<double>{433.0e6}}};
            gr::property_map forward{{"frequency", std::vector<double>{433.0e6}}};
            src.settingsChanged({}, changed, forward);
            passed.set_value();
        });
        const bool returnedWhileHeld = pass.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
        letGo.set_value();
        consumer.join();
        src._control.settle();
        expect(returnedWhileHeld) << "the settings pass returns while a tune holds the control mutex";
        std::lock_guard lock(src._controlProperties.mutex);
        expect(offersOffset(src._controlProperties.lastSurface)) << "and it sent the subscriber the surface the rate write moved";
        src._deviceUp.store(false, std::memory_order_release);
    });

    cases("controls.uhd-states-its-rate-and-frequency-from-the-truth", [] {
        using capture::uhd::DeviceTruth;
        using capture::uhd::Sink;
        using capture::uhd::Source;
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};

        // A programmable clock over one continuous span on a USB 3 link, as a B2xx states it.
        DeviceTruth b2xx;
        b2xx.mclkMinHz      = 220.0e3;
        b2xx.mclkMaxHz      = 61.44e6;
        b2xx.mclkContinuous = true;
        b2xx.mclkHz         = 16.0e6;
        b2xx.linkBytesPerS  = 500.0e6;
        b2xx.freqRanges     = {{70.0e6, 6000.0e6}};
        const std::pair<double, double> clockSpan{220.0e3 / 256.0, 61.44e6};
        capture::test::expectRateAndFrequency(Source::describeControls(b2xx, ctx), capture::uhd::sampleRatesFor(b2xx), clockSpan, b2xx.freqRanges, "a programmable clock");

        // The same radio on a USB 2 link: the link ceiling in the wire format named ends the span.
        DeviceTruth usb2   = b2xx;
        usb2.linkBytesPerS = 53.248e6;
        capture::test::expectRateAndFrequency(Source::describeControls(usb2, ctx), capture::uhd::sampleRatesFor(usb2, "sc16"), std::pair{clockSpan.first, 13.312e6}, usb2.freqRanges, "sc16 over USB 2");
        capture::test::expectRateAndFrequency(Source::describeControls(usb2, ctx, 0.0, std::nullopt, "sc8"), capture::uhd::sampleRatesFor(usb2, "sc8"), std::pair{clockSpan.first, 26.624e6}, usb2.freqRanges, "sc8 over USB 2");

        // A clock a write pinned is a fixed clock: the list of that clock's rates and no span.
        capture::test::expectRateAndFrequency(Source::describeControls(b2xx, ctx, 0.0, 30.72e6), capture::uhd::sampleRatesFor(capture::uhd::ladderTruthFor(b2xx, 30.72e6)), std::nullopt, b2xx.freqRanges, "a pinned clock");

        // A fixed clock stating its own ladder, with a coverage of two spans.
        DeviceTruth fixed;
        fixed.mclkMinHz     = 100.0e6;
        fixed.mclkMaxHz     = 100.0e6;
        fixed.mclkHz        = 100.0e6;
        fixed.linkBytesPerS = 125.0e6;
        for (int n = 4; n <= 512; n += 2) {
            fixed.rates.emplace_back(100.0e6 / n, 100.0e6 / n);
        }
        fixed.freqRanges = {{2300.0e6, 2900.0e6}, {50.0e6, 2200.0e6}};
        capture::test::expectRateAndFrequency(Source::describeControls(fixed, ctx), capture::uhd::sampleRatesFor(fixed), std::nullopt, fixed.freqRanges, "a fixed clock");
        capture::test::expectRateAndFrequency(Sink::describeControls(fixed, ctx), capture::uhd::sampleRatesFor(fixed), std::nullopt, fixed.freqRanges, "a transmit block on a fixed clock");

        // A truth stating no coverage states the rate picks and no frequency.
        const auto bare = Source::describeControls(DeviceTruth{}, ctx);
        expect(capture::test::descriptorOf(bare, capture::kSampleRateControl).has_value()) << "the picks for a caller holding no probe";
        expect(!capture::test::descriptorOf(bare, capture::kFrequencyControl).has_value()) << "and no frequency range the device has not stated";
    });

    cases("controls.uhd-states-its-filter-and-its-gain-mode", [] {
        using capture::uhd::DeviceTruth;
        using capture::uhd::Source;
        capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 100.0e6};

        // A frontend whose property tree carries the loop's enable, as a B2xx's does.
        DeviceTruth withLoop;
        withLoop.hasAgc = true;
        for (const bool on : {false, true}) {
            ctx.gainMode    = on;
            const auto mode = capture::test::descriptorOf(Source::describeControls(withLoop, ctx), capture::kGainModeControl);
            expect(mode.has_value() && mode->kind == 2 && !mode->isGain) << "a frontend with an AGC states GAIN_MODE, a toggle and not a gain";
            expect(mode.has_value() && mode->defValue == (on ? 1.0 : 0.0)) << "at the mode the context holds";
        }
        expect(!capture::test::descriptorOf(Source::describeControls(DeviceTruth{}, ctx), capture::kGainModeControl).has_value()) << "a frontend without one states none";
        expect(!capture::test::descriptorOf(capture::uhd::Sink::describeControls(withLoop, ctx), capture::kGainModeControl).has_value()) << "and a transmit block states none";

        // An unstarted source states the mode it holds staged.
        gr::BlockWrapper<Source> model;
        model.init(std::make_shared<gr::Sequence>());
        auto& src = model.blockRef();
        expect(!capture::detail::stagedContext(src).gainMode) << "the setting's default is off";
        expect(src.settings().setStaged({{"gain_mode", true}}).empty());
        expect(capture::detail::stagedContext(src).gainMode) << "a staged mode reaches the context the probe is given";

        // A source's settings drain moves the default its surface states.
        Source running{};
        running._openTruth = withLoop;
        running.publishSurfaceLocked();
        running.gain_mode = true;
        gr::property_map changed{{"gain_mode", true}};
        gr::property_map forwarded;
        running.settingsChanged({}, changed, forwarded);
        const auto held = capture::test::descriptorOf(running.controlSurface(), capture::kGainModeControl);
        expect(held.has_value() && held->defValue == 1.0) << "the drain's mode is the one the block's surface states";

        // The filter the device settled on is a reading, a fixed one included.
        running._bwCurHz = 40.0e6;
        std::vector<capture::SensorReading> readings;
        running.appendReadings(readings);
        const auto filter = capture::analogBandwidthIn(readings);
        expect(filter.has_value() && *filter == 40.0e6) << "the analog bandwidth is stated in hertz";
        const auto stated = std::ranges::find_if(readings, [](const capture::SensorReading& r) { return r.id == capture::kAnalogBandwidthReading; });
        expect(stated != readings.end() && stated->brief && stated->good && stated->value == "40000000 Hz") << "on a glance row, the width with its unit";
        running._bwCurHz = 0.0;
        readings.clear();
        running.appendReadings(readings);
        expect(!capture::analogBandwidthIn(readings).has_value()) << "and a block that has opened nothing states no filter";
    });

    cases("controls.uhd-an-unstarted-source-states-the-ladder-of-its-staged-format", [] {
        using capture::uhd::DeviceTruth;
        using capture::uhd::Source;
        // A programmable clock on a USB 2 link, whose ceiling differs between the formats.
        DeviceTruth usb2;
        usb2.mclkMinHz      = 220.0e3;
        usb2.mclkMaxHz      = 61.44e6;
        usb2.mclkContinuous = true;
        usb2.mclkHz         = 16.0e6;
        usb2.linkBytesPerS  = 53.248e6;
        usb2.freqRanges     = {{70.0e6, 6000.0e6}};
        const double floorHz = 220.0e3 / 256.0;
        expect(fatal(capture::uhd::sampleRatesFor(usb2, "sc8") != capture::uhd::sampleRatesFor(usb2, "sc16"))) << "the two formats cut two ladders";

        gr::BlockWrapper<Source> model(gr::property_map{{"sample_rate", 2.0e6}});
        model.init(std::make_shared<gr::Sequence>());
        auto&      src        = model.blockRef();
        const auto wireFormat = [](const std::vector<capture::ControlDesc>& surface) { return capture::test::descriptorOf(surface, "WIRE_FORMAT").value_or(capture::ControlDesc{}).defValue; };

        const capture::TruthContext held = capture::detail::stagedContext(src);
        expect(held.wireFormat == "sc16") << "the context carries the format the block holds set";
        const auto sc16 = Source::describeControls(usb2, held);
        capture::test::expectRateAndFrequency(sc16, capture::uhd::sampleRatesFor(usb2, "sc16"), std::pair{floorHz, 13.312e6}, usb2.freqRanges, "sc16 held");
        expect(wireFormat(sc16) == 0.0) << "and WIRE_FORMAT states sc16";

        expect(src.settings().setStaged({{"wire_format", std::string("sc8")}}).empty());
        const capture::TruthContext staged = capture::detail::stagedContext(src);
        expect(staged.wireFormat == "sc8") << "a staged format reaches the context the probe is given";
        const auto sc8 = Source::describeControls(usb2, staged);
        capture::test::expectRateAndFrequency(sc8, capture::uhd::sampleRatesFor(usb2, "sc8"), std::pair{floorHz, 26.624e6}, usb2.freqRanges, "sc8 staged");
        expect(wireFormat(sc8) == 1.0) << "and WIRE_FORMAT states sc8";
        const auto sink = capture::test::descriptorOf(capture::uhd::Sink::describeControls(usb2, staged), capture::kSampleRateControl);
        expect(sink.has_value() && sink->listValues == capture::uhd::sampleRatesFor(usb2, "sc8")) << "a transmit block's ladder follows the context's format too";
        const auto running = capture::test::descriptorOf(Source::describeControls(usb2, staged, 0.0, std::nullopt, "sc16"), capture::kSampleRateControl);
        expect(running.has_value() && running->listValues == capture::uhd::sampleRatesFor(usb2, "sc16")) << "and the format a running block names outranks the context's";
    });

    cases("controls.uhd-a-wait-for-room-ends-at-the-consume-or-the-bound", [] {
        /*| frame: the members awaitProgress reads from a source, a progress counter and a rate.
                A core whose counter offers a timed wait takes that arm, and any other core
                reads the counter across the bound. The same relations hold on both arms. */
        struct ProgressStandIn {
            std::shared_ptr<gr::Sequence> progress = std::make_shared<gr::Sequence>();
            struct {
                double value = 0.0;
            } sample_rate;
        };
        constexpr auto timedArm  = []<typename Block>(Block*) { return requires(Block& blk) { blk.progress->waitUntil(0UZ, std::chrono::steady_clock::time_point{}); }; };
        constexpr bool kTimedArm = timedArm(static_cast<ProgressStandIn*>(nullptr));
        std::printf("awaitProgress arm: %s\n", kTimedArm ? "timed wait" : "counter reads");

        ProgressStandIn blk;
        blk.sample_rate.value = 1.0e6;
        constexpr std::size_t kEdge  = 4000UZ;
        const auto            bound = capture::roomWaitBound(kEdge, blk.sample_rate.value);
        expect(bound == std::chrono::milliseconds(2)) << "half of a 4000-sample edge at 1 MS/s";

        const std::size_t before = blk.progress->value();
        blk.progress->incrementAndGet();
        expect(capture::awaitProgress(blk, before, kEdge)) << "a counter that already left seen ends the wait";

        const std::size_t seen  = blk.progress->value();
        const auto        start = std::chrono::steady_clock::now();
        expect(!capture::awaitProgress(blk, seen, kEdge)) << "a counter that stays put ends the wait unmoved";
        expect(std::chrono::steady_clock::now() - start >= bound) << "and only once the bound has passed";

        // At the 0.1 s limit a consume on another thread ends the wait.
        blk.sample_rate.value = 0.0;
        const std::size_t held = blk.progress->value();
        std::thread       reader([&blk] {
            blk.progress->incrementAndGet();
            blk.progress->notify_all();
        });
        const bool moved = capture::awaitProgress(blk, held, kEdge);
        reader.join();
        expect(moved) << "a consume on another thread ends the wait moved";
        expect(blk.progress->value() == held + 1UZ) << "the one consume moved the counter";
    });

    return cases.report();
}
