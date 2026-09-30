/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the USRP sink block's decisions a radio is not needed for: the transmit connector, the
        transmit filter, the queue in front of the streamer and the device listing. No device is
        opened.
    why: every one of these is a pure function, a ring the case builds itself or an address the
        case spells, so the rule it pins is checked here rather than on the bench.
*/
#include <capture/common/Sink.hpp>
#include <capture/uhd/Sink.hpp>
#include <capture/uhd/Source.hpp>

#include <gnuradio-4.0/Graph.hpp>

#include "support/Cases.hpp"
#include "support/Labels.hpp"
#include "support/Held.hpp"
#include "support/Readings.hpp"
#include "support/ScheduledGraph.hpp"

#include "UnparsedAddress.hpp"

#include <uhd/property_tree.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <expected>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace {

/*| role: a stand-in for the part of a USRP the reference wait and the sensor sweep read: the
        clock and time selections, the motherboard's sensors and each frontend's, each read
        counted. No device is opened.
*/
struct SensorStandIn {
    std::string                                  clockSource = "internal";
    bool                                         clockThrows = false;
    int                                          lockAfter   = 0; // ref_locked reads true from this read on, and never where negative
    int                                          refReads    = 0;
    int                                          rxReads     = 0;
    int                                          txReads     = 0;
    std::vector<std::string>                     mboard{"ref_locked", "gps_time"};
    std::vector<std::string>                     rx{"lo_locked", "rssi"};
    std::vector<std::string>                     tx{"lo_locked"};
    int                                          slowRecordMs = 8; // a GPSDO's serial record, slower than a poll affords

    std::string get_clock_source(std::size_t) const {
        if (clockThrows) {
            throw std::runtime_error("no clock source");
        }
        return clockSource;
    }
    std::string                                  timeSource = "none";
    std::string                  get_time_source(std::size_t) const { return timeSource; }
    ::uhd::time_spec_t           get_time_last_pps(std::size_t) const { return ::uhd::time_spec_t(12.5); }
    std::vector<std::string>     get_mboard_sensor_names(std::size_t) const { return mboard; }
    std::vector<std::string>     get_rx_sensor_names(std::size_t) const { return rx; }
    std::vector<std::string>     get_tx_sensor_names(std::size_t) const { return tx; }
    ::uhd::sensor_value_t        get_mboard_sensor(const std::string& name, std::size_t) {
        if (name == "ref_locked") {
            ++refReads;
            return {"Ref", lockAfter >= 0 && refReads > lockAfter, "locked", "unlocked"};
        }
        if (name == "mimo_locked") {
            return {"MIMO", false, "locked", "unlocked"};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(slowRecordMs));
        return {"GPS epoch", std::string("1790000000"), "seconds"};
    }
    ::uhd::sensor_value_t get_rx_sensor(const std::string& name, std::size_t) {
        ++rxReads;
        return name == "rssi" ? ::uhd::sensor_value_t{"RSSI", -40.0, "dB"} : ::uhd::sensor_value_t{"LO", true, "locked", "unlocked"};
    }
    ::uhd::sensor_value_t get_tx_sensor(const std::string&, std::size_t) {
        ++txReads;
        return {"LO", false, "locked", "unlocked"};
    }
};

/*| role: the sensor stand-in with a device whose property tree the EEPROM read reaches: empty
        until a case creates the EEPROM node. refuse makes the enumeration throw.
*/
struct EepromStandIn : SensorStandIn {
    struct Device {
        ::uhd::property_tree::sptr tree = ::uhd::property_tree::make();
        ::uhd::property_tree::sptr get_tree() const { return tree; }
    };
    std::shared_ptr<Device>  device = std::make_shared<Device>();
    bool                     refuse = false;
    std::shared_ptr<Device>  get_device() const { return device; }
    std::vector<std::string> get_mboard_sensor_names(std::size_t i) const {
        if (refuse) {
            throw std::runtime_error("enumeration refused");
        }
        return SensorStandIn::get_mboard_sensor_names(i);
    }
};

/*| role: a stand-in for a transmit streamer's asynchronous channel: the messages it hands out,
        one per read, in order, and nothing once they are spent. No device is opened.
*/
struct AsyncChannelStandIn {
    std::vector<::uhd::async_metadata_t::event_code_t> codes;
    std::size_t                                        next = 0;

    bool recv_async_msg(::uhd::async_metadata_t& md, double) {
        if (next >= codes.size()) {
            return false;
        }
        md.event_code = codes[next++];
        return true;
    }
};

/*| role: a stand-in for a transmit streamer: a send places what it is handed, or nothing where
        placeNothing is raised, and counts the ends of burst it placed; the asynchronous channel
        hands out nothing. No device is opened.
*/
struct StreamStandIn : ::uhd::tx_streamer {
    std::atomic<bool> placeNothing{false};
    std::atomic<int>  endsPlaced{0};

    std::size_t get_num_channels() const override { return 1UZ; }
    std::size_t get_max_num_samps() const override { return 363UZ; }
    std::size_t send(const buffs_type&, const std::size_t nsamps, const ::uhd::tx_metadata_t& md, const double) override {
        if (placeNothing.load()) {
            return 0UZ;
        }
        if (md.end_of_burst) {
            endsPlaced.fetch_add(1);
        }
        return nsamps;
    }
    bool recv_async_msg(::uhd::async_metadata_t&, double) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return false;
    }
    void post_output_action(const std::shared_ptr<::uhd::rfnoc::action_info>&, const std::size_t) override {}
};


/*| role: a stand-in for a transmit streamer that keeps where each send read from, how many
        samples it placed and the samples themselves, and asks the drain to stop once it holds
        total samples. No device is opened.
*/
struct RecordingStream : ::uhd::tx_streamer {
    struct Send {
        const std::complex<float>* at;
        std::size_t                n;
    };
    capture::uhd::Sink&              sink;
    std::size_t                      total;
    std::vector<Send>                sends;
    std::vector<std::complex<float>> placed;

    RecordingStream(capture::uhd::Sink& s, std::size_t n) : sink(s), total(n) {}
    std::size_t get_num_channels() const override { return 1UZ; }
    std::size_t get_max_num_samps() const override { return 363UZ; }
    std::size_t send(const buffs_type& buffs, const std::size_t nsamps, const ::uhd::tx_metadata_t& md, const double) override {
        if (md.end_of_burst || nsamps == 0UZ) {
            return nsamps;
        }
        const auto* p = static_cast<const std::complex<float>*>(buffs[0]);
        sends.push_back({p, nsamps});
        placed.insert(placed.end(), p, p + nsamps);
        if (placed.size() >= total) {
            sink._drain.requestStop();
        }
        return nsamps;
    }
    bool recv_async_msg(::uhd::async_metadata_t&, double) override { return false; }
    void post_output_action(const std::shared_ptr<::uhd::rfnoc::action_info>&, const std::size_t) override {}
};

/*| role: a stand-in for a transmit streamer that places what it is handed and counts the data
        sends and the samples they carried, where another thread can read them. No device is
        opened.
*/
struct CountingStream : ::uhd::tx_streamer {
    std::atomic<std::size_t> sends{0};
    std::atomic<std::size_t> samples{0};

    std::size_t get_num_channels() const override { return 1UZ; }
    std::size_t get_max_num_samps() const override { return 363UZ; }
    std::size_t send(const buffs_type&, const std::size_t nsamps, const ::uhd::tx_metadata_t& md, const double) override {
        if (!md.end_of_burst) {
            sends.fetch_add(1);
            samples.fetch_add(nsamps);
        }
        return nsamps;
    }
    bool recv_async_msg(::uhd::async_metadata_t&, double) override { return false; }
    void post_output_action(const std::shared_ptr<::uhd::rfnoc::action_info>&, const std::size_t) override {}
};

/*| role: the two ends a graph gives a device block under test, a feed of zeros and a drain.
*/
struct ZeroFeed : gr::Block<ZeroFeed> {
    gr::PortOut<std::complex<float>> out;
    GR_MAKE_REFLECTABLE(ZeroFeed, out);

    [[nodiscard]] std::complex<float> processOne() const noexcept { return {}; }
};

struct Drain : gr::Block<Drain> {
    gr::PortIn<std::complex<float>> in;
    GR_MAKE_REFLECTABLE(Drain, in);

    gr::work::Status processBulk(gr::InputSpanLike auto& /*input*/) noexcept { return gr::work::Status::OK; }
};

/*| role: how a run under the default scheduler ended.
*/
struct RunEnd {
    bool                     returned = false;
    bool                     failed   = false;
    std::string              error;
    std::vector<std::string> errors;

    [[nodiscard]] bool names(std::string_view text) const {
        return std::ranges::any_of(errors, [text](const std::string& e) { return e.find(text) != std::string::npos; });
    }
};

/*| contract: run the graph with runAndWait() under the default scheduler, as a program's main()
        does, and answer whether the call returned inside five seconds, whether it failed, the
        error it failed with, and the error events the run left on the message plane.
    trap: a run that never ends keeps its thread and its scheduler. Both are left behind on the
        heap rather than destroyed under a scheduler still running, and the case reads the
        answer as a run that did not return.
*/
RunEnd runToEnd(gr::Graph flow) {
    RunEnd end;
    auto*  scheduled = new capture::test::ScheduledGraph<>();
    if (!scheduled->adopt(std::move(flow)).has_value()) {
        delete scheduled;
        return end;
    }
    auto        outcome = std::make_shared<std::atomic<int>>(0); // 0 running, 1 succeeded, 2 failed
    auto        error   = std::make_shared<std::string>();         // written before outcome, read after it
    std::thread runner([scheduled, outcome, error] {
        const auto result = scheduled->runAndWait();
        if (!result.has_value()) {
            *error = result.error().message;
        }
        outcome->store(result.has_value() ? 1 : 2, std::memory_order_release);
    });
    const auto  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (outcome->load(std::memory_order_acquire) == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (outcome->load(std::memory_order_acquire) == 0) {
        runner.detach();
        return end;
    }
    runner.join();
    end.returned = true;
    end.failed   = outcome->load(std::memory_order_acquire) == 2;
    end.error    = *error;
    end.errors   = scheduled->errors();
    delete scheduled;
    return end;
}

} // namespace

int main(int argc, char** argv) {
    using namespace boost::ut;
    capture::test::Cases cases(argc, argv);

    cases("sink.uhd-antenna-and-bandwidth", [] {
        using capture::uhd::Sink;
        const std::vector<std::string> b2xx{"TX/RX"};
        const std::vector<std::string> twoPort{"A", "TX/RX", "B"};

        // A caller who named a connector the device offers gets it.
        expect(capture::uhd::Sink::chooseAntenna(twoPort, "B") == "B") << "the connector asked for";
        expect(capture::uhd::Sink::chooseAntenna(b2xx, "TX/RX") == "TX/RX");

        // A caller who named nothing, or named a connector this model does not have, gets
        // TX/RX where the device offers it.
        expect(capture::uhd::Sink::chooseAntenna(twoPort, "") == "TX/RX") << "TX/RX is the default where it exists";
        expect(capture::uhd::Sink::chooseAntenna(twoPort, "RX2") == "TX/RX") << "and stands in for a connector this model does not have";
        expect(capture::uhd::Sink::chooseAntenna(b2xx, "") == "TX/RX");

        // A model with no TX/RX transmits from its first port rather than from a name it does
        // not have.
        const std::vector<std::string> named{"TX1", "TX2"};
        expect(capture::uhd::Sink::chooseAntenna(named, "") == "TX1") << "the device's own first entry";
        expect(capture::uhd::Sink::chooseAntenna(named, "TX2") == "TX2");
        expect(capture::uhd::Sink::chooseAntenna({}, "TX/RX").empty()) << "a device offering nothing is left alone";

        // The filter follows the rate, inside the range the device states.
        expect(capture::uhd::bandwidthFor(2.0e6, 0.2e6, 56.0e6) == 2.0e6) << "the rate itself, where the range holds it";
        expect(capture::uhd::bandwidthFor(80.0e6, 0.2e6, 56.0e6) == 56.0e6) << "a rate above the range takes the ceiling";
        expect(capture::uhd::bandwidthFor(0.1e6, 0.2e6, 56.0e6) == 0.2e6) << "and one below it the floor";
        expect(capture::uhd::bandwidthFor(2.0e6, 0.0, 0.0) == 2.0e6) << "a frontend stating no range leaves the rate as it stands";

        // The control surface states the connector only where the device offers a choice, and
        // the filter only where its range has room in it.
        capture::uhd::DeviceTruth fixedFilter;
        fixedFilter.antennas = b2xx;
        fixedFilter.antenna  = "TX/RX";
        fixedFilter.bwMinHz  = 40.0e6;
        fixedFilter.bwMaxHz  = 40.0e6;
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};
        const auto                  narrow = capture::uhd::Sink::describeControls(fixedFilter, ctx);
        expect(narrow.size() == 1UZ && narrow.front().id == capture::kSampleRateControl) << "a single connector and a fixed filter leave a caller the rate alone to move";

        capture::uhd::DeviceTruth wide;
        wide.antennas = twoPort;
        wide.antenna  = "TX/RX";
        wide.bwMinHz  = 0.2e6;
        wide.bwMaxHz  = 56.0e6;
        wide.gains.push_back({"PGA", 0.0, 89.75, 0.25, 10.0});
        const auto offered = capture::uhd::Sink::describeControls(wide, ctx);
        expect(offered.size() == 4UZ) << "a gain element, the connector, the filter and the rate";
        expect(offered.front().id == "PGA") << "the gain element keeps the name the device gave it";
        expect(offered[1].id == "ANTENNA") << "the connector is an index into the device's own list";
        expect(offered[1].options.size() == 3UZ);
        expect(offered[1].defValue == 1.0) << "defaulting to the port a transmit caller gets";
        expect(offered[2].id == "BANDWIDTH") << "and the filter is stated in MHz";
        expect(offered[2].defValue == 2.0) << "derived from the rate where the block holds no width yet";
        expect(offered[2].rateDerived) << "and re-derived at every rate change";
    });

    cases("sink.uhd-both-blocks-describe-one-truth-alike", [] {
        /*| frame: one unit's truth described by both blocks. The gain elements and the filter
                are the device's, so both surfaces state them field for field alike, and both
                state the shared descriptors.
        */
        capture::uhd::DeviceTruth t;
        t.gains.push_back({"PGA", 0.0, 76.0, 1.0, 30.0});
        t.gains.push_back({"ADC-pga", 0.0, 6.0, 0.5, 3.0});
        t.antennas = {"TX/RX", "RX2"};
        t.antenna  = "TX/RX";
        t.bwMinHz  = 0.2e6;
        t.bwMaxHz  = 56.0e6;
        t.bwCurHz  = 2.0e6;
        const auto same = [](const capture::ControlDesc& a, const capture::ControlDesc& b) {
            return a.id == b.id && a.label == b.label && a.unit == b.unit && a.kind == b.kind && a.min == b.min && a.max == b.max && a.step == b.step && a.defValue == b.defValue && a.options == b.options &&
                   a.listValues == b.listValues && a.isGain == b.isGain && a.inoperativeAboveHz == b.inoperativeAboveHz && a.rateDerived == b.rateDerived && a.sessionOnly == b.sessionOnly &&
                   a.appliesAt == b.appliesAt;
        };
        for (const double rate : {0.1e6, 2.0e6, 80.0e6}) {
            for (const double held : {0.0, 1.5e6}) {
                const capture::TruthContext       ctx{.deviceParams = {}, .sampleRate = rate, .centerFreq = 915.0e6};
                const auto                        rx     = capture::uhd::Source::describeControls(t, ctx, held);
                const auto                        tx     = capture::uhd::Sink::describeControls(t, ctx, held);
                std::vector<capture::ControlDesc> shared = capture::uhd::gainElementControls(t);
                shared.push_back(capture::uhd::bandwidthControl(t, ctx, held).value());
                expect(shared.size() == 3UZ) << "two gain elements and the filter";
                for (const auto& want : shared) {
                    const auto inRx = std::ranges::find_if(rx, [&want](const capture::ControlDesc& c) { return c.id == want.id; });
                    const auto inTx = std::ranges::find_if(tx, [&want](const capture::ControlDesc& c) { return c.id == want.id; });
                    expect(inRx != rx.end() && same(*inRx, want)) << "the receive surface states the shared " << want.id;
                    expect(inTx != tx.end() && same(*inTx, want)) << "and so does the transmit one";
                }
            }
        }
        capture::uhd::DeviceTruth fixed = t;
        fixed.bwMinHz                   = 40.0e6;
        fixed.bwMaxHz                   = 40.0e6;
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};
        expect(!capture::uhd::bandwidthControl(fixed, ctx, 0.0).has_value()) << "a fixed filter is no control in either direction";
    });

    cases("sink.uhd-snaps-the-rate-as-the-source-does", [] {
        /*| frame: the transmit records of an N210, a fixed 100 MHz clock over a 1 Gbit link whose
                transmit DSP states every whole interpolation, odd ones among them, and of a
                B205mini, whose clock is programmable.
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

        capture::uhd::Sink   sink;
        capture::uhd::Source source;
        sink._openTruth = n210;
        expect(sink.rateToWrite(8.0e6) == 100.0e6 / 12.0) << "8 MS/s on an N210 is interpolation 12, both halfbands in, rather than 13 on the CIC alone";
        expect(sink.rateToWrite(30.72e6) == 25.0e6) << "and 30.72 MS/s is the most the link carries in sc16";
        sink._openTruth = b205;
        expect(sink.rateToWrite(8.0e6) == 8.0e6) << "a B205mini reaches 8 MS/s by choosing its clock";
        expect(sink.rateToWrite(30.72e6) == 30.72e6) << "and 30.72 MS/s the same way";
        for (const auto* t : {&n210, &b205}) {
            sink._openTruth   = *t;
            source._openTruth = *t;
            for (const double request : {250.0e3, 2.0e6, 8.0e6, 30.72e6}) {
                expect(sink.rateToWrite(request) == source.rateToWrite(request)) << "the sink snaps as the source does";
            }
        }
        sink._openTruth  = n210;
        sink._wireFormat = "sc8";
        expect(sink.rateToWrite(30.72e6) == 25.0e6) << "sc8 reaches 50 MS/s, and 25 MS/s is still the nearest even interpolation";
        expect(sink.rateToWrite(45.0e6) == 50.0e6) << "a request past the sc16 ceiling reaches 50 MS/s in sc8";
    });

    cases("sink.uhd-a-refused-rate-write-reads-the-rate-back", [] {
        for (const auto dir : {capture::uhd::Direction::Receive, capture::uhd::Direction::Transmit}) {
            const auto refused = capture::uhd::writeRateWith(8.0e6, [](double) { throw std::runtime_error("refused"); }, [] { return 2.0e6; }, dir, "case");
            expect(!refused.landed) << "a refused write says so";
            expect(refused.heldHz == 2.0e6) << "and states the rate the device still holds";

            const auto mute = capture::uhd::writeRateWith(
                8.0e6, [](double) { throw std::runtime_error("refused"); }, []() -> double { throw std::runtime_error("unreachable"); }, dir, "case");
            expect(!mute.landed && mute.heldHz == 0.0) << "a device that cannot be asked either states no rate";

            double     wrote  = 0.0;
            const auto landed = capture::uhd::writeRateWith(8.0e6, [&wrote](double r) { wrote = r; }, [] { return 7.999e6; }, dir, "case");
            expect(wrote == 8.0e6) << "the rate asked for is the rate written";
            expect(landed.landed && landed.heldHz == 7.999e6) << "and the rate stated is the one the device answers";
        }
    });

    cases("sink.uhd-waits-for-the-reference", [] {
        SensorStandIn internal;
        const auto    none = capture::uhd::waitForReference(internal, "case", 60.0, 1.0);
        expect(!none.waited && none.waitedMs == 0.0) << "an internal clock is not waited for";
        expect(internal.refReads == 0) << "and its reference sensor is not read";

        SensorStandIn unstated;
        unstated.clockThrows = true;
        expect(!capture::uhd::waitForReference(unstated, "case", 60.0, 1.0).waited) << "a radio stating no clock source has no reference to wait for";

        SensorStandIn late;
        late.clockSource = "external";
        late.lockAfter   = 3;
        const auto locked = capture::uhd::waitForReference(late, "case", 1500.0, 1.0);
        expect(locked.waited && locked.locked) << "an external reference is waited for until it locks";
        expect(late.refReads == 4) << "and polled until the read that answers locked";

        SensorStandIn never;
        never.clockSource = "external";
        never.lockAfter   = -1;
        const auto unlocked = capture::uhd::waitForReference(never, "case", 60.0, 1.0);
        expect(unlocked.waited && !unlocked.locked) << "a reference that never locks is given up";
        expect(unlocked.waitedMs >= 60.0 && unlocked.waitedMs < 1000.0) << "after the limit and not much later";

        capture::uhd::Sink sink;
        expect(sink.refLockWaitMs() == 0.0) << "a sink that has not started has waited for nothing";
    });

    cases("sink.uhd-reads-the-transmit-sensors", [] {
        /*| frame: the brief set is a timing the host makes, so the limit here sits far above a read
                preempted on a loaded host and the slow record far above the limit.
        */
        SensorStandIn device;
        device.slowRecordMs = 400;
        const auto names    = capture::uhd::sensorNamesOf(device, capture::uhd::Direction::Transmit, "case", 50.0);
        expect(names.frontend == std::vector<std::string>{"lo_locked"}) << "the transmit frontend's names";
        expect(device.txReads == 1 && device.rxReads == 0) << "timed through the transmit frontend alone";
        expect(std::ranges::find(names.brief, "gps_time") == names.brief.end()) << "a slow record is left out of the brief set";
        expect(std::ranges::find(names.brief, "ref_locked") != names.brief.end()) << "and the reference lock is in it";

        const auto live = [] { return true; };
        std::vector<capture::SensorReading> out;
        expect(capture::uhd::readDeviceSensors(out, device, names, capture::uhd::Direction::Transmit, capture::uhd::SensorSweep::Full, live)) << "a full sweep runs through";
        const auto find = [&out](const std::string& id) { return std::ranges::find_if(out, [&id](const capture::SensorReading& r) { return r.id == id; }); };
        expect(find("ref_locked") != out.end() && find("ref_locked")->value == "locked") << "the reference lock, under the unit's own name";
        expect(find("tx_lo_locked") != out.end() && find("tx_lo_locked")->value == "unlocked" && !find("tx_lo_locked")->good) << "the transmit synthesizer's lock, under a tx_ id";
        expect(find("tx_lo_locked") != out.end() && find("tx_lo_locked")->label == "TX LO lock") << "and labeled as the transmit one, in an operator's words";
        expect(find("ref_locked") != out.end() && find("ref_locked")->label == "Reference lock") << "a terse device name gives way to an operator's";
        expect(find("gps_time") != out.end() && find("gps_time")->label == "GPS epoch") << "and a name nothing replaces is the device's own";
        expect(find("lo_locked") == out.end() && find("rssi") == out.end()) << "no receive frontend reading";
        expect(find("gps_time") != out.end()) << "a full sweep reads the slow record";
        expect(find("clock_source") != out.end() && find("clock_source")->value == "internal") << "the reference selection beside the sensors";
        expect(!find("clock_source")->brief && !find("time_source")->brief) << "the selections the controls state are no glance rows";
        expect(find("time_source") != out.end() && find("time_last_pps") != out.end()) << "and the timing selection and the last edge";

        out.clear();
        capture::uhd::readDeviceSensors(out, device, names, capture::uhd::Direction::Transmit, capture::uhd::SensorSweep::Brief, live);
        expect(find("gps_time") == out.end() && find("tx_lo_locked") != out.end()) << "a brief sweep leaves the slow record out";

        const auto rxNames = capture::uhd::sensorNamesOf(device, capture::uhd::Direction::Receive, "case", 50.0);
        out.clear();
        capture::uhd::readDeviceSensors(out, device, rxNames, capture::uhd::Direction::Receive, capture::uhd::SensorSweep::Full, live);
        expect(find("lo_locked") != out.end() && find("lo_locked")->value == "locked" && find("rssi") != out.end()) << "the receive frontend's readings keep their own names";

        int        reads   = 0;
        const auto dropped = [&reads] { return ++reads < 2; };
        out.clear();
        expect(!capture::uhd::readDeviceSensors(out, device, names, capture::uhd::Direction::Transmit, capture::uhd::SensorSweep::Full, dropped)) << "a device going down mid-sweep ends it";
        expect(out.empty()) << "with nothing published";
    });

    cases("sink.uhd-readings-are-what-the-stream-produces", [] {
        /*| frame: no device is opened. Each block's own readings follow the device's sensors;
                the connector and the wire format the settings choose are not readings.
        */
        capture::uhd::Source src;
        src._seqErrors.store(2U);
        std::vector<capture::SensorReading> rx;
        src.appendReadings(rx);
        expect(capture::test::readingIds(rx) == std::vector<std::string>{"rx_overflows", "rx_dropped_samples", "rx_sequence_errors", "rx_dropped_markers"}) << "what the receive stream has lost";
        expect(capture::test::briefLabels(rx) == std::vector<std::string>{"Overflows", "Dropped samples"}) << "the overflows and the dropped samples on the glance rows";
        expect(capture::test::readingOf(rx, "rx_sequence_errors").label == "Lost packets" && capture::test::readingOf(rx, "rx_sequence_errors").value == "2") << "the link's losses are a diagnostic";

        capture::uhd::Sink                  sink;
        std::vector<capture::SensorReading> tx;
        sink.appendReadings(tx);
        expect(capture::test::readingIds(tx) == std::vector<std::string>{"tx_underrun_events", "tx_sequence_errors", "tx_late_packets", "tx_discarded_at_rate_change", "tx_unsent_at_stop", "tx_burst_acks", "tx_queued_samples"})
            << "what the transmit stream has lost and holds";
        expect(capture::test::briefLabels(tx) == std::vector<std::string>{"Underruns"}) << "the underruns on the glance row";
    });

    cases("sink.uhd-names-a-gpsdo-the-open-did-not-find", [] {
        using capture::uhd::gpsdoUndetected;
        const std::vector<std::string> plain{"mimo_locked", "ref_locked"};
        const std::vector<std::string> disciplined{"gps_gpgga", "gps_gprmc", "gps_time", "gps_locked", "gps_servo", "mimo_locked", "ref_locked"};
        expect(gpsdoUndetected("internal", plain)) << "an EEPROM naming an internal GPSDO beside no GPS sensor is a GPSDO the open did not find";
        expect(gpsdoUndetected("onboard", plain)) << "and so is an onboard one";
        expect(!gpsdoUndetected("internal", disciplined)) << "a GPSDO the open found publishes its GPS sensors";
        expect(!gpsdoUndetected("none", plain) && !gpsdoUndetected("", plain)) << "a unit whose EEPROM names none, or keeps no such field, has none to find";

        SensorStandIn device;
        device.mboard       = plain;
        device.slowRecordMs = 0;
        auto names          = capture::uhd::sensorNamesOf(device, capture::uhd::Direction::Receive, "case", 50.0);
        expect(!names.gpsdoUndetected) << "a device with no EEPROM to read states no missing GPSDO";

        EepromStandIn named;
        named.mboard       = plain;
        named.slowRecordMs = 0;
        expect(capture::uhd::eepromGpsdo(named).empty()) << "a tree that holds no EEPROM node answers no GPSDO";
        expect(!capture::uhd::sensorNamesOf(named, capture::uhd::Direction::Receive, "case", 50.0).gpsdoUndetected) << "and states none missing";
        ::uhd::usrp::mboard_eeprom_t eeprom;
        eeprom["gpsdo"] = "internal";
        named.device->tree->create<::uhd::usrp::mboard_eeprom_t>("/mboards/0/eeprom").set(eeprom);
        expect(capture::uhd::eepromGpsdo(named) == "internal") << "the EEPROM read answers the node's gpsdo key";
        expect(capture::uhd::sensorNamesOf(named, capture::uhd::Direction::Receive, "case", 50.0).gpsdoUndetected) << "and an open with no GPS sensor states the GPSDO missing";
        named.refuse = true;
        expect(!capture::uhd::sensorNamesOf(named, capture::uhd::Direction::Receive, "case", 50.0).gpsdoUndetected) << "a refused enumeration states nothing about a GPSDO";
        named.refuse = false;
        named.mboard = disciplined;
        expect(!capture::uhd::sensorNamesOf(named, capture::uhd::Direction::Receive, "case", 50.0).gpsdoUndetected) << "and an open that published the GPS sensors states none missing";

        const auto live = [] { return true; };
        std::vector<capture::SensorReading> out;
        const auto find = [&out](const std::string& id) { return std::ranges::find_if(out, [&id](const capture::SensorReading& r) { return r.id == id; }); };
        expect(capture::uhd::readDeviceSensors(out, device, names, capture::uhd::Direction::Receive, capture::uhd::SensorSweep::Full, live));
        expect(find("gpsdo") == out.end()) << "and publishes no reading about one";

        names.gpsdoUndetected = true;
        for (const auto sweep : {capture::uhd::SensorSweep::Full, capture::uhd::SensorSweep::Brief}) {
            out.clear();
            expect(capture::uhd::readDeviceSensors(out, device, names, capture::uhd::Direction::Receive, sweep, live));
            const auto gpsdo = find("gpsdo");
            expect(fatal(gpsdo != out.end())) << "a GPSDO the open did not find is a reading of its own";
            expect(!gpsdo->good && gpsdo->brief) << "marked failed, on a glance row";
            expect(gpsdo->value.find("EEPROM") != std::string::npos) << "it says the EEPROM names the GPSDO";
            expect(gpsdo->value.find("power-cycle") != std::string::npos) << "and names the power cycle that makes UHD search again";
            expect(std::ranges::none_of(out, [](const capture::SensorReading& r) { return r.id.starts_with("gps_"); })) << "no reading takes a GPS sensor's gps_ prefix";
        }
    });

    cases("sink.uhd-an-empty-element-name-is-the-overall-gain", [] {
        capture::uhd::DeviceTruth wbx;
        wbx.gains.push_back({"PGA0", 0.0, 31.5, 0.5, 0.0});
        expect(capture::uhd::gainElementNamed(wbx, "", "case")) << "the empty name is the overall gain, in both blocks";
        expect(capture::uhd::gainElementNamed(wbx, "PGA0", "case")) << "a name the device gives is written";
        expect(!capture::uhd::gainElementNamed(wbx, "PGA", "case")) << "and one it does not give is refused, beside the names it does";
        expect(!capture::uhd::gainElementNamed({}, "PGA0", "case")) << "a device naming no element refuses every name";

        capture::uhd::Sink sink;
        expect(!sink.setElementGain("", 10.0)) << "a sink with no device writes nothing";
    });

    cases("sink.uhd-a-full-ring-consumes-nothing", [] {
        // The work call stages what the ring has room for and consumes exactly that. A ring
        // with no room takes nothing, so the scheduler offers the same samples again rather
        // than the block dropping them or waiting on the device inside a work call.
        using CF32 = std::complex<float>;
        capture::SpscRing<CF32> ring(8UZ);
        expect(capture::ringRoom(ring) == 8UZ) << "a fresh ring is all room";

        const std::vector<CF32> block(6, CF32{0.25f, -0.25f});
        expect(capture::writeWhatFits(ring, block.data(), block.size()) == 6UZ) << "a block that fits goes in whole";
        expect(capture::ringFill(ring) == 6UZ);
        expect(capture::writeWhatFits(ring, block.data(), block.size()) == 2UZ) << "a block larger than the room takes the room";
        expect(capture::ringRoom(ring) == 0UZ) << "which fills it";
        expect(capture::writeWhatFits(ring, block.data(), block.size()) == 0UZ) << "and a full ring takes nothing at all";
        expect(ring.dropped.load() == 0UL) << "nothing is dropped, so no sample the caller handed over goes missing";

        ring.advanceTo(5UZ);
        expect(capture::ringRoom(ring) == 5UZ) << "the drain gives the room back";
        expect(capture::writeWhatFits(ring, block.data(), block.size()) == 5UZ);
        expect(ring.dropped.load() == 0UL);

        // The queue the block sizes from a rate is about a quarter of a second of it.
        expect(capture::ringSamplesFor(8.0e6) == 2097152UZ) << "a quarter second at 8 MS/s, rounded up to the power of two the ring indexes by";
        expect(capture::ringSamplesFor(2.0e6) == 524288UZ) << "and the floor holds at the slowest rate these devices run";
        expect(capture::ringSamplesFor(0.0) == 524288UZ) << "a rate that says nothing takes that floor";
        expect(capture::ringSamplesFor(20.0e6) > capture::ringSamplesFor(8.0e6)) << "a faster rate needs more room for the same time";
    });

    cases("sink.uhd-a-rate-change-ends-the-burst", [] {
        // A rate change ends the burst in flight and writes the rate; the next samples open a
        // fresh burst at it. A replay of the rate already in force writes nothing, which is
        // what keeps a running radio from being disturbed for a value it already holds.
        const auto unchanged = capture::rateChangeFor(2.0e6, 2.0e6, true);
        expect(!unchanged.write) << "the rate in force is not written again";
        expect(!unchanged.endBurst) << "so the burst is left alone";

        const auto midBurst = capture::rateChangeFor(4.0e6, 2.0e6, true);
        expect(midBurst.write) << "a new rate reaches the device";
        expect(midBurst.endBurst) << "and the burst in flight ends first, the samples staged at the old rate having gone out";

        const auto beforeStart = capture::rateChangeFor(2.0e6, 0.0, false);
        expect(beforeStart.write) << "the first rate of a run is written";
        expect(!beforeStart.endBurst) << "with no burst to end";

        expect(!capture::rateChangeFor(0.0, 2.0e6, true).write) << "a rate that is not positive asks for nothing";
        expect(!capture::rateChangeFor(std::numeric_limits<double>::quiet_NaN(), 2.0e6, true).write) << "nor one that is not a number";

        // The bound every wait is sized from grows with the queue and shrinks with the rate.
        expect(capture::playOutMs(2097152U, 8.0e6).count() >= 262) << "a full queue at 8 MS/s covers the 262 ms it holds";
        expect(capture::playOutMs(2097152U, 2.0e6).count() > capture::playOutMs(2097152U, 8.0e6).count()) << "a slower rate plays the same queue out over a longer time";
        expect(capture::playOutMs(0U, 0.0).count() >= 250) << "and a sink with no rate still names a deadline";
    });

    cases("sink.uhd-listing-pairs-are-the-sources", [] {
        // The sink offers the device list the source offers, from one body both blocks call,
        // so a caller choosing a unit for transmit reads the entry it chose for receive.
        uhd::device_addr_t b205;
        b205["type"]    = "b200";
        b205["serial"]  = "30D3AB9";
        b205["product"] = "B205mini";
        const auto entry = capture::uhd::listingEntryFor(b205);
        expect(entry.label == "USRP B205mini 30D3AB9") << "the label names the product and the serial";
        expect(entry.deviceString == "driver=uhd,type=b200,serial=30D3AB9") << "and the device string carries the keys that select it";
        expect(entry.unspellableKey.empty()) << "which this address can be spelled with";

        // Two transports answering for one unit share an identity, so the listing keeps one
        // entry for it.
        uhd::device_addr_t again;
        again["type"]   = "b200";
        again["serial"] = "30D3AB9";
        expect(capture::uhd::listingEntryFor(again).identity == entry.identity) << "an address of the same keys is the same unit";

        uhd::device_addr_t other;
        other["type"]   = "b200";
        other["serial"] = "30D3ABA";
        expect(capture::uhd::listingEntryFor(other).identity != entry.identity) << "and a different serial is a different unit";

        uhd::device_addr_t noSerial;
        noSerial["type"] = "usrp2";
        noSerial["addr"] = "192.168.10.2";
        const auto network = capture::uhd::listingEntryFor(noSerial);
        expect(network.label == "USRP usrp2") << "a unit stating no serial is named by its type";
        expect(network.deviceString == "driver=uhd,type=usrp2,addr=192.168.10.2") << "and selected by the address it answers on";

        // A value a device string cannot spell leaves the unit out rather than naming another
        // radio or none.
        uhd::device_addr_t odd;
        odd["type"]   = "b200";
        odd["serial"] = "30D,3AB9";
        const auto refused = capture::uhd::listingEntryFor(odd);
        expect(refused.unspellableKey == "serial") << "the listing names the key it could not spell";
        expect(refused.identity.empty()) << "and the entry carries no identity";
        expect(capture::uhd::deviceStringSafe("30D3AB9")) << "an ordinary serial is spellable";
        expect(!capture::uhd::deviceStringSafe("a,b")) << "one carrying the separator between two pairs is not";
        expect(!capture::uhd::deviceStringSafe("a=b")) << "nor one carrying the separator between a key and a value";
    });

    cases("sink.uhd-refuses-a-rate-that-is-not-a-positive-number", [] {
        using capture::uhd::Sink;
        // The rate applier is the only body that reads the device's coverage, and a rate that
        // is not positive asks it for nothing. Left to run, the block would transmit at
        // whatever rate and frequency the previous program left the radio on, with every tune
        // of the run refused against an empty coverage.
        expect(capture::uhd::Sink::startRefusal(2.0e6).empty()) << "an ordinary rate starts";
        expect(capture::uhd::Sink::startRefusal(61.44e6).empty()) << "and so does the top of a B2xx";
        expect(!capture::uhd::Sink::startRefusal(0.0).empty()) << "a rate of zero does not";
        expect(!capture::uhd::Sink::startRefusal(-1.0).empty()) << "nor a negative one";
        expect(!capture::uhd::Sink::startRefusal(std::numeric_limits<double>::quiet_NaN()).empty()) << "nor one that is not a number";
        expect(capture::uhd::Sink::startRefusal(0.0).find("0") != std::string::npos) << "the refusal states the rate it was given";

        // What the refusal protects: an empty coverage refuses every tune of the run.
        const std::vector<std::pair<double, double>> none;
        expect(!capture::frequencyInRange(915.0e6, none).has_value()) << "no frequency is inside a coverage nothing filled";
    });

    cases("sink.uhd-marks-a-lock-only-where-it-applies", [] {
        SensorStandIn device;
        device.lockAfter  = -1; // the reference never locks, and no MIMO cable is fitted
        device.mboard     = {"ref_locked", "mimo_locked"};
        const auto names  = capture::uhd::sensorNamesOf(device, capture::uhd::Direction::Receive, "case", 50.0);
        const auto live   = [] { return true; };
        std::vector<capture::SensorReading> out;
        const auto find  = [&out](const std::string& id) { return std::ranges::find_if(out, [&id](const capture::SensorReading& r) { return r.id == id; }); };
        const auto sweep = [&] {
            out.clear();
            capture::uhd::readDeviceSensors(out, device, names, capture::uhd::Direction::Receive, capture::uhd::SensorSweep::Full, live);
        };
        sweep();
        expect(find("ref_locked") != out.end() && find("ref_locked")->value == "unlocked" && find("ref_locked")->good) << "an internal clock's reference reading is published and not marked";
        expect(!find("ref_locked")->brief) << "and is no glance row";
        expect(find("mimo_locked") != out.end() && find("mimo_locked")->good && !find("mimo_locked")->brief) << "nor is a MIMO lock with no MIMO source";
        device.clockSource = "external";
        sweep();
        expect(find("ref_locked") != out.end() && !find("ref_locked")->good && find("ref_locked")->brief) << "an external reference that has not locked is marked, on a glance row";
        expect(find("mimo_locked") != out.end() && find("mimo_locked")->good) << "and the MIMO lock still does not apply";
        device.clockSource = "mimo";
        sweep();
        expect(find("mimo_locked") != out.end() && !find("mimo_locked")->good) << "a MIMO clock source marks its own lock";
        device.clockSource = "internal";
        device.timeSource  = "mimo";
        sweep();
        expect(find("mimo_locked") != out.end() && !find("mimo_locked")->good && find("ref_locked")->good) << "and so does a MIMO time source";
        device.clockThrows = true;
        device.timeSource  = "none";
        sweep();
        expect(find("ref_locked") != out.end() && find("ref_locked")->good) << "a radio that states no clock source has no reference to mark";
    });

    cases("sink.uhd-bounds-the-transmit-gain-at-the-device", [] {
        using capture::uhd::Sink;
        // UHD coerces a gain past the frontend's range without refusing it, so a request of
        // 200 dB reaches the device's maximum power with only the read-back to say so.
        expect(capture::uhd::boundedGainDb(40.0, 0.0, 89.75) == 40.0) << "a request inside the range is passed through";
        expect(capture::uhd::boundedGainDb(200.0, 0.0, 89.75) == 89.75) << "one above it takes the ceiling";
        expect(capture::uhd::boundedGainDb(-20.0, 0.0, 89.75) == 0.0) << "and one below it the floor";
        expect(capture::uhd::boundedGainDb(std::numeric_limits<double>::quiet_NaN(), 0.0, 89.75) == 0.0) << "a request that is not a number takes the quiet end";
        expect(capture::uhd::boundedGainDb(40.0, 0.0, 0.0) == 40.0) << "a device stating no usable range leaves a finite request alone";
        expect(capture::uhd::boundedGainDb(40.0, 10.0, 5.0) == 40.0) << "and so does one whose range is reversed";

        // The two cross: a frontend stating no overall range and a gain element whose ends are
        // equal both reach the unusable-range branch, and a value that is not a number passed
        // through it reaches the device.
        expect(capture::uhd::boundedGainDb(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0) == 0.0) << "an unusable range still refuses a value that is not a number";
        expect(capture::uhd::boundedGainDb(std::numeric_limits<double>::quiet_NaN(), 10.0, 5.0) == 10.0) << "and so does a reversed one";
        expect(std::isfinite(capture::uhd::boundedGainDb(std::numeric_limits<double>::quiet_NaN(), 40.0e6, 40.0e6))) << "a fixed stage answers the one value it has";

        bool insideRange = true;
        for (int dB = -100; dB <= 300; ++dB) {
            const double bounded = capture::uhd::boundedGainDb(static_cast<double>(dB), 0.0, 38.0);
            if (bounded < 0.0 || bounded > 38.0) {
                insideRange = false;
            }
        }
        expect(insideRange) << "nothing reaches the device outside the range it states, over the whole span";
    });

    cases("sink.uhd-states-the-frequency-the-caller-asked-for", [] {
        using capture::uhd::Sink;
        const std::vector<std::pair<double, double>> coverage{{49.0e6, 6001.0e6}};

        // The tune is the request divided by one plus the fraction, which is exact. The
        // first-order product differs by the request times the square of the fraction, ten
        // hertz at 1 GHz and 100 ppm, against a tuner that steps in hundredths of a hertz.
        const double commanded = capture::uhd::tuneFrequencyFor(1.0e9, 100.0, coverage).value();
        expect(commanded < 1.0e9) << "a fast reference is commanded low";
        expect(std::abs(commanded - 1.0e9 / 1.0001) < 1e-3) << "by the exact scale";
        expect(std::abs(commanded - 1.0e9 * (1.0 - 100.0 / 1e6)) > 5.0) << "which sits about ten hertz from the product form";
        expect(std::abs(commanded - 1.0e9 * (1.0 - 100.0 / 1e6)) < 15.0);

        // The read-back is carried back into the caller's own frame, so the stated value can
        // be written back without the correction being applied twice.
        expect(std::abs(capture::uhd::tunedFrequencyFrom(commanded, 100.0) - 1.0e9) < 1e-3) << "the frequency stated is the one asked for";
        expect(capture::uhd::tunedFrequencyFrom(914'999'999.404, 0.0) == 914'999'999.404) << "with no correction the read-back stands as it is";
        expect(capture::uhd::tunedFrequencyFrom(100.0e6, -1.0e6) == 100.0e6) << "minus a million ppm has no inverse and states the read-back as it stands";

        // And the block states that value rather than the one the synthesizer was commanded.
        capture::uhd::Sink sink{};
        sink._appliedRequestHz = 1.0e9;
        sink._appliedFreqHz    = commanded;
        sink._actualGainDb     = 20.0;
        sink._deviceUp.store(true);
        expect(sink.tunedRfHz() == 1.0e9) << "the block states the frequency the caller asked for";
        expect(sink.describe().find("1000.000000 MHz") != std::string::npos) << "and the status line carries the same number";
        sink._deviceUp.store(false);

        // A frequency outside the coverage the device states is refused, and the refusal
        // leaves the request the block last accepted where it was.
        capture::uhd::Sink refused{};
        refused._freqRanges = coverage;
        refused.frequency   = std::vector<double>{7.0e9};
        expect(!refused.applyFrequencyLocked()) << "a tune outside the radio's transmit coverage is refused";
        expect(refused._appliedRequestHz == 0.0) << "and nothing is stated for a tune that never happened";
        expect(refused.frequency->front() == 7.0e9) << "a block that has reached no center leaves the request where the caller staged it";
        refused.frequency = std::vector<double>{};
        expect(!refused.applyFrequencyLocked()) << "a settings map naming no frequency is refused too";

        // The setting carries the center the tuner reached, so a caller drawing its frequency
        // axis from the setting reads the same number the reader states.
        capture::uhd::Sink outside{};
        outside._freqRanges       = coverage;
        outside._appliedRequestHz = 915.0e6; // a tune that landed earlier in the run
        outside.frequency         = std::vector<double>{7.0e9};
        expect(!outside.applyFrequencyLocked()) << "a request outside the coverage leaves the tuner where it is";
        expect(outside.frequency->front() == 915.0e6) << "and the setting goes back to the center the radio is on";
        outside._deviceUp.store(true);
        expect(outside.tunedRfHz() == 915.0e6) << "which is the center the reader states beside it";
        outside._deviceUp.store(false);

        // The value stated is written once, from the read-back. The settings map is replayed
        // with the same frequency shortly after the start, and a second write of the raw
        // request there would put the read-back's own answer back to the number typed.
        capture::uhd::Sink replayed{};
        replayed._freqRanges          = coverage;
        replayed.frequency            = std::vector<double>{1.0e9};
        replayed.frequency_correction = 100.0;
        replayed._freqWrittenHz       = commanded;       // the value the first write left in force
        replayed._appliedRequestHz    = 1'000'000'000.5; // carried out of that write's read-back
        expect(replayed.applyFrequencyLocked()) << "a replay of the value in force writes nothing";
        expect(replayed._appliedRequestHz == 1'000'000'000.5) << "and leaves the read-back's own answer where it was";
        expect(replayed.frequency->front() == 1'000'000'000.5) << "with the setting carrying the center the tuner reached";

        /*| frame: what the framework's own parameter map answers for a member this block
                writes back. A caller that draws its frequency axis from settings().get()
                rather than from the reader asks this question, and the write-back is only
                worth making if the map follows it.
            trap: the applier is driven from the test body. The block's settings hook returns
                where it holds no handle, so a sweep on a block with no device runs no applier
                at all: what a case without a radio can pin is the map's own rule, that it
                carries a copy of the members taken by a sweep rather than a read of them.
        */
        capture::uhd::Sink stored{};
        stored._freqRanges       = coverage;
        stored._appliedRequestHz = 915.0e6; // a tune that landed earlier in the run
        stored.frequency         = std::vector<double>{7.0e9};
        std::ignore              = stored.settings().applyStagedParameters();
        const auto asked = stored.settings().get("frequency");
        expect(asked.has_value()) << "a settings sweep publishes the member into the parameter map";

        expect(!stored.applyFrequencyLocked()) << "a request outside the coverage is refused";
        expect(stored.frequency->front() == 915.0e6) << "and the member carries the center the radio is on";
        const auto read = stored.settings().get("frequency");
        expect(read.has_value()) << "the parameter map still names the setting";
        const auto* const centers = read.has_value() ? read->get_if<gr::Tensor<double>>() : nullptr;
        expect(centers != nullptr) << "as a tensor of centers";
        expect(centers != nullptr && centers->size() == 1UZ) << "carrying one of them";
        expect(centers != nullptr && centers->data()[0] == 7.0e9) << "which is the value the sweep published: the map is a copy and not a read of the member";

        std::ignore                    = stored.settings().applyStagedParameters();
        const auto        republished  = stored.settings().get("frequency");
        const auto* const afterSweep   = republished.has_value() ? republished->get_if<gr::Tensor<double>>() : nullptr;
        expect(afterSweep != nullptr && afterSweep->data()[0] == 915.0e6) << "and a sweep takes that copy again, so a write-back made outside a sweep reaches the map at the next sweep";
    });

    cases("sink.uhd-every-control-it-describes-it-writes", [] {
        using capture::uhd::Sink;
        // A caller drawing the surface the block states can write every part of it. A surface
        // with no applier behind it is a set of controls a caller can move with nothing at the
        // other end.
        capture::uhd::DeviceTruth wide;
        wide.antennas = {"TX/RX", "RX2"};
        wide.antenna  = "TX/RX";
        wide.bwMinHz  = 0.2e6;
        wide.bwMaxHz  = 56.0e6;
        wide.gains.push_back({"PGA", 0.0, 89.75, 0.25, 10.0});
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};
        for (const auto& c : capture::uhd::Sink::describeControls(wide, ctx)) {
            const bool staged = c.id == capture::kSampleRateControl || c.id == capture::kFrequencyControl;
            expect(capture::uhd::Sink::controlIsWritable(wide, c.id) != staged) << "every id the surface states is one the block writes, the rate and the frequency as settings";
        }
        expect(capture::uhd::Sink::controlIsWritable(wide, "PGA")) << "the gain element keeps the name the device gave it";
        expect(!capture::uhd::Sink::controlIsWritable(wide, "ADC-digital")) << "and an element this device does not name is refused";
        expect(!capture::uhd::Sink::controlIsWritable(wide, "BIAS_TEE")) << "as is a control no sink here carries";

        // A model naming three elements offers three writable ids and no more.
        capture::uhd::DeviceTruth three;
        three.gains.push_back({"ADC-digital", 0.0, 6.0, 0.5, 0.0});
        three.gains.push_back({"ADC-fine", 0.0, 0.5, 0.05, 0.0});
        three.gains.push_back({"PGA0", 0.0, 31.5, 0.5, 0.0});
        expect(capture::uhd::Sink::controlIsWritable(three, "PGA0"));
        expect(!capture::uhd::Sink::controlIsWritable(three, "PGA")) << "the names belong to the model that is open";

        // The answer is the surface's, so a device that states neither control offers neither.
        // A caller drawing from a fixed list would offer a WBX's 40 MHz filter as a slider and
        // setControl would refuse it.
        capture::uhd::DeviceTruth fixedFilter;
        fixedFilter.antennas = {"TX/RX"};
        fixedFilter.antenna  = "TX/RX";
        fixedFilter.bwMinHz  = 40.0e6;
        fixedFilter.bwMaxHz  = 40.0e6;
        const auto fixedSurface = capture::uhd::Sink::describeControls(fixedFilter, ctx);
        expect(fixedSurface.size() == 1UZ && fixedSurface.front().id == capture::kSampleRateControl) << "a single connector and a fixed filter state no control of the device's own, the rate ladder alone";
        expect(!capture::uhd::Sink::controlIsWritable(fixedFilter, "BANDWIDTH")) << "so the fixed filter is not offered as writable";
        expect(!capture::uhd::Sink::controlIsWritable(fixedFilter, "ANTENNA")) << "nor the single connector";

        // The two answers are one set, in the direction a caller can fail on.
        const auto surface = capture::uhd::Sink::describeControls(wide, ctx);
        for (const std::string& id : {std::string("PGA"), std::string("ANTENNA"), std::string("BANDWIDTH"), std::string("BIAS_TEE")}) {
            const bool stated = std::ranges::any_of(surface, [&id](const capture::ControlDesc& c) { return c.id == id; });
            expect(capture::uhd::Sink::controlIsWritable(wide, id) == stated) << "every id one answers is an id the other answers";
        }
    });

    cases("sink.uhd-a-send-reads-the-queue-in-place", [] {
        /*| frame: a queue of 4096 whose next free slot is 3000, so the 4000 samples staged wrap
                past its end, drained by the drain loop itself on this thread over a stand-in
                that states a 363-sample frame.
        */
        capture::uhd::Sink sink{};
        capture::resizeRing(sink._ring, 4096UZ);
        sink._ring.head.store(3000UZ);
        sink._ring.tail.store(3000UZ);
        std::vector<std::complex<float>> staged(4000UZ);
        for (std::size_t i = 0UZ; i < staged.size(); ++i) {
            staged[i] = {static_cast<float>(i) / 4096.0f, -static_cast<float>(i) / 4096.0f};
        }
        expect(capture::writeWhatFits(sink._ring, staged.data(), staged.size()) == staged.size()) << "the queue takes every staged sample";
        sink._maxSendSamples = 363UZ;
        sink.publishSendCapLocked(); // no rate in force, so one frame per send
        auto stream          = std::make_shared<RecordingStream>(sink, staged.size());
        sink._stream         = stream;
        sink._drain.clearStop();
        sink.drainLoop();
        sink._stream.reset();

        expect(stream->placed == staged) << "every sample reaches the streamer once and in order";
        const auto* first = sink._ring.buf.data();
        const auto* end   = first + sink._ring.buf.size();
        expect(std::ranges::all_of(stream->sends, [&](const auto& s) { return s.at >= first && s.at + s.n <= end; })) << "every send reads the queue's own storage and none reads past its end";
        expect(std::ranges::all_of(stream->sends, [](const auto& s) { return s.n <= 363UZ; })) << "and none places more than one frame";
        expect(capture::ringFill(sink._ring) == 0UZ) << "the samples leave the queue once they are sent";
    });

    cases("sink.uhd-a-send-takes-the-span-up-to-the-cap", [] {
        using capture::uhd::Sink;
        expect(Sink::sendCapSamples(61.44e6, 2040UZ) == 301UZ * 2040UZ) << "10 ms at 61.44 MS/s in whole frames of a B205mini's 2040 samples";
        expect(Sink::sendCapSamples(61.44e6, 2040UZ) <= 614400UZ && Sink::sendCapSamples(61.44e6, 2040UZ) > 614400UZ - 2040UZ) << "within one frame of the 10 ms";
        expect(Sink::sendCapSamples(250.0e3, 2040UZ) == 2040UZ) << "and never below one frame where 10 ms is less";
        expect(Sink::sendCapSamples(0.0, 363UZ) == 363UZ) << "one frame where no rate is in force";
        expect(Sink::sendCapSamples(1.0e6, 0UZ) == 10000UZ) << "and 10 ms alone where the streamer states no frame";

        /*| frame: a queue of 65536 at 1 MS/s over a stand-in stating a 363-sample frame, so the cap
                is 27 frames, 9801 samples. The 30000 samples staged start 5536 samples before the
                queue's end, and the drain runs on this thread until the stand-in holds them all.
        */
        Sink sink{};
        capture::resizeRing(sink._ring, 65536UZ);
        sink._ring.head.store(60000UZ);
        sink._ring.tail.store(60000UZ);
        std::vector<std::complex<float>> staged(30000UZ);
        for (std::size_t i = 0UZ; i < staged.size(); ++i) {
            staged[i] = {static_cast<float>(i % 1000UZ) / 1000.0f, 0.0f};
        }
        expect(sink.stage(staged.data(), staged.size()) == staged.size()) << "the queue takes every staged sample";
        sink._maxSendSamples = 363UZ;
        sink._rateActualHz   = 1.0e6;
        sink.publishSendCapLocked();
        auto stream  = std::make_shared<RecordingStream>(sink, staged.size());
        sink._stream = stream;
        sink._drain.clearStop();
        sink.drainLoop();
        sink._stream.reset();

        expect(stream->placed == staged) << "every sample reaches the streamer once and in order";
        expect(stream->sends.size() == 4UZ) << "in four sends rather than one per frame";
        expect(stream->sends.size() == 4UZ && stream->sends[0].n == 5536UZ) << "the first takes the whole span up to the queue's end";
        expect(stream->sends.size() == 4UZ && stream->sends[1].n == 9801UZ && stream->sends[2].n == 9801UZ) << "the next ones take the cap";
        expect(stream->sends.size() == 4UZ && stream->sends[3].n == 30000UZ - 5536UZ - 2UZ * 9801UZ) << "and the last takes what is left";
    });

    cases("sink.uhd-a-write-wakes-the-waiting-drain", [] {
        using namespace std::chrono_literals;
        /*| frame: the block's own drain loop on its own thread over a stand-in that counts what
                it places. The case reads the bell's flag to see the drain waiting and then
                stages samples as the work call does.
        */
        capture::uhd::Sink sink{};
        capture::resizeRing(sink._ring, 4096UZ);
        sink._maxSendSamples = 363UZ;
        sink._rateActualHz   = 1.0e6;
        sink.publishSendCapLocked();
        auto stream  = std::make_shared<CountingStream>();
        sink._stream = stream;
        sink._drain.start([&sink] { sink.drainLoop(); });

        const auto waited = [&sink](std::chrono::milliseconds limit) {
            const auto deadline = std::chrono::steady_clock::now() + limit;
            while (!sink._drainBell.waiting.load() && std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(1ms);
            }
            return sink._drainBell.waiting.load();
        };
        expect(waited(2000ms)) << "a drain with nothing queued waits on the bell";
        const std::uint32_t before = sink._drainBell.epoch.load();
        std::this_thread::sleep_for(20ms);
        expect(sink._drainBell.epoch.load() == before && sink._drainBell.waiting.load()) << "and stays there while nothing rings";

        const std::vector<std::complex<float>> staged(1000UZ, std::complex<float>{0.25f, 0.0f});
        expect(sink.stage(staged.data(), staged.size()) == staged.size()) << "a work call stages samples";
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (stream->samples.load() < staged.size() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
        }
        expect(stream->samples.load() == staged.size()) << "and the drain wakes and sends them";
        expect(waited(2000ms)) << "then waits on the bell again";

        sink.pause();
        const auto parkDeadline = std::chrono::steady_clock::now() + 2s;
        while (!sink._drainParked.load() && std::chrono::steady_clock::now() < parkDeadline) {
            std::this_thread::sleep_for(1ms);
        }
        expect(sink._drainParked.load()) << "a pause wakes the waiting drain, which parks";
        sink.resume();

        sink._drain.requestStop();
        sink.wakeDrain();
        auto joined = std::async(std::launch::async, [&sink] { sink._drain.joinIfRunning(); });
        expect(joined.wait_for(2s) == std::future_status::ready) << "a stop request and a ring end the drain";
        sink._stream.reset();
    });

    cases("sink.uhd-a-rate-change-reuses-the-queue-that-fits", [] {
        using capture::uhd::Sink;
        Sink sink{};
        sink._maxSendSamples = 2040UZ;
        sink._rateActualHz   = 8.0e6;
        sink.resizeQueueForRateLocked(true);
        const auto* const at8 = sink._ring.buf.data();
        expect(sink._ring.buf.size() == (1UZ << 21)) << "a quarter second at 8 MS/s rounds up to 2^21 samples";

        sink._rateActualHz = 6.0e6;
        expect(sink.queueStorageFor(sink.ringWantSamplesLocked()).empty()) << "6 MS/s asks for the same size, so nothing is allocated ahead of the mutex";
        sink.resizeQueueForRateLocked(true);
        expect(sink._ring.buf.data() == at8) << "and the rate change keeps the storage it has";

        auto ahead = sink.queueStorageFor(capture::ringSamplesFor(20.0e6, Sink::ringFloorSamples(2040UZ)));
        expect(ahead.size() == (1UZ << 23)) << "20 MS/s asks for 2^23 samples, allocated before the mutex is taken";
        const auto* const given = ahead.data();
        sink._queueSpare.swap(ahead);
        sink._rateActualHz = 20.0e6;
        sink.resizeQueueForRateLocked(true);
        expect(sink._ring.buf.data() == given) << "the rate change takes that storage";
        expect(sink._queueSpare.data() == at8) << "and hands back the storage it replaced for a free outside the mutex";
        expect(sink._ringCapacity.load() == (1UZ << 23)) << "with the capacity published beside it";
    });

    cases("sink.uhd-the-queue-follows-its-settings", [] {
        using capture::uhd::Sink;
        Sink sink{};
        sink._maxSendSamples = 2040UZ;
        sink._rateActualHz   = 61.44e6;
        sink.takeQueueBoundLocked();
        expect(sink.ringWantSamplesLocked() == (1UZ << 24)) << "the default settings hold a quarter second at 61.44 MS/s";
        sink._rateActualHz = 2.0e9;
        expect(sink.ringWantSamplesLocked() * sizeof(std::complex<float>) == (4UZ << 30)) << "and a quarter second of CF32 at 2 GS/s, 4 GiB";

        sink.queue_duration  = 0.05;
        sink.queue_max_bytes = 64.0 * 1024.0 * 1024.0;
        expect(sink.ringWantSamplesLocked() * sizeof(std::complex<float>) == (4UZ << 30)) << "a setting reaches the queue at the next sizing and not before";
        sink.takeQueueBoundLocked();
        expect(sink.ringWantSamplesLocked() * sizeof(std::complex<float>) == (64UZ << 20)) << "the byte ceiling caps the queue at 2 GS/s";
        sink._rateActualHz = 8.0e6;
        expect(sink.ringWantSamplesLocked() == (1UZ << 19)) << "and the duration sets it where the ceiling does not bind";
        expect(Sink::playOutBoundMs(1UZ << 24, 8.0e6, 2040UZ, sink._queueBound) == capture::playOutMs(1UZ << 19, 8.0e6)) << "every wait is sized from the queue the settings give";
    });

    cases("sink.uhd-the-stop-bound-reads-the-stated-buffer", [] {
        using capture::uhd::Sink;
        expect(Sink::deviceHeldBytesFor(::uhd::device_addr_t("serial=30D3AB9,type=b200")) == Sink::kDeviceHeldBytes) << "an address that states no send buffer takes 1 MiB";
        expect(Sink::deviceHeldBytesFor(::uhd::device_addr_t("addr=192.168.10.3,send_buff_size=4194304")) == 4194304UZ) << "an address that states one takes it";
        expect(Sink::deviceHeldBytesFor(::uhd::device_addr_t("send_buff_size=0")) == Sink::kDeviceHeldBytes) << "a stated zero takes 1 MiB";
        expect(Sink::deviceHeldBytesFor(::uhd::device_addr_t("send_buff_size=lots")) == Sink::kDeviceHeldBytes) << "and so does a value that is not a number";

        Sink stated{};
        stated.send_buff_size = 4194304.0;
        expect(Sink::deviceHeldBytesFor(stated.openAddress()) == 4194304UZ) << "the block's own send_buff_size reaches the address the bound reads";

        constexpr std::size_t kHeld = 4UZ << 20;
        expect(Sink::burstAckBoundMs(250.0e3, "sc16", kHeld) == capture::playOutMs(kHeld / 4UZ, 250.0e3)) << "the wait is the play-out of the stated buffer";
        expect(Sink::burstAckBoundMs(250.0e3, "sc16", kHeld) > Sink::burstAckBoundMs(250.0e3, "sc16")) << "longer than the 1 MiB a device stating none is given";

        const auto stream = std::make_shared<StreamStandIn>();
        Sink       unconfirmed{};
        unconfirmed._bursting.store(true);
        unconfirmed.endBurstAtStop(stream, 61.44e6, "sc16", kHeld);
        expect(unconfirmed.stopBurstEnd() == Sink::BurstEnd::Unconfirmed) << "a stop whose acknowledgment never came";
        expect(unconfirmed.unconfirmedByFlushBoundSamples() == kHeld / 4UZ) << "states the stated buffer as what the device can still hold";
    });

    cases("sink.uhd-a-work-call-clips-into-the-queue", [] {
        capture::SpscRing<std::complex<float>> ring(8UZ);
        ring.head.store(6UZ);
        ring.tail.store(6UZ);
        const std::vector<std::complex<float>> in{{0.5f, -0.5f}, {1.5f, 0.0f}, {0.0f, -2.0f}, {0.25f, 0.25f}, {-3.0f, 4.0f}};
        const auto clip = [](std::complex<float> s) { return capture::clipUnit(s); };
        expect(capture::writeWhatFitsWith(ring, in.data(), in.size(), clip) == in.size()) << "what fits goes in";
        for (std::size_t i = 0UZ; i < in.size(); ++i) {
            expect(ring.at(6UZ + i) == capture::clipUnit(in[i])) << "clipped on its way in, across the wrap, sample" << std::to_string(i);
        }
        expect(capture::writeWhatFitsWith(ring, in.data(), in.size(), clip) == 3UZ) << "a fuller queue takes what fits and no more";
        expect(capture::writeWhatFitsWith(ring, in.data(), in.size(), clip) == 0UZ && ring.dropped.load() == 0UL) << "a full one takes nothing and drops nothing";
    });

    cases("sink.uhd-the-buffers-outlive-the-threads", [] {
        // Members are destroyed in reverse declaration order, so a buffer declared after a
        // thread member is freed before that member's destructor joins the thread. The drain
        // sends from the ring on every pass, and a teardown that raised — which the guard
        // swallows — leaves it reading storage that has gone.
        capture::uhd::Sink  sink{};
        const auto        at     = [](const void* p) { return reinterpret_cast<const char*>(p); };
        const std::less<const char*> before;
        expect(before(at(&sink._ring), at(&sink._drain))) << "the queue is declared before the drain that reads it";
        expect(before(at(&sink._ring), at(&sink._async))) << "both threads are declared after every buffer";
        expect(before(at(&sink._drain), at(&sink._teardown))) << "and the teardown guard is last of all, so it runs first";
        expect(before(at(&sink._async), at(&sink._teardown)));
    });

    cases("sink.uhd-a-queue-reading-reads-no-storage", [] {
        // Three threads take this reading and a rate change replaces the ring's storage under
        // them, so a reader that asked the vector how large it is would ask it while it is
        // being reassigned. The capacity is published beside the ring instead.
        capture::SpscRing<std::complex<float>>  ring(16UZ);
        const std::vector<std::complex<float>> staged(10, std::complex<float>{0.5f, 0.5f});
        expect(capture::writeWhatFits(ring, staged.data(), staged.size()) == 10UZ);
        expect(capture::ringFillBounded(ring, 16UZ) == 10UZ) << "an ordinary reading is what the ring holds";
        ring.advanceTo(20UZ); // a consumer that has passed the head this reading would see
        expect(capture::ringFillBounded(ring, 16UZ) == 16UZ) << "and no reading exceeds the capacity it was given";
        expect(capture::ringFillBounded(ring, 16UZ) == capture::ringFill(ring)) << "which is the bound the ring's own size gives a caller that owns it";

        capture::uhd::Sink sink{};
        sink._deviceUp.store(true);
        expect(sink._ringCapacity.load() == sink._ring.buf.size()) << "the block publishes the capacity its queue was built with";
        expect(sink.queuedSamples() == 0UL) << "an empty queue reads empty";
        const std::vector<std::complex<float>> more(64, std::complex<float>{0.25f, 0.25f});
        expect(capture::writeWhatFits(sink._ring, more.data(), more.size()) == 64UZ);
        expect(sink.queuedSamples() == 64UL) << "and a staged queue reads what it holds";
        sink._deviceUp.store(false);
    });

    cases("sink.uhd-the-queue-follows-the-rate", [] {
        using capture::uhd::Sink;
        // Every wait this block makes is sized from the queue, and the queue is a quarter of a
        // second at the rate in force. A floor of 2^19 samples is that quarter second at
        // 2 MS/s and 2.62 s at the 250 kS/s a B205mini also runs, and a rate change and a
        // stop would then cost that.
        constexpr std::size_t kUsbSend = 2040UZ; // a B205mini's streamer
        constexpr std::size_t kNetSend = 363UZ;  // an N210's
        expect(capture::uhd::Sink::ringFloorSamples(kUsbSend) == 8UZ * kUsbSend) << "the floor is eight of the streamer's own maximum send";
        expect(capture::uhd::Sink::ringFloorSamples(kNetSend) == 4096UZ) << "a streamer whose send is small takes the block's own least queue";
        expect(capture::uhd::Sink::ringFloorSamples(0UZ) == 4096UZ) << "as does one that states no maximum at all";

        // The floor binds at and below four times the floor, whatever the maximum send. Stated
        // against the send instead, the rule holds only where eight sends reach 4096 samples.
        expect(capture::ringSamplesFor(65280.0, capture::uhd::Sink::ringFloorSamples(kUsbSend)) == 16384UZ) << "a B205mini's floor binds at 65280 S/s, four times itself";
        expect(capture::ringSamplesFor(65280.0 * 1.01, capture::uhd::Sink::ringFloorSamples(kUsbSend)) == 32768UZ) << "and the rate's own quarter second binds above it";
        expect(4UZ * capture::uhd::Sink::ringFloorSamples(kNetSend) == 16384UZ) << "an N210's floor is the least queue, so its crossing is 16384 S/s";
        expect(capture::ringSamplesFor(16384.0, capture::uhd::Sink::ringFloorSamples(kNetSend)) == 4096UZ) << "which the floor binds at";
        expect(capture::ringSamplesFor(16384.0 * 1.01, capture::uhd::Sink::ringFloorSamples(kNetSend)) == 8192UZ) << "and not above";
        expect(32UZ * kNetSend == 11616UZ) << "where 32 times the send names 11616 S/s, below the crossing the floor makes";

        bool everyRateIsQuick       = true;
        bool everyRateCoversThePark = true;
        for (const double rate : {195312.5, 250000.0, 1.0e6, 2.0e6, 2.1e6, 8.0e6, 20.0e6, 61.44e6}) {
            const std::size_t queue  = capture::ringSamplesFor(rate, capture::uhd::Sink::ringFloorSamples(kUsbSend));
            const double      heldMs = 1000.0 * static_cast<double>(queue) / rate;
            if (heldMs > 800.0) {
                everyRateIsQuick = false;
            }
            if (heldMs < 100.0) {
                everyRateCoversThePark = false;
            }
        }
        expect(everyRateIsQuick) << "no rate these radios run holds more than eight hundred milliseconds of queue";
        expect(everyRateCoversThePark) << "and every one of them holds more than the scheduler's hundred-millisecond park";

        const std::size_t slow = capture::ringSamplesFor(250000.0, capture::uhd::Sink::ringFloorSamples(kUsbSend));
        expect(1000.0 * static_cast<double>(slow) / 250000.0 < 400.0) << "a quarter of a second at 250 kS/s rather than the 2.6 s a fixed floor holds";
        expect(capture::ringSamplesFor(250000.0) > slow) << "which is the floor a radio running no slower than 2 MS/s keeps";
        expect(capture::playOutMs(slow, 250000.0).count() < 800) << "so the play-out a rate change waits for stays inside eight hundred milliseconds";
    });

    cases("sink.uhd-a-stop-that-took-the-handle-calls-no-library", [] {
        // A start reads the lifecycle state and then takes the control mutex, and a stop that
        // lands between the two tears the device down under that same mutex. Every body the
        // start then runs would hand a released handle to libuhd.
        capture::uhd::Sink sink{};
        sink._freqRanges = std::vector<std::pair<double, double>>{{49.0e6, 6001.0e6}};
        sink.frequency   = std::vector<double>{915.0e6};
        sink.tx_gains    = std::vector<double>{20.0};
        sink.tx_antennae = std::vector<std::string>{"TX/RX"};
        expect(sink._usrp == nullptr) << "a block that opened nothing holds no handle";

        sink.applyReferenceSourcesLocked();
        sink.applyRateLocked();
        sink.applyGainLocked();
        sink.applyAntennaLocked();
        sink.applyBandwidthLocked(2.0e6);
        sink.refreshFreqRangesLocked();
        expect(!sink.applyFrequencyLocked()) << "a tune with no handle behind it is refused";
        expect(sink._rateActualHz == 0.0) << "and no applier states a value it never wrote";
        expect(sink._actualAntenna.empty());
        expect(sink._bwCurHz == 0.0);
    });

    cases("sink.uhd-a-rate-change-counts-what-it-discards", [] {
        // The play-out before a rate change is bounded, so a device that stopped consuming
        // leaves samples in the queue when the bound expires. The resize behind it hands both
        // ends fresh storage, and those samples reach no air at either rate.
        capture::SpscRing<std::complex<float>>  ring(64UZ);
        const std::vector<std::complex<float>> staged(40, std::complex<float>{0.5f, 0.5f});
        expect(capture::writeWhatFits(ring, staged.data(), staged.size()) == 40UZ) << "the queue holds what the play-out could not place";
        expect(capture::resizeRingCountingDiscard(ring, 128UZ) == 40UZ) << "and the resize answers how many it threw away";
        expect(capture::ringFill(ring) == 0UZ) << "leaving the queue empty";
        expect(ring.buf.size() == 128UZ) << "at the capacity the new rate asked for";
        expect(capture::resizeRingCountingDiscard(ring, 64UZ) == 0UZ) << "an empty queue discards nothing";

        // The block states that count beside the one a pause keeps, which stays zero on this
        // family: a USRP pause ends the burst and holds its queue.
        capture::uhd::Sink sink{};
        expect(sink.discardedAtRateChangeSamples() == 0UL) << "a block that has changed no rate has discarded nothing";
        sink._counters.discardedAtRateChange.fetch_add(40UL);
        expect(sink.discardedAtRateChangeSamples() == 40UL) << "the counter the applier adds to is the one the reader states";
        expect(sink.discardedAtHoldSamples() == 0UL) << "and a pause on this family throws nothing away";
        sink._counters.reset();
        expect(sink.discardedAtRateChangeSamples() == 0UL) << "the start of a run clears it";
    });

    cases("sink.uhd-a-held-stop-ends-at-once", [] {
        // The drain parks on the hold and takes nothing out of the ring, so a wait for the
        // queue to reach the air cannot end by the queue emptying: it would run its whole
        // deadline and leave the queue exactly where it was. A stop under a hold is a discard,
        // and the samples it leaves are counted as unsent.
        capture::uhd::Sink                       sink{};
        const std::vector<std::complex<float>> staged(4096, std::complex<float>{0.25f, -0.25f});
        expect(capture::writeWhatFits(sink._ring, staged.data(), staged.size()) == staged.size()) << "the queue holds what the work calls staged";
        expect(capture::playOutMs(staged.size(), 1.0e6).count() >= 250) << "and the deadline a wait for it would take is a quarter second at least";

        sink._held.store(true);
        const auto began = std::chrono::steady_clock::now();
        sink.playOutAt(1.0e6, 2040UZ);
        const auto heldMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - began).count();
        expect(heldMs < 50) << "a stop made while the carrier is held waits for nothing";
        expect(capture::ringFill(sink._ring) == staged.size()) << "and leaves the queue for the stop to count as unsent";

        sink._held.store(false);
        const auto resumed = std::chrono::steady_clock::now();
        sink.playOutAt(1.0e6, 2040UZ);
        const auto runningMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - resumed).count();
        expect(runningMs >= 250) << "while a running carrier is given the queue's own time to reach the air";
    });

    cases("sink.uhd-a-parked-drain-ends-the-burst-once", [] {
        using capture::uhd::Sink;
        // The end of burst is a send, and against a device that stopped consuming it runs the
        // whole send timeout and leaves the carrier up. Attempted on every pass of the parked
        // loop, a pause against such a device costs one attempt a second for as long as it
        // lasts, and a stop then joins behind the attempt in flight.
        expect(capture::uhd::Sink::endBurstOnThisPass(false)) << "the pass that parks the drain ends the burst";
        expect(!capture::uhd::Sink::endBurstOnThisPass(true)) << "and the passes behind it do not attempt it again";

        // A rate change clears the flag before it raises the break, so its own park is a fresh
        // attempt rather than the one a pause already made.
        capture::uhd::Sink sink{};
        sink._held.store(true);
        sink._drainParked.store(true);
        sink.parkDrainForRateChange();
        expect(capture::uhd::Sink::endBurstOnThisPass(sink._drainParked.load())) << "so a rate change behind a pause ends the burst again";
        sink._rateBreak.store(false);
        sink._held.store(false);
    });

    cases("sink.uhd-a-rate-change-waits-for-a-fresh-park", [] {
        // The drain raises the parked flag for a pause as well as for a rate change. A flag
        // left standing by a pause would answer at once for a drain that took the resume and
        // is already back in the copy, and the resize behind that answer replaces the storage
        // the drain is reading.
        capture::uhd::Sink sink{};
        sink._held.store(true);
        sink._drainParked.store(true); // what a pause leaves behind
        expect(sink.parkDrainForRateChange()) << "a block with no drain running has no consumer to stand off";
        expect(sink._rateBreak.load()) << "and the drain is asked all the same";
        expect(!sink._drainParked.load()) << "the answer waited for being a fresh one rather than the pause's";
        sink._rateBreak.store(false);
        sink._held.store(false);

        /*| frame: the drain below is a thread of the block's own drain member, shaped like the
                real loop's park branch: it watches the break and answers it late. A case that
                starts no thread never reaches the wait at all.
        */
        capture::uhd::Sink answering{};
        answering._drainParked.store(true); // again, what a pause leaves behind
        answering._drain.start([&answering] {
            while (!answering._drain.stopRequested()) {
                if (answering._rateBreak.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(15));
                    answering._drainParked.store(true);
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        const auto began  = std::chrono::steady_clock::now();
        const bool parked = answering.parkDrainForRateChange(std::chrono::milliseconds(2000));
        const auto tookMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - began).count();
        answering._drain.requestStop();
        answering._drain.joinIfRunning();
        expect(parked) << "a drain that stands off the ring answers the park";
        expect(tookMs >= 10) << "and the wait waits for that answer rather than taking the flag a pause left";
        answering._rateBreak.store(false);
    });

    cases("sink.uhd-a-resize-down-gives-the-storage-back", [] {
        using capture::uhd::Sink;
        // vector::assign keeps the capacity it already holds when the new count is smaller, so
        // a block that ran at its top rate would hold that peak for the life of the block.
        capture::SpscRing<std::complex<float>> ring(1024UZ);
        const auto* const small = ring.buf.data();
        capture::resizeRing(ring, 65536UZ);
        expect(ring.buf.size() == 65536UZ) << "a resize up gives the ring the room the rate asks for";
        expect(ring.buf.data() != small) << "in storage of its own";
        const auto* const large = ring.buf.data();

        capture::resizeRing(ring, 4096UZ);
        expect(ring.buf.size() == 4096UZ) << "a resize down sizes it for the new rate";
        expect(ring.buf.data() != large) << "in fresh storage, the block the old rate held going back with the vector it was swapped out of";
        expect(capture::ringFill(ring) == 0UZ) << "with both ends at the start of it";

        // What the queue costs at the rates the family runs.
        constexpr std::size_t kMiB = 1024UZ * 1024UZ;
        expect(capture::ringSamplesFor(61.44e6, capture::uhd::Sink::ringFloorSamples(2040UZ)) * sizeof(std::complex<float>) == 128UZ * kMiB) << "a USRP queue at 61.44 MS/s is 128 MiB of CF32";
        expect(capture::ringSamplesFor(2.0e6, capture::uhd::Sink::ringFloorSamples(2040UZ)) * sizeof(std::complex<float>) == 4UZ * kMiB) << "and 4 MiB at 2 MS/s";
    });

    cases("sink.uhd-a-rate-change-before-the-first-send-resizes-the-queue", [] {
        using capture::uhd::Sink;
        // A burst opens on the first send, so a rate staged before any samples flow has no
        // burst to end. Gated on that, the resize is skipped and the queue keeps the size the
        // previous rate gave it: 65536 samples after a start at 250 kS/s, which at 61.44 MS/s
        // is 1.07 ms of queue against the scheduler's hundred-millisecond park.
        const auto beforeFirstSend = capture::rateChangeFor(61.44e6, 250000.0, false);
        expect(beforeFirstSend.write) << "the new rate reaches the device";
        expect(!beforeFirstSend.endBurst) << "with no burst in flight to end";
        expect(capture::ringSamplesFor(250000.0, capture::uhd::Sink::ringFloorSamples(2040UZ)) == 65536UZ) << "the queue a start at 250 kS/s leaves behind";
        expect(capture::ringSamplesFor(61.44e6, capture::uhd::Sink::ringFloorSamples(2040UZ)) == 16777216UZ) << "against the queue the new rate asks for";
        expect(1000.0 * 65536.0 / 61.44e6 < 1.1) << "which is the 1.07 ms a queue left at the old size holds";
        expect(1000.0 * 65536.0 / 61.44e6 < 100.0) << "far inside the park a scheduler takes on the progress counter";

        // The block sizes the queue from the rate the device took, with no burst open.
        capture::uhd::Sink sink{};
        sink._maxSendSamples = 2040UZ;
        sink._rateActualHz   = 250000.0;
        sink.resizeQueueForRateLocked(true);
        expect(!sink._bursting.load()) << "nothing has been sent, so no burst is open";
        expect(sink._ring.buf.size() == 65536UZ) << "and the queue is the one the start's rate asked for";

        sink._rateActualHz = 2.0e6;
        sink.resizeQueueForRateLocked(true);
        expect(sink._ring.buf.size() == 524288UZ) << "a rate change before the first send sizes the queue for the new rate all the same";
        expect(sink._ringCapacity.load() == sink._ring.buf.size()) << "with the capacity published beside it";
        expect(1000.0 * static_cast<double>(sink._ring.buf.size()) / 2.0e6 > 100.0) << "so the queue holds more than the scheduler's park at the rate in force";
    });

    cases("sink.uhd-a-refused-park-keeps-the-queue", [] {
        using capture::uhd::Sink;
        // The resize hands both ends of the queue fresh storage, and a drain that has not
        // stood off the ring reads that storage between its own availability test and the two
        // copies out of it. A queue kept at the previous rate's length is a wrong length of
        // time; a queue reassigned under a live consumer is a fault.
        capture::uhd::Sink sink{};
        sink._rateActualHz                     = 2.0e6;
        sink._maxSendSamples                   = 2040UZ;
        const std::vector<std::complex<float>> staged(4096, std::complex<float>{0.25f, -0.25f});
        expect(capture::writeWhatFits(sink._ring, staged.data(), staged.size()) == staged.size()) << "the queue holds what the play-out could not place";
        const auto* const before = sink._ring.buf.data();
        const std::size_t was    = sink._ring.buf.size();

        sink.resizeQueueForRateLocked(false);
        expect(sink._ring.buf.data() == before) << "a refused park leaves the storage where it is";
        expect(sink._ring.buf.size() == was) << "at the size it had";
        expect(capture::ringFill(sink._ring) == staged.size()) << "with the queue it was holding";
        expect(sink.discardedAtRateChangeSamples() == 0UL) << "and nothing counted as thrown away";

        sink.resizeQueueForRateLocked(true);
        expect(sink._ring.buf.size() == capture::ringSamplesFor(2.0e6, capture::uhd::Sink::ringFloorSamples(2040UZ))) << "a park that arrived sizes the queue for the rate the device took";
        expect(capture::ringFill(sink._ring) == 0UZ) << "leaving it empty";
        expect(sink.discardedAtRateChangeSamples() == staged.size()) << "and counting what it threw away";

        // A live drain that never answers is refused inside its own bound.
        capture::uhd::Sink live{};
        live._drain.start([&live] {
            while (!live._drain.stopRequested()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        const auto began  = std::chrono::steady_clock::now();
        const bool parked = live.parkDrainForRateChange(std::chrono::milliseconds(20));
        const auto tookMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - began).count();
        live._drain.requestStop();
        live._drain.joinIfRunning();
        expect(!parked) << "a drain that does not stand off the ring inside the bound refuses the park";
        expect(tookMs >= 20) << "the wait spending the whole of that bound";
        expect(live._rateBreak.load()) << "with the break left for the applier to clear";
        live._rateBreak.store(false);
    });

    cases("sink.uhd-a-play-out-is-bounded-by-the-rate-in-force", [] {
        using capture::uhd::Sink;
        // A refused park keeps the queue's sample count and not its length of time, so a block
        // taken from 61.44 MS/s to 250 kS/s holds 16777216 samples at a rate that plays them
        // out over 67 s. Sized from the count alone the next stop would hold the control mutex
        // and the teardown mutex for the whole of that while every status reader answers empty.
        constexpr std::size_t kFastQueue = 16777216UZ;
        expect(capture::ringSamplesFor(61.44e6, capture::uhd::Sink::ringFloorSamples(2040UZ)) == kFastQueue) << "the queue 61.44 MS/s asks for";
        expect(capture::playOutMs(kFastQueue, 250000.0).count() == 67358) << "which at 250 kS/s is a deadline of 67 s sized from the count alone";

        const auto bounded = capture::uhd::Sink::playOutBoundMs(kFastQueue, 250000.0, 2040UZ);
        expect(bounded.count() < 1000) << "the bound the rate in force gives is under a second";
        expect(bounded == capture::playOutMs(capture::ringSamplesFor(250000.0, capture::uhd::Sink::ringFloorSamples(2040UZ)), 250000.0)) << "being the capacity that rate asks for";

        // A queue no larger than the rate in force asks for is waited out whole.
        expect(capture::uhd::Sink::playOutBoundMs(65536UZ, 250000.0, 2040UZ) == capture::playOutMs(65536UL, 250000.0)) << "a queue sized for the rate in force is not shortened";
        expect(capture::uhd::Sink::playOutBoundMs(1000UZ, 2.0e6, 2040UZ) == capture::playOutMs(1000UL, 2.0e6)) << "nor is a queue below it";
        for (const double rate : {250000.0, 2.0e6, 20.0e6, 61.44e6}) {
            expect(capture::uhd::Sink::playOutBoundMs(kFastQueue, rate, 2040UZ).count() < 1000) << "no rate this family runs names a deadline of a second";
        }

        // The block itself, through the sequence that leaves the queue oversized.
        capture::uhd::Sink sink{};
        sink._maxSendSamples = 2040UZ;
        sink._rateActualHz   = 61.44e6;
        sink.resizeQueueForRateLocked(true);
        expect(sink._ring.buf.size() == kFastQueue) << "the fast rate sizes the queue for itself";

        sink._rateActualHz = 250000.0;
        sink.resizeQueueForRateLocked(false);
        expect(sink._ring.buf.size() == kFastQueue) << "and a refused park keeps that sample count";
        expect(capture::uhd::Sink::playOutBoundMs(sink._ringCapacity.load(), sink._rateActualHz, sink._maxSendSamples).count() < 1000) << "while the wait sized from it stays under a second";

        sink._rateActualHz = 250000.0;
        sink.resizeQueueForRateLocked(true);
        expect(sink._ring.buf.size() == 65536UZ) << "and the next park that arrives gives the storage back";
    });

    cases("sink.uhd-a-refused-park-is-retried-until-it-lands", [] {
        using capture::uhd::Sink;
        // Nothing asks the rate applier for the resize a second time: a rate already written is
        // not written again. Left at that, a block taken from 61.44 MS/s to 250 kS/s with the
        // park refused holds 16777216 samples for the rest of the run at that rate, 67 s of
        // transmit latency, and the stop behind it abandons up to 16711680 of them.
        constexpr std::size_t kFastQueue = 16777216UZ;
        constexpr std::size_t kSlowQueue = 65536UZ;

        capture::uhd::Sink sink{};
        sink._maxSendSamples = 2040UZ;
        sink._rateActualHz   = 61.44e6;
        sink.resizeQueueForRateLocked(true);
        expect(sink._ring.buf.size() == kFastQueue) << "the fast rate sizes the queue for itself";
        expect(!sink.queueOutsizesRate()) << "and a queue sized at the rate it runs at asks for no resize";
        expect(!sink.askParkForOwedResize()) << "so the drain asks for nothing at the head of its loop";

        const std::vector<std::complex<float>> staged(4096, std::complex<float>{0.25f, -0.25f});
        expect(capture::writeWhatFits(sink._ring, staged.data(), staged.size()) == staged.size()) << "the queue holds what the play-out could not place";

        sink._rateActualHz = 250000.0;
        sink.resizeQueueForRateLocked(false);
        expect(sink._ring.buf.size() == kFastQueue) << "a refused park keeps that sample count";
        expect(sink.queueOutsizesRate()) << "which is a queue holding more storage than the rate in force asks for";
        expect(sink.discardedAtRateChangeSamples() == 0UL) << "with nothing counted as thrown away yet";

        // The work call takes nothing while the drain still has the ring: the storage it would
        // replace is the storage that drain indexes.
        sink._deviceUp.store(true);
        sink.takeOwedQueueResize();
        expect(sink._ring.buf.size() == kFastQueue) << "a work call takes no resize while the drain is on the ring";
        expect(capture::ringFill(sink._ring) == staged.size()) << "and the queue stays where the drain left it";

        // The drain asks for the park again at the head of its own loop, and the pass behind
        // that ask ends the burst and stands off the ring.
        expect(sink.askParkForOwedResize()) << "the drain asks for the park again";
        expect(sink._rateBreak.load()) << "through the same flag a rate change raises";
        sink._drainParked.store(true); // the pass behind the ask

        sink.takeOwedQueueResize();
        expect(sink._ring.buf.size() == kSlowQueue) << "so the work call gives the storage back at the rate in force";
        expect(sink._ringCapacity.load() == kSlowQueue) << "with the capacity published beside it";
        expect(sink.discardedAtRateChangeSamples() == staged.size()) << "and what the queue still held counted under the rate change";
        expect(!sink.queueOutsizesRate()) << "leaving nothing owed";
        expect(!sink._rateBreak.load()) << "and the drain free to take the ring again";
        expect(!sink.askParkForOwedResize()) << "which it is not asked to give up a second time";

        sink.takeOwedQueueResize();
        expect(sink.discardedAtRateChangeSamples() == staged.size()) << "a later work call takes nothing and counts nothing";
        expect(capture::uhd::Sink::playOutBoundMs(sink._ringCapacity.load(), sink._rateActualHz, sink._maxSendSamples) == capture::playOutMs(kSlowQueue, 250000.0)) << "and every wait behind it is the rate's own quarter second";
        sink._deviceUp.store(false);
        sink._drainParked.store(false);
    });

    cases("sink.uhd-a-rate-change-does-not-wait-on-a-send", [] {
        // The rate applier runs on the block's own worker thread, and every other block in the
        // job waits behind it. It hands the burst end to the drain through a flag rather than
        // taking the send mutex, which a device that stopped consuming would hold for the
        // whole send timeout.
        capture::uhd::Sink sink{};
        const auto       began = std::chrono::steady_clock::now();
        sink.parkDrainForRateChange();
        const auto took = std::chrono::steady_clock::now() - began;
        expect(sink._rateBreak.load()) << "the drain is asked to stand off the ring";
        expect(std::chrono::duration_cast<std::chrono::milliseconds>(took).count() < 250) << "and a block with no drain running answers at once";
        expect(std::chrono::duration_cast<std::chrono::milliseconds>(took).count() < static_cast<long>(capture::uhd::Sink::kSendTimeoutS * 1000.0))
            << "far inside the send timeout the old path could wait";
        sink._rateBreak.store(false);
    });

    cases("sink.uhd-a-stop-request-waits-for-the-control-mutex", [] {
        /*| frame: a stop request runs the block's own stop on the thread that makes it, and
                that stop tears the hardware down, which takes the control mutex. The case holds
                the mutex on its own thread and makes the request on a second one, so the wait
                is read rather than entered.
        */
        capture::uhd::Sink sink{};
        std::unique_lock   held(sink._ctrlMutex);

        std::atomic<bool> ended{false};
        std::thread       asker([&sink, &ended] {
            sink.requestStop();
            ended.store(true, std::memory_order_release);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        expect(!ended.load(std::memory_order_acquire)) << "the stop behind the request waits for the control mutex";
        held.unlock();
        asker.join();
        expect(ended.load(std::memory_order_acquire)) << "and takes it once the holder gives it back";
        expect(sink.state() == gr::lifecycle::State::REQUESTED_STOP) << "leaving the block asked to stop";

        // No body of this block therefore requests a stop while it holds that mutex. Such a
        // request would wait on the thread making it.
        expect(sink._ctrlMutex.try_lock()) << "the mutex is free again once the stop has run";
        sink._ctrlMutex.unlock();
    });

    cases("sink.uhd-a-start-that-cannot-parse-its-address-returns", [] {
        /*| frame: the start parses its address inside the same try that opens the device, and
                the parser refuses a pair with two separators before any device is asked for. An
                address like that drives the start into the catch a failed open takes, with no
                device and no enumeration, so the case enters the catch rather than reading the
                rule. The block lives on the heap and is kept where the start does not return,
                the thread still holding it.
        */
        auto* sink             = new capture::uhd::Sink{};
        sink->device_parameter = std::string("a=b=c");
        bool parses            = true;
        try {
            std::ignore = sink->openAddress();
        } catch (const std::exception&) {
            parses = false;
        }
        expect(!parses) << "the parser refuses the address before any device is asked for";
        if (parses) {
            delete sink;
            return; // a start with this address would reach the device list, so it is not driven
        }

        auto        returned = std::make_shared<std::atomic<bool>>(false);
        std::thread starter([sink, returned] {
            std::ignore = sink->changeStateTo(gr::lifecycle::State::INITIALISED);
            std::ignore = sink->changeStateTo(gr::lifecycle::State::RUNNING);
            returned->store(true, std::memory_order_release);
        });
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!returned->load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!returned->load(std::memory_order_acquire)) {
            starter.detach();
            expect(false) << "the start did not return inside five seconds, waiting on its own control mutex";
            return;
        }
        starter.join();
        expect(sink->state() == gr::lifecycle::State::ERROR) << "the start throws, which leaves the block in ERROR";
        expect(!sink->deviceUp()) << "with no device up";
        expect(sink->_ctrlMutex.try_lock()) << "and the control mutex free";
        sink->_ctrlMutex.unlock();
        delete sink;
    });

    cases("sink.uhd-a-failed-start-throws", [] {
        capture::uhd::Sink blk{};
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

    cases("sink.uhd-a-failed-start-ends-the-run", [] {
        // A transmit graph whose radio does not open, run the way a program's main() runs one.
        expect(fatal(capture::uhd::test::uhdRefusesAddress())) << "UHD's parser refuses the address, so no device is asked for";
        gr::Graph flow;
        auto&     feed = flow.emplaceBlock<ZeroFeed>();
        auto&     sink = flow.emplaceBlock<capture::uhd::Sink>({{"device_parameter", std::string("a=b=c")}});
        expect(flow.connect<"out", "in">(feed, sink).has_value()) << "the feed wires to the sink";
        const RunEnd end = runToEnd(std::move(flow));
        expect(end.returned) << "runAndWait() returns by itself";
        expect(end.failed) << "and fails, since the sink never opened";
        expect(end.error.contains("uhd make(a=b=c)")) << "with the reason as its error";
        expect(end.names("uhd make(a=b=c)")) << "and on the message plane";
    });

    cases("sink.uhd-a-failed-source-start-ends-the-run", [] {
        // A receive graph whose radio does not open, run the way a program's main() runs one.
        expect(fatal(capture::uhd::test::uhdRefusesAddress())) << "UHD's parser refuses the address, so no device is asked for";
        gr::Graph flow;
        auto&     src   = flow.emplaceBlock<capture::uhd::Source>({{"device_parameter", std::string("a=b=c")}});
        auto&     drain = flow.emplaceBlock<Drain>();
        expect(flow.connect<"out", "in">(src, drain).has_value()) << "the source wires to the drain";
        const RunEnd end = runToEnd(std::move(flow));
        expect(end.returned) << "runAndWait() returns by itself";
        expect(end.failed) << "and fails, since the source never opened";
        expect(end.error.contains("uhd make(a=b=c)")) << "with the reason as its error";
        expect(end.names("uhd make(a=b=c)")) << "and on the message plane";
    });

    cases("sink.uhd-a-release-runs-behind-the-stop", [] {
        /*| frame: the case hands over two shared pointers in place of a streamer and a device.
                Each deleter sleeps 300 ms and records its place in the order. No device is
                opened.
        */
        using Clock = std::chrono::steady_clock;
        auto order  = std::make_shared<std::vector<int>>();
        auto guard  = std::make_shared<std::mutex>();
        auto slow   = [order, guard](int id) {
            return std::shared_ptr<int>(new int(id), [order, guard](int* p) {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                std::lock_guard lock(*guard);
                order->push_back(*p);
                delete p;
            });
        };
        auto stream = slow(1);
        auto device = slow(2);

        const auto began = Clock::now();
        capture::uhd::releaseLater(std::move(stream), std::move(device));
        const auto handed = Clock::now() - began;
        expect(stream == nullptr && device == nullptr) << "the caller holds nothing of the device once it has handed it over";
        expect(std::chrono::duration_cast<std::chrono::milliseconds>(handed).count() < 100) << "and returns without waiting for the release";
        {
            std::lock_guard lock(*guard);
            expect(order->empty()) << "the release is still running when the stop returns";
        }

        capture::uhd::awaitReleases();
        const auto waited = Clock::now() - began;
        expect(std::chrono::duration_cast<std::chrono::milliseconds>(waited).count() >= 600) << "an open waits for the whole release";
        std::lock_guard lock(*guard);
        expect(*order == std::vector<int>{1, 2}) << "the streamer goes before the device";
    });

    cases("sink.uhd-a-destroyed-block-leaves-the-release-running", [] {
        /*| frame: a release that takes 300 ms is in flight while a source and a sink are made
                and destroyed, as a caller that destroys its blocks at every stop does. No device
                is opened.
        */
        using Clock = std::chrono::steady_clock;
        capture::uhd::releaseLater(std::shared_ptr<int>(new int(0), [](int* p) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            delete p;
        }));
        const auto began = Clock::now();
        {
            capture::uhd::Source source{};
            capture::uhd::Sink   sink{};
        }
        const auto destroyed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - began).count();
        expect(destroyed < 100) << "both blocks go without waiting for the release in flight";
        capture::uhd::awaitReleases();
        expect(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - began).count() >= 250) << "and the wait a caller asks for waits it out";
    });

    cases("sink.uhd-the-release-ends-before-the-library-at-exit", [] {
        /*| frame: the case forks a child that hands a release off and leaves main at once, as
                a program does right after a stop. The child makes the registry first, as the
                wait ahead of every open makes it, and a static object after it, as the vendor
                library's open makes its own. The release writes 'r' to a pipe when it ends, and
                the static object writes 'l' when it is destroyed.
        */
        int ends[2] = {-1, -1};
        expect(pipe(ends) == 0) << "the pipe opens";
        std::fflush(nullptr);
        const pid_t child = fork();
        if (child == 0) {
            close(ends[0]);
            std::ignore = std::freopen("/dev/null", "w", stdout);
            std::ignore = std::freopen("/dev/null", "w", stderr);
            static int out = ends[1];
            struct Library {
                ~Library() {
                    const char c = 'l';
                    std::ignore  = write(out, &c, 1);
                }
            };
            capture::uhd::awaitReleases();
            static Library library;
            capture::uhd::releaseLater(std::shared_ptr<int>(new int(0), [](int* p) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                const char c = 'r';
                std::ignore  = write(out, &c, 1);
                delete p;
            }));
            std::exit(0);
        }
        close(ends[1]);
        std::string order;
        char        c = 0;
        while (read(ends[0], &c, 1) == 1) {
            order += c;
        }
        close(ends[0]);
        int status = 0;
        expect(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0) << "the child leaves main and exits cleanly";
        expect(order == std::string("rl")) << "the release ends before the library's static objects go, read " << order;
    });

    cases("sink.uhd-a-second-wait-waits-for-the-release-in-flight", [] {
        /*| frame: a release is handed off and runs until the case lets it end. A first caller
                waits for the releases on a thread of its own, and a second caller starts 100 ms
                later on another, while the first is joining. Each notes whether the release had
                ended when its wait returned. The case lets the release end 100 ms after the
                second caller starts. No device is opened.
        */
        std::promise<void>       letGo;
        std::shared_future<void> go    = letGo.get_future().share();
        auto                     ended = std::make_shared<std::atomic<bool>>(false);
        capture::uhd::releaseLater(std::shared_ptr<int>(new int(0), [go, ended](int* p) {
            std::ignore = go.wait_for(std::chrono::seconds(5));
            ended->store(true);
            delete p;
        }));
        std::atomic<int> firstSaw{-1};
        std::atomic<int> secondSaw{-1};
        std::thread      first([&firstSaw, ended] {
            capture::uhd::awaitReleases();
            firstSaw.store(ended->load() ? 1 : 0);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::thread second([&secondSaw, ended] {
            capture::uhd::awaitReleases();
            secondSaw.store(ended->load() ? 1 : 0);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const int secondBeforeTheEnd = secondSaw.load();
        letGo.set_value();
        first.join();
        second.join();
        expect(secondBeforeTheEnd == -1) << "the second wait is still waiting while the first joins the release";
        expect(firstSaw.load() == 1) << "the first wait returns once the release has ended";
        expect(secondSaw.load() == 1) << "and so does the second, which found nothing left to join";
    });

    cases("sink.uhd-a-rate-read-back-that-fails-states-no-rate", [] {
        int        reads = 0;
        const auto once  = capture::uhd::writeRateWith(
            8.0e6, [](double) {},
            [&reads]() -> double {
                if (reads++ == 0) {
                    throw std::runtime_error("busy");
                }
                return 8.0e6;
            },
            capture::uhd::Direction::Transmit, "case");
        expect(once.landed) << "the write went through";
        expect(once.heldHz == 0.0 && reads == 1) << "and a read-back that failed states no rate, read once under its own words";
        expect(!capture::uhd::filterFollowsRate(once, 0.2e6, 56.0e6)) << "so no filter follows a rate nobody read";
        expect(capture::uhd::filterFollowsRate({.landed = true, .heldHz = 8.0e6}, 0.2e6, 56.0e6)) << "a landed write that was read back moves the filter";
        expect(!capture::uhd::filterFollowsRate({.landed = true, .heldHz = 8.0e6}, 40.0e6, 40.0e6)) << "a fixed filter takes no write";
        expect(!capture::uhd::filterFollowsRate({.landed = false, .heldHz = 8.0e6}, 0.2e6, 56.0e6)) << "nor does a refused rate";
    });

    cases("sink.uhd-names-a-control-value-it-moves", [] {
        capture::uhd::DeviceTruth t;
        t.bwMinHz = 0.2e6;
        t.bwMaxHz = 56.0e6;
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};
        const auto                  tx = capture::uhd::Sink::describeControls(t, ctx);
        expect(capture::uhd::boundedControlValue(tx, "BANDWIDTH", 80.0, "case") == 56.0) << "a filter past the range takes its top";
        expect(capture::uhd::movedControlLine("uhd::Sink", "BANDWIDTH", 80.0, 56.0) == "[uhd::Sink] BANDWIDTH 80 is outside what this radio states; 56 is used") << "and the move is named";
        expect(capture::uhd::movedControlLine("uhd::Sink", "BANDWIDTH", 10.0, 10.0).empty()) << "a value inside the range is not";
        expect(capture::uhd::movedControlLine("uhd::Source", "LO_OFFSET", std::numeric_limits<double>::quiet_NaN(), 0.0) == "[uhd::Source] LO_OFFSET is not a number; 0 is used") << "and a value that is not a number has words of its own";
    });

    cases("sink.uhd-a-gain-that-is-not-a-number-is-its-floor", [] {
        // One element reading 20 dB when the device was read, which is its descriptor's default.
        capture::uhd::DeviceTruth t;
        t.gains.push_back({"PGA0", 0.0, 31.5, 0.5, 20.0});
        const double                nan = std::numeric_limits<double>::quiet_NaN();
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 2.0e6, .centerFreq = 915.0e6};
        const auto                  tx = capture::uhd::Sink::describeControls(t, ctx);
        const auto                  rx = capture::uhd::Source::describeControls(t, ctx);
        expect(capture::uhd::boundedControlValue(tx, "PGA0", nan) == 0.0) << "a NaN through the control is the element's floor";
        expect(capture::uhd::boundedGainFor(t, "PGA0", nan, "case") == 0.0) << "as it is through setElementGain";
        expect(capture::uhd::boundedControlValue(rx, "PGA0", nan) == 0.0) << "in either direction";
        expect(capture::uhd::boundedControlValue(tx, "PGA0", 40.0) == 31.5) << "a number past the range still takes its end";
    });

    cases("sink.uhd-a-control-writes-through-its-setting", [] {
        /*| frame: a sink whose device is up with no handle behind it. The settings hook returns
                where it holds no handle, so a sweep moves the members and writes no device, and
                the next sweep carrying the members writes their values.
        */
        capture::uhd::Sink sink{};
        sink._openTruth.antennas  = {"TX/RX", "RX2"};
        sink._openTruth.antenna   = "TX/RX";
        sink._openTruth.gainMinDb = 0.0;
        sink._openTruth.gainMaxDb = 89.75;
        sink._openTruth.gains.push_back({"PGA", 0.0, 89.75, 0.25, 10.0});
        sink.tx_antennae = std::vector<std::string>{"TX/RX"};
        sink._deviceUp.store(true);
        expect(capture::uhd::Sink::settingForControl(sink._openTruth, "ANTENNA", 1.0).contains("tx_antennae")) << "the connector control names its setting";
        expect(capture::uhd::Sink::settingForControl(sink._openTruth, "PGA", 1.0).empty()) << "and a gain element has none";
        expect(sink.setControl("ANTENNA", 1.0)) << "the control is staged";
        std::ignore = sink.settings().applyStagedParameters();
        expect(sink.tx_antennae->size() == 1UZ && sink.tx_antennae->front() == "RX2") << "and the setting carries the connector it chose, so a replay writes that one";
        expect(!sink.setControl("ANTENNA", 2.0)) << "an index past the list is refused";
        expect(sink.setElementGain("", 20.0)) << "the overall gain is staged";
        std::ignore = sink.settings().applyStagedParameters();
        expect(sink.tx_gains->size() == 1UZ && sink.tx_gains->front() == 20.0) << "through tx_gains";
        expect(sink.setElementGain("", 200.0));
        std::ignore = sink.settings().applyStagedParameters();
        expect(sink.tx_gains->front() == 89.75) << "bounded by the range the device states";
        sink._deviceUp.store(false);
        expect(!sink.setControl("ANTENNA", 0.0) && !sink.setElementGain("", 1.0)) << "a block that is down stages nothing";
    });

    cases("sink.uhd-times-its-sensors-at-the-first-sweep", [] {
        // The shared timing stops when its live predicate falls.
        SensorStandIn stopped;
        const auto    none = capture::uhd::sensorNamesOf(stopped, capture::uhd::Direction::Transmit, "case", 1000.0, [] { return false; });
        expect(none.brief.empty() && stopped.refReads == 0 && stopped.txReads == 0) << "a stop ends the timing before a read";

        // The block times the names at its first sweep, once, and reads them at every sweep.
        capture::uhd::Sink                  sink{};
        SensorStandIn                       device;
        std::vector<capture::SensorReading> out;
        expect(!sink._sensorNamesTaken) << "a start times nothing";
        sink._deviceUp.store(true);
        expect(sink.sweepDeviceSensors(out, device, capture::uhd::SensorSweep::Full)) << "the first sweep publishes";
        expect(sink._sensorNamesTaken) << "and keeps the names it timed";
        expect(device.refReads == 2 && device.txReads == 2) << "one read to time each sensor and one to publish it";
        expect(sink.sweepDeviceSensors(out, device, capture::uhd::SensorSweep::Full));
        expect(device.refReads == 3 && device.txReads == 3) << "a later sweep times nothing again";
        sink._deviceUp.store(false);
        expect(!sink.sweepDeviceSensors(out, device, capture::uhd::SensorSweep::Full)) << "a device that is down is not swept";
        expect(device.refReads == 3) << "and not read";
    });

    cases("sink.uhd-states-the-rate-in-force", [] {
        capture::uhd::Sink sink{};
        sink._rateActualHz = 100.0e6 / 12.0;
        sink.sample_rate   = 8.0e6; // a replay's request, before the applier writes the rate back
        expect(sink.rateInForceHz() == 0.0) << "a block that is down states no rate";
        sink._deviceUp.store(true);
        expect(sink.rateInForceHz() == 100.0e6 / 12.0) << "a running block states the rate the device runs, whatever the setting carries";
        std::unique_lock busy(sink._ctrlMutex);
        expect(sink.rateInForceHz() == 0.0) << "and a busy control mutex answers no rate rather than waiting";
        busy.unlock();
        sink._deviceUp.store(false);
    });

    cases("sink.uhd-a-loaded-host-keeps-the-brief-sensors", [] {
        const auto briefHas = [](int recordMs) {
            capture::uhd::Sink                  sink{};
            SensorStandIn                       device;
            std::vector<capture::SensorReading> out;
            device.slowRecordMs = recordMs;
            sink._deviceUp.store(true);
            std::ignore = sink.sweepDeviceSensors(out, device, capture::uhd::SensorSweep::Brief);
            return std::ranges::any_of(out, [](const capture::SensorReading& r) { return r.id == "gps_time"; });
        };
        expect(briefHas(12)) << "a read that took 12 ms on a loaded host stays in the brief set";
        expect(!briefHas(120)) << "and a record slower than a poll affords stays out";
    });

    cases("sink.uhd-a-stop-ends-the-reference-wait", [] {
        using namespace std::chrono;
        // The shared wait ends when its live predicate falls, well inside its limit.
        SensorStandIn external;
        external.clockSource = "external";
        external.lockAfter   = -1; // a reference that never locks
        std::atomic<bool> live{true};
        std::thread       stopper([&live] {
            std::this_thread::sleep_for(milliseconds(30));
            live.store(false);
        });
        const auto started = steady_clock::now();
        const auto wait    = capture::uhd::waitForReference(external, "test", 1500.0, 5.0, [&live] { return live.load(); });
        stopper.join();
        expect(steady_clock::now() - started < milliseconds(1000)) << "a stop ends the wait inside a poll rather than at the limit";
        expect(wait.waited && !wait.locked && wait.stopped) << "and the wait says a stop ended it";
        SensorStandIn unlocked;
        unlocked.clockSource = "external";
        unlocked.lockAfter   = -1;
        const auto full      = capture::uhd::waitForReference(unlocked, "test", 40.0, 5.0);
        expect(!full.stopped && !full.locked && full.waitedMs >= 40.0) << "with nothing ending it the wait runs to its limit";

        // The block's teardown lowers the predicate before anything else and then waits for a
        // start's wait to let go of its handle.
        capture::uhd::Sink sink{};
        expect(sink.referenceWaitLive()) << "a block that is not stopping lets the wait run";
        std::unique_lock  held(sink._startMutex); // a start inside its wait, holding the snapshot
        std::atomic<bool> returned{false};
        std::thread       stopping([&sink, &returned] {
            sink.stop();
            returned.store(true);
        });
        const auto deadline = steady_clock::now() + seconds(5);
        while (sink.referenceWaitLive() && steady_clock::now() < deadline) {
            std::this_thread::sleep_for(milliseconds(1));
        }
        expect(!sink.referenceWaitLive()) << "the teardown lowers the answer first";
        std::this_thread::sleep_for(milliseconds(20));
        expect(!returned.load()) << "and waits for the wait to let go of its handle";
        held.unlock();
        stopping.join();
        expect(returned.load()) << "then goes on to release the device";
    });

    cases("sink.uhd-an-address-that-pins-the-clock-fixes-the-ladder", [] {
        /*| frame: a B205mini opened with master_clock_rate in its address. The device still states
                its whole clock range, and its rate set is the ladder of the 30.72 MHz clock the
                key fixed.
        */
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220'000.0;
        b205.mclkMaxHz = 61'440'000.0;
        b205.mclkHz    = 30.72e6;
        for (int n = 1; n <= 256; ++n) {
            const double r = 30.72e6 / static_cast<double>(n);
            b205.rates.emplace_back(r, r);
        }
        expect(capture::uhd::clockInForceAtOpen(::uhd::device_addr_t("type=b200,master_clock_rate=30720000"), b205) == 30.72e6) << "the key fixes the clock the device read";
        expect(capture::uhd::clockInForceAtOpen(::uhd::device_addr_t("type=b200"), b205) == 0.0) << "an address without it leaves the clock to UHD";

        // The sink's address carries its setting, and the rate it hands the device follows.
        capture::uhd::Sink sink{};
        sink.device_parameter  = std::string("type=b200");
        sink.master_clock_rate = 30.72e6;
        sink._openTruth        = b205;
        sink.takeClockInForceLocked();
        expect(sink.rateToWrite(2.0e6) == 30.72e6 / 16.0) << "2 MS/s lands on the even decimation 16 rather than on 15, odd";
        capture::uhd::Sink chosen{};
        chosen.device_parameter = std::string("type=b200");
        chosen._openTruth       = b205;
        chosen.takeClockInForceLocked();
        expect(chosen.rateToWrite(2.0e6) == 2.0e6) << "with no key UHD chooses a clock for the request";

        // The source's address carries a key the caller typed into the device string as well.
        capture::uhd::Source typed{};
        typed.device_parameter = std::string("type=b200,master_clock_rate=30.72e6");
        typed._openTruth       = b205;
        typed.takeClockInForceLocked();
        expect(typed.rateToWrite(2.0e6) == 30.72e6 / 16.0) << "the receive ladder is the fixed clock's too";
        capture::uhd::Source unpinned{};
        unpinned.device_parameter = std::string("type=b200");
        unpinned._openTruth       = b205;
        unpinned.takeClockInForceLocked();
        expect(unpinned.rateToWrite(2.0e6) == 2.0e6) << "and a device string with no key leaves it to UHD";
    });

    cases("sink.uhd-the-device-says-whether-its-clock-is-fixed", [] {
        /*| frame: a B205mini a second block opens in one process. The device's own node says
                whether the clock is fixed, whatever this block's address carries.
        */
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220'000.0;
        b205.mclkMaxHz = 61'440'000.0;
        b205.mclkHz    = 30.72e6;
        for (int n = 1; n <= 256; ++n) {
            const double r = 30.72e6 / static_cast<double>(n);
            b205.rates.emplace_back(r, r);
        }
        b205.autoTickRate = false; // the first open's key fixed the clock
        expect(capture::uhd::clockInForceAtOpen(::uhd::device_addr_t("type=b200"), b205) == 30.72e6) << "a clock the device says is fixed is fixed, with no key in this address";
        b205.autoTickRate = true; // the first open carried no key, and this one's is ignored
        expect(capture::uhd::clockInForceAtOpen(::uhd::device_addr_t("type=b200,master_clock_rate=30720000"), b205) == 0.0) << "and a clock the device says UHD chooses is left to UHD, key or none";
        b205.autoTickRate.reset();
        expect(capture::uhd::clockInForceAtOpen(::uhd::device_addr_t("type=b200,master_clock_rate=30720000"), b205) == 30.72e6) << "a device without the node falls back on the address";

        capture::uhd::Sink second{};
        second.device_parameter = std::string("type=b200");
        second._openTruth       = b205;
        second._openTruth.autoTickRate = false;
        second.takeClockInForceLocked();
        expect(second.rateToWrite(2.0e6) == 30.72e6 / 16.0) << "so a sink opened beside a source that fixed the clock snaps to that clock's ladder";
    });

    cases("sink.uhd-counts-the-burst-acknowledgment", [] {
        using Code = ::uhd::async_metadata_t;
        capture::uhd::Sink  sink{};
        AsyncChannelStandIn channel{.codes = {Code::EVENT_CODE_BURST_ACK, Code::EVENT_CODE_UNDERFLOW, Code::EVENT_CODE_BURST_ACK}};
        expect(sink.takeAsyncMessage(channel, 0.0)) << "a message arrived";
        expect(sink.burstAcks() == 1UZ) << "an acknowledgment is counted";
        expect(sink.underrunEvents() == 0UZ) << "and is no gap";
        expect(sink.takeAsyncMessage(channel, 0.0));
        expect(sink.burstAcks() == 1UZ && sink.underrunEvents() == 1UZ) << "an underflow is a gap and no acknowledgment";
        expect(sink.takeAsyncMessage(channel, 0.0));
        expect(sink.burstAcks() == 2UZ) << "each acknowledgment is counted";
        expect(!sink.takeAsyncMessage(channel, 0.0)) << "a quiet channel counts nothing";
        expect(sink.burstAcks() == 2UZ && sink.underrunEvents() == 1UZ);
    });

    cases("sink.uhd-counts-transmit-events-by-bit", [] {
        using Code = ::uhd::async_metadata_t;
        const auto both = [](Code::event_code_t a, Code::event_code_t b) { return static_cast<Code::event_code_t>(a | b); };
        capture::uhd::Sink  sink{};
        AsyncChannelStandIn channel{.codes = {both(Code::EVENT_CODE_UNDERFLOW, Code::EVENT_CODE_SEQ_ERROR), Code::EVENT_CODE_SEQ_ERROR_IN_BURST, Code::EVENT_CODE_TIME_ERROR,
                                        both(Code::EVENT_CODE_UNDERFLOW_IN_PACKET, Code::EVENT_CODE_BURST_ACK), Code::EVENT_CODE_USER_PAYLOAD}};
        expect(sink.takeAsyncMessage(channel, 0.0));
        expect(sink.underrunEvents() == 1UZ) << "a message carrying an underflow and a sequence error is a gap";
        expect(sink.sequenceErrors() == 1UZ) << "and a lost packet, each counted once";
        expect(sink.takeAsyncMessage(channel, 0.0));
        expect(sink.sequenceErrors() == 2UZ) << "a sequence error inside a burst is a lost packet";
        expect(sink.underrunEvents() == 1UZ) << "and no gap of the host's making";
        expect(sink.takeAsyncMessage(channel, 0.0));
        expect(sink.latePackets() == 1UZ) << "a time error is a late packet";
        expect(sink.underrunEvents() == 1UZ && sink.sequenceErrors() == 2UZ) << "and nothing else";
        expect(sink.takeAsyncMessage(channel, 0.0));
        expect(sink.underrunEvents() == 2UZ && sink.burstAcks() == 1UZ) << "a packet that ran out and an acknowledgment in one message count as both";
        expect(sink.takeAsyncMessage(channel, 0.0));
        expect(sink.underrunEvents() == 2UZ && sink.sequenceErrors() == 2UZ && sink.latePackets() == 1UZ && sink.burstAcks() == 1UZ) << "a user payload counts as none of them";
    });

    cases("sink.uhd-a-stop-waits-for-the-burst-acknowledgment", [] {
        using capture::uhd::Sink;
        using std::chrono::milliseconds;
        // The wait ends on the acknowledgment rather than at its bound.
        std::atomic<std::uint64_t> acks{3};
        std::thread                device([&acks] {
            std::this_thread::sleep_for(milliseconds(20));
            acks.fetch_add(1);
        });
        const auto started = std::chrono::steady_clock::now();
        expect(Sink::waitForBurstAck([&acks] { return acks.load(); }, 4, milliseconds(5000))) << "the acknowledgment arrived";
        device.join();
        expect(std::chrono::steady_clock::now() - started < milliseconds(4000)) << "and the wait ended when it did";
        // A count short of the target does not answer for it.
        expect(!Sink::waitForBurstAck([&acks] { return acks.load(); }, 5, milliseconds(20))) << "no acknowledgment came inside the bound";

        // The bound is the play-out of what the device can hold, so it grows as the rate falls
        // and as the wire format narrows.
        const auto n210Slow = Sink::burstAckBoundMs(250.0e3, "sc16");
        expect(n210Slow > milliseconds(1000)) << "an N210's SRAM holds about a second at 250 kS/s";
        expect(n210Slow == capture::playOutMs(static_cast<std::uint64_t>(Sink::kDeviceHeldBytes / 4UZ), 250.0e3));
        expect(Sink::burstAckBoundMs(250.0e3, "sc8") > n210Slow) << "a narrower sample holds more of them";
        expect(Sink::burstAckBoundMs(2.0e6, "sc16") < n210Slow) << "and a faster rate plays them out sooner";
        expect(Sink::burstAckBoundMs(61.44e6, "sc16") - capture::playOutMs(0, 61.44e6) < milliseconds(10)) << "down to a few milliseconds past the margin every play-out carries";

        // A fresh sink has placed no end of burst at a stop.
        const Sink fresh{};
        expect(fresh.stopBurstEnd() == Sink::BurstEnd::NoneOpen);
        expect(fresh.burstAcks() == 0UZ);
    });

    cases("sink.uhd-a-start-a-stop-ended-arms-nothing", [] {
        const auto stream = std::make_shared<StreamStandIn>();
        int        built  = 0;
        const auto make   = [&stream, &built](const ::uhd::stream_args_t&) -> ::uhd::tx_streamer::sptr {
            ++built;
            return stream;
        };
        // A stop ended the wait: its teardown lowered the answer and waits on _startMutex.
        capture::uhd::Sink stopped{};
        stopped._stopping.store(true);
        expect(stopped.armAfterReferenceWait(make).empty());
        expect(built == 0) << "a start whose wait a stop ended builds no streamer";
        expect(!stopped.deviceUp() && !stopped._drain.thread.joinable() && !stopped._async.thread.joinable()) << "raises no device and starts no thread";
        // A stop that took the handle leaves nothing to build a streamer from.
        capture::uhd::Sink taken{};
        expect(taken.armAfterReferenceWait([](const ::uhd::stream_args_t&) -> ::uhd::tx_streamer::sptr { return nullptr; }).empty());
        expect(!taken.deviceUp() && !taken._drain.thread.joinable()) << "nor does a start whose handle a stop took";
        // A live start arms, and the stop behind it joins what the start began.
        capture::uhd::Sink live{};
        expect(live.armAfterReferenceWait(make).empty());
        expect(built == 1 && live.deviceUp()) << "a live start builds the streamer and raises the device";
        expect(live._drain.thread.joinable() && live._async.thread.joinable()) << "and starts both threads";
        live.stop();
        expect(!live.deviceUp() && !live._drain.thread.joinable() && !live._async.thread.joinable()) << "the stop joins both";
        expect(live._stream == nullptr) << "the block holds no streamer once the stop returns";
        capture::uhd::awaitReleases();
        expect(stream.use_count() == 1) << "and no snapshot of the streamer outlives the release";
    });

    cases("sink.uhd-a-stop-waits-for-its-own-acknowledgment", [] {
        using capture::uhd::Sink;
        using Code        = ::uhd::async_metadata_t;
        const auto stream = std::make_shared<StreamStandIn>();
        Sink       idle{};
        idle.endBurstAtStop(stream, 61.44e6, "sc16");
        expect(idle.stopBurstEnd() == Sink::BurstEnd::NoneOpen && idle.unconfirmedByFlushBoundSamples() == 0UZ) << "a stop with no burst open places nothing and owes nothing";

        // A pause ended an earlier burst, whose acknowledgment arrives during the stop; the
        // stop's own never does.
        Sink paused{};
        paused._bursting.store(true);
        expect(paused.endBurstWith(stream) == Sink::EndOfBurst::Placed) << "the pause places its end";
        paused._bursting.store(true); // the resume opened a fresh burst
        std::thread earlier([&paused] {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            paused.countAsyncEvent(Code::EVENT_CODE_BURST_ACK);
        });
        paused.endBurstAtStop(stream, 61.44e6, "sc16");
        earlier.join();
        expect(stream->endsPlaced.load() == 2 && paused.burstAcks() == 1UZ) << "the stop placed its own end and one acknowledgment came";
        expect(paused.stopBurstEnd() == Sink::BurstEnd::Unconfirmed) << "the earlier burst's acknowledgment does not answer for the stop's";
        expect(paused.unconfirmedByFlushBoundSamples() == Sink::kDeviceHeldBytes / 4UZ) << "and the stop states what the device can still hold";

        // The stop's own acknowledgment arrives.
        Sink acked{};
        acked._bursting.store(true);
        std::thread own([&acked] {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            acked.countAsyncEvent(Code::EVENT_CODE_BURST_ACK);
        });
        acked.endBurstAtStop(stream, 61.44e6, "sc16");
        own.join();
        expect(acked.stopBurstEnd() == Sink::BurstEnd::Acknowledged && acked.unconfirmedByFlushBoundSamples() == 0UZ) << "an acknowledged end owes nothing";

        // The send places no end, and the carrier stays up.
        stream->placeNothing.store(true);
        Sink keyed{};
        keyed._bursting.store(true);
        keyed.endBurstAtStop(stream, 61.44e6, "sc8");
        expect(keyed.stopBurstEnd() == Sink::BurstEnd::NotPlaced) << "a stop whose end was not placed says so rather than that no burst was open";
        expect(keyed._bursting.load()) << "with the burst flag back up";
        expect(keyed.unconfirmedByFlushBoundSamples() == Sink::kDeviceHeldBytes / 2UZ) << "and the bound in the wire format in force";
    });

    cases("sink.uhd-a-start-configures-the-rate-first", [] {
        /*| frame: an N210's transmit truth: a fixed 100 MHz clock stating its divisors, a 1 Gbit
                link and a fixed filter. The block holds no device, so the rate write is the
                stand-in the case passes and every other device write finds no handle.
        */
        capture::uhd::DeviceTruth n210;
        n210.mclkMinHz     = 100.0e6;
        n210.mclkMaxHz     = 100.0e6;
        n210.mclkHz        = 100.0e6;
        n210.linkBytesPerS = 125.0e6;
        n210.bwMinHz       = 40.0e6;
        n210.bwMaxHz       = 40.0e6;
        n210.freqRanges    = {{50.0e6, 2.2e9}};
        for (int n = 1; n <= 512; ++n) {
            const double r = 100.0e6 / static_cast<double>(n);
            n210.rates.emplace_back(r, r);
        }
        std::vector<double> asked;
        const auto          lands = [&asked](double hz) {
            asked.push_back(hz);
            return capture::uhd::RateWrite{.landed = true, .heldHz = hz};
        };
        capture::uhd::Sink sink{};
        sink.sample_rate = 8.0e6;
        sink.frequency   = std::vector<double>{915.0e6};
        sink._openTruth  = n210;
        const std::string failed = sink.configureOpenDeviceLocked(lands);
        expect(asked.size() == 1UZ && std::abs(asked.front() - 100.0e6 / 12.0) < 1.0) << "the block writes the snapped rate, interpolation 12";
        expect(std::abs(static_cast<double>(sink.sample_rate) - 100.0e6 / 12.0) < 1.0 && sink._rateActualHz == asked.front()) << "and states the rate the device holds";
        expect(failed == "uhd: the center frequency was refused") << "the rate lands before the tune, which finds no handle here";
        expect(sink.applyRateWith(lands) && asked.size() == 1UZ) << "a replay of the rate in force writes nothing";

        capture::uhd::Sink refused{};
        refused.sample_rate = 8.0e6;
        refused.frequency   = std::vector<double>{915.0e6};
        refused._openTruth  = n210;
        expect(refused.configureOpenDeviceLocked([](double) { return capture::uhd::RateWrite{.landed = false, .heldHz = 5.0e6}; }) == "uhd: the sample rate was refused") << "a refused rate fails the start before the tune";

        // The receive applier takes the same write, and a programmable clock takes the request.
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220.0e3;
        b205.mclkMaxHz = 61.44e6;
        b205.mclkHz    = 16.0e6;
        b205.rates     = {{2.0e6, 2.0e6}};
        capture::uhd::Source source{};
        source.sample_rate = 2.0e6;
        source._openTruth  = b205;
        asked.clear();
        source.applyRateWith(lands);
        source.applyRateWith(lands);
        expect(asked.size() == 1UZ && asked.front() == 2.0e6) << "the source writes the request once and a replay nothing";
        expect(source._rateActualHz == 2.0e6);
    });

    cases("sink.uhd-a-refused-start-leaves-the-rate-asked-for", [] {
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz  = 220.0e3;
        b205.mclkMaxHz  = 61.44e6;
        b205.freqRanges = {{70.0e6, 6.0e9}};
        capture::uhd::Sink sink{};
        sink.sample_rate = 8.0e6;
        sink.frequency   = std::vector<double>{915.0e6};
        sink._openTruth  = b205;
        const auto refused = [](double) { return capture::uhd::RateWrite{.landed = false, .heldHz = 5.0e6}; };
        expect(sink.configureOpenDeviceLocked(refused) == "uhd: the sample rate was refused");
        expect(static_cast<double>(sink.sample_rate) == 8.0e6) << "the setting keeps the request rather than the rate a released device held";
        expect(!sink.applyRateWith(refused) && static_cast<double>(sink.sample_rate) == 5.0e6) << "while a running block's refused change states the rate the device holds";
    });

    cases("sink.uhd-a-rate-nobody-read-back-is-written-again", [] {
        // The first write lands and its read-back fails; the next one is read back.
        int        writes = 0;
        const auto flaky  = [&writes](double hz) { return capture::uhd::RateWrite{.landed = true, .heldHz = writes++ == 0 ? 0.0 : hz}; };
        capture::uhd::DeviceTruth b205;
        b205.mclkMinHz = 220.0e3;
        b205.mclkMaxHz = 61.44e6;
        b205.rates     = {{2.0e6, 2.0e6}};
        capture::uhd::Sink sink{};
        sink.sample_rate = 2.0e6;
        sink._openTruth  = b205;
        expect(sink.applyRateWith(flaky) && writes == 1 && sink._rateActualHz == 0.0) << "a landed write nobody read back states no rate";
        expect(!sink._rateWrittenHz.has_value()) << "and is not recorded as written";
        expect(sink.applyRateWith(flaky) && writes == 2) << "so the replay writes again";
        expect(sink._rateActualHz == 2.0e6 && sink._rateWrittenHz == 2.0e6) << "and reads the rate in force";
        expect(sink.applyRateWith(flaky) && writes == 2) << "after which a replay writes nothing";

        writes = 0;
        capture::uhd::Source source{};
        source.sample_rate = 2.0e6;
        source._openTruth  = b205;
        source.applyRateWith(flaky);
        expect(writes == 1 && !source._rateWrittenHz.has_value()) << "the source records no write nobody read back";
        source.applyRateWith(flaky);
        expect(writes == 2 && source._rateActualHz == 2.0e6) << "and its replay writes again";
        expect(!capture::uhd::rateWriteSettled({.landed = true, .heldHz = 0.0}) && !capture::uhd::rateWriteSettled({.landed = false, .heldHz = 2.0e6}) && capture::uhd::rateWriteSettled({.landed = true, .heldHz = 2.0e6}));
    });

    cases("sink.uhd-the-surface-is-read-without-the-control-mutex", [] {
        /*| frame: the case holds the control mutex on its own thread, as a device call made
                under it holds it, and reads the surface on another. No device is opened.
        */
        capture::uhd::Sink                sink{};
        const auto                        unheld = sink.controlSurface();
        std::vector<capture::ControlDesc> held;
        expect(capture::test::returnsWhileHeld(sink._ctrlMutex, [&sink, &held] { held = sink.controlSurface(); })) << "the surface is read while a device call holds the control mutex";
        expect(held == unheld) << "and it is the surface the block states";
    });

    cases("sink.uhd-both-blocks-declare-their-labels", [] {
        capture::test::expectLabels<capture::uhd::Source>({"family/uhd", "role/source", "holds/device", "ingests/rf", "ingests/time"});
        capture::test::expectLabels<capture::uhd::Sink>({"family/uhd", "role/sink", "holds/device", "emits/rf", "ingests/time"});
    });

    return cases.report();
}
