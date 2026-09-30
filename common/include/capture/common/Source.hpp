/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <capture/common/Device.hpp>

#include <algorithm>
#include <chrono>
#include <complex>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <thread>

/*| role: the publish side every device source block shares.
    frame: the device-neutral parts — the device-string reader, the coverage test, the
        streaming-thread guard, the teardown guard, the control guard and the drop-newest ring —
        are in capture/common/Device.hpp, which this header carries for every source. What lives
        here is the hold-and-retry publish, the rate tag on a stream's first block, the wall
        clock its periodic timing tag is read from, and the gap marker beside it.
*/
namespace capture {

// Wall-clock nanoseconds for the SDR_WALLCLOCK timing tags.
inline std::uint64_t wallClockNs() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

/*| contract: when the first sample of a drained block was taken, from the clock read as it was
        drained. backlogSamples counts the samples the ring held when it was read, the chunk
        about to be published included, since the first sample of that chunk is the oldest;
        rateHz is the rate those samples were taken at.
    trap: the tag holdAndPublish emits carries TRIGGER_OFFSET 0, so it says the timestamp
        belongs to the first sample of the block. Every source that owns a ring samples the wall
        clock on the consuming thread at the moment it drains that ring, which is when the
        newest sample in it arrived, so the reading is late by the whole backlog: one block at
        steady state, the length of one device transfer, and after any stall the backlog
        itself, a full second for a 2 Mi-sample ring at 2 MS/s.
    why: a recorder turns that tag into core:datetime, the only statement a SigMF recording
        carries about when it began.
    invariant: a rate or a backlog of zero leaves the reading alone, and a correction larger
        than the reading itself floors at zero rather than wrapping.
    trap: rateHz is the rate those samples were taken at, which is the rate in force only while
        the ring holds nothing taken at another one. Every caller therefore drops what its ring
        holds inside a rate change and marks the discontinuity; a backlog carried across one is
        corrected by the wrong number, by the backlog times the difference of the two
        reciprocals. A source moving from 2.4 MS/s to 250 kS/s with a quarter-second backlog
        stamps its next block about two seconds in the past.
    verified-by: publish.tag-timestamps-the-first-sample
*/
inline std::uint64_t blockStartNs(std::uint64_t drainedNs, std::size_t backlogSamples, double rateHz) {
    if (rateHz <= 0.0 || backlogSamples == 0UZ) {
        return drainedNs;
    }
    const auto backlogNs = static_cast<std::uint64_t>(static_cast<double>(backlogSamples) * 1e9 / rateHz);
    return backlogNs < drainedNs ? drainedNs - backlogNs : 0UL;
}

/*| contract: the minimum interval between timing tags in nanoseconds, from the seconds a
        caller staged. An interval that is not positive reads as zero, which the decision below
        takes as a tag on every block.
    trap: tag_interval is a reflectable setting on every source and no driver validates it, so
        the value arrives from whatever staged it. Casting a negative or not-a-number product
        into an unsigned integer is undefined behavior, and on this target it yields the largest
        std::uint64_t — an interval of six hundred years, which stops every tag after the first
        for the life of the stream. The ceiling keeps the cast inside the type for a value
        nobody meant either.
    verified-by: publish.tag-interval-outside-the-useful-range
*/
inline std::uint64_t tagIntervalNs(float seconds) {
    constexpr double kCeilingSeconds = 1.0e6; // eleven days, and 1e15 ns
    if (!(seconds > 0.f)) {
        return 0UL;
    }
    return static_cast<std::uint64_t>(std::min(static_cast<double>(seconds), kCeilingSeconds) * 1e9);
}

/*| contract: whether the block about to be published carries a timing tag, and the time the
        next decision measures from. lastTagNs holds what the previous decision left and is
        zero before any tag has gone out; intervalNs comes from tagIntervalNs.
    invariant: the time this leaves is never earlier than the time it was given, so a stamp
        taken from behind cannot make the next interval read longer than it is.
    trap: tWallNs carries the backlog correction, so it moves backwards whenever a stall grows
        the backlog: the clock only ran forward, and the first sample of the new block was taken
        before the first sample of the last tagged one. An unsigned difference underflows there
        and reads as an interval of centuries, so every block after a stall passes the interval
        test however short the interval is.
    why: a stamp that moves backwards still carries a tag, because a reader aligning the stream
        to wall time needs that correction, and the next interval is measured from the later
        of the two times.
    verified-by: publish.tag-times-never-run-backwards
*/
struct TagDecision {
    bool          emit      = false;
    std::uint64_t lastTagNs = 0;
};

inline TagDecision tagDecision(std::uint64_t tWallNs, std::uint64_t lastTagNs, std::uint64_t intervalNs) {
    const std::uint64_t latest = std::max(tWallNs, lastTagNs);
    if (lastTagNs == 0UL || intervalNs == 0UL || tWallNs < lastTagNs) {
        return {true, latest};
    }
    if (tWallNs - lastTagNs >= intervalNs) {
        return {true, latest};
    }
    return {false, lastTagNs};
}

/*| contract: publish the gap marker at the head of the block the gap falls in front of, with
        the cause beside it where the caller names one. rx_overflow says a gap occurred and
        rx_overflow_cause names what caused it; a caller naming nothing publishes the key alone.
    frame: a cause is a short phrase in the words the device library uses, "restart" or "lost
        packet". Every reader of rx_overflow alone reads the same tag whether a cause is there
        or not.
    why: the families that know why a gap happened publish the same key as the families that do
        not, so a reader learns the cause where there is one without a second tag to look for.
    verified-by: publish.a-marker-carries-the-cause-where-one-is-named
*/
template <typename Block>
void emitOverflowTag(Block& blk, std::string_view cause = {}) {
    auto tagMap = blk.out.makeTagMap();
    gr::tag::put(tagMap, "rx_overflow", true);
    if (!cause.empty()) {
        gr::tag::put(tagMap, "rx_overflow_cause", std::string(cause));
    }
    blk.out.publishTag(std::move(tagMap), 0UZ);
    noteDroppedTags(blk);
}

/*| contract: publish the tags the first sample of a block carries: sample_rate on the first
        block a stream publishes after its start, the value of the block's own sample_rate
        setting, and the periodic timing tag stamped tWallNs where tagDecision says one is due.
        Every source writes the rate the device runs into that setting before it streams, a rate
        the device snapped included, so a reader that takes its rate from tags reads the rate the
        samples arrive at from the first one. A rate changed while streaming reaches the stream
        as the settings forward.
    contract: tWallNs is a time in nanoseconds or a callable returning one. A callable runs only
        where the block emits timing tags, and a block with no tag due builds no tag map.
    frame: the tags go on the sample at the writer's position, the first sample the next
        publish hands downstream, so a caller holding a reservation publishes them before it
        publishes the reservation.
    verified-by: publish.the-first-block-of-a-stream-carries-the-rate-in-force
    verified-by: publish.a-block-with-no-tag-due-reads-no-clock
*/
template <typename Block, typename WallNs>
void publishBlockHeadTags(Block& blk, WallNs&& tWallNs) {
    const bool    rateOwed = blk._outputState.rateTagOwed.load(std::memory_order_relaxed) && blk._outputState.rateTagOwed.exchange(false, std::memory_order_acq_rel);
    bool          timing   = false;
    std::uint64_t tagNs    = 0;
    if (blk.emit_timing_tags) {
        if constexpr (std::invocable<WallNs>) {
            tagNs = tWallNs();
        } else {
            tagNs = tWallNs;
        }
        const auto decision = tagDecision(tagNs, blk._lastTagTimeNs, tagIntervalNs(blk.tag_interval.value));
        timing              = decision.emit;
        blk._lastTagTimeNs  = decision.lastTagNs;
    }
    if (!rateOwed && !timing) {
        return;
    }
    auto tagMap = blk.out.makeTagMap();
    if (rateOwed) {
        gr::tag::put(tagMap, gr::tag::SAMPLE_RATE, blk.sample_rate.value);
    }
    if (timing) {
        gr::tag::put(tagMap, gr::tag::TRIGGER_NAME, blk.trigger_name.value);
        gr::tag::put(tagMap, gr::tag::TRIGGER_TIME, tagNs);
        gr::tag::put(tagMap, gr::tag::TRIGGER_OFFSET, 0.f);
    }
    blk.out.publishTag(std::move(tagMap), 0UZ);
    noteDroppedTags(blk);
}

/*| contract: how long a source waiting for room on its output edge waits when no block of the
        graph moves: the time the stream takes to fill half the edge at rateHz, and at most
        kRoomWaitLimitS. A rate that is not a positive number waits the limit.
    frame: a block that consumes samples moves the graph's progress counter, and that ends the
        wait at once. The bound covers a reader whose consume moves no counter this source
        waits on: after half the edge's time at the rate, a reader that keeps pace has freed
        room for the next receive. The timer slack of the waiting thread, 50 us by default on
        Linux, floors every timed wait.
    why: a source that sleeps a fixed interval between looks moves at most one edge per
        interval once the edge fills, which caps the rate the source can carry by a constant.
    invariant: kRoomWaitLimitS bounds how late a stop or a staged setting reaches a source
        whose edge stays full.
    verified-by: publish.a-full-edge-waits-for-the-reader
*/
inline constexpr double kRoomWaitLimitS = 0.1;

inline std::chrono::nanoseconds roomWaitBound(std::size_t edgeSamples, double rateHz) {
    const double limitNs = kRoomWaitLimitS * 1e9;
    const double halfNs  = rateHz > 0.0 ? static_cast<double>(edgeSamples) * 0.5e9 / rateHz : limitNs; // a rate that is not a number fails the test
    return std::chrono::nanoseconds(static_cast<std::int64_t>(std::min(halfNs, limitNs)));
}

/*| contract: wait until the graph's progress counter leaves seen, or roomWaitBound of the edge
        at the block's rate passes, and answer whether the counter moved. The caller reads seen
        before it reads the room it lacks, so a consume that lands between the two ends the wait
        at once.
    frame: a core whose progress counter offers a timed wait ends the wait at the consume. On a
        core without one, the source reads the counter kRoomPollSteps times across the bound,
        and the wait ends at most one step after the consume.
    why: the untimed wait of such a core returns only when the counter moves, and a source whose
        edge stays full must still reach a stop or a staged setting within the bound.
*/
inline constexpr int kRoomPollSteps = 16;

template <typename Block>
bool awaitProgress(Block& blk, std::size_t seen, std::size_t edgeSamples) {
    const std::chrono::nanoseconds bound    = roomWaitBound(edgeSamples, static_cast<double>(blk.sample_rate.value));
    const auto                     deadline = std::chrono::steady_clock::now() + bound;
    if constexpr (requires { blk.progress->waitUntil(seen, deadline); }) {
        return blk.progress->waitUntil(seen, deadline);
    } else {
        const std::chrono::nanoseconds step = std::max(bound / kRoomPollSteps, std::chrono::nanoseconds(1));
        while (blk.progress->value() == seen) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::min<std::chrono::nanoseconds>(step, deadline - now));
        }
        return true;
    }
}

/*| role: hold-and-retry publish of one CF32 block into the source's output stream, with the
        periodic wall-clock timing tag and, on the first block of a stream, the rate in force.
    contract: it never drops samples the hardware already delivered: when the downstream ring is
        full it waits through awaitProgress, applies any staged settings, and retries until the
        block drains or keepGoing() returns false. Callers invoke this from their streaming
        thread; keepGoing encodes the per-device stop condition — a library-owned callback tests
        lifecycle only, an owned consumer thread also tests its stop flag. tWallNs is sampled once by the caller
        so every sample in the block shares one timestamp.
    invariant: the owning block keeps the framework from applying settings on the scheduler
        thread. applyChangedSettings() runs here, on the streaming thread, once per retry; it
        writes the block's reflected members, publishes a forward tag, notifies listeners and
        invalidates the port caches. gr::Block's own work path calls that same method from the
        scheduler thread at four sites, so a block reaching both paths applies one settings map
        from two threads. A device source overrides work() with a body that returns a status
        without reaching the base's work path, which is the whole of the guarantee: a source
        written with processBulk, the ordinary GNU Radio 4 shape, does not have it.
    trap: an output port with nothing connected to it never drains. The default buffer fills
        once, available() stays at zero, and this loop waits and retries while the device ring
        overflows — the base's own test for an unconnected downstream sits in a work path the
        overridden work() never reaches. The owning block tests out.isConnected() in work() and
        requests the stop.
    trap: the space the writer can take right now caps the request. tryReserve is all
        or nothing — it claims exactly the count asked for or hands back an empty span — so a
        request larger than the output edge holds can never be granted, however empty that edge
        is. Asking for the whole remainder therefore wedged this loop for good once a block was
        longer than the edge, the size a device ring emptied in one pass after a starvation
        hands it: the source went on streaming and dropping while nothing left it, work() kept
        reporting OK, and only a stop and start cleared it.
    contract: the first sample of each block carries the tags publishBlockHeadTags places.
    invariant: the graph's progress counter moves after the span holding the samples is
        released. A writer span hands its samples to the edge when it is released, not when
        publish() is called on it, so a reader woken earlier would find nothing new and wait
        again.
    verified-by: publish.block-longer-than-the-edge
*/
template <typename Block, typename KeepGoing>
void holdAndPublish(Block& blk, std::span<const std::complex<float>> samples, std::uint64_t tWallNs, KeepGoing keepGoing) {
    auto&             writer = blk.out.streamWriter();
    const std::size_t n      = samples.size();
    std::size_t       done   = 0UZ;
    while (done < n && keepGoing()) {
        const std::size_t want = std::min(n - done, writer.available());
        if (want == 0UZ) {
            const std::size_t seen = blk.progress->value();
            if (writer.available() == 0UZ) {
                awaitProgress(blk, seen, writer.buffer().size());
            }
            blk.applyChangedSettings();
            continue;
        }
        std::size_t nCopy = 0UZ;
        {
            auto span = writer.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(want);
            if (!span.empty()) {
                nCopy = std::min(n - done, span.size());
                std::memcpy(span.data(), samples.data() + done, nCopy * sizeof(std::complex<float>));
                if (done == 0UZ) {
                    publishBlockHeadTags(blk, tWallNs);
                }
                span.publish(nCopy);
            }
        } // the edge takes the samples when the span is released
        if (nCopy == 0UZ) {
            awaitProgress(blk, blk.progress->value(), writer.buffer().size());
            blk.applyChangedSettings();
            continue;
        }
        done += nCopy;
        blk.progress->incrementAndGet();
        blk.progress->notify_all();
    }
}

} // namespace capture
