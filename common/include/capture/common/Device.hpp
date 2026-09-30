/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/LifeCycle.hpp>
#include <gnuradio-4.0/Message.hpp>

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Properties.hpp>
#include <capture/common/Ring.hpp>
#include <capture/common/Threading.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <functional>
#include <mutex>
#include <source_location>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>

/*| role: the parts every device block shares, whichever direction it runs in.
    frame: each block still owns its device-specific logic: open and close, the apply helpers,
        the gain vocabulary, and whatever threading model its library forces — callback-push,
        read-pull, or an API-owned callback over a ring. This header holds the RAII
        streaming-thread guard, the teardown guard, the control guard and the two duties a
        source owes its output port. It carries
        capture/common/ControlDesc.hpp, where the control descriptors, the device-string reader
        and the coverage test live, and capture/common/Ring.hpp, where the drop-newest ring and
        its helpers live.
    contract: every apply helper reachable from settingsChanged() is idempotent and suppresses a
        write of an unchanged value.
    trap: the settings machinery replays the staged settings map shortly after streaming starts,
        so each applier runs at least twice with unchanged values — once before start and once
        mid-stream. A device can mishandle the redundant write, and a rate write that reaches a
        streaming device can stop its sample clock until the device is reset. Cache the last
        applied value and skip unchanged writes.
*/
namespace capture {

/*| contract: the scale a reference error of ppm parts per million puts on a tuner, one plus
        the fraction.
    invariant: a correction of minus a million parts per million or below drives the factor to
        zero or turns its sign over, and there this answers one, so the caller applies no
        correction rather than a division by zero or a frequency of the wrong sign.
*/
constexpr double referenceScale(double ppm) {
    const double factor = 1.0 + ppm / 1e6;
    return factor > 0.0 ? factor : 1.0;
}

/*| contract: the frequency to command a tuner for a request under a reference error of ppm
        parts per million.
    frame: a positive correction states that the reference runs fast. A fast reference makes
        the synthesizer land above where it was told to by the same fraction, so the command is
        the request divided by that factor and the radio ends on the frequency asked for.
    why: the exact form rather than its first-order expansion: the two differ by the request
        times the square of the fraction, which is 60 Hz at 6 GHz and 100 ppm against a tuning
        resolution of about 50 Hz. A radio that carries no correction register applies it in
        both directions, so the arithmetic and its bound are stated once.
    verified-by: tune.a-fast-reference-tunes-low
*/
constexpr double correctedTuneHz(double requestHz, double ppm) { return requestHz / referenceScale(ppm); }

/*| role: what a source keeps about its output port for the duties it owes that port.
    invariant: a member of every source, declared before the teardown guard.
*/
struct OutputState {
    std::atomic<bool>        connectedAtStart{false};
    std::atomic<std::size_t> droppedTags{0};   // the port's refused-tag count, as last published
    std::atomic<bool>        rateTagOwed{false}; // the stream's first block carries sample_rate
};

/*| contract: record whether the block's output port had a reader when the block started, and
        owe the stream that starts the rate in force as a tag on its first block. Each source
        calls this first in its start().
    frame: the scheduler connects every edge of a graph before it starts a block, so a port that
        reads unconnected here was never given a consumer.
*/
template <typename Block>
void noteOutputAtStart(Block& blk) {
    blk._outputState.connectedAtStart.store(blk.out.isConnected(), std::memory_order_release);
    blk._outputState.rateTagOwed.store(true, std::memory_order_release);
}

/*| contract: true where the block's output port reaches nothing, having asked the block to stop;
        false where the port reaches something and the work call goes on to look at the device.
        The fault is named only for a port that had no reader at the start. A port that had one
        and lost it is the ordinary end of a graph traveling up from its consumer, which
        releases its readers when it finishes, and the stop is asked for without a message, as
        the framework's own test for an unconnected downstream does. False as well where the
        block's disconnect_on_done setting is false: a caller that asked the block to stay when
        nothing reads it keeps the block, as that same test does.
    trap: a source whose output reaches nothing never ends its publish by itself. The default
        buffer fills once, the writer offers no room from then on, and the streaming thread
        sleeps and retries while the device keeps delivering into a ring that sheds. The base
        class tests for an unconnected downstream inside a work path a device source
        overrides, so the test belongs in the override.
    frame: each source calls this behind its own lifecycle test, so a block already asked to
        stop names no fault.
    verified-by: the device source's own case ending an-unconnected-output-ends-the-block,
        which also reads the source's droppedTags()
    verified-by: publish.a-block-kept-when-nothing-reads-it-is-not-ended
    verified-by: publish.an-ordinary-end-from-the-consumer-names-no-fault
*/
template <typename Block>
bool stopWhenOutputIsUnconnected(Block& blk) {
    if (!blk.disconnect_on_done || blk.out.isConnected()) {
        return false;
    }
    if (!blk._outputState.connectedAtStart.load(std::memory_order_acquire)) {
        blk.emitErrorMessage("work()", "the output port is connected to nothing");
    }
    blk.requestStop();
    return true;
}

/*| role: what a device block's start() throws when it cannot proceed.
    contract: what() answers the reason alone, and sourceLocation keeps the place of the throw.
        The framework's lifecycle quotes what() behind the block's name, so the error a run
        fails with reads as the block's own sentence.
*/
struct StartFailure : gr::exception {
    using gr::exception::exception;

    [[nodiscard]] const char* what() const noexcept override { return message.c_str(); }
};

/*| contract: end a start that cannot proceed. The reason goes to the console under the block's
        tag, and a StartFailure carrying it leaves start(). The framework catches it and puts
        the block in ERROR. The scheduler then winds the run down and runAndWait() fails with
        the reason.
    invariant: the caller has released whatever its start opened. A block whose start threw is
        never given stop(): ERROR leads only to the initialized state, the scheduler winds down
        only the blocks that reached RUNNING, and the base destructor acts only on an active
        block.
    verified-by: start.a-failed-source-start-ends-the-run
    verified-by: start.a-failed-sink-start-ends-the-run
*/
[[noreturn]] inline void throwStartFailure(std::string_view tag, std::string_view reason, std::source_location where = std::source_location::current()) {
    std::fprintf(stderr, "[%.*s] the start failed: %.*s\n", static_cast<int>(tag.size()), tag.data(), static_cast<int>(reason.size()), reason.data());
    throw StartFailure(reason, where);
}

/*| contract: carry the port's own count of refused tags into the block's copy. Called on the
        publishing thread after each tag the block publishes, which is the one thread that moves
        the port's count.
*/
template <typename Block>
void noteDroppedTags(Block& blk) {
    blk._outputState.droppedTags.store(blk.out.nTagsDropped, std::memory_order_relaxed);
}

/*| contract: the tags this block's output port could not publish since the graph was built, as
        of the last tag the block published. Safe from any thread.
    trap: a tag ring with no room makes publishTag refuse. The port counts the refusal and
        prints one line, and a gap marker lost that way leaves a hole the stream never reports —
        against the guarantee that a marker says which kind of gap occurred. The exposure is a
        run of very short chunks each carrying a marker, the consumer loops publishing at least
        one sample per marker.
    trap: the port's counter is a plain integer the publishing thread increments, and a sensor
        sweep runs on another thread. Read there, it is a data race under the language's memory
        model and a thread-sanitized consumer reports it, so the reading is the block's own
        atomic copy.
    why: read where the ring's own dropped count is read, so the two ways a gap goes unannounced
        are answered in one place.
    verified-by: publish.the-lost-marker-count-is-read-from-the-block
*/
template <typename Block>
std::size_t droppedTagCount(const Block& blk) {
    return blk._outputState.droppedTags.load(std::memory_order_relaxed);
}

/*| contract: name the calling streaming thread <tag>:<blockName>.
    frame: tag says which device library and which of its threads this is, one short word for
        a reader and another for the publisher behind it, and blockName says which block in the
        graph owns it, so a graph carrying two of one kind still reads apart. The form is the
        <library>:<role> the stock GNU Radio 4 sources use, which keeps an application's device
        threads and the framework's own sitting in one sorted run of top -H.
    trap: keep tag short enough that the block name survives: the budget is
        capture::threading::kMaxThreadNameLength characters for the whole thing, and a tag over
        eight characters starts eating the part that says which block it is.
    why: deliberately not gr::thread_pool::thread::setThreadName, which passes the name through
        to pthread_setname_np untouched and tests the result for rc < 0. The failure for an
        over-long name is ERANGE, a positive code, so that test never fires and the thread
        silently keeps the executable's name. Several device threads here carry a tag past
        that limit.
*/
inline void nameStreamingThread(std::string_view tag, std::string_view blockName) {
    std::string name(tag);
    name += ':';
    name += blockName;
    capture::threading::nameOwnThread(name);
}

/*| contract: place the calling streaming thread as priority and cpu ask, on the scale
        capture::threading::placeOwnThread states, and name on standard error under blockTag
        what was asked and what the system granted. A thread asked nothing is left alone and
        named nowhere. Answers the placement.
    why: a placement is the caller's choice, as a transport depth is, and a refusal costs the
        thread a share of the processor and never a result, so it is a line and not a failed
        start.
    verified-by: threading.an-asked-placement-is-applied-or-named
*/
inline capture::threading::Placement placeStreamingThread(std::string_view blockTag, std::string_view role, double priority, double cpu) {
    const auto placed = capture::threading::placeOwnThread(priority, cpu);
    if (placed.asked()) {
        std::fprintf(stderr, "[%.*s] %.*s thread: %s\n", static_cast<int>(blockTag.size()), blockTag.data(), static_cast<int>(role.size()), role.data(), placed.describe(priority, cpu).c_str());
    }
    return placed;
}

/*| role: a joinable streaming thread that is always stopped and joined on destruction.
    why: device blocks are gr aggregates: emplaceBlock aggregate-initializes the base
        from the settings map, so they cannot declare their own destructor. Yet a stop() racing
        a slow start() can move the block from the initialized state to REQUESTED_STOP without
        ever calling its stop() hook, leaving a live thread to be joined in
        ~BlockWrapper, which calls std::terminate().
    trap: observed as a SIGABRT when a graph was stopped during a rate switch. Holding the
        thread in a member with this destructor is the safety net; the block's own stop() still
        joins eagerly on the normal path.
*/
struct ConsumerThread {
    std::thread       thread;
    std::atomic<bool> stopFlag{true}; // true = not running / stop requested

    void requestStop() { stopFlag.store(true, std::memory_order_release); }
    bool stopRequested() const { return stopFlag.load(std::memory_order_acquire); }
    void clearStop() { stopFlag.store(false, std::memory_order_release); }

    /*| contract: run body on this member's thread. Whatever thread the member already holds is
            asked to stop and joined first, and the stop flag is clear before body can read it.
        trap: std::thread::operator= on a joinable target calls std::terminate. Assigning the
            member directly rests on nothing here ever being asked to start twice, which is a
            rule the lifecycle machine keeps somewhere else entirely — and the documented hole
            on this type, a stop racing a slow start, is exactly a sequence that leaves a thread
            behind for the next start to land on.
        verified-by: consumerthread.a-start-replaces-the-thread-it-holds
    */
    template <typename Body>
    void start(Body&& body) {
        requestStop();
        joinIfRunning();
        clearStop();
        thread = std::thread(std::forward<Body>(body));
    }

    void joinIfRunning() {
        if (thread.joinable()) {
            thread.join();
        }
    }

    /*| invariant: the destructor's bare join assumes that a thread parked in a device-library
            call which ignores stopFlag has already been unblocked by the owner's
            hardwareTeardown().
        why: the owner guarantees the ordering structurally, by declaring its TeardownGuard
            after every ConsumerThread member: the guard is destroyed first and runs teardown,
            which owns the library-specific unblocking, while these threads are still joinable.
    */
    ~ConsumerThread() {
        requestStop();
        joinIfRunning();
    }
};

/*| role: a thread of a block's own that makes the block's device control calls in the order
        they were asked for, away from the thread that streams.
    contract: start() runs the thread; post(key, work) queues work and returns at once. Work
        queued under a key that has not begun leaves its place, and the new work goes to the
        back, so the last write of a key wins and the writes that remain run in the order given.
        settle() returns once nothing is queued or running, and settleFor() does the same within
        a duration, answering whether it did. stop() drops what has not begun, waits for what is
        running and joins the thread; a post while stopped runs nothing.
    why: a device control call can outlast the buffering a stream has. A B205mini retune takes
        about 100 ms, and a receive thread that made the call would not drain the device for
        that long.
    invariant: one piece of work runs at a time, and the lock is not held across it, so the
        work may take the block's own locks and post() never waits for a device.
    trap: settle() and stop() wait for the running piece, so neither may be called while
        holding a lock that piece takes.
    verified-by: controlworker.writes-run-in-order-and-the-last-of-a-key-wins
*/
struct ControlWorker {
    ~ControlWorker() { stop(); }

    void start(std::string_view tag, std::string_view blockName) {
        stop();
        {
            std::lock_guard lock(_mutex);
            _stopping = false;
        }
        _thread = std::thread([this, name = std::string(tag) + ':' + std::string(blockName)] {
            capture::threading::nameOwnThread(name);
            run();
        });
    }

    void post(int key, std::function<void()> work) {
        {
            std::lock_guard lock(_mutex);
            if (_stopping) {
                return;
            }
            std::erase_if(_queue, [key](const auto& queued) { return queued.first == key; });
            _queue.emplace_back(key, std::move(work));
        }
        _wake.notify_all();
    }

    void settle() {
        std::unique_lock lock(_mutex);
        _wake.wait(lock, [this] { return _stopping || (_queue.empty() && !_running); });
    }

    /*| contract: settle() bounded by limit. True where nothing is queued or running at the
            return, or the worker is stopping; false where the limit passed first, the work
            still queued or running.
        verified-by: controlworker.a-settle-ends-by-its-bound
    */
    bool settleFor(std::chrono::duration<double> limit) {
        std::unique_lock lock(_mutex);
        return _wake.wait_for(lock, limit, [this] { return _stopping || (_queue.empty() && !_running); });
    }

    void stop() {
        {
            std::lock_guard lock(_mutex);
            _stopping = true;
            _queue.clear();
        }
        _wake.notify_all();
        if (_thread.joinable()) {
            _thread.join();
        }
    }

private:
    void run() {
        std::unique_lock lock(_mutex);
        for (;;) {
            _wake.wait(lock, [this] { return _stopping || !_queue.empty(); });
            if (_stopping) {
                break;
            }
            std::function<void()> work = std::move(_queue.front().second);
            _queue.pop_front();
            _running = true;
            lock.unlock();
            work();
            lock.lock();
            _running = false;
            _wake.notify_all();
        }
    }

    std::mutex                                        _mutex;
    std::condition_variable                           _wake;
    std::deque<std::pair<int, std::function<void()>>> _queue;
    bool                                              _running  = false;
    bool                                              _stopping = true;
    std::thread                                       _thread;
};

/*| role: run the block's idempotent hardwareTeardown() when the block is destroyed, and leave
        the lifecycle state one ~Block() will not act on.
    why: the stop-races-start hole described on ConsumerThread leaves more than a joinable
        thread behind: a vendor library's callback keeps writing into the block's ring after
        destruction, and open device handles leak. Each device block therefore has an
        idempotent hardwareTeardown(), its stop() body minus the end-of-stream publish, and
        holds this guard as its last data member — members are destroyed in reverse declaration
        order, so the guard runs teardown first, while the rings, the scratch buffers and the
        gr::Block base are all still alive.
    invariant: on the normal path stop() already ran teardown and the guard is a no-op.
    contract: hardwareTeardown() must not throw. This destructor is implicitly noexcept, so an
        exception leaving it ends the process, and the vendor calls a teardown makes are the
        kind that raise, a transport teardown for a device that has gone among them, and none of
        the hooks here is declared noexcept. The catch-all below turns such an
        escape into a leaked handle during block destruction, which is the smaller of the two
        failures and the only one a caller can act on.
    trap: the state drive covers the other way a block dies abnormally. A graph whose scheduler
        ends in ERROR is destroyed without its blocks ever leaving RUNNING, and ~Block() — the
        base destructor, running after every derived member is gone — invokes the stop() hook on
        any block still in an active state. Driving through REQUESTED_STOP here re-invokes the
        idempotent hook while the members it touches are still alive, and leaves a state the
        base destructor's net ignores.
*/
template <typename TBlock>
struct TeardownGuard {
    TBlock* self;
    ~TeardownGuard() {
        if (self == nullptr) {
            return;
        }
        try {
            self->hardwareTeardown();
        } catch (...) {
        }
        if (gr::lifecycle::isActive(self->state())) {
            std::ignore = self->changeStateTo(gr::lifecycle::State::REQUESTED_STOP);
            std::ignore = self->changeStateTo(gr::lifecycle::State::STOPPED);
        }
    }
};

/*| contract: take a block's control mutex and re-test device liveness under it. The guard is
        true only while the device handle is guaranteed to stay valid for the scope, so a caller
        writes CtrlGuard guard(_ctrlMutex, _deviceUp); if (!guard) { return false; } and
        then makes its device calls.
    frame: every teardown here has the same shape: clear the liveness flag first, then join the
        streaming thread, and only then take the control mutex to close the device and null its
        handle.
    trap: testing liveness before locking is not enough. A caller can pass the test, block on
        the mutex for as long as the join takes — a millisecond or more — and acquire it after
        the handle is already gone, writing through a null handle or racing the refcount control
        block where the handle is a shared_ptr. The window is not theoretical: a gain applier
        fires a burst of these calls the instant it sees the device, which is exactly when a
        start is most likely to be failing underneath it.
    why: testing only after the lock is held closes it, the close happening under this same
        mutex: once the lock is held the flag cannot be stale in the dangerous direction.
*/
class CtrlGuard {
public:
    CtrlGuard(std::mutex& ctrlMutex, const std::atomic<bool>& deviceUp) : _lock(ctrlMutex), _live(deviceUp.load(std::memory_order_acquire)) {}

    /*| contract: the same guard with a deadline. It gives up after waitFor and is then false,
            holding no lock, so a caller reads it exactly as it reads the blocking form and
            answers its own caller that the device could not be reached.
        why: for a read-only path called from the thread that serves an interface. The blocking
            form waits for whatever device call the streaming thread holds the mutex for, and
            one sensor sweep on an oscillator disciplined by a satellite fix waits for the next
            serial sentence, about a second. A window that stops for a second is worse than a
            reading that did not arrive.
        frame: a poll rather than a timed lock, because the mutex belongs to the block and
            std::mutex has no bounded acquire. The poll interval is short against any device
            call it can be waiting on.
        verified-by: ctrlguard.a-bounded-wait-gives-up-and-holds-no-lock
    */
    CtrlGuard(std::mutex& ctrlMutex, const std::atomic<bool>& deviceUp, std::chrono::milliseconds waitFor) : _lock(ctrlMutex, std::defer_lock), _live(false) {
        const auto deadline = std::chrono::steady_clock::now() + waitFor;
        for (;;) {
            if (_lock.try_lock()) {
                _live = deviceUp.load(std::memory_order_acquire);
                return;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }

    // True while the device handle is guaranteed to stay valid for this scope.
    explicit operator bool() const noexcept { return _live; }

private:
    std::unique_lock<std::mutex> _lock;
    bool                         _live;
};

} // namespace capture
