/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/SchedulerModel.hpp>

#include <chrono>
#include <expected>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace capture::test {

/*| role: a graph held by a GNU Radio 4 scheduler, with a reader on the scheduler's message
        output. A case runs the graph the two ways a program does: to its end on the calling
        thread, or on the scheduler's own thread until the case stops it.
    contract: every call goes to the typed scheduler or to its SchedulerWrapper. A case
        therefore builds against any GNU Radio 4 core that has the two.
    trap: a scheduler whose message output has no reader turns a block's error into an
        exception on its worker thread, and the block's reason is lost with it. adopt()
        connects the reader before the graph reaches the scheduler. The reader is declared
        ahead of the scheduler, so the scheduler is destroyed first.
    contract: send() writes to the scheduler's message input, as a program outside the graph
        does, and the scheduler hands the message to the block it names. Every message read
        off the output waits in one backlog until a call takes it, so await() and errors()
        each see what the other left.
*/
template <gr::scheduler::ExecutionPolicy policy = gr::scheduler::ExecutionPolicy::singleThreaded>
class ScheduledGraph {
public:
    // Connect the reader and hand the graph to the scheduler. The error is the refusal of
    // either step.
    [[nodiscard]] std::expected<void, gr::Error> adopt(gr::Graph&& graph) {
        if (auto connected = _scheduler.blockRef().msgOut.connect(_events); !connected) {
            return std::unexpected(connected.error());
        }
        if (auto connected = _requests.connect(_scheduler.blockRef().msgIn); !connected) {
            return std::unexpected(connected.error());
        }
        if (auto exchanged = _scheduler.blockRef().exchange(std::move(graph)); !exchanged) {
            return std::unexpected(exchanged.error());
        }
        return {};
    }

    // Run to the end on the calling thread, as a program's main() does.
    [[nodiscard]] std::expected<void, gr::Error> runAndWait() { return _scheduler.blockRef().runAndWait(); }

    // Start on the scheduler's own thread and return.
    void start() { _scheduler.start(); }

    // Request a stop and join the scheduler's thread.
    void stop() { _scheduler.stop(); }

    // The error reports on the message output since the last call, oldest first.
    [[nodiscard]] std::vector<std::string> errors() {
        drain();
        std::vector<std::string> reasons;
        for (const gr::Message& message : _backlog) {
            if (!message.data.has_value()) {
                reasons.push_back(message.data.error().message);
            }
        }
        _backlog.clear();
        return reasons;
    }

    // Write one message to the scheduler's message input.
    void send(gr::Message message) {
        auto span = _requests.streamWriter().template reserve<gr::SpanReleasePolicy::ProcessAll>(1UZ);
        span[0]   = std::move(message);
    }

    // The oldest message on the output the predicate accepts, taken out of the backlog, or
    // nothing once the wait has passed without one.
    template <typename Accept>
    [[nodiscard]] std::optional<gr::Message> await(Accept&& accept, std::chrono::milliseconds wait) {
        const auto deadline = std::chrono::steady_clock::now() + wait;
        for (;;) {
            drain();
            for (auto it = _backlog.begin(); it != _backlog.end(); ++it) {
                if (accept(*it)) {
                    gr::Message found = std::move(*it);
                    _backlog.erase(it);
                    return found;
                }
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return std::nullopt;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

private:
    void drain() {
        auto messages = _events.streamReader().get();
        _backlog.insert(_backlog.end(), messages.begin(), messages.end());
        std::ignore = messages.consume(messages.size());
    }

    gr::MsgPortIn                                       _events;
    gr::MsgPortOut                                      _requests;
    std::vector<gr::Message>                            _backlog;
    gr::SchedulerWrapper<gr::scheduler::Simple<policy>> _scheduler;
};

} // namespace capture::test
