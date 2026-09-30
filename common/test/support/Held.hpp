/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <chrono>
#include <future>
#include <mutex>
#include <thread>

namespace capture::test {

/*| contract: whether call returns within limit while the case's own thread holds mutex, as a
        thread making a device call under that mutex holds it. The call runs on a thread of its
        own, and the case lets the mutex go once the answer is known, so a call that waits on it
        fails the case rather than hanging it.
*/
template <typename Mutex, typename Call>
bool returnsWhileHeld(Mutex& mutex, Call&& call, std::chrono::milliseconds limit = std::chrono::milliseconds(1000)) {
    std::promise<void> returned;
    std::future<void>  answer = returned.get_future();
    std::unique_lock   held(mutex);
    std::thread        caller([&call, &returned] {
        call();
        returned.set_value();
    });
    const bool inTime = answer.wait_for(limit) == std::future_status::ready;
    held.unlock();
    caller.join();
    return inTime;
}

} // namespace capture::test
