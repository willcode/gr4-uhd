/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <uhd/types/device_addr.hpp>

#include <exception>
#include <string>

/*| role: the address the offline cases hand a start or a probe that has to fail before any
        device is asked for, and the check that UHD's parser refuses it.
    why: UHD 4.9.0.1's device_addr_t parser throws on a pair with two separators, so a start or
        a probe given this address never reaches the device list. A UHD that parsed it would
        pass the device list a hint with an unknown key, which B2xx discovery ignores, and the
        case would open any attached USRP. Each case checks the refusal first and fails rather
        than start.
*/
namespace capture::uhd::test {

inline const std::string kUnparsedAddress = "a=b=c";

inline bool uhdRefusesAddress(const std::string& address = kUnparsedAddress) {
    try {
        const ::uhd::device_addr_t parsed(address);
        return false;
    } catch (const std::exception&) {
        return true;
    }
}

} // namespace capture::uhd::test
