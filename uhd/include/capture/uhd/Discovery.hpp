/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/Discovery.hpp>
#include <capture/uhd/Sink.hpp>
#include <capture/uhd/Source.hpp>

namespace capture::uhd {

/*| role: the blocks the discovery library lists and probes for the USRP family: its source and
        its sink.
*/
using DiscoveryBlocks = capture::BlockList<Source, Sink>;

} // namespace capture::uhd
