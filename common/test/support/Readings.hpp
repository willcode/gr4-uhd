/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/ControlDesc.hpp>

#include <algorithm>
#include <string>
#include <string_view>
#include <vector>

namespace capture::test {

// The ids of a reading set, in the order the block published them.
inline std::vector<std::string> readingIds(const std::vector<SensorReading>& readings) {
    std::vector<std::string> ids;
    for (const SensorReading& r : readings) {
        ids.push_back(r.id);
    }
    return ids;
}

// The labels of the readings a glance row shows, in the order the block published them.
inline std::vector<std::string> briefLabels(const std::vector<SensorReading>& readings) {
    std::vector<std::string> labels;
    for (const SensorReading& r : readings) {
        if (r.brief) {
            labels.push_back(r.label);
        }
    }
    return labels;
}

// The reading published under id, and a reading with an empty id where the set holds none.
inline SensorReading readingOf(const std::vector<SensorReading>& readings, std::string_view id) {
    const auto it = std::ranges::find_if(readings, [id](const SensorReading& r) { return r.id == id; });
    return it == readings.end() ? SensorReading{} : *it;
}

} // namespace capture::test
