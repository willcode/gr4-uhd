/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the plain discovery functions of a build that selects the USRP family alone.
    frame: no case asks a device. The one probe names the address "a=b=c", which UHD's
        address parser refuses before any device is asked, and a listing is never taken,
        since a USRP listing broadcasts on every interface UHD reaches.
*/
#include <capture/common/DiscoveryLibrary.hpp>

#include "support/Cases.hpp"

#include "UnparsedAddress.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
    using namespace boost::ut;
    capture::test::Cases cases(argc, argv);

    cases("discovery.uhd-the-library-carries-the-usrp-family", [] {
        const auto families = capture::discovery::families();
        expect(fatal(families.size() == 1UZ)) << "one family";
        expect(families[0].family == "uhd") << "named as the USRP blocks' device setting names it: " << families[0].family;
        expect(families[0].receive && families[0].transmit) << "with its source and its sink";
    });

    cases("discovery.uhd-a-probe-names-an-address-it-cannot-open", [] {
        expect(fatal(capture::uhd::test::uhdRefusesAddress())) << "UHD's parser refuses the address, so no device is asked for";
        const capture::TruthContext ctx{.deviceParams = "a=b=c", .sampleRate = 1.0e6, .centerFreq = 100.0e6};
        for (const bool transmit : {false, true}) {
            const auto probed = capture::discovery::probeControls("uhd", transmit, ctx);
            expect(fatal(!probed.has_value())) << "no surface, transmit " << transmit;
            expect(probed.error().find("could not open the device 'a=b=c'") != std::string::npos) << "the error names the address: " << probed.error();
        }
    });

    cases("discovery.uhd-a-family-the-library-lacks-is-refused", [] {
        const auto probed = capture::discovery::probeControls("none", false, {.deviceParams = "", .sampleRate = 1.0e6, .centerFreq = 100.0e6});
        expect(fatal(!probed.has_value())) << "no surface";
        expect(probed.error().find("no source of that family is compiled in") != std::string::npos) << "the error names the cause: " << probed.error();
    });

    const int  rc     = cases.report();
    const bool failed = boost::ut::cfg<boost::ut::override>.run({.report_errors = true});
    std::fflush(stdout);
    std::fflush(stderr);
    /*| trap: the binary links GNU Radio 4, whose global destructors run after its threads are
            gone, so the process ends without static teardown once the verdict is taken.
    */
    std::_Exit(failed ? 1 : rc);
}
