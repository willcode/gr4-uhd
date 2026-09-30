# Copyright 2026 Jeff Long
# SPDX-License-Identifier: MIT

# The USRP: capture::uhd::Source and capture::uhd::Sink over libuhd.

# UHD 4 is the API the USRP source and sink are written against: multi_usrp's 4.x
# streamer interface, its property-tree paths and the metadata error codes. The
# detection therefore names a minimum. Without one, an older libuhd is reported found
# here and fails in a compile: in the header parse where the top runs one, or inside a
# consumer's own tree.
# UHD_VERSION is MAJOR * 1000000 + API * 10000 + ABI * 100 + PATCH.
set(CAPTURE_UHD_MINIMUM 4000000)
find_library(CAPTURE_UHD_LIB NAMES uhd libuhd.so.4)
find_path(CAPTURE_UHD_INCLUDE uhd/usrp/multi_usrp.hpp)
set(CAPTURE_UHD_FOUND FALSE)
set(CAPTURE_UHD_VERSION "")
if(CAPTURE_UHD_LIB AND CAPTURE_UHD_INCLUDE AND EXISTS "${CAPTURE_UHD_INCLUDE}/uhd/version.hpp")
  file(STRINGS "${CAPTURE_UHD_INCLUDE}/uhd/version.hpp" CAPTURE_UHD_VERSION_LINE
       REGEX "^#define[ \t]+UHD_VERSION[ \t]")
  string(REGEX MATCH "[0-9]+$" CAPTURE_UHD_VERSION "${CAPTURE_UHD_VERSION_LINE}")
  if(CAPTURE_UHD_VERSION AND NOT CAPTURE_UHD_VERSION LESS CAPTURE_UHD_MINIMUM)
    set(CAPTURE_UHD_FOUND TRUE)
  endif()
endif()
if(CAPTURE_UHD_FOUND)
  message(STATUS "capture: libuhd FOUND (${CAPTURE_UHD_LIB}, UHD_VERSION ${CAPTURE_UHD_VERSION})")
elseif(CAPTURE_UHD_LIB AND CAPTURE_UHD_INCLUDE)
  message(STATUS "capture: libuhd NOT USED (UHD_VERSION '${CAPTURE_UHD_VERSION}' is below ${CAPTURE_UHD_MINIMUM})")
else()
  message(STATUS "capture: libuhd NOT FOUND (no library or headers)")
endif()

capture_device(uhd INCLUDE ${CAPTURE_UHD_INCLUDE} LIBRARY ${CAPTURE_UHD_LIB} VERDICT ${CAPTURE_UHD_FOUND})

if(NOT CAPTURE_TESTS OR NOT CAPTURE_GR4)
  return()
endif()

# A USRP states its control surface instead of carrying a table, so its decisions are a
# binary of their own, linking libuhd.
if(CAPTURE_UHD_FOUND)
  capture_test(
    qa_controls_uhd
    DIR test
    CASES controls.uhd-frequency-out-of-range-refused controls.uhd-agc-hands-the-gain-back
          controls.uhd-counts-what-the-device-shed controls.uhd-forwards-the-frequency-the-tuner-holds
          controls.uhd-rate-ladder-from-the-device controls.uhd-skips-an-unchanged-write
          controls.uhd-reference-lock-wait-is-bounded controls.uhd-tag-time-comes-from-the-device
          controls.uhd-offers-only-the-controls-the-device-has controls.uhd-master-clock-ladder
          controls.uhd-a-clock-span-takes-any-clock-in-it
          controls.uhd-transport-sizing controls.uhd-device-time-from-the-pps
          controls.uhd-describes-the-unit controls.uhd-holds-the-tick-grid-across-a-clock-set
          controls.uhd-reference-selection-comes-from-the-device-list
          controls.uhd-states-when-each-control-applies
          controls.uhd-reports-a-rate-the-host-cannot-carry controls.uhd-a-pinned-clock-is-a-fixed-clock
          controls.uhd-a-replay-of-the-clock-in-force-writes-nothing
          controls.uhd-the-overall-gain-writes-through-its-setting controls.uhd-a-bounded-gain-is-written-back
          controls.uhd-the-drain-writes-the-bounded-gain-back
          controls.uhd-the-stated-coverage-is-the-coverage-in-force
          controls.uhd-the-probe-reads-the-surface-the-block-runs-on
          controls.uhd-a-replay-of-the-selection-in-force-says-nothing
          controls.uhd-bounds-the-lo-offset controls.uhd-a-silent-receive-counts-whatever-it-says
          controls.uhd-a-gap-marker-names-its-cause controls.uhd-an-unconnected-output-ends-the-block controls.uhd-a-failed-start-throws
          controls.uhd-a-thread-that-does-not-start-ends-the-start
          controls.uhd-a-retune-forwards-nothing-ahead-of-its-samples controls.uhd-a-retune-tag-lands-on-the-first-sample-it-reached
          controls.uhd-an-overflow-during-a-retune-is-named-for-it controls.uhd-a-shed-no-counter-explains-is-marked
          controls.uhd-stamps-off-the-sample-grid-count-only-lost-samples
          controls.uhd-a-receive-asks-for-whole-frames
          controls.uhd-a-refused-clock-range-keeps-the-clock controls.uhd-the-transport-keys-follow-the-direction
          controls.uhd-reads-a-disciplined-second controls.uhd-device-listing-names-one-unit-each
          controls.uhd-the-source-rate-answers-for-two-radios controls.uhd-a-refused-filter-write-states-no-width
          controls.uhd-an-mpm-device-is-cut-at-its-own-rate controls.uhd-the-link-ceiling-is-the-rate-uhd-states
          controls.uhd-several-clock-rates-are-a-variable-clock
          controls.uhd-a-listing-asks-for-dpdk-where-asked
          controls.uhd-a-refused-read-costs-its-own-field controls.uhd-bounds-every-control-value
          controls.uhd-a-settings-pass-never-waits-on-a-tune
          controls.uhd-states-its-rate-and-frequency-from-the-truth
          controls.uhd-a-radio-stating-clocks-above-the-converter-picks
          controls.uhd-a-product-table-offers-the-clocks-of-a-fixed-clock-radio
          controls.uhd-a-table-clock-applies-at-the-next-open
          controls.uhd-a-list-takes-its-entries-alone controls.uhd-a-failed-open-clears-the-clock-pin
          controls.uhd-a-settings-write-clears-the-pins
          controls.uhd-the-receive-depth-is-stated-in-time controls.uhd-a-drain-ends-by-duration-at-any-rate
          controls.uhd-thread-placement-is-the-callers
          controls.uhd-states-its-filter-and-its-gain-mode
          controls.uhd-an-unstarted-source-states-the-ladder-of-its-staged-format
          controls.uhd-a-wait-for-room-ends-at-the-consume-or-the-bound
          controls.uhd-states-the-antennas-the-unit-names controls.uhd-the-antenna-writes-through-its-setting
          controls.uhd-refuses-an-antenna-the-unit-does-not-offer
    LIBS ${CAPTURE_GR4} ${CAPTURE_UHD_LIB}
    INCLUDES ${CAPTURE_UHD_INCLUDE})

  capture_test(
    qa_sink_uhd
    DIR test
    CASES sink.uhd-antenna-and-bandwidth
          sink.uhd-a-full-ring-consumes-nothing sink.uhd-a-send-reads-the-queue-in-place
          sink.uhd-a-work-call-clips-into-the-queue
          sink.uhd-a-send-takes-the-span-up-to-the-cap sink.uhd-a-write-wakes-the-waiting-drain
          sink.uhd-a-rate-change-reuses-the-queue-that-fits sink.uhd-the-stop-bound-reads-the-stated-buffer
          sink.uhd-the-queue-follows-its-settings
          sink.uhd-a-rate-change-ends-the-burst
          sink.uhd-listing-pairs-are-the-sources
          sink.uhd-refuses-a-rate-that-is-not-a-positive-number
          sink.uhd-bounds-the-transmit-gain-at-the-device
          sink.uhd-states-the-frequency-the-caller-asked-for
          sink.uhd-every-control-it-describes-it-writes
          sink.uhd-the-buffers-outlive-the-threads
          sink.uhd-a-queue-reading-reads-no-storage
          sink.uhd-the-queue-follows-the-rate
          sink.uhd-a-stop-that-took-the-handle-calls-no-library
          sink.uhd-a-rate-change-counts-what-it-discards
          sink.uhd-a-held-stop-ends-at-once
          sink.uhd-a-parked-drain-ends-the-burst-once
          sink.uhd-a-rate-change-waits-for-a-fresh-park
          sink.uhd-a-refused-park-keeps-the-queue
          sink.uhd-a-rate-change-before-the-first-send-resizes-the-queue
          sink.uhd-a-resize-down-gives-the-storage-back
          sink.uhd-a-play-out-is-bounded-by-the-rate-in-force
          sink.uhd-a-refused-park-is-retried-until-it-lands
          sink.uhd-a-rate-change-does-not-wait-on-a-send
          sink.uhd-a-stop-request-waits-for-the-control-mutex
          sink.uhd-a-start-that-cannot-parse-its-address-returns sink.uhd-a-failed-start-throws sink.uhd-a-failed-start-ends-the-run sink.uhd-a-failed-source-start-ends-the-run sink.uhd-a-release-runs-behind-the-stop
          sink.uhd-a-destroyed-block-leaves-the-release-running sink.uhd-the-release-ends-before-the-library-at-exit
          sink.uhd-a-second-wait-waits-for-the-release-in-flight
          sink.uhd-both-blocks-describe-one-truth-alike
          sink.uhd-snaps-the-rate-as-the-source-does
          sink.uhd-a-refused-rate-write-reads-the-rate-back
          sink.uhd-waits-for-the-reference
          sink.uhd-reads-the-transmit-sensors
          sink.uhd-readings-are-what-the-stream-produces
          sink.uhd-names-a-gpsdo-the-open-did-not-find
          sink.uhd-marks-a-lock-only-where-it-applies
          sink.uhd-an-empty-element-name-is-the-overall-gain
          sink.uhd-a-gain-that-is-not-a-number-is-its-floor
          sink.uhd-names-a-control-value-it-moves
          sink.uhd-a-rate-read-back-that-fails-states-no-rate
          sink.uhd-a-control-writes-through-its-setting
          sink.uhd-times-its-sensors-at-the-first-sweep
          sink.uhd-a-stop-ends-the-reference-wait
          sink.uhd-an-address-that-pins-the-clock-fixes-the-ladder
          sink.uhd-counts-the-burst-acknowledgment
          sink.uhd-counts-transmit-events-by-bit
          sink.uhd-a-stop-waits-for-the-burst-acknowledgment
          sink.uhd-a-start-a-stop-ended-arms-nothing
          sink.uhd-a-stop-waits-for-its-own-acknowledgment
          sink.uhd-a-start-configures-the-rate-first
          sink.uhd-a-rate-nobody-read-back-is-written-again
          sink.uhd-a-refused-start-leaves-the-rate-asked-for
          sink.uhd-the-device-says-whether-its-clock-is-fixed
          sink.uhd-a-loaded-host-keeps-the-brief-sensors
          sink.uhd-states-the-rate-in-force
          sink.uhd-the-surface-is-read-without-the-control-mutex
          sink.uhd-both-blocks-declare-their-labels
    LIBS ${CAPTURE_GR4} ${CAPTURE_UHD_LIB}
    INCLUDES ${CAPTURE_UHD_INCLUDE})

  # The discovery library's source built over this directory's fragment alone. No case asks
  # a device: the one probe names an address UHD's parser refuses before any device is asked.
  capture_test(
    qa_discovery_uhd
    DIR test
    CASES discovery.uhd-the-library-carries-the-usrp-family
          discovery.uhd-a-probe-names-an-address-it-cannot-open
          discovery.uhd-a-family-the-library-lacks-is-refused
    LIBS ${CAPTURE_GR4} ${CAPTURE_UHD_LIB}
    INCLUDES ${CAPTURE_UHD_INCLUDE})
  capture_discovery_sources(qa_discovery_uhd uhd)
else()
  message(STATUS "capture: the USRP cases are SKIPPED (libuhd is absent or below the minimum version)")
endif()

# The USRP cases that need the radio itself. They open a device, so they are off by
# default, carry the label hardware rather than the offline cases' label, and are never
# part of the hardware-free suite. One invocation runs one case, which opens the device and
# releases it; CAPTURE_HW_UHD_ARGS in the environment selects a unit other than the default.
option(CAPTURE_HW_UHD "Build the USRP cases that open a radio" OFF)
if(CAPTURE_HW_UHD AND CAPTURE_UHD_FOUND)
  set(CAPTURE_HW_UHD_CASES
      uhd.open-stream-close uhd.rate-ladder uhd.tuning-and-range uhd.frequency-correction-sign
      uhd.gain uhd.hardware-information uhd.agc uhd.antenna uhd.analog-bandwidth uhd.sensors
      uhd.overflow-under-a-stalled-consumer uhd.teardown-and-restart uhd.corrections
      uhd.correction-defaults uhd.device-string-keys uhd.clock-and-time-source
      uhd.reference-lock-wait uhd.lo-offset uhd.device-time uhd.device-time-source
      uhd.device-time-set-midstream uhd.tag-time-after-a-stall uhd.property-tree-link-rate
      uhd.rate-change-midstream uhd.a-drain-at-the-top-rate-ends-by-duration uhd.accessors-after-a-stop uhd.stop-latency
      uhd.coverage-past-the-stated-range uhd.vendor-agc-claim
      uhd.wire-format uhd.transport-sizing uhd.master-clock uhd.tx-burst uhd.tx-loopback
      uhd.tx-underrun)
  capture_test(
    hw_uhd
    DIR test
    CASES ${CAPTURE_HW_UHD_CASES}
    LIBS ${CAPTURE_GR4} ${CAPTURE_UHD_LIB}
    INCLUDES ${CAPTURE_UHD_INCLUDE})
  # Each of these runs a radio for seconds at a time, and the rate ladder restarts the
  # graph once per rate, so the ceiling is far above the arithmetic cases'.
  set_tests_properties(${CAPTURE_HW_UHD_CASES} PROPERTIES TIMEOUT 180 LABELS hardware)
  message(STATUS "capture: the USRP hardware cases are ENABLED (label hardware)")
endif()
