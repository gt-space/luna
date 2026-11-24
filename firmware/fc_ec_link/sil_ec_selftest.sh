#!/bin/sh
# Builds sil_ec (with sanitizers) and the FC-side test client, then runs the
# end-to-end SIL self-test: Hello -> Arm -> demands -> safing -> Terminate ->
# ClearSafing, plus the fault-injection flags. See test_sil_ec.c.
#
#   ./sil_ec_selftest.sh            # build + test
#   SIL_TEST_VERBOSE=1 ./sil_ec_selftest.sh   # also dump sil_ec's EVENT lines
#
# Exit status 0 = every check passed. `make test` runs the same thing.
set -eu
cd "$(dirname "$0")"
make sil_ec sil_ec_san test_sil_ec
./test_sil_ec ./sil_ec_san
