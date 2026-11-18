CTV flight computer check (pi_test)
===================================

What this is
  A self-contained check that the CTV bench program runs on the flight
  computer. It flies the simulated CTV hop 5 times against a FAKE engine
  controller that runs on the flight computer itself (127.0.0.1), then
  tests that killing the program makes the fake EC safe itself.

  It never talks to the real EC or anything on the network, and it doesn't
  need the EC, BMS or actuators connected. It installs nothing and only
  writes inside its own folder plus one results file in your home folder.
  The luna checkout on the Pi is not touched.

  It takes about 3 minutes and needs no input.

1. Copy it to the flight computer (from a laptop on the same network)
     scp pi_test-aarch64.tar.gz ubuntu@flight.local:~

2. Run it (on the flight computer)
     ssh ubuntu@flight.local
     tar xzf pi_test-aarch64.tar.gz
     cd pi_test-aarch64
     ./pi_check.sh

   If any other flight software is running on the Pi (flight2 or a
   ctv-flight-computer process), stop it first: the check needs UDP ports
   4574 and 8378 on the Pi itself.

3. Send the results back
   The script ends with a PASS/FAIL table and the name of one results file:
     ~/pi_results-<hostname>-<date>.tar.gz
   Copy it back to the laptop and send it to Aaditya:
     scp ubuntu@flight.local:~/pi_results-*.tar.gz .
   Please send it even if the table says FAIL; that is what it is for.

Reading the table
  OS / glibc        The program needs glibc 2.39 or newer (Ubuntu 24.04 has
                    2.39). Older systems can't start it; the check then
                    stops early but still packs the system info.
  program starts    FAIL here usually means wrong OS or glibc, or a missing
                    program loader. Nothing else runs; send the results.
  run 1-5           touchdown should be about 15.97 s each time (PASS
                    between 14 and 18 s); overruns
                    should be 0; "demand gap" is the longest time between
                    two commands to the EC and must stay under 30 ms (the
                    EC's timeout), flagged above 15 ms; there must be no
                    mid-run safing.
  kill -9           PASS means the fake EC safed itself on its own timeout
                    when the flight program was killed.

Files in this folder
  ctv-flight-computer   the flight program (sim-plant build)
  fake_ec               the fake engine controller, built static
  pi_check.sh           this check
  gains/, bench.json, plant.json, bench.env, run.sh, eth_soak.sh,
  BENCH_CHECKLIST.md
                        the normal bench package (run.sh, eth_soak.sh and
                        bench.env are for later tests over the network;
                        pi_check.sh doesn't use them)
  MANIFEST.txt          checksums of every file; pi_check.sh verifies them
