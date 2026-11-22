# Host builds of the fc_ec_link tests and the laptop fake EC. On the EC,
# compile fc_ec_link.c and ec_side.c into the firmware like any other source;
# nothing here is needed there.

CC ?= cc
WARNINGS = -std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
CFLAGS ?= $(WARNINGS) -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all

LINK = fc_ec_link.c fc_ec_link.h
EC = ec_side.c ec_side.h $(LINK)
ESC = esc.c esc.h $(EC)
MEAS = measurement.c measurement.h measurement_stm32h745.c measurement_stm32h745.h \
	temperature.c temperature.h fc_ec_link.h

test: test_fc_ec_link test_ec_side test_esc test_measurement test_pa12 test_sil_ec sil_ec_san
	./test_fc_ec_link
	./test_ec_side
	./test_esc
	./test_measurement
	./test_pa12
	./test_sil_ec ./sil_ec_san

test_fc_ec_link: test_fc_ec_link.c $(LINK)
	$(CC) $(CFLAGS) -o $@ test_fc_ec_link.c fc_ec_link.c -lm

test_ec_side: test_ec_side.c $(EC)
	$(CC) $(CFLAGS) -o $@ test_ec_side.c ec_side.c fc_ec_link.c -lm

test_esc: test_esc.c $(ESC)
	$(CC) $(CFLAGS) -o $@ test_esc.c esc.c ec_side.c fc_ec_link.c -lm

test_measurement: test_measurement.c $(MEAS)
	$(CC) $(CFLAGS) -o $@ test_measurement.c measurement.c measurement_stm32h745.c temperature.c -lm

test_pa12: test_pa12.c pa12_drv.c pa12_drv.h measurement.c measurement.h $(EC)
	$(CC) $(CFLAGS) -o $@ test_pa12.c pa12_drv.c measurement.c ec_side.c fc_ec_link.c -lm

# No sanitizers: this one is meant to be left running.
fake_ec: fake_ec.c $(EC)
	$(CC) $(WARNINGS) -O2 -o $@ fake_ec.c ec_side.c fc_ec_link.c -lm

# Software-in-the-loop EC: the real ec_side/esc/pa12_drv logic with simulated
# PA-12s and ESC, behind UDP (see sil_ec.c, sil_profile.h). Host only; never
# part of the firmware build. sil_ec_san is the same program with sanitizers,
# which `make test` drives from test_sil_ec.
SIL = sil_ec.c sil_profile.h esc.c esc.h pa12_drv.c pa12_drv.h measurement.c measurement.h $(EC)
SIL_SRC = sil_ec.c esc.c pa12_drv.c measurement.c ec_side.c fc_ec_link.c

sil_ec: $(SIL)
	$(CC) $(WARNINGS) -O2 -o $@ $(SIL_SRC) -lm

sil_ec_san: $(SIL)
	$(CC) $(CFLAGS) -o $@ $(SIL_SRC) -lm

test_sil_ec: test_sil_ec.c $(LINK)
	$(CC) $(CFLAGS) -o $@ test_sil_ec.c fc_ec_link.c -lm

# The EC firmware builds copies of the link sources (CubeIDE needs them in
# the project); its CMake build fails if they differ from these.
BOARD = ../ctv_engine_ctrl/CM7/Core
SHARED = ec_side fc_ec_link esc measurement temperature pa12_drv

check-board:
	@for f in $(SHARED); do \
		cmp -s $$f.c $(BOARD)/Src/$$f.c && cmp -s $$f.h $(BOARD)/Inc/$$f.h || \
		{ echo "board copy of $$f differs: run make sync-board"; exit 1; }; \
	done; echo "board copies match"

sync-board:
	for f in $(SHARED); do cp $$f.c $(BOARD)/Src/ && cp $$f.h $(BOARD)/Inc/; done

clean:
	rm -f test_fc_ec_link test_ec_side test_esc test_measurement test_pa12 fake_ec \
		sil_ec sil_ec_san test_sil_ec

.PHONY: test clean check-board sync-board
