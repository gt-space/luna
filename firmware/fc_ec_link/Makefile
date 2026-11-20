# Host builds of the fc_ec_link tests and the laptop fake EC. On the EC,
# compile fc_ec_link.c and ec_side.c into the firmware like any other source;
# nothing here is needed there.

CC ?= cc
WARNINGS = -std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
CFLAGS ?= $(WARNINGS) -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all

LINK = fc_ec_link.c fc_ec_link.h
EC = ec_side.c ec_side.h $(LINK)

test: test_fc_ec_link test_ec_side
	./test_fc_ec_link
	./test_ec_side

test_fc_ec_link: test_fc_ec_link.c $(LINK)
	$(CC) $(CFLAGS) -o $@ test_fc_ec_link.c fc_ec_link.c -lm

test_ec_side: test_ec_side.c $(EC)
	$(CC) $(CFLAGS) -o $@ test_ec_side.c ec_side.c fc_ec_link.c -lm

# No sanitizers: this one is meant to be left running.
fake_ec: fake_ec.c $(EC)
	$(CC) $(WARNINGS) -O2 -o $@ fake_ec.c ec_side.c fc_ec_link.c -lm

# The EC firmware builds copies of the link sources (CubeIDE needs them in
# the project); its CMake build fails if they differ from these.
BOARD = ../ctv_engine_ctrl/CM7/Core
SHARED = ec_side fc_ec_link

check-board:
	@for f in $(SHARED); do \
		cmp -s $$f.c $(BOARD)/Src/$$f.c && cmp -s $$f.h $(BOARD)/Inc/$$f.h || \
		{ echo "board copy of $$f differs: run make sync-board"; exit 1; }; \
	done; echo "board copies match"

sync-board:
	for f in $(SHARED); do cp $$f.c $(BOARD)/Src/ && cp $$f.h $(BOARD)/Inc/; done

clean:
	rm -f test_fc_ec_link test_ec_side fake_ec

.PHONY: test clean check-board sync-board
