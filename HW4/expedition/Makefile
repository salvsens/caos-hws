# Сборка модели «Планетарная исследовательская экспедиция» (ИДЗ №4, TCP)
CC     ?= cc
CFLAGS ?= -std=c17 -O2 -Wall -Wextra -Wpedantic
CFLAGS += -D_POSIX_C_SOURCE=200809L
SRC    := $(wildcard src/*.c)
HDR    := $(wildcard src/*.h)

expedition: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ $(SRC)

run: expedition
	./run_separate.sh

test: expedition
	./tests/run_tests.sh

clean:
	rm -f expedition expedition.log tests/*.log

.PHONY: run test clean
