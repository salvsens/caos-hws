# Сборка модели «Планетарная исследовательская экспедиция» (ИДЗ №1, вариант 50)
# make — сборка, make run — демонстрация, make test — тесты
CC     ?= cc
CFLAGS ?= -std=c17 -O2 -Wall -Wextra -Wpedantic
CFLAGS += -D_POSIX_C_SOURCE=200809L
SRC    := $(wildcard src/*.c)

expedition: $(SRC) src/model.h
	$(CC) $(CFLAGS) -o $@ $(SRC)

run: expedition
	./expedition -m

test: expedition
	./tests/run_tests.sh

clean:
	rm -f expedition expedition.log tests/*.log

.PHONY: run test clean
