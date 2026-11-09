# Сборка модели «Планетарная исследовательская экспедиция» из процессов (ИДЗ №3, вариант 50)
# make — сборка (именованные семафоры), make expedition_u — безымянные семафоры в
# разделяемой памяти (только Linux), make run — демонстрация, make test — тесты
CC     ?= cc
CFLAGS ?= -std=c17 -O2 -Wall -Wextra -Wpedantic
CFLAGS += -D_POSIX_C_SOURCE=200809L
LDLIBS := $(if $(filter Linux,$(shell uname -s)),-lrt,)
SRC    := $(wildcard src/*.c)

expedition: $(SRC) src/model.h
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDLIBS)

expedition_u: $(SRC) src/model.h
	$(CC) $(CFLAGS) -DUNNAMED -o $@ $(SRC) $(LDLIBS)

run: expedition
	./expedition -m

# В Linux проверяются обе сборки, в macOS — только именованные семафоры
test: expedition
	./tests/run_tests.sh ./expedition
ifeq ($(shell uname -s),Linux)
	$(MAKE) expedition_u
	./tests/run_tests.sh ./expedition_u
endif

clean:
	rm -f expedition expedition_u expedition.log tests/*.log

.PHONY: run test clean
