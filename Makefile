PIO ?= pio

BOARD1_DIR := board1
BOARD2_DIR := board2
BOARD3_DIR := board3
BOARD4_DIR := board4
BOARD1_ENV := genericSTM32F103TB
BOARD2_ENV := genericSTM32F103TB
BOARD3_ENV := genericSTM32F103TB
BOARD4_ENV := genericSTM32F103TB
BOARD1_DEBUG_ENV := genericSTM32F103TB_debug
BOARD2_DEBUG_ENV := genericSTM32F103TB_debug
BOARD3_DEBUG_ENV := genericSTM32F103TB_debug
BOARD4_DEBUG_ENV := genericSTM32F103TB_debug

.PHONY: help board1 board2 board3 board4 board1-debug board2-debug board3-debug board4-debug build-board1 build-board2 build-board3 build-board4 build-all upload-board1 upload-board2 upload-board3 upload-board4 upload-board1-debug upload-board2-debug upload-board3-debug upload-board4-debug list info monitor monitor-board1 monitor-board2 monitor-board3 monitor-board4 restructure-board1 restructure-board2 restructure-board3 restructure-board4

help:
	@printf '%s\n' \
		'make board1            Upload board1 release build' \
		'make board2            Upload board2 release build' \
		'make board3            Upload board3 release build' \
		'make board4            Upload board4 release build' \
		'make board1-debug      Upload board1 debug build with printf logs' \
		'make board2-debug      Upload board2 debug build with printf logs' \
		'make board3-debug      Upload board3 debug build with printf logs' \
		'make board4-debug      Upload board4 debug build with printf logs' \
		'make build-board1      Build board1 without upload' \
		'make build-board2      Build board2 without upload' \
		'make build-board3      Build board3 without upload' \
		'make build-board4      Build board4 without upload' \
		'make build-all         Build every board project' \
		'make monitor           Open the shared ITM monitor' \
		'make monitor-board1    Alias of make monitor' \
		'make monitor-board2    Alias of make monitor' \
		'make monitor-board3    Alias of make monitor' \
		'make monitor-board4    Alias of make monitor'

board1: upload-board1

board2: upload-board2

board3: upload-board3

board4: upload-board4

board1-debug: upload-board1-debug

board2-debug: upload-board2-debug

board3-debug: upload-board3-debug

board4-debug: upload-board4-debug

build-board1:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD1_ENV)

build-board2:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD2_ENV)

build-board3:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD3_ENV)

build-board4:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD4_ENV)

build-all: build-board1 build-board2 build-board3 build-board4

upload-board1:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD1_ENV) -t upload

upload-board2:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD2_ENV) -t upload

upload-board3:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD3_ENV) -t upload

upload-board4:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD4_ENV) -t upload

upload-board1-debug:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD1_DEBUG_ENV) -t upload

upload-board2-debug:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD2_DEBUG_ENV) -t upload

upload-board3-debug:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD3_DEBUG_ENV) -t upload

upload-board4-debug:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD4_DEBUG_ENV) -t upload

list:
	$(PIO) device list

info:
	st-info --probe

monitor-board1:
	./stlink_monitor.zsh

monitor-board2:
	./stlink_monitor.zsh

monitor-board3:
	./stlink_monitor.zsh

monitor-board4:
	./stlink_monitor.zsh

monitor:
	./stlink_monitor.zsh

restructure-board1:
	cd ./board1 && ./restructure.sh

restructure-board2:
	cd ./board2 && ./restructure.sh

restructure-board3:
	cd ./board3 && ./restructure.sh

restructure-board4:
	cd ./board4 && ./restructure.sh
