PIO ?= pio

BOARD1_DIR := board1
BOARD2_DIR := board2
BOARD3_DIR := board3
BOARD4_DIR := board4
BOARD4U_DIR := board4u
BOARD1_ENV := genericSTM32F103TB
BOARD2_ENV := genericSTM32F103TB
BOARD3_ENV := genericSTM32F103TB
BOARD4_ENV := genericSTM32F103TB
BOARD4U_ENV := nucleo_f446re
BOARD1_DEBUG_ENV := genericSTM32F103TB_debug
BOARD2_DEBUG_ENV := genericSTM32F103TB_debug
BOARD3_DEBUG_ENV := genericSTM32F103TB_debug
BOARD4_DEBUG_ENV := genericSTM32F103TB_debug
BOARD4U_DEBUG_ENV := nucleo_f446re_debug
MONITOR_BOARD ?= $(if $(BOARD),$(BOARD),board4)

.PHONY: help board1 board2 board3 board4 board4u board1-debug board2-debug board3-debug board4-debug board4u-debug build-board1 build-board2 build-board3 build-board4 build-board4u build-all upload-board1 upload-board2 upload-board3 upload-board4 upload-board4u upload-board1-debug upload-board2-debug upload-board3-debug upload-board4-debug upload-board4u-debug list info monitor monitor-board1 monitor-board2 monitor-board3 monitor-board4 monitor-board4u restructure-board1 restructure-board2 restructure-board3 restructure-board4 restructure-board4u

help:
	@printf '%s\n' \
		'make board1            Upload board1 release build' \
		'make board2            Upload board2 release build' \
		'make board3            Upload board3 release build' \
		'make board4            Upload board4 release build' \
		'make board4u           Upload board4u release build' \
		'make board1-debug      Upload board1 debug build with printf logs' \
		'make board2-debug      Upload board2 debug build with printf logs' \
		'make board3-debug      Upload board3 debug build with printf logs' \
		'make board4-debug      Upload board4 debug build with printf logs' \
		'make board4u-debug     Upload board4u debug build with printf logs' \
		'make build-board1      Build board1 without upload' \
		'make build-board2      Build board2 without upload' \
		'make build-board3      Build board3 without upload' \
		'make build-board4      Build board4 without upload' \
		'make build-board4u     Build board4u without upload' \
		'make build-all         Build every board project' \
		'make monitor           Open the ITM monitor for MONITOR_BOARD or BOARD (default: board4)' \
		'make monitor BOARD=board4u' \
		'make monitor-board1    Open the ITM monitor for board1' \
		'make monitor-board2    Open the ITM monitor for board2' \
		'make monitor-board3    Open the ITM monitor for board3' \
		'make monitor-board4    Open the ITM monitor for board4' \
		'make monitor-board4u   Open the ITM monitor for board4u'

board1: upload-board1

board2: upload-board2

board3: upload-board3

board4: upload-board4

board4u: upload-board4u

board1-debug: upload-board1-debug

board2-debug: upload-board2-debug

board3-debug: upload-board3-debug

board4-debug: upload-board4-debug

board4u-debug: upload-board4u-debug

build-board1:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD1_ENV)

build-board2:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD2_ENV)

build-board3:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD3_ENV)

build-board4:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD4_ENV)

build-board4u:
	$(PIO) run --project-dir $(BOARD4U_DIR) -e $(BOARD4U_ENV)

build-all: build-board1 build-board2 build-board3 build-board4 build-board4u

upload-board1:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD1_ENV) -t upload

upload-board2:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD2_ENV) -t upload

upload-board3:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD3_ENV) -t upload

upload-board4:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD4_ENV) -t upload

upload-board4u:
	$(PIO) run --project-dir $(BOARD4U_DIR) -e $(BOARD4U_ENV) -t upload

upload-board1-debug:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD1_DEBUG_ENV) -t upload

upload-board2-debug:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD2_DEBUG_ENV) -t upload

upload-board3-debug:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD3_DEBUG_ENV) -t upload

upload-board4-debug:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD4_DEBUG_ENV) -t upload

upload-board4u-debug:
	$(PIO) run --project-dir $(BOARD4U_DIR) -e $(BOARD4U_DEBUG_ENV) -t upload

list:
	$(PIO) device list

info:
	st-info --probe

monitor-board1:
	./stlink_monitor.zsh board1

monitor-board2:
	./stlink_monitor.zsh board2

monitor-board3:
	./stlink_monitor.zsh board3

monitor-board4:
	./stlink_monitor.zsh board4

monitor-board4u:
	./stlink_monitor.zsh board4u

monitor:
	./stlink_monitor.zsh $(MONITOR_BOARD)

restructure-board1:
	cd ./board1 && ./restructure.sh

restructure-board2:
	cd ./board2 && ./restructure.sh

restructure-board3:
	cd ./board3 && ./restructure.sh

restructure-board4:
	cd ./board4 && ./restructure.sh

restructure-board4u:
	cd ./board4u && ./restructure.sh
