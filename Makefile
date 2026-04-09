PIO ?= pio

BOARD1_DIR := board1
BOARD2_DIR := board2
BOARD3_DIR := board3
BOARD4_DIR := board4
BOARD_ENV := genericSTM32F103TB

.PHONY: help board1 board2 board3 board4 build-board1 build-board2 build-board3 build-board4 build-all upload-board1 upload-board2 upload-board3 upload-board4 list info monitor-board1 monitor-board2 monitor-board3 monitor-board4 restructure-board1 restructure-board2 restructure-board3 restructure-board4

help:
	@printf '%s\n' \
		'make board1            Upload board1 release build' \
		'make board2            Upload board2 release build' \
		'make board3            Upload board3 release build' \
		'make board4            Upload board4 release build' \
		'make build-board1      Build board1 without upload' \
		'make build-board2      Build board2 without upload' \
		'make build-board3      Build board3 without upload' \
		'make build-board4      Build board4 without upload' \
		'make build-all         Build every board project' \
		'make monitor-board1    Open ITM monitor for board1' \
		'make monitor-board2    Open ITM monitor for board2' \
		'make monitor-board3    Open ITM monitor for board3' \
		'make monitor-board4    Open ITM monitor for board4'

board1: upload-board1

board2: upload-board2

board3: upload-board3

board4: upload-board4

build-board1:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD_ENV)

build-board2:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD_ENV)

build-board3:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD_ENV)

build-board4:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD_ENV)

build-all: build-board1 build-board2 build-board3 build-board4

upload-board1:
	$(PIO) run --project-dir $(BOARD1_DIR) -e $(BOARD_ENV) -t upload

upload-board2:
	$(PIO) run --project-dir $(BOARD2_DIR) -e $(BOARD_ENV) -t upload

upload-board3:
	$(PIO) run --project-dir $(BOARD3_DIR) -e $(BOARD_ENV) -t upload

upload-board4:
	$(PIO) run --project-dir $(BOARD4_DIR) -e $(BOARD_ENV) -t upload

list:
	$(PIO) device list

info:
	st-info --probe

monitor-board1:
	./board1/stlink_monitor.zsh

monitor-board2:
	./board2/stlink_monitor.zsh

monitor-board3:
	./board3/stlink_monitor.zsh

monitor-board4:
	./board4/stlink_monitor.zsh

restructure-board1:
	cd ./board1 && ./restructure.sh

restructure-board2:
	cd ./board2 && ./restructure.sh

restructure-board3:
	cd ./board3 && ./restructure.sh

restructure-board4:
	cd ./board4 && ./restructure.sh
