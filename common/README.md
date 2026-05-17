# Common Distribution Board Modules

This directory contains PlatformIO local libraries shared by the board projects.

Each board enables these libraries with:

```ini
lib_extra_dirs = ../common/lib
```

Board-specific constants such as CAN IDs, LED pins, and encoder timing live in
each board's `include/board_config.h`.

Shared libraries:

- `led`: LED API used by board1-4.
- `encoder`: single RS485 encoder API used by board2-4.
- `can_control`: encoder CAN TX and optional board-specific CAN RX command handling.
- `encoder_can_publisher`: common encoder polling and CAN publish loop used by board2-4.
