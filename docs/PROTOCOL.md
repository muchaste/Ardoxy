# Ardoxy-OS Serial Protocol Reference

This document consolidates the USB serial protocol used by `ardoxy_gui.py` to
configure and control the two Ardoxy-OS sketches. It mirrors the header
comments in [`ardoxy_live.ino`](../examples/ardoxy_live/ardoxy_live.ino) and
[`ardoxy_standalone.ino`](../examples/ardoxy_standalone/ardoxy_standalone.ino)
(also used by `ardoxy_standalone_20x4lcd.ino`) — refer to those files if this
document and the code ever disagree.

All communication is line-based ASCII over USB serial at **19200 baud**,
newline (`\n`) terminated.

---

## Live Mode (`ardoxy_live.ino`)

Single set of channels (1-4), Arduino stays connected to the PC for the whole
experiment.

### PC -> Arduino

| Command | Description |
|---|---|
| `CFG:MODE:<MEASURE\|SETPOINT\|SEQUENCE>` | Set operating mode |
| `CFG:NCHANNELS:<1-4>` | Number of FireSting channels to read |
| `CFG:RELAY:<ch>:<pin>` | Assign GPIO pin to channel (`ch` = 0-based) |
| `CFG:INTERVAL:<ms>` | Sampling interval |
| `CFG:DURATION:<minutes>` | Experiment duration |
| `CFG:SETPOINT:<float>` | Target DO, SETPOINT mode only |
| `CFG:KP:<float>` / `CFG:KI:<float>` / `CFG:KD:<float>` | PID gains |
| `CFG:NPHASES:<n>` | Number of sequence phases, SEQUENCE mode only |
| `CFG:PHASE:<idx>:<sp>:<min>:<t>` | Phase definition; `t` = `c` (change), `h` (hold), `p` (pause) |
| `CMD:START` / `CMD:STOP` / `CMD:PAUSE` / `CMD:RESUME` / `CMD:STATUS` | Runtime control |

### Arduino -> PC

| Message | Description |
|---|---|
| `ACK:OK` | Last command accepted |
| `ACK:ERR:<msg>` | Last command rejected |
| `STATUS:<IDLE\|CONFIGURED\|RUNNING\|PAUSED>` | Current state |
| `DATA:<ms>,<do_ch1[,do_ch2...]>,<temp>,<output[,output...]>,<sp>,<phase>,<ptype>` | Measurement row |
| `MSG:<text>` | Free-text status/log message |
| `DONE` | Experiment finished |

---

## Standalone Mode (`ardoxy_standalone.ino` / `ardoxy_standalone_20x4lcd.ino`)

Up to 8 channels across two FireSting sensors, each channel independently
configured, scheduled, and run. The PC is only needed to send configuration;
the Arduino then runs autonomously and logs to its SD card.

### PC -> Arduino — shared hardware config

| Command | Description |
|---|---|
| `CFG:NCHANNELS:<1-8>` | Total number of channels |
| `CFG:SENSORS:<1\|2>` | Number of FireSting sensors (default 1) |
| `CFG:S1CHANNELS:<n>` | Channels on sensor 1; required when `SENSORS=2` |
| `CFG:RELAY:<ch>:<pin>` | Assign GPIO pin to channel (`ch` = 0-based) |
| `CFG:INTERVAL:<ms>` | Sampling interval |
| `CFG:TANKID:<ch>:<id>` | Tank label, up to 6 chars |
| `CFG:KP:<ch>:<float>` / `CFG:KI:<ch>:<float>` / `CFG:KD:<ch>:<float>` | Per-channel valve PID gains |

### PC -> Arduino — per-channel mode config

| Command | Description |
|---|---|
| `CFG:CH:<ch>:MODE:<MEASURE\|SETPOINT\|SEQUENCE>` | Channel operating mode |
| `CFG:CH:<ch>:START:<Y>:<M>:<D>:<h>:<m>:<s>` | Scheduled start; `0:0:0:0:0:0` = start immediately |
| `CFG:CH:<ch>:SETPOINT:<float>` | Target DO, SETPOINT mode |
| `CFG:CH:<ch>:DURATION:<minutes>` | SETPOINT duration |
| `CFG:CH:<ch>:NPHASES:<n>` | Number of sequence phases (1..`MAX_PHASES`) |
| `CFG:CH:<ch>:PHASE:<idx>:<sp>:<dur_d>:<dur_h>:<dur_m>:<type>[:<params>]` | Phase definition; `type` = `h` (hold), `c` (change), `p` (pause), `d` (daily-cycle). `startDO` required when `type=c`; `min_sp`/`max_sp`/`peak_h` required when `type=d` |

### PC -> Arduino — commands

| Command | Description |
|---|---|
| `CMD:START` / `CMD:STOP` / `CMD:PAUSE` / `CMD:RESUME` / `CMD:STATUS` | Runtime control |
| `CMD:SAVECONFIG` | Write current config to `CONFIG.TXT` on SD |
| `CMD:TESTPIN:<pin>:<0\|1>` | Open (1) or close (0) a relay by pin number; blocked while RUNNING |
| `CMD:READCONFIG` | Emit config (loads from SD first if IDLE; uses in-memory config if CONFIGURED) |
| `CMD:RECOVER` | Resume experiment from `STATE.TXT`; blocked if IDLE or RUNNING |
| `CMD:LISTFILES` | List all `.csv` log files on SD with file sizes |
| `CMD:SENDFILE:<filename>` | Stream a log file over serial as `FLINE:` rows |
| `CMD:SETRTC:<Y>:<M>:<D>:<h>:<m>:<s>` | Set the real-time clock |

### Arduino -> PC

| Message | Description |
|---|---|
| `ACK:OK` | Last command accepted |
| `ACK:ERR:<msg>` | Last command rejected |
| `STATUS:<IDLE\|CONFIGURED\|RUNNING\|PAUSED>` | Overall Arduino state |
| `STATUS:CH:<ch>:<statusStr>:<phaseIdx>:<do>:<sp>` | Per-channel status; `statusStr` = `MEASURE` \| `SETPOINT` \| `SEQUENCE` \| `WAITING-MEASURING` \| `DONE` |
| `DATA:<elapsed_s>,<do_ch0[,do_ch1...]>,<temp1>[,<temp2>],<out_ms_ch0[,...]>` | Measurement row |
| `MSG:<text>` | Free-text status/log message |
| `DONE` | Experiment finished |

### SD card files

| File | Description |
|---|---|
| `CONFIG.TXT` | Saved config (`key=value`); auto-loaded on boot if no serial input arrives |
| `STATE.TXT` | Last-known experiment state; auto-restored on boot (power-outage recovery) |
| `YYYY_MM_DD_HH_MM.csv` | Semicolon-delimited measurement log, new file per day |

### Boot behaviour

1. Wait 10 s for any serial byte (a GUI config session).
2. If no serial input arrives: load `CONFIG.TXT`, then restore `STATE.TXT` if present (power-outage recovery), or start fresh. If `CONFIG.TXT` is absent, wait for serial configuration instead.

### Note on N2-only control

DO can only be decreased by bubbling N2; passive increase occurs through
mixing with ambient air. The PID controller runs in reverse mode so that
output is 0 when DO is already below setpoint (valve stays closed).
