# Starts the firmware after an upload. The upload enters the bootloader through
# the firmware's USB port, which sets a "boot into download mode" flag that the
# usual RTS reset keeps (the board hung in the bootloader until power was cut).
# A watchdog reset clears it. esptool then reports the vanished port as an error,
# so the upload itself ends without reset and this step ignores that error.
Import("env")

def reset_with_watchdog(source, target, env):
    env.Execute(env.VerboseAction(
        '"$PYTHONEXE" "$UPLOADER" --chip $BOARD_MCU --port "$UPLOAD_PORT" '
        "--before no_reset --after watchdog_reset read_mac || exit 0",
        "Starting firmware (watchdog reset)"))

env.AddPostAction("upload", reset_with_watchdog)
