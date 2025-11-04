**REMOTE FLASH/DEBUG GUIDE**:

1. Connect to the Raspberry Pi (contact the administrator or send an email to request access for this step).

2. In the desired project directory (e.g., `solaris-vX`), run the following commands:

```bash
idf.py set-target esp32s3
idf.py build
idf.py merge-bin
```

This sets the microcontroller target, builds the project (producing the three required binaries in the `build` directory), and merges them into a single binary file: `merged-binary.bin`.

3. To debug: *Run → Start Debugging* (or press *F5*).

4. To flash: *Terminal → Run Task → flash-esp32s3-remote-only*.

*Note*: Remote debugging uses **OpenOCD**, and the debugging interface is the **JTAG** protocol. See `tasks.json` for more information.
