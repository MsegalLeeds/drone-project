# Write and run C on your STM32H743

You can do the editing and building on your Mac with the board unplugged. When you want to see the result, the onboard blue LED only needs the board and a USB data cable.

## 1. Open the example in CLion

Choose **File → Open** in CLion and select the root `drone-project` folder. In the Project pane, open **software → stm32 → blue-led → double-blink.c**. Open the Terminal tool window in the repository root and run

```sh
cd software/stm32/blue-led
```

The file `double-blink.c` holds the program. In this small example, the STM32 starts in the function `blink()`. Its first lines set up the blue LED and a timer. The outer loop, `for (;;)`, repeats the pattern as long as the program runs.

## 2. Make a change you can see

Press **⌘F** in CLion and search for `flash < 3`. Change just the `3` to a `2` so that the line reads

```c
for (unsigned flash = 0; flash < 2; ++flash) {
```

Press **⌘S** to save. The counter takes the values 0 and 1, so the loop flashes the LED twice per group. The surrounding code makes the middle group longer. Your expected pattern is **two short flashes, two long flashes, two short flashes, then a pause**.

## 3. Build on your Mac

In CLion’s CMake targets, select **stm32_blink** and build it, or run this command in the same Terminal window.

```sh
./"Load blink.command" --build-only
```

The script compiles your saved C code for the STM32 and creates `double-blink-ram.hex`, the file the board can run after loading. **Build complete.** means it succeeded. If you see a compiler error, look for the first line containing `error:`. The number after `double-blink.c:` tells you which line to check. Fix it in CLion, save, and build again. You can stop here with the board unplugged.

## 4. Run it on the board when you’re ready

1. With USB unplugged, disconnect the barometer and set the STM32 on the wooden desk. Connect it directly to the Mac with the USB data cable that worked before.
2. In the same Terminal window, run `./"Load blink.command"`. It rebuilds the saved code, then waits for you to press Return.
3. Hold **BOOT**. While holding it, press and release **RST**. Release **BOOT**. Click **Allow** if macOS asks about the USB accessory.
4. Press **Return** in Terminal. The script uses STM32CubeProgrammer to load, verify, and start your program. Watch the blue LED for the pattern from step 2.

The loader puts this example in **RAM**, the board’s temporary memory. Press **RST** by itself to restart the board’s saved program. To try another version, edit the C file, save, and repeat step 4’s upload sequence. If Terminal says **Target device not found**, run the loader again, repeat the BOOT/RST sequence, and press Return while the board is still connected. If the CubeProgrammer window already has an active connection, click **Disconnect** there first.

## One more exercise

Find `wait_ticks(600);` near the end of `blink()` and double it to `wait_ticks(1200);`. Predict which pause will get longer, then save and rebuild. This changes the pause after the whole pattern. The shorter pauses inside each group come from a different `wait_ticks` call.

The loader handles the USB programming commands for you. [ST’s STM32CubeProgrammer manual](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_Command_Lines.html)
