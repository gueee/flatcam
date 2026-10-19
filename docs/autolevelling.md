# Autolevelling

Autolevelling measures the real height of the copper surface across the board and then rewrites the
Z heights of a CNC job so the cutting depth follows that surface. It exists because a PCB blank is
never perfectly flat and never perfectly parallel to the machine bed. Without it, a 0.05 mm isolation
cut can cut air in one corner and bite into the substrate in another.

The correction is an offset, not a replacement: your programmed depth is kept and the measured
surface height is added to it.

```
Z_output = Z_programmed + Z_surface_at(X, Y)
```

## What you need before you start

Levelling can only work on a **CNC job made from a Geometry object with segmented G-code**. A long
straight cut is a single move with two endpoints, and there is nothing in the middle to correct. The
Milling plugin can break those long moves into shorter ones.

In the **Milling** plugin, under the common parameters:

1. Tick **Allow levelling**.
2. Set **Segment X size** and **Segment Y size** to a non-zero value. Both must be greater than
   zero. Something in the range of 1 to 5 mm is typical: smaller segments follow the surface more
   closely but produce a larger G-code file.
3. Generate the CNC job.

If you skip this, the Levelling panel opens but stays greyed out. That is the single most common
reason for "the panel does nothing".

Excellon-based jobs (drilling) are not levelled. Neither are jobs made with the Roland or HPGL
preprocessors, because those do not emit G-code moves at all; if you try, the plugin says
`Nothing was levelled` rather than pretending it worked.

## Opening the plugin

Three ways in, all the same panel:

- The **Levelling** button on the Plugins toolbar
- Menu **Options -> Experimental -> Levelling**
- The **Levelling** button in a CNC job's Properties tab

Set **Source Object** at the top of the panel to the CNC job you want to level.

## Step 1: place the probe points

### Parameters

| Field | Meaning |
|---|---|
| **Probe Tip Dia** | Diameter of your probe tip. Used to draw the points and to keep them clear of drill holes. |
| **Probe Z travel** | Safe height the probe moves at between points. Positive. |
| **Probe Z depth** | How far down the probe is allowed to go looking for the surface. Negative. |
| **Probe Feedrate** | Speed of the downward probing move. Slow is accurate; 20 to 60 mm/min is usual. |

### Mode

**Grid** lays out points automatically over the job's bounding box, using the **Rows** and
**Columns** values. This is what you want most of the time.

**Manual** lets you click points on the canvas. Left click adds a point, right click finishes.
A point outside the job area or on top of an Excellon drill hole is refused with a message, so you
cannot accidentally probe into a hole. Manual mode always uses the Voronoi method.

Tick **Avoid Excellon holes** to have grid points that land on a drill hole nudged aside. **Avoid
Step** is how far each nudge moves the point.

### Method

**Voronoi** divides the board into one cell per probe point and every cut inside a cell uses that
point's height. It accepts points in any arrangement, and it produces small steps in depth at the
cell borders.

**Bilinear** interpolates smoothly between the four probe points surrounding each position. It needs
a complete rectangular grid of at least 2 rows by 2 columns. If the grid is incomplete, for example
because points were moved to avoid drill holes, the plugin falls back to the nearest probe point for
the affected areas rather than producing nonsense.

Bilinear gives a smoother result on a gently warped board. Voronoi is more forgiving of an irregular
set of points.

Press **Add Probe Points**. The points appear on the canvas and in the **Probe Points Table**
(switch the app to Advanced level if you do not see the table, or use the **Show** checkbox next to
it). The table has three columns: the point number, its X-Y coordinates, and its measured **Height**,
which is 0.0 until you actually probe.

## Step 2: measure the heights

Pick your **Controller**. The path differs.

### GRBL: probe from inside FlatCAM

This is the route that closes the loop without leaving the application. A general purpose sender such
as gSender can run a probe file and will print the results in its console, but it will not build a
height map from them and it will not rewrite your cutting file, so the probe-then-cut cycle cannot
be completed there.

Select **GRBL** as the controller. Three tabs appear.

**Connect tab.** Press **Search** to list the serial ports, pick yours, pick the baud rate (115200
for most GRBL boards; use **New** and **Add** if your rate is not listed), then press
**Disconnected** to connect. The button turns green and reads **Connected** only after the
controller actually answers, and the Control and Sender tabs unlock. If it stays red, the port is
wrong, the board is busy, or another program is holding the port open. Press it again to disconnect;
that is refused while a job is being sent.

A command the controller refuses is never silent: its answer, `error:15` for a jog past the soft
limits for instance, is printed in the shell, and the status bar says which state the controller was
in, so you can tell an alarm that needs `$X` or a homing from a hold that needs **Pause/Resume**.
Homing, **Probe Z** and commands typed in the command box run in the background, so the window stays
responsive while the machine moves.

**Control tab.** Jog the machine, zero an axis, run a homing cycle, touch off a tool. Use this to set
your work zero before you start, exactly as you would for any job. **Step** and **Feedrate** control
the jog increment and speed.

The **Zero** buttons make the place the machine stands at right now the origin of the work coordinate
system. They move nothing and they leave the machine coordinates alone, so jogging the tool down onto
the surface and pressing **Zero Z** is a complete manual touch-off.

**Probe Z** does the same with the probe instead of your eye. It probes straight down, at most
**Probe travel** far and at **Probe feedrate**, then gives the point it touched the Z of **Plate
thickness** and lifts to the travel height. Leave **Plate thickness** at 0 when the clip is on the
bit and the copper itself is the contact; set it to the thickness of the plate when you probe onto a
touch plate. Keep the feedrate low, 80 mm/min or less on a Masuter 3S. If the probe touches nothing
within Probe travel, no origin is set, the failure is reported, and the controller is left in an
alarm state that `$X` clears.

**Sender tab.** Press **Apply AutoLevelling**. FlatCAM walks every probe point: it lifts to
Probe Z travel, moves to the point, probes down at Probe Feedrate, and reads the height back from
the controller's `[PRB:...]` answer. The measured heights fill the Probe Points Table as it goes.

If a probe reaches Probe Z depth without touching anything, the run stops and tells you which point
failed instead of recording a bogus zero. Check that your probe clip is attached and that Probe Z
depth is deep enough.

When probing finishes, the correction is applied automatically. **Save height map** next to the
button stores the raw controller output if you want to keep it.

Note that with GRBL selected, the **Save Probing GCode** and **Import Height Map** buttons are
hidden, because GRBL probes live rather than through a file. If you have a height map from a previous
session and want to load it, switch the controller to LinuxCNC to reveal the import button.

### MACH3, MACH4, LinuxCNC: probe with a file

These controllers get a probing program that you run on the machine, and they write the results to a
file that you bring back.

1. Press **Save Probing GCode** and save the file. Use the button beside it to view or edit the
   program first.
2. Run that program on your machine. MACH3 and MACH4 use `G31` with `M40`/`M41` to open and close the
   output file. LinuxCNC uses `G38.2` with `(PROBEOPEN a_probing_points_file.txt)`; the file lands in
   the LinuxCNC configuration folder.
3. Back in FlatCAM, press **Import Height Map** and select the results file. The heights load into
   the table and the correction is applied immediately.

The height map file is plain text, one probe point per line, as `x,y,z`. Spaces work as separators
too:

```
10.0,10.0,-0.032
30.0,10.0,-0.015
10.0,30.0,0.021
30.0,30.0,0.008
```

The lines must be in the same order as the probe points in the table, because that is how they are
matched up.

## Step 3: check the result

The corrected program replaces the job's G-code. Open the CNC job's **Code Review** to see it. Where
the original had:

```
G00 Z1.0000
G00 X10.0000 Y10.0000
G01 Z-0.0500
G01 X30.0000 Y10.0000
```

the levelled version reads, for a surface sitting 0.020 mm high at the first point and 0.015 mm high
at the second:

```
G00 Z1.0000
G00 X10.0000 Y10.0000 Z1.0200
G01 Z-0.0300
G01 X30.0000 Y10.0000 Z-0.0350
```

The programmed 0.05 mm depth became 0.03 mm where the board sits 0.02 mm high, which is the point of
the exercise. Cutting moves that had no Z of their own now carry one, so the tool tracks the surface
along the path. Travel moves get the same offset, keeping their clearance above the surface.

The opening `G00 Z1.0000` is left alone because at that point the machine has no X-Y position yet, so
there is no surface height to look up. Comments, feedrates, spindle commands and probing lines are
never touched. Arcs keep their own Z if they have one and never get one added, since that would turn
them into helixes.

Applying the correction twice would add the heights twice, so a job that is already levelled refuses
a second apply. Regenerate the CNC job from the Milling plugin if you want to level it again with new
probing data.

## Step 4: run the job

Export the G-code as usual and cut it with whatever sender you prefer. The exported file is the
levelled one.

Or send it straight from FlatCAM over the GRBL connection you already have open: on the **Sender**
tab press **Send Job**. A counter shows the progress as `Line N / total`.

Levelling and sending are separate. The Controller section with its Connect, Control and Sender tabs
is usable for **any** CNC job, drilling included, and the Source Object dropdown lists all of them.
Only the probing side of the panel, the probe points table, **Add Probe Points**, **Save Probing
GCode**, **Import Height Map** and **Apply AutoLevelling**, needs a Geometry job with segmented
G-code. So a drill job is connected to, jogged, touched off and sent here like any other, it just
cannot be height-corrected.

Lines are streamed with GRBL's character counting protocol: FlatCAM keeps the controller's 128 byte
receive buffer filled instead of waiting for each line to be acknowledged before sending the next
one. This matters for levelled jobs, which are made of thousands of very short segments. Waiting for
a round trip per segment empties the controller's motion planner between moves, and the machine
stutters and dwells at every segment border.

Comments are not sent, since GRBL has no use for them, with one exception: the text of a
`(MSG, ...)` comment is printed in the shell and in the progress label, because that is how a
preprocessor talks to the operator.

| Control | What it does |
|---|---|
| **Pause/Resume** (Control tab) | Holds and releases the machine mid-job. |
| **Stop** | Brings the machine to a controlled stop, then soft-resets the controller so it throws away the lines it still holds. The position is kept. |
| **Continue** | Lets a job that stopped at a program pause, a tool change for instance, carry on. |
| **Resume from line** | Sends the job again starting at the given line, restoring the state first. |

While a job streams, everything that drives the machine by hand is greyed out: jogging, the Zero
buttons, homing, **Probe Z**, the custom command box, **Get Report** and **Get Parameter**. They all
read the controller's replies, and one of those pressed mid-stream would swallow an acknowledgement
the sender is counting. They come back as soon as the job stops at a program pause or ends. **Stop**,
**Pause/Resume** and the controller **Reset** stay available at all times.

**Stop** does more than stop sending. A held GRBL still has up to 128 bytes of the job in its buffer
and would carry on with them at the next cycle start, and it refuses to jog while it holds. So after
the feed hold has brought the machine to rest, FlatCAM soft-resets the controller, which empties the
buffer. A reset at rest does not cost the position, and the status bar reports the state the
controller came back in. **Reset** resets at once, without the controlled stop first: it is the
emergency button, and a reset in mid-move costs the position, so the controller will want a homing
afterwards.

If the controller reports an `error:` or an `ALARM:`, the stream stops and the message appears in the
shell together with the line that caused it. Long moves can take a while to acknowledge, which is
normal and not treated as a lost connection.

Your work origin is safe throughout. FlatCAM never sends a homing cycle (`$H`) and never changes the
work coordinate system on its own, so nothing is lost between probing, cutting and a second job. It
also does not send a program end (`M2`) after probing, because GRBL resets every modal state to its
default when a program ends, which would reselect `G54` and stop the spindle in the middle of your
session.

### Tool changes and tool length

GRBL does not implement `M6`. A line that holds it is answered with `error:20` and, in a plain
sender, the job dies there. FlatCAM removes the `M6` word before sending and keeps the rest of the
line, so the `T` number still goes out.

What actually stops the machine is the `M0` that the preprocessors emit right after the tool change
message. That pause is never sent to the controller. FlatCAM holds the job itself: it waits for the
machine to come to a stop, shows you the preprocessor's message, and enables **Continue**. The
machine stays exactly where it is and, because it is idle rather than held, it can be driven by hand.
GRBL refuses jog commands while it holds, which is why the pause is kept on this side of the port.

So at a tool change you have the whole Control tab: jog the spindle somewhere you can reach, change
the tool, jog it down onto the surface and press **Zero Z**, or clip the probe on and press
**Probe Z**. Then press **Continue** and the job carries on from the next line, which is a lift to
clearance in every preprocessor. The modal state is sent again first, so a probe that failed and left
the controller in relative mode cannot turn the rest of the job into relative moves. If the
controller is in an alarm state when you press **Continue**, the job refuses to go on and asks you to
clear it with `$X` and set Z again, because a machine in alarm has lost its position.

Which tool to mount is in the message itself: `Change to Tool Dia = 0.8`, and for drilling also the
number of holes that tool will make.

Setting the length of the new tool is the part that has to be right, because a tool that sits 2 mm
deeper than the previous one cuts 2 mm deeper. You can do it by hand at the pause as described above,
or let the preprocessor do it. Three preprocessors handle it, and all three work on GRBL:

| Preprocessor | How the tool length is set |
|---|---|
| `Toolchange_Probe_GRBL` | Probes the surface with `G38.2` and zeroes Z on it. |
| `Toolchange_Manual` | You lower the tool onto the material by hand and tighten it there. |
| `GRBL_11_no_M6` | Not at all; set it yourself at the pause with **Zero Z** or **Probe Z**. |

`Toolchange_Probe_GRBL` is the one to pick if your machine has a probe input. At every tool change it
stops for the tool and the probe clip, then touches the surface twice, the second time at half the
feed rate for accuracy, and makes that point Z zero with `G92 Z0`. Then it lifts clear and stops once
more so you can take the clip off. Its probe depth and probe feed rate come from the Milling
plugin's **Z Probe depth** and **Feedrate Probe**; keep the feed rate low, 80 mm/min or less.

`G92` is used rather than a `G43.1` tool length offset because the rest of the program, and the
height map from the levelling step, are written against the work coordinate system. Both are volatile
in GRBL: a soft reset, an alarm or a power cycle drops them, and the tool length has to be probed
again. That is what the two probing stops at every tool change are for.

### Resuming a job

A job that stopped, because you pressed **Stop**, because of an alarm, or because the controller
rejected a line, can be picked up again. FlatCAM counts the lines it sent, so after the job ends
**Resume from line** is enabled and the box beside it holds the line after the last one the
controller acknowledged. That number counts only the lines that were actually sent: comments and
empty lines are not counted, which is also how the `Line N / total` counter reads.

Before the first line goes out, FlatCAM replays the state that the skipped lines had set: the units,
the distance and feed rate modes, the plane, the work coordinate system, the last feed rate, and the
spindle with its speed unless the last spindle command was a stop. Then it lifts the tool to the
highest Z the skipped part of the job used, which is your travel or tool change height, so the first
move cannot drag the tool through the material.

Two things it cannot do for you. If the resume line is itself a plunge or a cut, the tool goes
straight back down to depth at that point, so pick a line at the start of a path rather than in the
middle of one. And if the controller was reset, which **Stop** does, or powered down in between, a
`G92` tool length that `Toolchange_Probe_GRBL` set is gone and has to be probed again. A work zero set
with the Zero buttons or **Probe Z** is not affected: those write the `G54` offset, which GRBL keeps
in its settings memory through resets and power cycles.

## Troubleshooting

**The Levelling panel is greyed out.** The selected object is not a Geometry CNC job with segmented
G-code. Regenerate it with **Allow levelling** ticked and non-zero segment sizes. The Controller
section stays usable either way: you can still connect, jog and send the job, you just cannot
height-correct it.

**"There is no height data."** The probe points exist but all their heights are still zero. Probe
them, or import a height map, before applying.

**"Nothing was levelled."** The job holds no G-code moves. This happens with the Roland and HPGL
preprocessors, whose output is not G-code.

**"The GCode of this object is already levelled."** Regenerate the CNC job to start from an
uncorrected program.

**Connect stays red.** No answer from the controller. Wrong port or baud rate, or another program
has the port; the status bar names the reason the port could not be opened.

**A jog or zero button does nothing.** Look at the shell and the status bar: the controller's answer
and its state are reported there. `error:8` or `error:9` with an alarm state means the controller is
locked; home it, or unlock it with `$X`. `error:15` means the jog would leave the travel the soft
limits allow. **Go to origin** lifts to the travel height in work coordinates, so before Z zero is set
on the work it can ask for a height above the top of the machine.

**`error:20` from the controller.** An unsupported command. FlatCAM strips `M6` before sending, but
a file you exported and ran in another sender still holds it; use the `GRBL_11_no_M6` or
`Toolchange_Probe_GRBL` preprocessor for those.

**`ALARM:4` or `ALARM:5` after a tool change or a Probe Z.** The probe. `ALARM:5` means it never
touched anything within the travel, `ALARM:4` means it was already triggered when the move started.
Check the clip and that the travel reaches past the surface. The controller is in an alarm state
afterwards and nothing was zeroed: unlock it with `$X` from the Control tab, set the tool length
again, then continue or resume.

**Probe Z did nothing and reported a failure.** Same cause as above. The remaining commands are
deliberately not sent when the probe does not touch, so the origin keeps the value it had rather than
being set to a height the machine never reached.

**The job stopped and the machine is standing still.** That is a program pause waiting for you. Read
the message in the shell or the progress label, do what it asks, which at a tool change may include
**Zero Z** or **Probe Z**, then press **Continue**.

**Depth is right in the middle and wrong at the edges.** Your probe points do not reach far enough
out. Add rows and columns, or place manual points nearer the corners. Outside the probed area the
nearest point's height is reused, which is a guess.

---

# Other behaviour that changed

Short notes on the rest, for anyone who knew the old behaviour.

**Gerber editor, aperture codes.** Editing an aperture code to a code that is not in use renames the
aperture, as before. Editing it to a code that is already in use now merges the pads into that
aperture, and only when the two apertures would draw the same pad: same type, and for rectangular and
obround apertures the same width and height. A mismatch is refused and the old code is restored.
Previously this scaled the pad geometry by the ratio of the two code numbers, which is not a
meaningful scale factor.

**DXF import, array blocks.** A block inserted as an array produced extra copies. It now produces
exactly rows times columns copies.

**NCC, path simplification.** The tolerance used when simplifying the generated paths was fixed at
0.01. It is now **Simplification** in the NCC preferences, still defaulting to 0.01. A larger value
means fewer points and a smaller G-code file, at the cost of accuracy.

**Milling, exclusion areas.** The **Add exclusion areas** checkbox keeps its state when you leave and
re-enter the plugin, and it is stored with the preferences.

**Units in Preferences.** Changing the units in Preferences now converts the project: every object is
scaled, the grid values are converted, and the status bar updates. It used to change the setting
without converting anything until a restart.

**SVG and QR code import.** A length that carries a unit suffix, such as `width="100mm"` or
`r="2mm"`, is now converted instead of being read as a bare number.

**Tcl shell.** The output no longer jumps to the bottom when you have scrolled up to read something,
and a command that fails inside a worker thread now reports its error in the shell instead of only in
the log file.
