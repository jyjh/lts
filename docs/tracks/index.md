---
layout: page
title: Tracks
permalink: /tracks/
---

## Track files

`2026enduro` and other real circuits are loaded from `.mat` files in the
main repository's `tracks/` folder via `lts.components.WaypointTrack.loadMat`.
These files are produced by the separate
[`fsae track image tool`](https://github.com/jyjh/fsae-track-image-tool),
which traces a track image into `[x, y]` waypoints.

**Travel direction.** The exporter bakes the requested clockwise/anticlockwise
direction into the ordering of `points_m` and also records it in a `direction`
field. `loadMat` honors that order by default; an explicit override forces a
direction:

```matlab
track = lts.components.WaypointTrack.loadMat('tracks/<file>.mat', 'Direction', 'anticlockwise');
```

If the override conflicts with the direction stored in the file (for example
because the file is a **stale copy** that was re-exported the other way), the
waypoints are reversed — keeping the start/finish point fixed — and a warning is
emitted. `lts.app.run_simulation` loads the endurance track without a
`Direction` override, so the stored `direction` field is honored as-is, and
prints the resolved direction at startup, so a wrong or stale track is obvious
immediately. A file with no direction field at all also warns.

**Updating a track.** The exporter writes to its own `examples/` directory; it
does **not** touch this repository's `tracks/`. After re-running the exporter
(with a changed `Direction` or otherwise), copy the new `.mat` into `tracks/`
yourself. `Direction` only reorders points *inside* the file — the filename is
unchanged — so a re-export silently overwrites the previous output.

**Variable track widths.** As of the cone-aware exporter, the `.mat`/`.csv`
carry the real track corridor, not a single width: each waypoint has its own
`width_m` plus asymmetric `left_width_m`/`right_width_m` derived from the cone
marks. `WaypointTrack.loadMat` reads these into `LeftWidth`/`RightWidth`, and
the simulator consumes them with full per-side fidelity — the off-track margin,
edge slowdown/steering, racing-line offset, and feasibility all use the actual
local half-width on whichever side of the centerline the car is on (positive
lateral error = left of the line, bounded by `left_width_m`; negative by
`right_width_m`). When a direction override reverses the waypoint order, the
left and right sides are swapped to stay consistent with the new travel
direction. Scalar-width files (no `left_width_m`/`right_width_m`) load exactly
as before and run with a symmetric `Width/2` corridor.

```matlab
track = lts.components.WaypointTrack.loadMat('tracks/<file>.mat');
[leftWidth, rightWidth] = track.getTrackSideWidths();   % per-waypoint [m]
```

The app entry points (`run_simulation`, `run_all`, `CorrelationAppSupport`)
load the file's widths as-is; do not override `track.Width`, since that would
discard the exported corridor and force a uniform width back on.

**Built-in test tracks** (`straight`, `skidpad`, `autocross`, `slalom`, ...)
are provided by `lts.components.TestTrack` and need no files — see the
[Architecture & Usage](../) page.
