# Olympus Air A01 — observed behavior notes

Facts about the camera discovered empirically while debugging this project, on top of what's documented in `OPC_Communication_Protocol_EN_1.0a.pdf`. The protocol doc describes the API shape; this file records how the camera *actually* behaves in practice — timings, reset quirks, string lengths — since none of that is written down anywhere else.

If any of this needs re-verifying later, `main.c` has a serial debug console (see below) built for exactly this kind of investigation.

## Shooting mode (TAKEMODE)

- The camera does **not** retain `TAKEMODE` across the `standalone` → `rec` mode transition. Confirmed by polling `get_camprop.cgi?com=get&propname=TAKEMODE` continuously through a full gallery visit: the value stays unchanged (e.g. `S`) for the entire time the user is browsing the gallery in `play` mode, and only flips to `iAuto` at the exact moment `switch_cameramode.cgi?mode=standalone` is issued (part of this app's gallery-exit sequence, itself a workaround for `changelvqty` returning 520 when starting liveview directly from `play` mode).
- So: entering `play` mode by itself does not reset the shooting mode. It's specifically the trip through `standalone` mode that resets it to `iAuto`.
- A `set_camprop.cgi` call for `TAKEMODE` returns HTTP 200 with body `<response><notset></notset></response>` — this response shape does **not** indicate failure; it's the normal response format for a successful set. Always verify with a follow-up `get` rather than trusting the HTTP status/body of the `set`.
- Measured time from firing a `TAKEMODE` set to the change being confirmed on a subsequent `get`: **350–550 ms** across multiple real-hardware samples (5 samples: 360, 387, 402, 502, 536 ms). Not the multi-second delay we originally assumed — polling every ~100 ms and stopping as soon as it's confirmed is both correct and fast.
- **iA (Intelligent Auto) mode locks out all manual exposure control.** While in iA: aperture, shutter, ISO, and exposure compensation are not adjustable (confirmed both by the camera's own `field_selectable` semantics and by testing — sets to these props are accepted at the HTTP layer but silently ignored), and white balance is forced to `WB_AUTO` (see below). Restoring a non-iA mode must be confirmed *before* attempting to push any of these dependent properties, or the pushes are silently lost.

## White balance (WB)

- WB gets reset to `WB_AUTO` as a side effect of the same `standalone`-transition / iA-mode behavior above — it is not an independently-triggered reset. A `TAKEMODE`-changed push event and a `WB`-changed push event both arrive around the same time during this transition.
- Unlike aperture/shutter/ISO/exposure-comp, **WB is not present in the RTP liveview stream's extension header** — those four self-correct live every frame; WB does not. Its current value is only obtainable via `get_camprop.cgi?com=get&propname=WB` or an unsolicited push event (`event=206`, `<prop>WB</prop>`). Code that wants to track "true current WB" must listen for that event or poll explicitly — it will not just show up in per-frame telemetry like the others do.
- **WB API value strings vary a lot in length.** The full table (`api` values as sent to `set_camprop.cgi`):
  `WB_AUTO`, `MWB_FINE`, `MWB_SHADE`, `MWB_CLOUD`, `MWB_LAMP`, `MWB_FLUORESCENCE1`, `MWB_WATER_1`, `WB_CUSTOM1`.
  Longest is `MWB_FLUORESCENCE1` — 17 characters, 18 bytes with the NUL. A buffer sized for the shorter common values (e.g. 12 bytes, which happens to fit `MWB_LAMP`/`MWB_SHADE`/`WB_CUSTOM1` fine) will silently truncate this one to something the camera doesn't recognize (`MWB_FLUORES` at 12 bytes), and the set is then silently ignored — no error anywhere in the chain, it just quietly doesn't take. Cost this project a real bug. Use at least 24 bytes for any buffer meant to hold a WB value string.

## Push events (`event=206`, "Camera Property Value Changed")

- Fires for **any** property change, regardless of source — whether triggered by this app's own `set_camprop.cgi` calls, or by something else entirely (the camera's native app, physical controls if the body has any, or an internal side-effect reset like the iA transition above). This is the only way to learn about a property change this app didn't itself initiate.
- Delivered over a separate long-lived TCP connection opened via `start_pushevent.cgi?port=N`, format: 4-byte header (`app_id`, `event`, big-endian `xml_len`) followed by an XML body, e.g. `<?xml version="1.0"?><prop>WB</prop>`.

## HTTP server behavior

- The embedded HTTP server can return **520** for a request issued while the camera is mid-transition (e.g., right after a mode switch, or starting liveview right after leaving `play` mode). A short retry (a few hundred ms) after a 520 is generally enough.
- Issuing `set_camprop.cgi` calls before the *first video frame* of a freshly-restarted liveview stream has arrived can leave the stream stuck indefinitely (observed as a black screen that never recovers) — the video/HTTP pipeline appears to still be settling from the just-completed mode transition. Always wait for stream health (first frame received) before issuing further property changes, not just for the mode-switch HTTP calls themselves to return.

## `get_camprop.cgi` command variants

Confirmed against both the protocol PDF and live testing:
- `com=get` — current value of one property (`<value>X</value>`).
- `com=getlist` — the list of valid values for a property in the camera's *current* context (e.g. aperture range depends on lens/mode).
- `com=desc` — a descriptor (not used by this app).

There is no single "get all current settings" endpoint — each property must be queried individually via `com=get`.

## Debugging tool

`main.c` includes a serial debug console (`debug_console_task`, started unconditionally in `app_main`) that accepts plain-text commands over the same USB serial port used for `idf.py monitor`, prefixing all its output with `DBG> ` so it's easy to pick out from the regular log noise:

```
get <path>          raw HTTP GET, e.g. get /get_camprop.cgi?com=get&propname=TAKEMODE
set <PROP> <value>  set_camprop.cgi via cam_set_prop (synchronous)
prop <PROP>         get_camprop.cgi?com=get via cam_get_prop
status               dump current in-RAM shoot mode / exposure state
```

This is how all of the above was actually measured — e.g. polling `prop TAKEMODE` in a tight loop from a host script while triggering a real gallery round-trip on the device, correlating against the ESP_LOGI timestamps already in the log (millisecond-resolution, printed as `I (<ms since boot>)`).
