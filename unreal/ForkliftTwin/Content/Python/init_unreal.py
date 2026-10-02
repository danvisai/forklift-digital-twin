"""Editor startup hook for running automated tests inside one open editor (nothing happens without the flags).

  -PIEAutoStart            begin Play-In-Editor as soon as the map has loaded.
  -PIETestBatch=<file>     JSON list of {"name": ..., "args": "PickupBackCm=120 PickupYawDeg=8"}. Starts Play once, then for each
                           case writes the args to Saved/AutoPickupCase.txt and requests a scene reset via
                           Saved/AutoPickupRun.txt (the game mode resets inside the running Play session), waits for
                           Saved/AutoPickupResult.txt and finally writes Saved/AutoPickupBatch.txt. The editor and
                           Play session stay open the whole time.
"""
import json
import os
import time

import unreal

_cmd = unreal.SystemLibrary.get_command_line()
_saved = unreal.Paths.project_saved_dir()
_case_file = os.path.join(_saved, "AutoPickupCase.txt")
_result_file = os.path.join(_saved, "AutoPickupResult.txt")
_run_file = os.path.join(_saved, "AutoPickupRun.txt")
_summary_file = os.path.join(_saved, "AutoPickupBatch.txt")
_CASE_TIMEOUT_S = 150.0


def _batch_path():
    for tok in _cmd.split():
        if tok.startswith("-PIETestBatch="):
            return tok.split("=", 1)[1].strip('"')
    return None


def _les():
    return unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)


if "-PIEAutoStart" in _cmd and not _batch_path():
    _s = {"frames": 0, "started": False}

    def _tick_single(_dt):
        _s["frames"] += 1
        if not _s["started"] and _s["frames"] > 60:
            _les().editor_request_begin_play()
            _s["started"] = True

    unreal.register_slate_post_tick_callback(_tick_single)

elif _batch_path():
    with open(_batch_path(), encoding="utf-8") as f:
        _cases = json.load(f)
    _b = {"frames": 0, "i": -1, "phase": "wait_map", "t0": 0.0, "results": []}

    def _remove(path):
        try:
            os.remove(path)
        except OSError:
            pass

    def _finish_case(text):
        _b["results"].append("%-14s %s" % (_cases[_b["i"]]["name"], text))
        unreal.log("[PIEBatch] %s" % _b["results"][-1])
        _b["phase"] = "next"

    def _tick_batch(_dt):
        _b["frames"] += 1
        les = _les()
        ph = _b["phase"]
        if ph == "wait_map":
            if _b["frames"] > 60:
                _b["phase"] = "next"
        elif ph == "next":
            _b["i"] += 1
            if _b["i"] >= len(_cases):
                with open(_summary_file, "w", encoding="utf-8") as f:
                    f.write("\n".join(_b["results"]) + "\n")
                unreal.log("[PIEBatch] done\n" + "\n".join(_b["results"]))
                _b["phase"] = "done"
                return
            _remove(_result_file)
            with open(_case_file, "w", encoding="utf-8") as f:
                f.write(_cases[_b["i"]].get("args", ""))
            if _b["i"] == 0:
                les.editor_request_begin_play()  # first case: start Play once; later cases reset inside it
            else:
                with open(_run_file, "w", encoding="utf-8") as f:
                    f.write("run")  # the game mode resets the scene inside the running Play session
            _b["t0"] = time.time()
            _b["phase"] = "running"
        elif ph == "running":
            if os.path.exists(_result_file):
                with open(_result_file, encoding="utf-8") as f:
                    _finish_case(f.read().strip())
            elif time.time() - _b["t0"] > _CASE_TIMEOUT_S:
                _finish_case("FAIL wall-clock timeout")

    unreal.register_slate_post_tick_callback(_tick_batch)
