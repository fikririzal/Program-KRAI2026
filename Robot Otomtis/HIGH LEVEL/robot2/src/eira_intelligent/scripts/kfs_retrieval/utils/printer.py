import os
import re
import sys


_FORCE_COLOR = os.getenv("FORCE_COLOR", "0") == "1"
_ENABLE_COLOR = sys.stdout.isatty() or _FORCE_COLOR

_RESET = "\033[0m"

_COLOR_MAP = {
    "ROTATE": "\033[91m",     # red
    "MOVE": "\033[92m",       # green
    "ENTER": "\033[92m",      # green
    "EXIT": "\033[92m",       # green
    "APPROACH": "\033[93m",   # yellow
    "PICK_UP": "\033[94m",    # blue
    "PUT_DOWN": "\033[95m",   # magenta
    "VALIDATE": "\033[96m",   # cyan
    "REPLAN": "\033[93m",     # yellow
}

_TARGET_COLOR = "\033[93m"

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def _colorize(text, code):
    text = str(text)

    if not _ENABLE_COLOR or not code:
        return text

    return f"{code}{text}{_RESET}"


def _strip_ansi(text):
    return _ANSI_RE.sub("", str(text))


def _pad_colored(text, width, align="left"):
    text = str(text)
    visible = _strip_ansi(text)
    padding = max(0, width - len(visible))

    if align == "right":
        return " " * padding + text

    return text + " " * padding


def _safe_get(data, key, default="-"):
    value = data.get(key, default)
    return default if value is None else value


def print_strategy_log(strategy, planner_name=None):
    planner_name = planner_name or strategy.get("planner", "Planner")

    print("\n" + "=" * 100)
    print(f"THE MEIHUA FOREST STRATEGY PLAN | WINNER: {planner_name}")
    print("=" * 100)

    print(f"R1 Recommendation  : {strategy.get('r1_rec', [])}")
    print(f"R2 Accumulated Cost: {strategy.get('r2_total_cost', 0.0):.2f}")

    plan = strategy.get("r2_plan", [])

    if not plan:
        print("\n[R2 ACTION PLAN DETAIL]")
        print("No valid R2 plan generated.")
        print("=" * 100 + "\n")
        return

    print("\n[R2 ACTION PLAN DETAIL]")

    widths = {
        "step": 5,
        "node": 5,
        "action": 10,
        "elev": 8,
        "delta": 7,
        "direction": 12,
        "head": 6,
        "target": 8,
        "cost": 7,
    }

    header = (
        f"{'Step':<{widths['step']}} | "
        f"{'Node':<{widths['node']}} | "
        f"{'Action':<{widths['action']}} | "
        f"{'Elev':<{widths['elev']}} | "
        f"{'Delta':<{widths['delta']}} | "
        f"{'Direction':<{widths['direction']}} | "
        f"{'Head':<{widths['head']}} | "
        f"{'Target':<{widths['target']}} | "
        f"{'Cost':<{widths['cost']}}"
    )

    print(header)
    print("-" * len(header))

    for item in plan:
        action = str(_safe_get(item, "action"))
        is_turn = action == "ROTATE"

        target_raw = "(Turn)" if is_turn else _safe_get(item, "target")
        target_text = _colorize(target_raw, _TARGET_COLOR) if is_turn else str(target_raw)

        action_text = _colorize(action, _COLOR_MAP.get(action))

        cost_value = _safe_get(item, "cost", 0.0)
        try:
            cost_text = f"{float(cost_value):.2f}"
        except (TypeError, ValueError):
            cost_text = str(cost_value)

        print(
            f"{_pad_colored(_safe_get(item, 'step'), widths['step'])} | "
            f"{_pad_colored(_safe_get(item, 'node'), widths['node'])} | "
            f"{_pad_colored(action_text, widths['action'])} | "
            f"{_pad_colored(_safe_get(item, 'elevation'), widths['elev'])} | "
            f"{_pad_colored(_safe_get(item, 'delta_height'), widths['delta'])} | "
            f"{_pad_colored(_safe_get(item, 'direction'), widths['direction'])} | "
            f"{_pad_colored(_safe_get(item, 'heading'), widths['head'])} | "
            f"{_pad_colored(target_text, widths['target'])} | "
            f"{_pad_colored(cost_text, widths['cost'])}"
        )

    print("=" * 100 + "\n")


def print_test_input(test_name, config):
    print("\n" + "=" * 100)
    print("KFS RETRIEVAL SINGLE PLANNING TEST")
    print("=" * 100)

    print(f"Test Case         : {test_name}")
    print(f"Team              : {config.get('team')}")
    print(f"R2 Position        : {config.get('r2_pos')}")
    print(f"R2 Heading         : {config.get('r2_heading')}")
    print(f"R2 Targets         : {config.get('r2_targets')}")
    print(f"R1 Targets         : {config.get('r1_targets')}")
    print(f"Fake Walls         : {config.get('fake_walls')}")
    print(f"Dynamic Obstacles  : {config.get('dynamic_obstacles')}")
    print(f"Inventory          : {config.get('inventory')}")
    print(f"Validated Empty    : {config.get('validated_empty')}")
    print(f"Visited History    : {config.get('visited_history')}")
    print(f"Retrieval Mode     : {config.get('retrieval_mode')}")

    print("=" * 100 + "\n")


def print_test_result(success, message=""):
    if success:
        print(_colorize("[TEST RESULT] SUCCESS: Valid R2 plan generated.", "\033[92m"))
    else:
        print(_colorize("[TEST RESULT] FAILED: No valid R2 plan generated.", "\033[91m"))

    if message:
        print(f"[DETAIL] {message}")