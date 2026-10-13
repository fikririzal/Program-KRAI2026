WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 765

SCALE_W = WINDOW_WIDTH / 1280.0
SCALE_H = WINDOW_HEIGHT / 765.0

def sw(val): return int(val * SCALE_W)
def sh(val): return int(val * SCALE_H)
def sf(val): return int(val * min(SCALE_W, SCALE_H))

WINDOW_TITLE = "EIRA ROBOT 2 AUTOMATIC | CONFIGURATION MATCH"

APP_MARGIN = sw(15)
LEFT_PANEL_WIDTH = sw(360)
LOG_MIN_HEIGHT = sh(160)

SPEARHEAD_OPTIONS = ["1", "2", "3", "4", "5", "6"]
STRATEGY_OPTIONS = [
    "MEIHUA FOREST",
    "TIC TAC TOE CUSTOM",
]
KFS_RETRIEVAL_OPTIONS = ["2", "3", "4"]
KFS_LIMITS = {1: 3, 2: 4, 3: 1}

RETRY_OPTIONS = [
    {
        "key": "retry_spearhead",
        "label": "SPEARHEAD",
    },
    {
        "key": "retry_forest",
        "label": "FOREST",
    },
    {
        "key": "retry_arena",
        "label": "ARENA",
    },
    {
        "key": "retry_1",
        "label": "RETRY 1",
    },
    {
        "key": "retry_2",
        "label": "RETRY 2",
    },
    {
        "key": "emergency",
        "label": "EMERGENCY",
    },
]

DEFAULT_RETRY_STAGE = "retry_spearhead"
DEFAULT_START_CHECKPOINT = "martial_start"

THEMES = {
    "light": {
        "bg": "#E2E8F0", "panel": "#FFFFFF", "text": "#0F172A", "accent": "#1D4ED8",
        "border": "#94A3B8", "log_bg": "#F8FAFC", "log_text": "#0F172A",
        "box_title": "#1E293B", "btn_text": "#FFFFFF",
        "blue_zone": "#93C5FD", "red_zone": "#FCA5A5",
        "grid_med": "#22C55E", "grid_dark": "#166534", "grid_light": "#86EFAC",
        "kfs1": "#FACC15", "kfs2": "#38BDF8", "fake": "#F43F5E",
        "blue_team": "#1D4ED8", "red_team": "#B91C1C",
        "state_run": "#059669", "state_cancel": "#DC2626", "unselected": "#CBD5E1", "btn_hover": "#F1F5F9",
        "combo_btn": "#F8FAFC"
    },
    "dark": {
        "bg": "#0F172A", "panel": "#1E293B", "text": "#F8FAFC", "accent": "#38BDF8",
        "border": "#475569", "log_bg": "#020617", "log_text": "#4ADE80",
        "box_title": "#94A3B8", "btn_text": "#F8FAFC",
        "blue_zone": "#1E3A8A", "red_zone": "#7F1D1D",
        "grid_med": "#15803D", "grid_dark": "#064E3B", "grid_light": "#22C55E",
        "kfs1": "#FACC15", "kfs2": "#38BDF8", "fake": "#F43F5E",
        "blue_team": "#2563EB", "red_team": "#DC2626",
        "state_run": "#059669", "state_cancel": "#EF4444", "unselected": "#334155", "btn_hover": "#475569",
        "combo_btn": "#1E293B"
    }
}
