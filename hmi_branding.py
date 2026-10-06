"""Shared application identity and icon helpers for the trainer HMIs."""

import os
import sys


APP_USER_MODEL_ID = "TrainerCell.ControlSystem.HMI"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ICON_ICO = os.path.join(BASE_DIR, "assets", "trainer-cell.ico")
ICON_PNG = os.path.join(BASE_DIR, "assets", "trainer-cell.png")


def configure_process_branding():
    """Give Windows a product identity instead of the Python interpreter ID."""
    if sys.platform != "win32":
        return False
    try:
        import ctypes

        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_USER_MODEL_ID)
        return True
    except Exception:
        return False


def apply_window_branding(root):
    """Apply the Trainer Cell icon without making assets a startup dependency."""
    applied = False
    if sys.platform == "win32" and os.path.isfile(ICON_ICO):
        try:
            root.iconbitmap(default=ICON_ICO)
            applied = True
        except Exception:
            pass

    if os.path.isfile(ICON_PNG):
        try:
            import tkinter as tk

            image = tk.PhotoImage(master=root, file=ICON_PNG)
            root.iconphoto(True, image)
            root._trainer_cell_icon = image
            applied = True
        except Exception:
            pass
    return applied