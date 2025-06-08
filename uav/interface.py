import tkinter as tk
import threading

# Shared exit flag checked by main loop
exit_flag = [False]


def start_gui():
    """Launch a simple GUI with a STOP button in a separate thread."""

    def stop():
        exit_flag[0] = True
        print("\ud83d\udd34 STOP button pressed")

    root = tk.Tk()
    root.title("SLAM UAV Control")
    root.geometry("200x100")
    tk.Button(root, text="STOP", font=("Arial", 20), command=stop).pack(expand=True)

    # run the tkinter event loop in a daemon thread so it doesn't block shutdown
    threading.Thread(target=root.mainloop, daemon=True).start()
