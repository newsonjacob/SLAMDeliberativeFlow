import tkinter as tk
import threading

# Shared exit flag
exit_flag = [False]
exit_event = threading.Event()

def start_gui():
    """Launch GUI with a STOP button; shuts down when exit_event is set."""

    def stop():
        exit_flag[0] = True
        exit_event.set()
        print("🟥 STOP button pressed")

    def check_exit():
        if exit_event.is_set():
            root.destroy()
        else:
            root.after(100, check_exit)

    root = tk.Tk()
    root.title("SLAM UAV Control")
    root.geometry("200x100")

    tk.Button(root, text="STOP", font=("Arial", 20), command=stop).pack(expand=True)
    root.protocol("WM_DELETE_WINDOW", stop)

    root.after(100, check_exit)
    root.mainloop()
