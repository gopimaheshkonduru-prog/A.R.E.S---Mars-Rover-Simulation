"""ARES dashboard - post-run reporting.

main_3d.py calls install(sim). When the program quits (Esc, closing the
window, Ctrl+C) this saves the telemetry and runs reports/analyze.py, so
the plots in reports/*.png always match the run you just did.

Run by hand any time:  python dashboard/dashboard.py
"""
import atexit
import os
import subprocess
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV = os.path.join(BASE, "reports", "telemetry.csv")
SEED_FILE = os.path.join(BASE, "reports", "telemetry_seed.txt")
ANALYZE = os.path.join(BASE, "reports", "analyze.py")


def run_report():
    """Run reports/analyze.py (headless matplotlib) and show its summary."""
    if not os.path.exists(CSV) or os.path.getsize(CSV) < 200:
        print("[dashboard] no telemetry to analyse - skipping report.")
        return False
    print("[dashboard] running reports/analyze.py ...")
    env = dict(os.environ, MPLBACKEND="Agg")      # no pop-up plot windows
    try:
        out = subprocess.run([sys.executable, ANALYZE], cwd=BASE, env=env,
                             capture_output=True, text=True, timeout=180)
    except Exception as err:
        print("[dashboard] report failed:", err)
        return False
    print(out.stdout.strip())
    if out.returncode != 0:
        print("[dashboard] analyze.py error:\n" + out.stderr.strip())
        return False
    return True


def recorded_seed():
    """Map seed of the run stored in telemetry.csv (None if unknown)."""
    try:
        with open(SEED_FILE) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def _on_quit(sim):
    if sim.telemetry is not None and sim.telemetry.data:
        sim.save_telemetry(CSV)
        with open(SEED_FILE, "w") as f:
            f.write(str(sim.seed))
    run_report()


def install(sim):
    """Save telemetry and run the report when the program exits."""
    atexit.register(_on_quit, sim)


if __name__ == "__main__":
    run_report()
