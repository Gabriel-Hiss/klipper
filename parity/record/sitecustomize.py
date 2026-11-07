# Sitecustomize-style hook.
#
# Installed on PYTHONPATH by parity/record/record.py. Python imports
# `sitecustomize` automatically at interpreter startup, so the
# recorders are live before the scenario runs. The driver sets
# KLIPPY_RECORD_DIR, KLIPPY_RECORD_UNITS, KLIPPY_RECORD_SCENARIO,
# and KLIPPY_RECORD_HOME (the parity/record directory) in the
# environment.
import os
import sys


def _install():
    home = os.environ.get("KLIPPY_RECORD_HOME")
    if not home:
        home = os.path.dirname(os.path.abspath(__file__))
    if home not in sys.path:
        sys.path.insert(0, home)
    if not os.environ.get("KLIPPY_RECORD_DIR"):
        return
    try:
        import unit_recorder
        unit_recorder.install_from_environ()
    except Exception:
        pass
    try:
        import cffi_recorder
        # Patch at once so every get_ffi() caller sees the wrapper;
        # failures here must not break the scenario run.
        cffi_recorder.activate()
    except Exception:
        pass


_install()
