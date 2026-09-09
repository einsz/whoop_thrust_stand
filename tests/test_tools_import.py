"""Every tool in tools/ must import cleanly.

A dead import took down tools/spindown_probe.py unnoticed: its module-level
HERE line used os before `import os`, raising NameError on any invocation,
while nothing imported the tools to catch it. Module-level code in these tools
is kept side-effect free by construction -- hardware and serial work happens in
main() -- so importing each one is a cheap, real check.
"""
import importlib
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
TOOLS_DIR = pathlib.Path(__file__).resolve().parents[1] / "tools"


class ToolImportTests(unittest.TestCase):
    def test_every_tool_imports(self):
        failures = []
        for path in sorted(TOOLS_DIR.glob("*.py")):
            if path.name.startswith("__"):
                continue
            try:
                importlib.import_module("tools." + path.stem)
            except Exception as exc:  # noqa: BLE001 - report every failure
                failures.append("%s: %s: %s" % (path.name, type(exc).__name__, exc))
        self.assertEqual([], failures)


if __name__ == "__main__":
    unittest.main()
