"""Regenerate SHA256SUMS of the row_executor package (run after every change; the tests check it).

    python -B tools/row_executor/write_sha256sums.py

The file is in coreutils format, so the Jetson can check it with `cd <src>/row_executor && sha256sum -c
SHA256SUMS`; executor_main also re-verifies it inside the container before starting.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.dont_write_bytecode = True

from row_executor.executor_main import SUMS_FILE, render_sums  # noqa: E402

if __name__ == '__main__':
    text = render_sums(HERE)
    with open(os.path.join(HERE, SUMS_FILE), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    print('%s: %d files' % (SUMS_FILE, text.count('\n')))
