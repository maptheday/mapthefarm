#!/usr/bin/env python3
"""
check_log.py -- verify a captured Wokwi serial log against a
scenario's expected-output manifest.

Usage:
    check_log.py <expected_file> <log_file>

Manifest line formats (see expected/*.expected.txt):
    MUST:<substring>              must appear, at or after the byte
                                   offset where the previous MUST
                                   matched -- i.e. all MUST lines
                                   must occur IN ORDER.
    FORBID:<substring>            must never appear anywhere in the log.
    FORBID_COUNT_GT_1:<substring> must not appear more than once in
                                   the whole log.
    # comment / blank line        ignored.

Exit code 0 = pass, 1 = fail. Prints a PASS/FAIL line per check.
"""
import sys


def load_manifest(path):
    checks = []
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.rstrip("\n")
            if not line.strip() or line.strip().startswith("#"):
                continue
            if line.startswith("MUST:"):
                checks.append(("MUST", line[len("MUST:"):]))
            elif line.startswith("FORBID_COUNT_GT_1:"):
                checks.append(("FORBID_COUNT_GT_1", line[len("FORBID_COUNT_GT_1:"):]))
            elif line.startswith("FORBID:"):
                checks.append(("FORBID", line[len("FORBID:"):]))
            else:
                print(f"WARN: unrecognized manifest line, skipping: {line!r}", file=sys.stderr)
    return checks


def main():
    if len(sys.argv) != 3:
        print("usage: check_log.py <expected_file> <log_file>", file=sys.stderr)
        return 2

    expected_path, log_path = sys.argv[1], sys.argv[2]
    checks = load_manifest(expected_path)

    with open(log_path, encoding="utf-8", errors="replace") as f:
        log = f.read()

    ok = True
    cursor = 0  # byte offset MUST checks must search forward from

    for kind, pattern in checks:
        if kind == "MUST":
            idx = log.find(pattern, cursor)
            if idx == -1:
                print(f"FAIL  MUST (not found{'  after prior match' if cursor else ''}): {pattern!r}")
                ok = False
            else:
                print(f"pass  MUST: {pattern!r}")
                cursor = idx + len(pattern)

        elif kind == "FORBID":
            if pattern in log:
                print(f"FAIL  FORBID (found but should not appear): {pattern!r}")
                ok = False
            else:
                print(f"pass  FORBID (absent, as required): {pattern!r}")

        elif kind == "FORBID_COUNT_GT_1":
            count = log.count(pattern)
            if count > 1:
                print(f"FAIL  FORBID_COUNT_GT_1 (appeared {count}x): {pattern!r}")
                ok = False
            else:
                print(f"pass  FORBID_COUNT_GT_1 (appeared {count}x): {pattern!r}")

    print()
    if ok:
        print(f"RESULT: PASS  ({log_path})")
    else:
        print(f"RESULT: FAIL  ({log_path})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
