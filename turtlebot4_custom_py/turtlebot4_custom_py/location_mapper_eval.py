#!/usr/bin/env python3
"""Score the LLM location mapper against a fixed set of prompts.

A pure desk experiment: needs the GGUF model and llama.cpp (i.e. run it on
the demo laptop), but no robot, no map, no ROS graph:

    ros2 run turtlebot4_custom_py location_mapper_eval

The cases are written against the default locations_map.txt names
(Harold's Room, John's Room, dock) — if those change, update CASES too.

The abstain cases (expected None) matter more than the others: a wrong
location that the LLM is confident about sends a real robot somewhere real,
while a missed abstain just logs a warning. Exit code is the number of
failed cases, so this can gate a demo from a script.
"""
import argparse
import sys

from turtlebot4_custom_py.llm_location_mapper import LLMLocationMapper

# (command, expected location name or None for "must abstain")
CASES = [
    # Straightforward destinations
    ("Go to Harold's Room", "Harold's Room"),
    ("Navigate to John's room", "John's Room"),
    ("Navigate to the dock", "dock"),
    # Person implies their room
    ("Bring Harold this book", "Harold's Room"),
    ("Take this package to John", "John's Room"),
    ("Go visit Harold", "Harold's Room"),
    # Charging synonyms map to the dock
    ("Go recharge yourself", "dock"),
    ("Return to the charging station", "dock"),
    ("Go back home", "dock"),
    # Must abstain: unknown places and non-navigation chatter
    ("Go to the kitchen", None),
    ("Take this to Alice", None),
    ("What time is it?", None),
    ("Spin around in a circle", None),
]


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Score the LLM location mapper against fixed prompts')
    parser.add_argument('--model-path', default=None,
                        help='GGUF model (default: the one llm_navigation uses)')
    parser.add_argument('--locations-file', default=None,
                        help='locations file (default: locations_map.txt)')
    parser.add_argument('--n-threads', type=int, default=4)
    parsed = parser.parse_args(args)

    mapper = LLMLocationMapper(model_path=parsed.model_path,
                               locations_file=parsed.locations_file,
                               n_threads=parsed.n_threads)

    failures = []
    abstain_total = abstain_passed = 0
    for command, expected in CASES:
        got = mapper.extract_location_name(command)
        passed = got == expected
        if expected is None:
            abstain_total += 1
            abstain_passed += passed
        if not passed:
            failures.append((command, expected, got))
        print(f"{'PASS' if passed else 'FAIL'}  '{command}' -> "
              f"{got!r} (expected {expected!r})")

    print('\n' + '=' * 60)
    print(f'{len(CASES) - len(failures)}/{len(CASES)} cases passed '
          f'({abstain_passed}/{abstain_total} abstain cases)')
    if failures:
        print('Failed:')
        for command, expected, got in failures:
            print(f"  '{command}': expected {expected!r}, got {got!r}")
        wrong_goal = [f for f in failures if f[1] is None and f[2] is not None]
        if wrong_goal:
            print('Abstain failures above would drive the robot to a wrong, '
                  'real place — treat those as blockers for hardware demos.')
    sys.exit(len(failures))


if __name__ == '__main__':
    main()
