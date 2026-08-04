#!/usr/bin/env python3
import argparse
import re
from pathlib import Path


def replace_yaml_scalar(text: str, key: str, value) -> str:
    pattern = re.compile(rf'^(\s*{re.escape(key)}\s*:\s*).*$',
                         flags=re.MULTILINE)
    if isinstance(value, int):
        format_value = str(value)
    else:
        format_value = f'"{value}"'
    text, count = pattern.subn(lambda match: match.group(1) + format_value,
                               text, count=1)
    if count != 1:
        raise RuntimeError(f"Could not find yaml key: {key}")
    return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--local-ip", required=True)
    parser.add_argument("--dog-ip", required=True)
    parser.add_argument("--local-port", required=True, type=int)
    args = parser.parse_args()

    text = Path(args.input).read_text(encoding="utf-8")
    text = replace_yaml_scalar(text, "local_ip", args.local_ip)
    text = replace_yaml_scalar(text, "dog_ip", args.dog_ip)
    text = replace_yaml_scalar(text, "local_port", args.local_port)
    Path(args.output).write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
