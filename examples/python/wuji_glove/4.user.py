#!/usr/bin/env python3
"""
Wuji SDK local user example.

Calibration data is scoped by SDK user and glove serial number. Run this
example before calibration when you want to create, inspect, or switch the
current local SDK user.

Usage:
  python 4.user.py
  python 4.user.py --list
  python 4.user.py --user-name Alice --user-description "right hand"
  python 4.user.py --default-user
"""

import argparse
from typing import Any, Optional

from wuji_sdk import SdkManager, WujiException, set_log_level


def print_user(label: str, user: dict[str, Any]) -> None:
    print(f"{label}:")
    print(f"  user_id:      {user.get('user_id')!r}")
    print(f"  display_name: {user.get('display_name')}")
    print(f"  description:  {user.get('description')}")
    print(f"  is_default:   {user.get('is_default')}")


def find_user_by_display_name(manager: SdkManager, display_name: str) -> Optional[dict[str, Any]]:
    matches = [
        user for user in manager.list_users()
        if user.get("display_name") == display_name
    ]
    if len(matches) > 1:
        raise RuntimeError(
            f"Multiple SDK users named {display_name!r}. "
            "Please rename or delete duplicates before switching."
        )
    return matches[0] if matches else None


def switch_or_create_user(manager: SdkManager, args: argparse.Namespace) -> dict[str, Any]:
    if args.default_user:
        return manager.switch_to_default_user()

    if not args.user_name:
        return manager.current_user()

    user = find_user_by_display_name(manager, args.user_name)
    if user is None:
        user = manager.create_user(args.user_name, description=args.user_description)
    elif args.user_description is not None and user.get("description") != args.user_description:
        user = manager.update_user(user["user_id"], description=args.user_description)

    return manager.switch_user(user["user_id"])


def list_users(manager: SdkManager) -> None:
    print("SDK users:")
    for user in manager.list_users():
        marker = "*" if user.get("user_id") == manager.current_user().get("user_id") else " "
        print(
            f"{marker} {user.get('display_name')} "
            f"user_id={user.get('user_id')!r} "
            f"description={user.get('description')!r}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--user-name", default=None, help="Local SDK user display name.")
    group.add_argument("--default-user", action="store_true", help="Switch to the default SDK user.")
    parser.add_argument("--user-description", default=None, help="Optional note for --user-name.")
    parser.add_argument("--list", action="store_true", help="List local SDK users.")
    parser.add_argument(
        "--log-level",
        default="warn",
        choices=["trace", "debug", "info", "warn", "warning", "error", "off"],
    )
    args = parser.parse_args()

    if args.user_description is not None and not args.user_name:
        parser.error("--user-description requires --user-name")

    set_log_level(args.log_level)
    manager = SdkManager.instance()

    previous = manager.current_user()
    current = switch_or_create_user(manager, args)

    print_user("previous current user", previous)
    print_user("current SDK user", current)

    if args.list:
        print()
        list_users(manager)


if __name__ == "__main__":
    try:
        main()
    except (WujiException, RuntimeError) as e:
        print(f"Error: {e}")
        raise SystemExit(1) from e
