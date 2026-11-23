#!/usr/bin/env python3
"""
Purpose:
    Manage local SDK users and export or import their data.
Prerequisites:
    Install wuji-sdk. No device is required.
Usage:
    python 4_user.py
    python 4_user.py --list
    python 4_user.py --user-name Alice --user-description "right hand"
    python 4_user.py --default-user
    python 4_user.py --delete-user USER_ID --list
    python 4_user.py --export user_data.zip
    python 4_user.py --export user_data.zip --user-id USER_ID
    python 4_user.py --preview user_data.zip
    python 4_user.py --import user_data.zip
Expected Behavior:
    Print the previous and selected SDK user. The selection remains active.
    Use --list to list users, --user-name NAME to create or select a user,
    --user-description TEXT to update its description, or --default-user.
    --delete-user USER_ID deletes that user and its stored data.
    Deleting the current user switches to the default user.
    Export hand models and tactile calibration data for the current user,
    unless --user-id selects another user. Preview validates without writing.
    Import restores the bundle owner's data, creating that user if needed and
    overwriting matching files. It does not switch the current user.
    Data operations print confirmation instead of user selection details.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import sys
from typing import Any, Optional

from wuji_sdk import SdkManager


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


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--user-name", default=None, help="Local SDK user display name.")
    group.add_argument("--default-user", action="store_true", help="Switch to the default SDK user.")
    group.add_argument("--delete-user", metavar="USER_ID", help="Delete a user and its stored data.")
    group.add_argument("--export", dest="export_path", metavar="FILE.zip")
    group.add_argument("--preview", dest="preview_path", metavar="FILE.zip")
    group.add_argument("--import", dest="import_path", metavar="FILE.zip")
    parser.add_argument("--user-id", help="User to export; an empty ID selects the default user.")
    parser.add_argument("--user-description", default=None, help="Optional note for --user-name.")
    parser.add_argument("--list", action="store_true", help="List local SDK users.")
    args = parser.parse_args()

    if args.user_description is not None and not args.user_name:
        parser.error("--user-description requires --user-name")

    paths = (args.export_path, args.preview_path, args.import_path)
    if any(path == "" for path in paths):
        parser.error("bundle path must not be empty")
    if args.user_id is not None and args.export_path is None:
        parser.error("--user-id requires --export")
    if args.list and any(path is not None for path in paths):
        parser.error("--list cannot be combined with --export, --preview, or --import")
    if args.delete_user == "":
        parser.error("--delete-user must not be empty")
    if args.user_name == "":
        parser.error("--user-name must not be empty")
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        if args.export_path is not None:
            operation = "read current SDK user"
            user_id = args.user_id if args.user_id is not None else manager.current_user()["user_id"]
            operation = "export user data"
            manager.export_user_data(user_id, args.export_path)
            print(f"Exported user data: {args.export_path}")
            return 0
        if args.preview_path is not None:
            operation = "validate user data"
            manager.preview_user_data(args.preview_path)
            print(f"Validated user data: {args.preview_path}")
            return 0
        if args.import_path is not None:
            operation = "import user data"
            manager.import_user_data(args.import_path)
            print(f"Imported user data: {args.import_path}")
            return 0

        operation = "read current SDK user"
        previous = manager.current_user()
        if args.delete_user is not None:
            operation = "delete SDK user"
            manager.delete_user(args.delete_user)
            print(f"Deleted user: {args.delete_user}")
        operation = "switch or create SDK user"
        current = switch_or_create_user(manager, args)

        print_user("previous current user", previous)
        print_user("current SDK user", current)

        if args.list:
            print()
            operation = "list SDK users"
            list_users(manager)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
