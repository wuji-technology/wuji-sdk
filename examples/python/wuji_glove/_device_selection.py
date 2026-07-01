"""Device selection helpers for Wuji Glove examples."""

from contextlib import suppress
from typing import Any

from wuji_sdk import WujiGlove


def scan_contains(scanned: list[Any], sn: str) -> bool:
    return any(device.sn == sn for device in scanned)


def _candidates(scanned: list[Any], sn: str | None) -> list[Any]:
    if sn is None:
        return scanned
    return [device for device in scanned if device.sn == sn]


def _disconnect_if_possible(manager: Any, device_name: str) -> None:
    with suppress(Exception):
        manager.disconnect(device_name)


def connect_gloves(
    manager: Any,
    *,
    sn: str | None = None,
    device_name_prefix: str = "glove",
    options: Any = None,
) -> tuple[list[WujiGlove], list[Any]]:
    """Connect every Wuji Glove from scan results and skip non-glove devices."""
    scanned = list(manager.scan())
    gloves: list[WujiGlove] = []

    for index, device in enumerate(_candidates(scanned, sn)):
        device_name = f"{device_name_prefix}_{index}"
        connected = manager.connect(sn=device.sn, device_name=device_name, options=options)
        if isinstance(connected, WujiGlove):
            gloves.append(connected)
        else:
            _disconnect_if_possible(manager, device_name)

    return gloves, scanned


def connect_first_glove(
    manager: Any,
    *,
    sn: str | None = None,
    device_name: str = "glove",
    options: Any = None,
) -> tuple[WujiGlove | None, list[Any]]:
    """Connect the first Wuji Glove from scan results and skip non-glove devices."""
    scanned = list(manager.scan())

    for device in _candidates(scanned, sn):
        connected = manager.connect(sn=device.sn, device_name=device_name, options=options)
        if isinstance(connected, WujiGlove):
            return connected, scanned
        _disconnect_if_possible(manager, device_name)

    return None, scanned
