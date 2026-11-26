"""Versión única de OpenBotV Studio."""

VERSION = "0.6.4-beta"

VERSION_NUMBER, _, VERSION_SUFFIX = VERSION.partition("-")
VERSION_TUPLE = tuple(int(part) for part in VERSION_NUMBER.split(".")) + (0,)
