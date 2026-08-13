#!/usr/bin/env python3
import json
import struct
import sys
import time

import gz.transport13 as transport


ENTITY_TYPES = {
    "NONE": 0,
    "LIGHT": 1,
    "MODEL": 2,
    "LINK": 3,
    "VISUAL": 4,
    "COLLISION": 5,
    "SENSOR": 6,
    "JOINT": 7,
    "ACTOR": 8,
    "WORLD": 9,
}


class MessageType:
    def __init__(self, full_name):
        self.DESCRIPTOR = type("Descriptor", (), {"full_name": full_name})()


def varint(value):
    value = int(value)
    out = bytearray()
    while value > 0x7F:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def key(field, wire_type):
    return varint((field << 3) | wire_type)


def string_field(field, value):
    data = str(value).encode("utf-8")
    return key(field, 2) + varint(len(data)) + data


def varint_field(field, value):
    return key(field, 0) + varint(value)


def double_field(field, value):
    return key(field, 1) + struct.pack("<d", float(value or 0.0))


def message_field(field, payload):
    return key(field, 2) + varint(len(payload)) + payload


def vector3d(values):
    return b"".join(
        [
            double_field(2, values.get("x", 0.0)),
            double_field(3, values.get("y", 0.0)),
            double_field(4, values.get("z", 0.0)),
        ]
    )


def entity_msg(entity):
    entity_type = entity.get("type", "MODEL")
    if isinstance(entity_type, str):
        entity_type = ENTITY_TYPES.get(entity_type.upper(), ENTITY_TYPES["MODEL"])
    payload = b""
    if entity.get("id"):
        payload += varint_field(2, entity["id"])
    payload += string_field(3, entity.get("name", ""))
    payload += varint_field(4, entity_type)
    return payload


def entity_wrench_msg(command):
    wrench = b"".join(
        [
            message_field(2, vector3d(command.get("force", {}))),
            message_field(3, vector3d(command.get("torque", {}))),
        ]
    )
    return b"".join(
        [
            message_field(2, entity_msg(command.get("entity", {}))),
            message_field(3, wrench),
        ]
    )


def main():
    node = transport.Node()
    publishers = {}

    def publisher(topic, msg_type):
        cache_key = (topic, msg_type)
        if cache_key not in publishers:
            publishers[cache_key] = node.advertise(topic, MessageType(msg_type))
            time.sleep(0.02)
        return publishers[cache_key]

    for line in sys.stdin:
        if not line.strip():
            continue
        command = json.loads(line)
        if command.get("clearTopic"):
            publisher(command["clearTopic"], "gz.msgs.Entity").publish_raw(
                entity_msg(command.get("entity", {})),
                "gz.msgs.Entity",
            )
        publisher(command["topic"], "gz.msgs.EntityWrench").publish_raw(
            entity_wrench_msg(command),
            "gz.msgs.EntityWrench",
        )
        sys.stdout.write(json.dumps({"published": command.get("t", 0)}) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
