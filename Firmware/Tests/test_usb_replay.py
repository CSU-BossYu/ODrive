from types import SimpleNamespace

from odrive_hil.replay import (decode_bundle, encode_bundle, make_bundle,
                                mark_stale_feedback)


def test_hil_replay_uses_versioned_production_format():
    status = SimpleNamespace(
        build_id="0011223344556677",
        manifest_identity="8899aabbccddeeff0011223344556677",
        protocol_release="4.5",
        protocol_version=4,
        schema_version=4,
    )
    bundle = make_bundle(
        status=status,
        raw_capture=b"",
        events=[{"type": "scope_data", "samples": [{
            "control_sequence": 1,
            "timestamp_cycles": 0xFFFFFFF0,
        }]}],
    )

    replay = decode_bundle(encode_bundle(bundle))
    stale = mark_stale_feedback(
        replay, now_cycles=5, maximum_age_cycles=20)

    assert replay["format"] == "odrive-usb-replay"
    assert replay["schema_version"] == 1
    assert stale["capture"]["events"][0]["samples"][0][
        "feedback_stale"] is True
