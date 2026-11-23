"""Run the pinned LimX SDK suite with the bundled controller's ServoP format."""

import os
from pathlib import Path
import sys


def resolve_states_files(cases, sdk_root: Path):
    """Find pinned playback fixtures when the SDK is launched from this repo."""
    for case in cases:
        if "states_file" not in case:
            continue
        path = Path(case["states_file"])
        if not path.is_absolute() and not path.is_file() and (sdk_root / path).is_file():
            case["states_file"] = str(sdk_root / path)
    return cases


def main() -> int:
    upstream = os.environ.get("TRON2_UPSTREAM")
    if not upstream:
        raise SystemExit("Set TRON2_UPSTREAM to the pinned LimX controller checkout.")
    sdk_root = Path(upstream).expanduser().resolve() / "Tron2_arm_tests"
    if not (sdk_root / "dual_arm_protocol_tests" / "cli.py").is_file():
        raise SystemExit(f"LimX SDK runner missing from {sdk_root}. Run scripts/get_upstream.sh first.")
    sys.path.insert(0, str(sdk_root))

    from dual_arm_protocol_tests import cli

    upstream_expand_cases = cli.expand_cases

    def expand_cases(document):
        return resolve_states_files(upstream_expand_cases(document), sdk_root)

    class ServoPClient(cli.DualArmProtocolClient):
        def request(self, title, data=None, **kwargs):
            if title == "request_servop" and data is not None and "pos" in data:
                data = data.copy()
                pose = data.pop("pos")
                if not isinstance(pose, (list, tuple)) or len(pose) != 14:
                    raise ValueError("ServoP pos must contain seven pose values per arm.")
                data.setdefault("left_pos", pose[:7])
                data.setdefault("right_pos", pose[7:])
            return super().request(title, data, **kwargs)

    # The pinned CLI constructs the client itself. Substitute its class in this
    # runner process so upstream cases, sampling, and scoring are used directly.
    cli.DualArmProtocolClient = ServoPClient
    cli.expand_cases = expand_cases
    return cli.main()


if __name__ == "__main__":
    raise SystemExit(main())
