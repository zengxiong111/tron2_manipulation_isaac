"""Run the pinned LimX SDK suite with the bundled controller's ServoP format."""

import os
from pathlib import Path
import sys


def main() -> int:
    upstream = os.environ.get("TRON2_UPSTREAM")
    if not upstream:
        raise SystemExit("Set TRON2_UPSTREAM to the pinned LimX controller checkout.")
    sdk_root = Path(upstream).expanduser().resolve() / "Tron2_arm_tests"
    if not (sdk_root / "dual_arm_protocol_tests" / "cli.py").is_file():
        raise SystemExit(f"LimX SDK runner missing from {sdk_root}. Run scripts/get_upstream.sh first.")
    sys.path.insert(0, str(sdk_root))

    from dual_arm_protocol_tests import cli

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
    return cli.main()


if __name__ == "__main__":
    raise SystemExit(main())
