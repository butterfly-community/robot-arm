"""Extract input components from the Runtime's interaction-profile definitions."""

import json
import sys
from pathlib import Path


def action_type(component: str) -> str | None:
    if component in {"click", "touch"}:
        return "boolean"
    if component in {"value", "force", "proximity", "x", "y"}:
        return "float"
    return None


source = json.loads(Path(sys.argv[1]).read_text())
profiles = []
for profile_path, definition in sorted(source["profiles"].items()):
    if not profile_path.startswith("/interaction_profiles/"):
        continue
    components = []
    poses = []
    for subpath, item in definition.get("subpaths", {}).items():
        for component in item.get("components", []):
            expanded = ["x", "y"] if component == "position" else [component]
            for leaf in expanded:
                suffix = f"{subpath}/{leaf}"
                if leaf == "pose":
                    poses.append(suffix)
                elif (kind := action_type(leaf)) is not None:
                    components.append(
                        {
                            "suffix": suffix,
                            "action_type": kind,
                            "localized_name": item.get("localized_name"),
                        }
                    )
    profiles.append(
        {
            "profile": profile_path,
            "user_paths": definition.get("subaction_paths", []),
            "components": components,
            "pose_suffixes": poses,
        }
    )

destination = Path(sys.argv[2])
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(json.dumps({"profiles": profiles}, indent=2) + "\n")
