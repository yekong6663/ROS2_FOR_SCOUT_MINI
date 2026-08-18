from pathlib import Path


TREES = Path(__file__).resolve().parents[1] / "behavior_trees"

GENERAL = {
    "navigate_to_pose_lane_safe.xml",
    "navigate_to_pose_outdoor.xml",
    "navigate_through_poses_outdoor2_continuous.xml",
}

PRECISION_OR_STAGING = {
    "navigate_to_pose_staging.xml",
    "navigate_to_pose_outdoor_precision.xml",
    "navigate_to_pose_indoor_precision.xml",
    "navigate_to_pose_indoor03_precision.xml",
}


def test_general_trees_include_inflation_escape():
    for name in GENERAL:
        text = (TREES / name).read_text(encoding="utf-8")
        assert "ScoutEscapeInflation" in text, name


def test_grasp_place_trees_omit_inflation_escape():
    for name in PRECISION_OR_STAGING:
        text = (TREES / name).read_text(encoding="utf-8")
        assert "ScoutEscapeInflation" not in text, name


if __name__ == "__main__":
    test_general_trees_include_inflation_escape()
    test_grasp_place_trees_omit_inflation_escape()
    print("ok")
