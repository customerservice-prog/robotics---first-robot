import re
from pathlib import Path


HTML = Path("static/index.html").read_text(encoding="utf-8")


def test_dashboard_has_one_inline_script():
    scripts = re.findall(r"<script>([\s\S]*?)</script>", HTML)
    assert len(scripts) == 1
    assert scripts[0].strip()


def test_dashboard_element_ids_are_unique():
    ids = re.findall(r'\bid="([^"]+)"', HTML)
    duplicates = sorted({element_id for element_id in ids if ids.count(element_id) > 1})
    assert duplicates == []


def test_navigation_dashboard_controls_exist():
    required = {
        "mapCanvas",
        "goalX",
        "goalY",
        "goalHeading",
        "planRoute",
        "startRoute",
        "cancelRoute",
        "replanRoute",
        "setDock",
        "planDock",
        "startDock",
        "resetPose",
        "clearMap",
        "odomPill",
        "locPill",
        "navPill",
        "dockState",
        "mapReference",
        "locConfidence",
        "locCorrection",
        "saveMap",
        "loadMap",
        "freezeMap",
        "resumeMap",
        "matchPose",
        "relocalizeWide",
        "locHintX",
        "locHintY",
        "locHintHeading",
    }
    ids = set(re.findall(r'\bid="([^"]+)"', HTML))
    assert required <= ids
