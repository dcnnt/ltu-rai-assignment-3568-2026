from ground_stage_region import resolve_region
from ground_stage_object import resolve_object, NO_OBJECT
from validation import verify_plan


def run_mission(instruction: str, scene: dict) -> dict:
    region_result = resolve_region(instruction, scene["regions"])

    if region_result["confidence"] == "low":
        return {
            "executable": False,
            "failed_check": "region_confidence",
            "reason": "ambiguous instruction when resolving the region",
            "region_result": region_result,
        }

    object_result = resolve_object(instruction, region_result["region_id"], scene["objects"])

    if object_result.get("object_id") is None:
        return {
            "executable": False,
            "failed_check": "no_candidates",
            "reason": object_result.get("reason", "no candidates in the resolved region"),
            "region_result": region_result,
            "object_result": object_result,
        }

    verification = verify_plan(region_result, object_result, scene)

    if not verification["executable"]:
        return {
            "executable": False,
            "failed_check": verification["failed_check"],
            "reason": verification["reason"],
            "region_result": region_result,
            "object_result": object_result,
        }

    region_id = region_result["region_id"]
    object_id = object_result["object_id"]

    if object_id == NO_OBJECT:
        return {
            "executable": True,
            "instruction": instruction,
            "action": "navigate_to_area",
            "region_id": region_id,
            "object_id": NO_OBJECT,
            "region_bbox": scene["regions"][region_id]["bbox"],
            "region_result": region_result,
            "object_result": object_result,
        }

    target = scene["objects"][object_id]["position"]
    return {
        "executable": True,
        "instruction": instruction,
        "action": "navigate_to",
        "region_id": region_id,
        "object_id": object_id,
        "target_position": target,
        "region_result": region_result,
        "object_result": object_result,
    }


def format_robot_instruction(plan: dict) -> str:
    if not plan["executable"]:
        return f"REJECTED PLAN AT '{plan['failed_check']}': {plan['reason']}"

    if plan["action"] == "navigate_to_area":
        return (
            f"ROBOT INSTRUCTION -> navigate_to_area(region={plan['region_id']}, "
            f"bbox={plan['region_bbox']}) -- exact point resolved by mission_bridge_node"
        )

    t = plan["target_position"]
    return (
        f"ROBOT INSTRUCTION -> {plan['action']}("
        f"object={plan['object_id']}, region={plan['region_id']}, "
        f"target=({t['x']}, {t['y']}))"
    )