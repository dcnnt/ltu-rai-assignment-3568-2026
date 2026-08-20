import math

SAFETY_RADIUS_M = 2.0
NO_OBJECT = "NO_OBJECT"


def check_existence(object_id, objects):
    if object_id not in objects:
        return False, f"object_id '{object_id}' no existe en el scene graph"
    return True, "ok"


def check_hierarchy(region_id, object_id, objects):
    actual_parent = objects[object_id]["parent_region"]
    if actual_parent != region_id:
        return False, (
            f"inconsistencia jerarquica: '{object_id}' pertenece a "
            f"'{actual_parent}', no a la region resuelta '{region_id}'"
        )
    return True, "ok"


def check_confidence(region_result, object_result):
    if region_result["confidence"] == "low":
        return False, "Low confidence region"
    if object_result["confidence"] == "low":
        return False, "Low confidence object"
    return True, "ok"


def check_human_safety(object_id, objects, radius_m=SAFETY_RADIUS_M):
    target = objects[object_id]["position"]
    for oid, o in objects.items():
        if o["type"] != "human":
            continue
        d = math.dist((target["x"], target["y"]), (o["position"]["x"], o["position"]["y"]))
        if d < radius_m:
            return False, f"humano '{oid}' a {d:.2f}m del objetivo (< {radius_m}m de seguridad)"
    return True, "ok"


def check_human_in_region(region_id, scene):
    x1, y1, x2, y2 = scene["regions"][region_id]["bbox"]
    xmin, xmax = min(x1, x2), max(x1, x2)
    ymin, ymax = min(y1, y2), max(y1, y2)
    for oid, o in scene["objects"].items():
        if o["type"] != "human":
            continue
        px, py = o["position"]["x"], o["position"]["y"]
        if xmin <= px <= xmax and ymin <= py <= ymax:
            return False, f"humano '{oid}' presente en la region objetivo '{region_id}'"
    return True, "ok"


def verify_plan(region_result, object_result, scene):
    objects = scene["objects"]
    object_id = object_result.get("object_id")
    region_id = region_result["region_id"]

    if object_id == NO_OBJECT:
        # plan de area: sin objeto concreto, target aun no existe
        checks = [
            ("confidence", lambda: check_confidence(region_result, object_result)),
            ("safety", lambda: check_human_in_region(region_id, scene)),
        ]
    else:
        checks = [
            ("existence", lambda: check_existence(object_id, objects)),
            ("hierarchy", lambda: check_hierarchy(region_id, object_id, objects)),
            ("confidence", lambda: check_confidence(region_result, object_result)),
            ("safety", lambda: check_human_safety(object_id, objects)),
        ]

    for name, check_fn in checks:
        ok, reason = check_fn()
        if not ok:
            return {"executable": False, "failed_check": name, "reason": reason}

    return {"executable": True, "failed_check": None, "reason": "todos los chequeos pasados"}