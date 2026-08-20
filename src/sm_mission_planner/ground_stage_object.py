import json
from ollama import chat

MODEL = "qwen3:8b"
NO_OBJECT = "NO_OBJECT"  

def resolve_object(instruction: str, region_id: str, objects: dict) -> dict:
    candidates = {oid: o for oid, o in objects.items() if o["parent_region"] == region_id}

    if not candidates:
        return {"object_id": None, "confidence": "low", "reason": f"no hay objetos en {region_id}"}

    object_ids = list(candidates.keys()) + [NO_OBJECT]
    candidates_text = "\n".join(
        f"- {oid}: type={o['type']}, pos=({o['position']['x']}, {o['position']['y']})"
        for oid, o in candidates.items()
    )

    schema = {
        "type": "object",
        "properties": {
            "object_id": {
                "type": "string",
                "enum": object_ids,
                "description": (
                    "Exact id of the referenced object, within the already resolved region. "
                    f"Use '{NO_OBJECT}' if the instruction only refers to the region/area in "
                    "general, with no specific object mentioned (e.g. 'go to the break area')."
                ),
            },
            "confidence": {
                "type": "string",
                "enum": ["high", "low"],
                "description": "low if the instruction could refer to more than one object of this region",
            },
        },
        "required": ["object_id", "confidence"],
    }

    response = chat(
        model=MODEL,
        format=schema,
        think=False,
        options={"temperature": 0},
        messages=[
            {
                "role": "system",
                "content": (
                    "You resolve natural-language instructions to an object id in a "
                    "warehouse scene graph. The instruction may be given in any "
                    f"language. If no specific object is mentioned, respond with "
                    f"'{NO_OBJECT}' rather than guessing one. Respond only with the "
                    "JSON object defined by the schema."
                ),
            },
            {
                "role": "user",
                "content": (
                    f"Available objects in region '{region_id}':\n{candidates_text}\n\n"
                    f'Instruction: "{instruction}"\n\n'
                    "Which object does this instruction refer to?"
                ),
            },
        ],
    )

    result = json.loads(response.message.content)

    assert result["object_id"] in candidates or result["object_id"] == NO_OBJECT, \
        "Error model results"
    return result