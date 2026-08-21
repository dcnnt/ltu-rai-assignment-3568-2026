"""
Stage 1 del grounding via LLM: resolver a que REGION se refiere la instruccion.
Modelo open-weights: Qwen3 8B servido localmente con Ollama.

Idea clave: en vez de tool_choice forzado (Anthropic), usamos el parametro
`format` de Ollama con un JSON Schema construido dinamicamente a partir de
los region_ids reales del scene graph (enum=region_ids). Ollama aplica
constrained decoding sobre ese schema: el modelo no puede muestrear un
token que produzca un region_id fuera del enum -- no es "menos probable",
es imposible a nivel de sampling.

La instruccion puede venir en cualquier idioma; el system prompt esta en
ingles y no asume ningun idioma concreto para el input del usuario.

Requiere:
    ollama pull qwen3:8b
    pip install ollama
"""

import json
from ollama import chat

MODEL = "qwen3:8b"


def resolve_region(instruction: str, regions: dict) -> dict:

    region_ids = list(regions.keys())
    candidates_text = "\n".join(f"- {rid}: {r['label']}" for rid, r in regions.items())

    schema = {
        "type": "object",
        "properties": {
            "region_id": {
                "type": "string",
                "enum": region_ids,
                "description": "Exact id of the region the instruction refers to",
            },
            "confidence": {
                "type": "string",
                "enum": ["high", "low"],
                "description": "low if the instruction could plausibly refer to more than one region",
            },
        },
        "required": ["region_id", "confidence"],
    }

    response = chat(
        model=MODEL,
        format=schema,
        think=False,  # decision trivial (seleccionar 1 de N); activar si se ven fallos de precision
        options={"temperature": 0},
        messages=[
            {
                "role": "system",
                "content": (
                    "You resolve natural-language instructions to a region id in a "
                    "warehouse scene graph. The instruction may be given in any "
                    "language. Respond only with the JSON object defined by the schema."
                ),
            },
            {
                "role": "user",
                "content": (
                    f"Available regions in the warehouse:\n{candidates_text}\n\n"
                    f'Instruction: "{instruction}"\n\n'
                    "Which region does this instruction refer to?"
                ),
            },
        ],
    )

    result = json.loads(response.message.content)

    assert result["region_id"] in regions, "modelo devolvio un id fuera del grafo (no deberia pasar, format lo impide)"
    return result


if __name__ == "__main__":
    with open("JSON/scene_graph.json") as f:
        scene = json.load(f)

    instruction = input("Instruction: ")
    result = resolve_region(instruction, scene["regions"])
    print(result)