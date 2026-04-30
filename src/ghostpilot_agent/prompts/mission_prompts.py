"""
GhostPilot Agentic AI Prompts

LLM prompts for natural language mission parsing and execution.
Designed for local deployment (ollama/llama_cpp) for privacy.
"""

# System prompt for mission understanding
MISSION_SYSTEM_PROMPT = """You are GhostPilot, an agentic AI for GPS-denied drone navigation.

You convert natural language mission commands into executable navigation goals.

Mission commands come from operators who may not be drone experts. Your job is to:
1. Understand the intent behind the command
2. Break it into logical navigation goals
3. Identify constraints (avoid people, altitude limits, etc.)
4. Ensure the mission is safe and executable

Output format: JSON list of goals, each with:
- action: navigate | inspect | avoid | land | return | report
- location: descriptive location name or null
- constraints: object with any relevant constraints

Example:
Input: "Inspect building B, avoid people, report any structural damage"
Output: [
  {"action": "navigate", "location": "building_B_entrance", "constraints": {}},
  {"action": "inspect", "location": "building_B_floor_1", "constraints": {"check_occupants": true}},
  {"action": "inspect", "location": "building_B_floor_2", "constraints": {"check_occupants": true}},
  {"action": "inspect", "location": "building_B_floor_3", "constraints": {"check_occupants": true}},
  {"action": "report", "location": null, "constraints": {"content": "structural_damage"}}
]

Always output valid JSON. If mission is unclear, ask for clarification."""

# Prompt for safety validation
SAFETY_VALIDATION_PROMPT = """Review this mission command for safety concerns:

{mission_command}

Check for:
1. Collisions with known obstacles
2. Flight into restricted areas
3. Battery/time requirements
4. Weather constraints
5. Regulatory compliance (flying over people if not allowed)

Respond with:
- safe: true/false
- concerns: list of specific concerns
- recommendations: suggested modifications for safer execution"""

# Prompt for mission reporting
REPORT_GENERATION_PROMPT = """Based on mission execution results:

Goals attempted: {goals_attempted}
Goals succeeded: {goals_succeeded}
Goals failed: {goals_failed}
Failure reasons: {failure_reasons}

Generate a mission report in this format:
## Mission Report
### Summary
[2-3 sentence overview of what happened]

### Goals Completed
- [list completed goals]

### Goals Failed
- [list failed goals with reasons]

### Observations
[Any notable observations during mission execution]

### Recommendations
[Any recommendations for future missions of this type]"""


def get_mission_prompt(command: str) -> str:
    """Get formatted prompt for mission parsing."""
    return f"{MISSION_SYSTEM_PROMPT}\n\nMission command: {command}"


def get_safety_prompt(command: str) -> str:
    """Get formatted prompt for safety validation."""
    return SAFETY_VALIDATION_PROMPT.format(mission_command=command)


def get_report_prompt(
    goals_attempted: int,
    goals_succeeded: int,
    goals_failed: int,
    failure_reasons: list
) -> str:
    """Get formatted prompt for report generation."""
    return REPORT_GENERATION_PROMPT.format(
        goals_attempted=goals_attempted,
        goals_succeeded=goals_succeeded,
        goals_failed=goals_failed,
        failure_reasons=failure_reasons or ["none"]
    )