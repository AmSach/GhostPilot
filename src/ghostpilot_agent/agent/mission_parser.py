#!/usr/bin/env python3
"""
GhostPilot Agentic AI - Mission Parser

Converts natural language mission commands into executable navigation goals.
Uses local LLM for privacy-preserving, battlefield-ready operation.
"""

import json
import re
from typing import List, Dict, Optional, Any
from dataclasses import dataclass


@dataclass
class Goal:
    """Represents a single navigation or action goal."""
    location: Optional[str] = None
    action: str = ""
    constraints: Dict[str, Any] = None
    waypoint: Optional[Dict[str, float]] = None
    
    def __post_init__(self):
        if self.constraints is None:
            self.constraints = {}
    
    def to_dict(self) -> dict:
        return {
            "location": self.location,
            "action": self.action,
            "constraints": self.constraints,
            "waypoint": self.waypoint
        }


class MissionParser:
    """Parses natural language mission commands into goal sequences."""
    
    # Action verbs and their semantic meanings
    ACTION_MAP = {
        "navigate": "navigate",
        "fly to": "navigate",
        "go to": "navigate",
        "move to": "navigate",
        "travel to": "navigate",
        "inspect": "inspect",
        "check": "inspect",
        "scan": "inspect",
        "search": "inspect",
        "look at": "inspect",
        "avoid": "avoid",
        "stay clear": "avoid",
        "keep away": "avoid",
        "land": "land",
        "touch down": "land",
        "return home": "return",
        "go back": "return",
        "report": "report",
        "broadcast": "report",
    }
    
    # Location patterns
    LOCATION_PATTERNS = [
        r'\b(building [A-Z])\b',
        r'\b(floor \d+)\b',
        r'\b(zone [A-Z])\b',
        r'\b(room \d+)\b',
        r'\b(warehouse|factory|mine)\b',
        r'\b(helipad|landing pad|home base)\b',
        r'\b(entrance|exit|door)\b',
        r'\b(central pillar|obstacle|barrier)\b',
        r'\b(east wing|west wing|north|south)\b',
    ]
    
    # Constraint patterns
    CONSTRAINT_PATTERNS = {
        "people": r'\b(avoid|clear|check for)\s+(people|occupants|humans|person)\b',
        "height": r'\b(altitude|height)\s+(\d+)\s*(m|meter)?',
        "speed": r'\b(speed|velocity)\s+(slow|fast|moderate)',
        "ceiling": r'\b(ceiling|indoor|inside|confined)\b',
    }
    
    def __init__(self, llm_provider: Optional[str] = None):
        """
        Initialize mission parser.
        
        Args:
            llm_provider: LLM backend ('ollama', 'llama_cpp', 'openai', None for rule-based)
        """
        self.llm_provider = llm_provider
    
    def parse(self, mission_command: str) -> List[Goal]:
        """
        Parse a mission command into a sequence of goals.
        
        Args:
            mission_command: Natural language mission description
            
        Returns:
            List of Goal objects representing the mission sequence
        """
        command = mission_command.lower().strip()
        
        if self.llm_provider:
            return self._parse_with_llm(command)
        return self._parse_rule_based(command)
    
    def _parse_rule_based(self, command: str) -> List[Goal]:
        """Parse using rule-based approach (no LLM required)."""
        goals = []
        
        # Split into sentences for sequential goals
        sentences = re.split(r'[,;]|\bthen\b|\band then\b|\bafter that\b', command)
        
        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue
            
            # Extract action
            action = self._extract_action(sentence)
            
            # Extract location
            location = self._extract_location(sentence)
            
            # Extract constraints
            constraints = self._extract_constraints(sentence)
            
            goal = Goal(
                location=location,
                action=action,
                constraints=constraints
            )
            goals.append(goal)
        
        # Add default return/home if implied
        if any(word in command for word in ['inspect', 'check', 'search', 'scan']):
            if not any(g.action == 'return' for g in goals):
                goals.append(Goal(action='return', location='home'))
        
        return goals
    
    def _parse_with_llm(self, command: str) -> List[Goal]:
        """Parse using LLM for more complex commands."""
        # Placeholder for LLM integration
        # In production, this would call ollama or llama_cpp
        return self._parse_rule_based(command)
    
    def _extract_action(self, sentence: str) -> str:
        """Extract the action verb from a sentence."""
        for pattern, action in self.ACTION_MAP.items():
            if pattern in sentence:
                return action
        
        # Default action
        return "navigate"
    
    def _extract_location(self, sentence: str) -> Optional[str]:
        """Extract location references from sentence."""
        for pattern in self.LOCATION_PATTERNS:
            match = re.search(pattern, sentence, re.IGNORECASE)
            if match:
                return match.group(0)
        
        # Check for implicit locations
        if 'return' in sentence or 'home' in sentence:
            return 'home'
        
        return None
    
    def _extract_constraints(self, sentence: str) -> Dict[str, Any]:
        """Extract navigation constraints from sentence."""
        constraints = {}
        
        for key, pattern in self.CONSTRAINT_PATTERNS.items():
            match = re.search(pattern, sentence, re.IGNORECASE)
            if match:
                if key == "height":
                    constraints["altitude"] = float(match.group(2))
                elif key == "speed":
                    constraints["speed"] = match.group(2)
                else:
                    constraints[key] = True
        
        return constraints
    
    def validate_goals(self, goals: List[Goal]) -> tuple[bool, Optional[str]]:
        """
        Validate that a goal sequence is executable.
        
        Returns:
            (is_valid, error_message)
        """
        if not goals:
            return False, "No goals generated"
        
        for i, goal in enumerate(goals):
            if not goal.action:
                return False, f"Goal {i+1} has no action"
        
        return True, None


def main():
    """Demo/test the mission parser."""
    parser = MissionParser()
    
    test_commands = [
        "Inspect building B, avoid people, report damage",
        "Fly to the third floor, check each room for occupants, land at the helipad",
        "Navigate to zone A, scan for obstacles, return home",
        "Search the warehouse for survivors, avoid the central pillar"
    ]
    
    print("GhostPilot Mission Parser Demo")
    print("=" * 50)
    
    for cmd in test_commands:
        print(f"\nCommand: '{cmd}'")
        goals = parser.parse(cmd)
        print(f"Goals ({len(goals)}):")
        for i, goal in enumerate(goals, 1):
            print(f"  {i}. {goal.action.upper()} -> {goal.location or 'unknown'}")
            if goal.constraints:
                print(f"     Constraints: {goal.constraints}")


if __name__ == "__main__":
    main()