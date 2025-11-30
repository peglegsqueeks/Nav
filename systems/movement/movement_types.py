#!/usr/bin/env python3
"""
Movement types and base classes for robot movement system
Contains MovementDirection enum, VelocityConfig, and ActionManager base class
"""
from abc import ABC, abstractmethod
from enum import Enum

class MovementDirection(Enum):
    """Movement direction enumeration for robot base control"""
    NONE = "NONE"
    FORWARDS = "FORWARDS"
    BACKWARDS = "BACKWARDS"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    FORWARDS_LEFT = "FORWARDS_LEFT"
    FORWARDS_RIGHT = "FORWARDS_RIGHT"
    BACKWARDS_LEFT = "BACKWARDS_LEFT"
    BACKWARDS_RIGHT = "BACKWARDS_RIGHT"

class VelocityConfig:
    """Configuration for velocity actions"""
    def __init__(self, direction: MovementDirection, speed: float):
        self.direction = direction
        self.speed = speed
    
    def __str__(self):
        return f"VelocityConfig(direction={self.direction.value}, speed={self.speed})"

class ActionManager(ABC):
    """Base class for an Action Manager"""
    
    def __init__(self, name) -> None:
        self.name = name

    @abstractmethod
    def perform_action(self, config):
        """Perform an action from this manager"""
        pass