# ==== REPLACE: systems/navigation/__init__.py ====
#!/usr/bin/env python3
"""
Navigation System Package
Potential field navigation for autonomous person following with obstacle avoidance
"""

from .field_calculations import PotentialFieldCalculations
from .navigation_state_machine import NavigationStateMachine, NavigationState
from .potential_field_navigator import PotentialFieldNavigator
from .navigation_diagnostics import NavigationDiagnosticsLogger


__all__ = [
    'PotentialFieldCalculations',
    'NavigationStateMachine', 
    'NavigationState',
    'PotentialFieldNavigator'
]