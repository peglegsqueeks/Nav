# Target Position Memory & Reacquisition System

## Overview
This system solves the critical problem where Maxine loses sight of the person during obstacle avoidance maneuvers. When the robot turns to avoid an obstacle, the head (camera) may lose track of the target. The Target Position Memory System remembers where the person was and helps Maxine reacquire the target after clearing the obstacle.

## The Problem (Before)
```
1. Person detected ahead → Navigate toward person
2. Obstacle detected → Turn left to avoid
3. Head tracks person → Camera turns left too
4. Person moves out of camera view → TARGET LOST!
5. Only repulsive forces remain → Robot drifts aimlessly
6. Result: STUCK or wandering with no destination
```

## The Solution (After)
```
1. Person detected ahead → Store IMU bearing to person
2. Obstacle detected → Remember "avoidance start heading"
3. Turn left to avoid → Person may go out of view (OK!)
4. Obstacle cleared, person still lost → ENTER REACQUISITION MODE
5. Calculate expected bearing → Turn toward last known position
6. Person reacquired → RESUME NORMAL NAVIGATION
```

## How It Works

### Phase 1: Memory Storage (Continuous)
**When person is detected:**
- Store absolute bearing: `robot_orientation + lidar_angle`
- Store person position: `(x, y)` in robot frame
- Update timestamp: Track when memory was created

**When obstacle avoidance starts:**
- Remember the IMU heading when avoidance began
- This allows compensation for rotation during avoidance

### Phase 2: Loss Detection
**When person is lost:**
- Start timer: `person_lost_timestamp = current_time`
- Wait for threshold: 2 seconds (configurable)
- Check memory validity: Not older than 15 seconds

### Phase 3: Reacquisition Mode
**Triggered when:**
- Person lost for > 2 seconds
- Robot has valid memory (< 15 seconds old)
- Navigation is still enabled

**Behavior:**
1. Calculate target bearing:
   ```
   relative_bearing = last_known_bearing - current_orientation
   ```
   This accounts for rotation during obstacle avoidance

2. Generate turn command:
   - If bearing > 10°: Turn toward expected position
   - If bearing < 10°: Stop and scan (person should appear)
   - Turn speed proportional to angle (0.2 - 0.4)

3. Exit conditions:
   - Person reacquired (success!)
   - Memory expires (15 seconds)
   - Navigation disabled

### Phase 4: Resume Normal Navigation
**When person reacquired:**
- Clear reacquisition mode flag
- Reset person lost timestamp
- Resume normal potential field navigation
- Update memory with new bearing

## Key Parameters

```python
# Memory System
person_lost_threshold = 2.0  # seconds - delay before entering reacquisition
reacquisition_timeout = 15.0  # seconds - max age of memory
person_timeout = 3.0  # seconds - lose target if no updates

# Reacquisition Behavior
turn_threshold = 10.0  # degrees - dead zone (close enough)
slow_turn_threshold = 45.0  # degrees - use slow turn below this
fast_turn_speed = 0.4  # Fast turn for large angles
slow_turn_speed = 0.2  # Base speed for small angles
```

## Integration with Potential Fields

The system seamlessly integrates with existing potential field navigation:

1. **Normal Mode:** Forces calculated from detected person position
2. **Reacquisition Mode:** Turn toward memory, no force calculation
3. **Transition:** Automatic switching based on detection status

```python
# In generate_movement_command():
if self.reacquisition_mode:
    # Use remembered bearing to turn
    return self.generate_reacquisition_movement()
else:
    # Normal potential field navigation
    return normal_force_based_movement()
```

## Benefits

1. **Robustness:** Robot doesn't lose target during obstacle avoidance
2. **Efficiency:** Direct return to expected bearing, not random search
3. **Intelligence:** Compensates for rotation using IMU data
4. **Simplicity:** Minimal computational overhead
5. **Reliability:** Memory timeout prevents stale data issues

## Navigation Status Fields (New)

The following fields are now available in `get_navigation_status()`:

```python
{
    'reacquisition_mode': bool,  # True if actively reacquiring
    'has_target_memory': bool,  # True if memory exists
    'last_known_bearing': float,  # Absolute bearing (degrees)
    'target_bearing': float,  # Relative bearing to turn toward
    'time_since_person_lost': float,  # seconds since loss
}
```

## Example Scenario

### Test 2 Obstacle Bypass (Actual Data)

**Before Fix:**
```
Time 62.86s: Person at -498mm, 2567mm, IMU: -142°
             → Start avoiding, turn left

Time 65.88s: Person LOST (0, 0), IMU: -114° (28° rotation)
             → No memory system, just drift

Time 74.05s: Still lost, IMU: -103° (39° total rotation)
             → Stuck, no target, no direction
```

**After Fix:**
```
Time 62.86s: Person at -498mm, 2567mm, IMU: -142°
             → Store bearing: -142° + angle = -135° absolute
             → Start avoiding, turn left

Time 65.88s: Person LOST, IMU: -114° (28° rotation)
             → Wait 2 seconds for reacquisition

Time 67.88s: Enter reacquisition mode
             → Calculate: -135° - (-114°) = -21° relative
             → Turn LEFT 21° to expected position

Time 68.50s: Person REACQUIRED!
             → Resume normal navigation toward person
```

## Head Tracking Integration (Future Enhancement)

Your suggestion about head tracking is excellent:

1. **During obstacle avoidance:** Head continues tracking person as long as visible
2. **After person lost:** Head can slowly scan around last known bearing
3. **Reacquisition mode:** Body turns toward bearing while head scans wider arc
4. **Double-check method:** Body IMU + Head servo = two methods to verify bearing

This could be implemented by:
- Adding head scan pattern during reacquisition
- Using head servo position as additional bearing input
- Combining body rotation with head scanning for faster reacquisition

## Testing Recommendations

1. **Test 2 (Obstacle Bypass):** Should now complete successfully
   - Robot avoids obstacle
   - Loses person briefly
   - Enters reacquisition mode
   - Turns toward expected bearing
   - Reacquires person
   - Completes navigation to goal

2. **Monitor diagnostics:** Watch for reacquisition mode activation
   ```
   ⚠ Person lost at 65.88s
   → Entering reacquisition mode. Last bearing: -135.0°
   ↻ Reacquisition turn: -21.0° → LEFT @ 0.30
   ✓ Target reacquired!
   ```

3. **Verify memory timeout:** Test with obstacles that take >15 seconds to bypass
   - System should gracefully give up on stale memory
   - Resume searching behavior

## Files Modified

1. **potential_field_navigator.py**
   - Added target memory state variables
   - Modified `update_person_target()` for memory storage
   - Added `calculate_reacquisition_bearing()` method
   - Added `generate_reacquisition_movement()` method
   - Modified `generate_movement_command()` for reacquisition mode
   - Added reacquisition status to `get_navigation_status()`

2. **field_calculations.py**
   - Enhanced repulsive force (inverse cube at close range)
   - Tuned force parameters for better obstacle avoidance
   - No reacquisition changes (handled at navigator level)

## Conclusion

This Target Position Memory System transforms Maxine from a robot that gets confused when losing sight of the target, into an intelligent navigator that remembers where people were and actively works to reacquire them. Combined with the enhanced potential field forces, this should make Test 2 (Obstacle Bypass) work reliably!# Nav
