# Legacy Code

This folder contains deprecated code that is no longer actively maintained.

## Contents

| File | Original Location | Description | Deprecated |
|:-----|:------------------|:------------|:-----------|
| `vostok1_integrated.py` | `plan/plan/vostok1.py` | Monolithic integrated navigation system | 11/12/2025 |

## Why Deprecated?

The **integrated Vostok1** was replaced by the **modular architecture** (OKO + SPUTNIK + BURAN) which offers:

- Better maintainability (separated concerns)
- Runtime parameter configuration via launch YAML
- A* path planning (hybrid + runtime modes)
- Hazard zone avoidance
- Enhanced SASS with Kalman filtering
- Easier debugging (test each node independently)

## Should I Use This?

**No.** Use the modular system instead:

```bash
# Recommended: Modular system
ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml
```

This code is kept for historical reference only.
