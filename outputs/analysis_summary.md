# Controller comparison results

The simulation compares P, PI and PID control for a 150 rad/s speed command and a
0.20 N m load disturbance at 2.50 s.

## Metrics

```text
            Rise time (s)  Overshoot (%)  Settling time (s)  Final absolute error (rad/s)  Load recovery time (s)
Controller                                                                                                       
P             not reached         0.0000        not reached                       51.3819             not reached
PI                  0.776         7.0722              1.831                        0.6987                   0.519
PID                 0.777         5.6828              1.748                        0.3241                   0.417
```

## Main finding

**PID control produced the smallest final absolute speed error** in this parameter study.
These gains are illustrative; a real design would require system identification, safety margins and hardware testing.
