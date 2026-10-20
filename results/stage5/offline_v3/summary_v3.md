# Stage 5 v3: generated buildings, 3 drift seeds per building (480 main runs)

| policy | drift | safe return (95 % CI) | runs with collision | rooms searched (mean) | n |
|---|---|---|---|---|---|
| Proposed | 2 % | 30/30 = 100 % (89-100) | 0/30 = 0 % | 4.5 | 30 |
| Proposed | 5 % | 30/30 = 100 % (89-100) | 0/30 = 0 % | 3.1 | 30 |
| Proposed | 10 % | 29/30 = 97 % (83-99) | 1/30 = 3 % | 1.6 | 30 |
| Proposed | 20 % | 20/30 = 67 % (49-81) | 3/30 = 10 % | 0.6 | 30 |
| No return decision | 2 % | 30/30 = 100 % (89-100) | 3/30 = 10 % | 7.0 | 30 |
| No return decision | 5 % | 4/30 = 13 % (5-30) | 6/30 = 20 % | 4.9 | 30 |
| No return decision | 10 % | 6/30 = 20 % (10-37) | 10/30 = 33 % | 3.3 | 30 |
| No return decision | 20 % | 6/30 = 20 % (10-37) | 17/30 = 57 % | 1.4 | 30 |
| Return on link loss | 2 % | 30/30 = 100 % (89-100) | 0/30 = 0 % | 0.7 | 30 |
| Return on link loss | 5 % | 23/30 = 77 % (59-88) | 0/30 = 0 % | 0.5 | 30 |
| Return on link loss | 10 % | 0/30 = 0 % (0-11) | 0/30 = 0 % | 0.4 | 30 |
| Return on link loss | 20 % | 0/30 = 0 % (0-11) | 0/30 = 0 % | 0.0 | 30 |
| No drift awareness | 2 % | 0/30 = 0 % (0-11) | 4/30 = 13 % | 5.9 | 30 |
| No drift awareness | 5 % | 0/30 = 0 % (0-11) | 0/30 = 0 % | 2.0 | 30 |
| No drift awareness | 10 % | 0/30 = 0 % (0-11) | 0/30 = 0 % | 1.0 | 30 |
| No drift awareness | 20 % | 0/30 = 0 % (0-11) | 3/30 = 10 % | 0.5 | 30 |

## Fisher exact tests (two-sided)

- safe return at 5-20 % drift, proposed 79/90 vs No return decision 16/90: p = 2.7e-22
- safe return at 5-20 % drift, proposed 79/90 vs Return on link loss 23/90: p = 7.4e-18
- safe return at 5-20 % drift, proposed 79/90 vs No drift awareness 0/90: p = 3.5e-39
- runs with a collision (all drifts), proposed 4/120 vs No return decision 36/120: p = 1.5e-08
- runs with a collision (all drifts), proposed 4/120 vs No drift awareness 7/120: p = 0.54
