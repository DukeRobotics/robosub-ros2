# tau_ij = t_i - t_j; positive means channel_i received later than channel_j.
string channel_i
string channel_j
float64 physical_bound_s
# Preserve plausible carrier-cycle alternatives. Arrays below have equal length.
float64[] candidate_delays_s
# JSON object per candidate; named estimator-specific scalar quality metrics.
string[] candidate_quality_json
# ok, invalid, ambiguous, not_implemented (never a successful implicit zero delay).
string status
string reason
