"""Online scale of the front-steer executor (PW-10, docs/steering/ACTUATOR_PRIOR_KO.md 4, OPERATOR_API_KO.md 6.1).

The vendored estimator's actuator prior (closed_loop/actuator_prior.py, weight 0 = record and monitor only) measures
the online scale k = LiDAR distance / prior distance over the last ~20 s of driving (monitor_window_s), where the prior
speed is table(applied duty) x prior_speed_scale and prior_speed_scale = speed_scale (k_field) x surface_scale x
battery_scale of the field tuning. Only scan-matched intervals enter; while the LiDAR speed fails that check (a failed
match, a scan gap), the vehicle stands or no status line arrives, the scale is held and the block says why.

This module keeps, per executor loop:
  block      the newest scale values of the current (or last) run, for the status and the screen
  offer      after a run ended: the proposed new k_field = prior_speed_scale x k / (surface x battery), rounded to
             PROPOSAL_DIGITS, with the reason when it can not be taken (why_not). The operator answers it once
             (accept: the executor applies speed_scale like a screen change, only at rest; dismiss); a new run
             supersedes an unanswered offer.
It reads nothing itself and never changes a decision; the executor feeds it the estimator block of every tick.
Standard library only. Every value is a placeholder-based quantity (자리값); nothing here was validated in the field.
"""
from __future__ import annotations

import math

PROPOSAL_DIGITS = 3            # k_field step of a proposal (the screen shows 0.85, the file keeps 3 digits)
NO_CHANGE = .5 * 10. ** -PROPOSAL_DIGITS
ANSWERS = ('accept', 'dismiss')
WHY_NOT = ('no_scale', 'no_forward_motion', 'scale_set_in_config', 'out_of_range', 'no_change', 'no_field_tuning')


def _number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def compact(prior):
    """The scale part of the estimator's 'actuator_prior' block (plain values)."""
    return dict(value=_number(prior.get('online_scale')), window_s=_number(prior.get('monitor_s')),
                target_s=_number(prior.get('monitor_window_s')), min_s=_number(prior.get('monitor_min_s')),
                frozen=prior.get('online_scale_frozen') is True, hold=prior.get('online_scale_hold'),
                age_s=_number(prior.get('online_scale_age_s')), mismatch=prior.get('table_mismatch') is True,
                unmatched=prior.get('unmatched_scans'))


def proposal(block, tuning_values, prior_scale, key_range, scale_range):
    """(k_field_new | None, why_not | None) of a run's last scale block.

    tuning_values: the resolved field tuning values (speed_scale, surface_scale, battery_scale) or None;
    prior_scale: the executor config's prior_speed_scale the run measured against; key_range: (low, high) of the
    speed_scale key; scale_range: (low, high) of the product speed_scale x surface x battery."""
    if tuning_values is None:
        return None, 'no_field_tuning'
    value, window, minimum = block.get('value'), block.get('window_s'), block.get('min_s')
    if value is None or window is None or minimum is None or window < minimum:
        return None, 'no_scale'
    if not value > 0.:
        return None, 'no_forward_motion'
    factors = float(tuning_values['surface_scale']) * float(tuning_values['battery_scale'])
    if abs(float(prior_scale) - float(tuning_values['speed_scale']) * factors) > 1e-6:
        return None, 'scale_set_in_config'       # an explicit prior_speed_scale of the executor config wins over k_field
    new = round(float(prior_scale) * value / factors, PROPOSAL_DIGITS)
    if not key_range[0] - 1e-9 <= new <= key_range[1] + 1e-9:
        return new, 'out_of_range'
    if not scale_range[0] - 1e-9 <= new * factors <= scale_range[1] + 1e-9:
        return new, 'out_of_range'
    if abs(new - float(tuning_values['speed_scale'])) < NO_CHANGE:
        return new, 'no_change'
    return new, None


class OnlineScale:
    """Per executor loop: the newest scale block of a run and the offer after it ended (module docstring)."""

    def __init__(self):
        self.run_id = None           # run of `block`
        self.block = None            # compact() of the newest estimator block of that run
        self.offer = None            # dict (offer()) after a run ended, until answered or superseded
        self._offered_run = None     # run id of the newest offer (one offer per run)

    def track(self, run_id, ended, prior, end_reason, tuning_values, prior_scale, key_range, scale_range):
        """One tick: the estimator block of run `run_id` (None while the prior is off). Returns the events to log
        [(name, fields)]: online_scale_offer when the run ended, online_scale_answer 'superseded' when a new run
        starts before the offer of the last one was answered."""
        events = []
        if run_id is None:
            return events
        if run_id != self.run_id and not ended:
            if self.offer is not None and self.offer['run_id'] != run_id:
                events.append(('online_scale_answer', dict(self.answer_fields(), answer='superseded', by_run=run_id)))
                self.offer = None
            self.run_id, self.block = run_id, None
        if prior is not None and run_id == self.run_id:
            self.block = compact(prior)
        if ended and run_id == self.run_id and self._offered_run != run_id and self.block is not None:
            self._offered_run = run_id
            new, why = proposal(self.block, tuning_values, prior_scale, key_range, scale_range)
            self.offer = dict(run_id=run_id, value=self.block['value'], window_s=self.block['window_s'],
                              end_reason=end_reason, prior_scale=prior_scale,
                              speed_scale_now=None if tuning_values is None else tuning_values['speed_scale'],
                              surface_scale=None if tuning_values is None else tuning_values['surface_scale'],
                              battery_scale=None if tuning_values is None else tuning_values['battery_scale'],
                              speed_scale_new=new, ok=why is None, why_not=why)
            events.append(('online_scale_offer', dict(self.offer)))
        return events

    def answer_fields(self):
        o = self.offer or {}
        return dict(run_id=o.get('run_id'), value=o.get('value'), window_s=o.get('window_s'),
                    speed_scale_now=o.get('speed_scale_now'), speed_scale_new=o.get('speed_scale_new'))

    def basis_changed(self, tuning_values):
        """True when the tuning's scale factors are not the ones the offer was made from (changed in between)."""
        o = self.offer
        return any(abs(float(tuning_values[key]) - float(o[name])) > 1e-9 for key, name in
                   (('speed_scale', 'speed_scale_now'), ('surface_scale', 'surface_scale'),
                    ('battery_scale', 'battery_scale')))

    def status(self, enabled, tuning_values, prior_scale):
        """The status block 'online_scale' (OPERATOR_API_KO.md 6.1)."""
        block = self.block or {}
        out = dict(enabled=bool(enabled), run_id=self.run_id if self.block is not None else None,
                   value=block.get('value'), window_s=block.get('window_s'), target_s=block.get('target_s'),
                   min_s=block.get('min_s'), frozen=block.get('frozen', False),
                   hold=block.get('hold') if enabled else 'monitor_off', age_s=block.get('age_s'),
                   mismatch=block.get('mismatch', False),
                   speed_scale=None if tuning_values is None else tuning_values['speed_scale'],
                   prior_scale=prior_scale if enabled else None, offer=None if self.offer is None else dict(self.offer))
        return out
