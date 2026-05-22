"""
Converts touch detector readings into concise natural-language descriptions
for the Ollama prompt.
"""

from __future__ import annotations

from petctl.perception.contact import ContactReading, ContactType
from petctl.perception.stroke import HoldReading, StrokeReading


def format_touch_state(
    stroke: StrokeReading | None,
    hold: HoldReading | None,
    contact: ContactReading | None,
) -> str:
    """Return a one-to-two sentence description of the current touch state."""
    if stroke is not None:
        direction = "head to tail" if stroke.direction == "head_to_tail" else "tail to head"
        speed_label = _speed_label(stroke.speed)
        intensity_label = _intensity_label(stroke.intensity)
        return (
            f"Stroke detected: moving {direction} at {speed_label} speed, "
            f"{intensity_label} intensity, on the {stroke.side} face. "
            f"Centroid at module {stroke.centroid:.1f}."
        )

    if contact is not None:
        hold = contact.hold
        ctype = contact.contact_type
        duration_s = f"{hold.duration:.1f}s"
        intensity_label = _intensity_label(hold.intensity)
        base = (
            f"Static contact: {ctype.name.lower()} on {hold.side} face, "
            f"centroid module {hold.centroid:.1f}, {duration_s} duration, "
            f"{intensity_label} intensity."
        )
        if ctype == ContactType.SQUEEZE:
            base += f" Pressure peak: {contact.pressure_peak:.2f}."
        elif ctype in (ContactType.RESTRICT, ContactType.WRENCH):
            base += f" Torque peak: {contact.torque_peak:.2f} Nm."
        return base

    if hold is not None:
        duration_s = f"{hold.duration:.1f}s"
        intensity_label = _intensity_label(hold.intensity)
        return (
            f"Gentle hold: centroid module {hold.centroid:.1f}, "
            f"{duration_s} duration, {intensity_label} intensity, "
            f"on the {hold.side} face."
        )

    return "No touch detected. The robot is idle."


def _speed_label(speed: float) -> str:
    if speed < 0.5:
        return "slow"
    if speed < 1.5:
        return "moderate"
    if speed < 3.0:
        return "fast"
    return "very fast"


def _intensity_label(intensity: float) -> str:
    if intensity < 0.2:
        return "very light"
    if intensity < 0.4:
        return "light"
    if intensity < 0.6:
        return "moderate"
    if intensity < 0.8:
        return "firm"
    return "strong"
