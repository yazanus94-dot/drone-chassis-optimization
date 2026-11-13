"""Typed exceptions for the drone chassis pipeline."""


class DroneChassisError(Exception):
    """Base class for all pipeline errors."""


class SizingError(DroneChassisError):
    """Raised when requirements or sizing inputs are physically inconsistent."""


class DesignConstraintError(DroneChassisError):
    """Raised when a design violates a hard constraint."""


class PipelineStageError(DroneChassisError):
    """Raised when a pipeline stage (CAD/mesh/structural/CFD/battery) fails."""
