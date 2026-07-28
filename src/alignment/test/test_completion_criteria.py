from alignment.completion_criteria import (
    AlignmentCompletionChecker,
    AlignmentMeasurement,
    AlignmentThresholds,
)


def test_alignment_requires_consecutive_valid_frames() -> None:
    checker = AlignmentCompletionChecker(AlignmentThresholds(0.035, 0.02, 0.03, 3))
    aligned = AlignmentMeasurement(0.02, 0.01, 0.02, True)

    assert checker.update(aligned) is False
    assert checker.update(aligned) is False
    assert checker.update(aligned) is True


def test_invalid_landmark_resets_completion_counter() -> None:
    checker = AlignmentCompletionChecker(AlignmentThresholds(0.035, 0.02, 0.03, 2))
    aligned = AlignmentMeasurement(0.0, 0.0, 0.0, True)

    assert checker.update(aligned) is False
    assert checker.update(AlignmentMeasurement(0.0, 0.0, 0.0, False)) is False
    assert checker.update(aligned) is False
