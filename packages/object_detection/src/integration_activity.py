from typing import Tuple


def DT_TOKEN() -> str:
    # TODO: change this to your duckietown token
    dt_token = "dt1-3nT7FDbT7NLPrXykNJmqqhP9kMhgdt235rZQhfP9rR8exQL-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfWXvZzwN8ZUNTMZ87tSRdLiN7vtotEURDT"
    return dt_token

def NUMBER_FRAMES_SKIPPED() -> int:
    # TODO: change this number to drop more frames
    # (must be a positive integer)
    return 3

def filter_by_classes(pred_class: int) -> bool:
    """
    Remember the class IDs:

        | Object    | ID    |
        | ---       | ---   |
        | Duckie    | 0     |
        | Cone      | 1     |
        | Truck     | 2     |
        | Bus       | 3     |


    Args:
        pred_class: the class of a prediction
    """
    # Right now, this returns True for every object's class
    # TODO: Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    # List of class IDs to detect
    target_classes = [3, 15, 16, 17]  # Assuming these are the correct IDs for the specified classes
    return pred_class in target_classes


def filter_by_scores(score: float) -> bool:
    """
    Args:
        score: the confidence score of a prediction
    """
    # Right now, this returns True for every object's confidence
    # TODO: Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    return score > 0.80


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    # TODO: Like in the other cases, return False if the bbox should not be considered.
    # If box is too small or too large, ignore. Else, send detection signal. 
    width = abs(bbox[2] - bbox[0])
    height = abs(bbox[1] - bbox[3])

    if width < 35 or height < 50:
        return False
    if width > 300 or height > 300:
        return False
    return True
