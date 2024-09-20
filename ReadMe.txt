Predict Step:
    Predict the new location of tracks based on their estimated Position and velocity.
    Input:  estimated Position, velocity
    Output: predicted Position

Association Step:
    Associate new detections with track predicted Position

Birth/Death Step:
    Create of Delete trackFiles based on persistence criteria

Correct Step:
    Estimate the tracks state with an update to the estimated position and vel given a state update equation.

TODO LIST
    [ ] Add gating scheme to tracks
    [ ] Fix infinty loop in AUCTION ALGORITHM