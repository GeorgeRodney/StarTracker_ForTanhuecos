Predict Step:
    Predict the new location of tracks based on their estimated Position and velocity.
    Input:  estimated Position, velocity
    Output: predicted Position

Association Step:
    Associate new detections with track predicted Position

Birth/Death Step:
    Create or Delete trackFiles based on persistence criteria

Correct Step:
    Estimate the tracks state with an update to the estimated position and vel given a state update equation.

TODO LIST
    [X] Add gating scheme to tracks
    [O] Fix infinty loop in AUCTION ALGORITHM
    [X] Hungarian Algorithm isnt excluding unlikely associations. Debug this.
    [X] Velocity isnt being filtered into state estimation 
    [X] Create visualization tool with python
    [X] Make the dets and tracks structure a member variable for the TFM
    [X] Fix binning association algorithm - Detections one is associating with multiple converged tracks
    [O] Gate the other association algorithms
    [O] Create GNN algorithm
    [O] Add single run command
        [O] 
    [O] Change explicit matrix multiplication to Armadillo / Eigen
    [O] INVESTIGATE WHY THE COVARIANCE MATRIX IS SHITTY
    [O] Add acceleration
    [O] Add history buffer
    [O] Add basic performance metric

