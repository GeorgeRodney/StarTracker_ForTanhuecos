Get project up and running:
    This is a difficult process to get set up. Thats just the nature of the beast.
    If you are able to grind through this part you will be set up with a build environment
    that is standard in the software development career field. Be patient with yourself.
    Setting up a development environment is difficult. Please reach out as soon as you get stuck. 

-1 Install VSCode for Mac - https://code.visualstudio.com/Download

-2 Install VSCode extensions:
    - CMake Tools
    - CMake
    - C/C++ by microsoft
    - Python by Microsoft
    - Python Debugger by Microsoft

-3 Download StarTracker_ForTanhuecos repo
    - Request access to my Github Repo: https://github.com/GeorgeRodney/StarTracker_ForTanhuecos
    - Set up your SSH keys (There are tutorials for this online. Call me if you are having trouble with this)
    - Clone the repo into your working folder on your computer

-4 Build the project
    - Create build folder and build project
    - Run these commands from where you cloned the repo (These might be different based on Mac terminal commands)
        - mkdir build
        - cd build
        - cmake ..
        - make

-5 Call me when you get here. 

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

    [X] Change explicit matrix multiplication to Armadillo / Eigen
    [X] INVESTIGATE WHY THE COVARIANCE MATRIX IS SHITTY
    [O] Add acceleration
    [O] Add history buffer
    [O] Add basic performance metric
        [X] Added unique ID to trackFiles
        [X] Added truth ID to detections
        [X] Added truth ID hand off from det to track during correlation
        [O] Create Track to Truth assignment matrix
        [O] Define a single metric and plot it
