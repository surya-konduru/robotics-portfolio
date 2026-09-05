# RoboV — Autonomous Robot Path Planning Simulator

RoboV is an interactive Python and Streamlit simulator for exploring autonomous robot path planning on a 10×10 grid environment.

The simulator allows the user to create obstacles, choose a start and goal position, run a path-planning algorithm, visualize the search process, and simulate a robot following the resulting path.

## Features

- Interactive 10×10 grid-based environment
- A* path planning
- Dijkstra's shortest-path algorithm
- Manhattan-distance heuristic for A*
- Interactive obstacle placement and removal
- Custom start and goal positions
- Visualization of explored cells and the resulting path
- Step-by-step robot movement simulation
- Robot position and heading telemetry
- Simulated front, back, left, and right distance sensors
- Sensor-based robot environment view
- Search statistics including:
  - Path length
  - Cells explored
  - Search time
- Dynamic obstacle mode
- Environment reset functionality

## How It Works

### 1. Grid Environment

The environment is represented as a 10×10 grid.

Each cell can represent:

- Free space
- An obstacle
- The robot's starting position
- The goal
- An explored cell
- A cell belonging to the calculated path

The user can modify the environment directly through the Streamlit interface.

### 2. A* Path Planning

A* searches for a path between the start and goal while using the Manhattan distance as its heuristic.

The estimated cost is:

f(n) = g(n) + h(n)

where:

- `g(n)` is the cost from the start to the current cell
- `h(n)` is the Manhattan-distance estimate from the current cell to the goal
- `f(n)` is the estimated total cost

The implementation records the explored cells and reconstructs the final path once the goal is reached.

### 3. Dijkstra's Algorithm

Dijkstra's algorithm is implemented as a second path-planning option.

Unlike A*, it does not use a heuristic. It expands cells based on their accumulated distance from the start and reconstructs the shortest path when the goal is reached.

This allows the two search strategies to be compared within the same environment.

## Robot Simulation

After a path is calculated, RoboV can simulate the robot moving through the path one cell at a time.

The simulation displays:

- Current position
- Robot heading
- Path progress
- Remaining moves
- Current sensor readings

The robot's direction is determined from its previous and current grid positions.

## Sensor Simulation

RoboV includes four simulated distance sensors:

- Front
- Back
- Left
- Right

Each sensor scans along its direction until it reaches an obstacle or the boundary of the grid.

The sensor data is then displayed as robot telemetry and used to create a sensor-view representation of the environment.

## Search Visualization

When path planning is executed, the simulator progressively displays the cells explored by the algorithm.

Once the search is complete, the final path is displayed separately from the explored region.

This makes it possible to visually understand how different path-planning algorithms explore the environment.

## Technologies

- Python
- Streamlit
- NumPy

## Project Structure

```text
RoboV/
└── RoboV_1.py

## Running the Project

Install the required Python packages:

pip install streamlit numpy

Run the application:

streamlit run RoboV_1.py

The Streamlit interface will open in your browser.

## What I Learned

Through this project, I explored:

- Grid-based robot navigation
- Graph search algorithms
- A* and Dijkstra's algorithms
- Heuristic-based path planning
- Path reconstruction
- Search-space visualization
- Basic robot state simulation
- Simulated distance sensing
- Building interactive robotics tools with      Streamlit
- Managing simulation state using Python
limitations

RoboV is a grid-based simulation rather than a physical robot or full robotics autonomy stack.

The robot moves between discrete grid cells, and the distance sensors are simulated from the grid rather than obtained from physical sensors.

The project is intended as a foundation for experimenting with path planning and robot navigation concepts.

## Future Improvements

Planned extensions include:

- Continuous robot motion instead of discrete grid movement
- More realistic sensor models
- Real-time obstacle detection
- Dynamic path replanning
- Integration with ROS2
- Gazebo-based simulation
- Physical robot integration
- Comparison of additional path-planning algorithms

## Author

Surya Konduru

Robotics and AI Engineering Student

GitHub: surya-konduru
