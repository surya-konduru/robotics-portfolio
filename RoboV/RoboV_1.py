import streamlit as st
import numpy as np
import time


def heuristic(a, b):
    """Manhattan distance between two grid cells."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(position, rows, cols, grid):
    """Return walkable neighboring cells."""
    r, c = position

    directions = [
        (-1, 0),  # Up
        (1, 0),   # Down
        (0, -1),  # Left
        (0, 1)    # Right
    ]

    neighbors = []

    for dr, dc in directions:
        new_r = r + dr
        new_c = c + dc

        # Stay inside the grid
        if 0 <= new_r < rows and 0 <= new_c < cols:
            # Don't walk through obstacles
            if grid[new_r, new_c] == 0:
                neighbors.append((new_r, new_c))

    return neighbors


def a_star(grid, start, goal):
    """Find the shortest path from start to goal using A*."""
    open_set = {start}
    came_from = {}

    g_score = {
        start: 0
    }

    f_score = {
        start: heuristic(start, goal)
    }

    explored = []

    while open_set:
        # Choose the cell with the lowest estimated total cost
        current = min(
            open_set,
            key=lambda position: f_score.get(
                position,
                float("inf")
            )
        )

        # Remember that we explored this cell
        explored.append(current)

        # We reached the goal
        if current == goal:
            path = []

            while current in came_from:
                path.append(current)
                current = came_from[current]

            path.append(start)
            path.reverse()

            return path, explored

        # Finished examining this cell
        open_set.remove(current)

        # Examine neighboring cells
        for neighbor in get_neighbors(
            current,
            grid.shape[0],
            grid.shape[1],
            grid
        ):
            # Moving to a neighboring cell costs 1
            tentative_g_score = g_score[current] + 1

            # Is this a better route?
            if tentative_g_score < g_score.get(
                neighbor,
                float("inf")
            ):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score

                f_score[neighbor] = (
                    tentative_g_score
                    + heuristic(neighbor, goal)
                )

                open_set.add(neighbor)

    # No path exists
    return None, explored

def dijkstra(grid, start, goal):
    """Find the shortest path using Dijkstra's algorithm."""

    open_set = {start}

    came_from = {}

    distance = {
        start: 0
    }

    explored = []

    while open_set:

        # Choose the cell with the smallest distance
        current = min(
            open_set,
            key=lambda position: distance.get(
                position,
                float("inf")
            )
        )

        explored.append(current)

        # Goal reached
        if current == goal:

            path = []

            while current in came_from:
                path.append(current)
                current = came_from[current]

            path.append(start)
            path.reverse()

            return path, explored

        open_set.remove(current)

        # Examine neighboring cells
        for neighbor in get_neighbors(
            current,
            grid.shape[0],
            grid.shape[1],
            grid
        ):

            new_distance = distance[current] + 1

            if new_distance < distance.get(
                neighbor,
                float("inf")
            ):

                distance[neighbor] = new_distance

                came_from[neighbor] = current

                open_set.add(neighbor)

    return None, explored


def get_direction(current, previous):
    """Return the robot's movement direction."""
    if previous is None:
        return "🤖"

    current_r, current_c = current
    previous_r, previous_c = previous

    if current_r < previous_r:
        return "⬆️"

    elif current_r > previous_r:
        return "⬇️"

    elif current_c < previous_c:
        return "⬅️"

    elif current_c > previous_c:
        return "➡️"

    return "🤖"


def get_sensor_data(position, grid):
    """Simulate four distance sensors around the robot."""

    r, c = position

    sensors = {
        "Front": 0,
        "Back": 0,
        "Left": 0,
        "Right": 0
    }

    # Front — upward
    distance = 0

    for i in range(r - 1, -1, -1):

        if grid[i, c] == 1:
            break

        distance += 1

    sensors["Front"] = distance

    # Back — downward
    distance = 0

    for i in range(r + 1, grid.shape[0]):

        if grid[i, c] == 1:
            break

        distance += 1

    sensors["Back"] = distance

    # Left
    distance = 0

    for j in range(c - 1, -1, -1):

        if grid[r, j] == 1:
            break

        distance += 1

    sensors["Left"] = distance

    # Right
    distance = 0

    for j in range(c + 1, grid.shape[1]):

        if grid[r, j] == 1:
            break

        distance += 1

    sensors["Right"] = distance

    return sensors

def detect_obstacle_ahead(position, grid, threshold=1):
    """Detect whether an obstacle is within the robot's forward sensing range."""

    r, c = position

    # Look directly ahead
    for distance in range(1, threshold + 1):

        next_r = r - distance

        if next_r < 0:
            break

        if grid[next_r, c] == 1:
            return True

    return False

def get_sensor_cells(position, grid):
    """Return cells currently visible to the robot's sensors."""

    r, c = position

    sensor_cells = set()

    # Front
    for i in range(r - 1, -1, -1):
        if grid[i, c] == 1:
            break
        sensor_cells.add((i, c))

    # Back
    for i in range(r + 1, grid.shape[0]):
        if grid[i, c] == 1:
            break
        sensor_cells.add((i, c))

    # Left
    for j in range(c - 1, -1, -1):
        if grid[r, j] == 1:
            break
        sensor_cells.add((r, j))

    # Right
    for j in range(c + 1, grid.shape[1]):
        if grid[r, j] == 1:
            break
        sensor_cells.add((r, j))

    return sensor_cells


def path_is_blocked(path, grid):
    """Check whether an obstacle blocks the current path."""
    for position in path:
        r, c = position

        if grid[r, c] == 1:
            return True

    return False


# -----------------------------
# Page Configuration
# -----------------------------

st.set_page_config(
    page_title="RoboV",
    page_icon=None,
    layout="centered"
)


# -----------------------------
# Header
# -----------------------------

st.title("RoboV")
st.subheader("Autonomous Robot Path Planning Simulator")

st.write(
    "Interactive simulation of autonomous navigation "
    "using graph-based path planning algorithms."
)

st.divider()


# -----------------------------
# Grid Configuration
# -----------------------------

ROWS = 10
COLS = 10


# -----------------------------
# Session State
# -----------------------------

if "grid" not in st.session_state:
    st.session_state.grid = np.zeros(
        (ROWS, COLS),
        dtype=int
    )

if "start" not in st.session_state:
    st.session_state.start = (0, 0)

if "goal" not in st.session_state:
    st.session_state.goal = (
        ROWS - 1,
        COLS - 1
    )

if "path" not in st.session_state:
    st.session_state.path = None

if "explored" not in st.session_state:
    st.session_state.explored = []

if "search_time" not in st.session_state:
    st.session_state.search_time = None

if "animating" not in st.session_state:
    st.session_state.animating = False

if "simulating" not in st.session_state:
    st.session_state.simulating = False

if "simulation_complete" not in st.session_state:
    st.session_state.simulation_complete = False



# -----------------------------
# Controls
# -----------------------------

st.write("### Environment Controls")

st.write("### Path Planning Algorithm")

algorithm = st.radio(
    "Select algorithm",
    [
        "A*",
        "Dijkstra"
    ],
    horizontal=True
)

mode = st.radio(
    "Select editing mode",
    [
        "Add Obstacle",
        "Remove Obstacle",
        "Set Start",
        "Set Goal",
        "Dynamic Obstacle"
    ],
    horizontal=True
)


# -----------------------------
# Grid
# -----------------------------

grid_placeholder = st.empty()


# -----------------------------
# Run A*
# -----------------------------

if st.button(
    "Run Path Planning",
    use_container_width=True
):
    start_time = time.perf_counter()

    if algorithm == "A*":

        path, explored = a_star(
            st.session_state.grid,
            st.session_state.start,
            st.session_state.goal
        )

    else:

        path, explored = dijkstra(
            st.session_state.grid,
            st.session_state.start,
            st.session_state.goal
        )

    st.session_state.path = None
    st.session_state.explored = []

    st.session_state.search_time = (
        time.perf_counter() - start_time
    )

    # No path
    if path is None:
        st.session_state.explored = explored

        st.error("No valid path exists!")

    else:
        # Animate the explored cells
        for i in range(len(explored)):
            st.session_state.explored = explored[:i + 1]

            with grid_placeholder.container():
                st.write("### Environment")

                for r in range(ROWS):
                    cols = st.columns(COLS)

                    for c in range(COLS):
                        position = (r, c)

                        if position == st.session_state.start:
                            label = "🤖"

                        elif position == st.session_state.goal:
                            label = "🎯"

                        elif st.session_state.grid[r, c] == 1:
                            label = "⬛"

                        elif position in st.session_state.explored:
                            label = "🟡"

                        else:
                            label = "⬜"

                        cols[c].button(
                            label,
                            key=f"anim_{r}_{c}_{i}",
                            use_container_width=True
                        )

            time.sleep(0.04)

        # Show final path
        st.session_state.explored = explored
        st.session_state.path = path

        st.session_state.simulation_complete = False
        st.session_state.step = 0

        st.success(
            f"✅ {algorithm} found a path! "
            f"{len(path) - 1} moves."
        )


# -----------------------------
# A* Statistics
# -----------------------------

if st.session_state.path is not None:
    st.write(
    f"### {algorithm} Search Statistics"
)

    metric1, metric2, metric3 = st.columns(3)

    with metric1:
        st.metric(
            "Path Length",
            f"{len(st.session_state.path) - 1} moves"
        )

    with metric2:
        st.metric(
            "Cells Explored",
            len(st.session_state.explored)
        )

    with metric3:
        st.metric(
            "Search Time",
            f"{st.session_state.search_time:.5f} s"
        )


# Robot Simulation

st.write("### Robot Simulation")

if st.session_state.path:

    path = st.session_state.path

    # Initialize simulation state
    if "step" not in st.session_state:
        st.session_state.step = 0

    current_position = path[st.session_state.step]

    # Robot orientation
    previous_position = (
        path[st.session_state.step - 1]
        if st.session_state.step > 0
        else None
    )

    direction = get_direction(
        current_position,
        previous_position
    )

    # Sensor readings
    sensor_data = get_sensor_data(
        current_position,
        st.session_state.grid
    )

    # Robot telemetry

    st.write("#### Robot Telemetry")

    telemetry_col1, telemetry_col2, telemetry_col3, telemetry_col4 = st.columns(4)

    with telemetry_col1:
        st.metric(
            "Position",
            f"({current_position[0]}, {current_position[1]})"
        )

    with telemetry_col2:
        st.metric(
            "Heading",
            direction
        )

    with telemetry_col3:
        st.metric(
            "Path Progress",
            f"{st.session_state.step}/{len(path) - 1}"
        )

    with telemetry_col4:
        remaining = len(path) - 1 - st.session_state.step

    st.metric(
        "Remaining Moves",
        remaining
    )


    st.write("#### Sensor Telemetry")

    sensor_col1, sensor_col2, sensor_col3, sensor_col4 = st.columns(4)

    with sensor_col1:
        st.metric("⬆️ Front", sensor_data["Front"])

    with sensor_col2:
        st.metric("⬇️ Back", sensor_data["Back"])

    with sensor_col3:
        st.metric("⬅️ Left", sensor_data["Left"])

    with sensor_col4:
        st.metric("➡️ Right", sensor_data["Right"])
    sensor_cells = get_sensor_cells(
        current_position,
        st.session_state.grid
    )

    

    # Status
    st.write(
        f"**Position:** {current_position}  |  "
        f"**Direction:** {direction}"
    )

    st.write(
        f"**Front:** {sensor_data['Front']}  |  "
        f"**Back:** {sensor_data['Back']}  |  "
        f"**Left:** {sensor_data['Left']}  |  "
        f"**Right:** {sensor_data['Right']}"
    )

    # Simulation controls
    col1, col2 = st.columns(2)

    with col1:
        if "simulating" not in st.session_state:
            st.session_state.simulating = False

        if "simulation_complete" not in st.session_state:
            st.session_state.simulation_complete = False

        control_col1, control_col2 = st.columns(2)

        with control_col1:
            if st.button("Start Simulation", key="start_simulation"):
                st.session_state.simulating = True
                st.session_state.simulation_complete = False
                st.rerun()

            with col2:
                if st.button("Restart Simulation", key="restart_simulation"):
                    st.session_state.step = 0
                    st.session_state.simulation_complete = False
                    st.rerun()

    # Goal reached message
    if st.session_state.simulation_complete:
        st.success("Robot reached the goal!")

    # Simulation grid
    st.write("#### Robot Sensor View")

    for r in range(ROWS):

        cols = st.columns(COLS)

        for c in range(COLS):

            position = (r, c)

            if position == current_position:
                cell = direction

            elif position == st.session_state.goal:
                cell = "🎯"

            elif st.session_state.grid[r, c] == 1:
                cell = "⬛"

            elif position in sensor_cells:
                cell = "🟢"

            elif position in path:
                cell = "🔵"

            else:
                cell = "⬜"

            cols[c].markdown(
                f"<div style='text-align:center; font-size:24px'>{cell}</div>",
                unsafe_allow_html=True
            )

# Automatic robot simulation

if st.session_state.simulating:

    if st.session_state.step < len(path) - 1:

        time.sleep(0.25)

        st.session_state.step += 1

        st.rerun()

    else:

        st.session_state.simulating = False
        st.session_state.simulation_complete = True
        st.rerun()
# -----------------------------
# Final Environment Grid
# -----------------------------

with grid_placeholder.container():
    st.write("### Environment")

    for r in range(ROWS):
        cols = st.columns(COLS)

        for c in range(COLS):
            position = (r, c)

            if position == st.session_state.start:
                label = "🤖"

            elif position == st.session_state.goal:
                label = "🎯"

            elif st.session_state.grid[r, c] == 1:
                label = "⬛"

            elif (
                st.session_state.path is not None
                and position in st.session_state.path
            ):
                label = "🔵"

            elif position in st.session_state.explored:
                label = "🟡"

            else:
                label = "⬜"

            if cols[c].button(
                label,
                key=f"cell_{r}_{c}",
                use_container_width=True
            ):
                if mode == "Add Obstacle":
                    if (
                        position != st.session_state.start
                        and position != st.session_state.goal
                    ):
                        st.session_state.grid[r, c] = 1
                        st.session_state.path = None
                        st.session_state.explored = []
                        st.session_state.search_time = None

                elif mode == "Remove Obstacle":
                    st.session_state.grid[r, c] = 0
                    st.session_state.path = None
                    st.session_state.explored = []
                    st.session_state.search_time = None

                elif mode == "Set Start":
                    if position != st.session_state.goal:
                        st.session_state.start = position
                        st.session_state.grid[r, c] = 0
                        st.session_state.path = None
                        st.session_state.explored = []
                        st.session_state.search_time = None

                elif mode == "Set Goal":
                    if position != st.session_state.start:
                        st.session_state.goal = position
                        st.session_state.grid[r, c] = 0
                        st.session_state.path = None
                        st.session_state.explored = []
                        st.session_state.search_time = None

                elif mode == "Dynamic Obstacle":
                    if (
                        position != st.session_state.start
                        and position != st.session_state.goal
                    ):
                        st.session_state.grid[r, c] = 1
                        st.session_state.path = None
                        st.session_state.explored = []
                        st.session_state.search_time = None

                st.rerun()


# -----------------------------
# Reset Environment
# -----------------------------

st.write("")

if st.button(
    "Reset Environment",
    use_container_width=True
):
    st.session_state.grid = np.zeros(
        (ROWS, COLS),
        dtype=int
    )

    st.session_state.start = (0, 0)

    st.session_state.goal = (
        ROWS - 1,
        COLS - 1
    )

    st.session_state.path = None
    st.session_state.explored = []
    st.session_state.search_time = None

    st.rerun()
