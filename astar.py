import math
import heapq as hq
import copy


# Constant variables
ACTIONS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
NUM_COL = 50
NUM_ROW = 30


def parse_file(filename):
    """
    Function to read through an input file, store start and goal coordinates, and the initial state into variables. 
    
    Params:
    filename: string

    Returns:
    start: tuple
    goal: tuple
    workspace: nested list
    """ 
    file = open(filename, "r")
    line1 = file.readline()
    coords = line1.split()

    start = (int(coords[0]), int(coords[1]))
    goal = (int(coords[2]), int(coords[3]))
    workspace = []

    for line in file:
        line = line.strip()
        if line:
            workspace.append(line.split())

    file.close()
    return start, goal, workspace


def write_output(filename, start, goal, workspace, solution_path, solution_f, depth, generated):
    """
    Creates and writes to a txt file, following designated format. Also modifies workspace to indicate solution path. 
    
    Params:
    filename: string file
    start: tuple of coordinates
    goal: tuple of coordinates
    workspace: nested list
    solution_path: list of actions
    solution_f: list of f(n) function values
    depth: depth of goal node
    generated: number of generated nodes added to search tree
    """
    file = open(filename, "w")
    file.write(str(depth) + "\n")
    file.write(str(generated) + "\n")
    file.write(" ".join([str(a) for a in solution_path]) + "\n")
    file.write(" ".join([str(f) for f in solution_f]) + "\n")

    curr = start
    for a in solution_path:
        curr = (curr[0] + ACTIONS[a][0], curr[1] + ACTIONS[a][1])
        if curr == goal:
            break
        i, j = coord_to_pos(curr)
        workspace[i][j] = "4"
    
    for row in workspace:
        file.write(" ".join(row) + "\n")
    file.close()


def coord_to_pos(coord):
    """
    Converts workspace coordinates to i, j positions used to index the nested list workspace. 
    
    Params:
    coord: tuple of coordinates

    Returns:
    i, j: positions for indexing
    """
    return NUM_ROW - 1 - coord[1], coord[0]


def print_workspace(workspace):
    """
    Function for testing purposes. Prints out current workspace. 
    """
    for row in workspace:
        print(row)


def angle_cost(before, after):
    """
    Calculates change in angle and divides result by 4. Equivalent to (delta theta) / 180. 
    
    Params:
    before: current angle
    after: angle to change to

    Returns:
    change / 4: value used to calculate step cost of angle. 
    """
    change = abs(after - before)
    if change > 4:
        change = 8 - change
    return change / 4


def heuristic(curr, goal):
    """
    Calculates euclidean distance of two coordinates. 
    
    Params:
    curr: tuple of current coordinates
    goal: tuple of goal coordinates
    """
    return math.sqrt((goal[0] - curr[0])**2 + (goal[1] - curr[1])**2)


def astar(start, goal, workspace, k):
    """
    A* search algorithm. f(n) = g(n) + h(n) where f(n) is search function, g(n) is path cost, h(n) is heuristic function. 
    
    Params:
    start: tuple of coordinates
    goal: tuple of coordinates
    workspace: nested list
    k: weight of angle step cost

    Returns:
    path: list of actions along solution path
    f_path: f(n) values of nodes along path
    d: depth of goal node
    generated: total number of generated nodes
    """
    h_root = heuristic(start, goal)
    root = [0 + h_root, start, 0, h_root, [], [0 + h_root], 0]
    frontier = []
    hq.heappush(frontier, root)
    reached = {}
    reached[start] = 0 + h_root
    generated = 1

    while frontier:
        f, coord, g, h, path, f_path, d = hq.heappop(frontier)
        if coord == goal:
            return path, f_path, d, generated
        
        for a in range(len(ACTIONS)):
            action = ACTIONS[a]
            if coord[0] + action[0] < 0 or coord[0] + action[0] > NUM_COL - 1 or coord[1] + action[1] < 0 or coord[1] + action[1] > NUM_ROW - 1:
                continue
            new_coord = (coord[0] + action[0], coord[1] + action[1])
            i, j = coord_to_pos(new_coord)

            if workspace[i][j] == '1':
                continue
            ca = (k * angle_cost(path[-1], a)) if path else 0
            cd = 1 if a % 2 == 0 else math.sqrt(2)
            new_g = g + ca + cd
            new_h = heuristic(new_coord, goal)
            new_f = new_g + new_h

            if new_coord in reached and reached[new_coord] <= new_f:
                continue
            generated += 1
            node = [new_f, new_coord, new_g, new_h, path+[a], f_path+[new_f], d + 1]
            hq.heappush(frontier, node)
            reached[new_coord] = new_f
    return None


def main():
    """
    Main function. Controls which files to parse, generate, and write to. 
    """
    k_vals = [0, 2, 4]
    input_files = ["Input1.txt", "Input2.txt", "Input3.txt"]

    for i in range(len(input_files)):
        start, goal, init_state = parse_file(input_files[i])

        for k in k_vals:
            workspace = copy.deepcopy(init_state)
            solution = astar(start, goal, workspace, k)
            if solution is None:
                raise Exception("There is no solution for {}".format(input_files[i]))
            solution_path, solution_f, depth, generated = solution
            out_filename = "Output{file_num}-k={k}.txt".format(file_num=i+1, k=k)
            write_output(out_filename, start, goal, workspace, solution_path, solution_f, depth, generated)
            print("File {input} with k value of {k} output generated at {output}".format(input=input_files[i], k=k, output=out_filename))


if __name__ == "__main__":
    main()

