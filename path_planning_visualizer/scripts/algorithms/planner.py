#! /usr/bin/env python

import rospy
import random


def planner(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
    # Open List contains the nodes, from which selection is to take place
    open_list = []
    
    # Closed List contains the nodes, which have already been explored
    closed_list = set()
    
    # Parents dictionary maps an index to it's parent
    parents = dict()

    # Initialize current_index and open_list
    current_index = start_index
    open_list.append(current_index)

    # Keep iterating while there exists an element in open_list
    while open_list:
        # Shuffle Open List and select the current_index
        # Shuffling ensures random selection
        random.shuffle(open_list)
        current_index = open_list.pop(0)
        
        # Add the current_index to closed_list
        closed_list.add(current_index)
        
        # Use grid_viz to set color in RViz
        grid_viz.set_color(current_index, "pale yellow")

        # Break condition
        if current_index == goal_index:
            break

        # Call function to get the 8 nearest neighbors
        # Returns neighbor index along with cost to move to that neighbor
        neighbors = find_neighbors(
            current_index, width, height, costmap, resolution)

        # Iterate over the neighbor indices
        for neighbor_index, _ in neighbors:
            # If neighbor_index in Closed List, already explored, nothing to do
            if neighbor_index in closed_list:
                continue

            # Assign parent, useful when giving final path to robot
            parents[neighbor_index] = current_index
            
            # Append neighbor index to Open List (next nodes to choose from)
            open_list.append(neighbor_index)

    # Initialize list containing indices
    path = []
    
    # Start with goal index
    current_index = goal_index
    path.append(goal_index)
    
    # Iterate back starting from goal index, reaching till start
    # Using the mapping derived from parents dictionary
    while current_index != start_index:
        current_index = parents[current_index]
        path.append(current_index)

    # Return reveresed path (starting from start index, ending at goal index)
    return path[::-1]

def find_neighbors(index, width, height, costmap, orthogonal_step_cost):
    """
    Identifies neighbor nodes inspecting the 8 adjacent neighbors
    Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
    Returns a list with valid neighbour nodes as [index, step_cost] pairs
    """
    neighbors = []
    # length of diagonal = length of one side by the square root of 2 (1.41421)
    diagonal_step_cost = orthogonal_step_cost * 1.41421
    # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
    lethal_cost = 150

    # Get the neighbor above the current cell
    upper = index - width
    if upper > 0:
        if costmap[upper] < lethal_cost:
            step_cost = orthogonal_step_cost + costmap[upper]/255
            neighbors.append([upper, step_cost])

    # Get the neighbor to the left of the current cell
    left = index - 1
    if left % width > 0:
        if costmap[left] < lethal_cost:
            step_cost = orthogonal_step_cost + costmap[left]/255
            neighbors.append([left, step_cost])

    # Get the neighbor to the top left of the current cell
    upper_left = index - width - 1
    if upper_left > 0 and upper_left % width > 0:
        if costmap[upper_left] < lethal_cost:
            step_cost = diagonal_step_cost + costmap[upper_left]/255
            neighbors.append([index - width - 1, step_cost])

    # Get the neighbor to the top right of the current cell
    upper_right = index - width + 1
    if upper_right > 0 and (upper_right) % width != (width - 1):
        if costmap[upper_right] < lethal_cost:
            step_cost = diagonal_step_cost + costmap[upper_right]/255
            neighbors.append([upper_right, step_cost])

    # Get the neighbor to the right of the current cell
    right = index + 1
    if right % width != (width + 1):
        if costmap[right] < lethal_cost:
            step_cost = orthogonal_step_cost + costmap[right]/255
            neighbors.append([right, step_cost])

    # Get the neighbor to the bottom left of the current cell
    lower_left = index + width - 1
    if lower_left < height * width and lower_left % width != 0:
        if costmap[lower_left] < lethal_cost:
            step_cost = diagonal_step_cost + costmap[lower_left]/255
            neighbors.append([lower_left, step_cost])

    # Get the neighbor to the bottom of the current cell
    lower = index + width
    if lower <= height * width:
        if costmap[lower] < lethal_cost:
            step_cost = orthogonal_step_cost + costmap[lower]/255
            neighbors.append([lower, step_cost])

    # Get the neighbor to the bottom right of the current cell
    lower_right = index + width + 1
    if (lower_right) <= height * width and lower_right % width != (width - 1):
        if costmap[lower_right] < lethal_cost:
            step_cost = diagonal_step_cost + costmap[lower_right]/255
            neighbors.append([lower_right, step_cost])

    return neighbors