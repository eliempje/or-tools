#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
# Copyright 2018 Google LLC
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# [START program]
"""Vehicle Routing example"""

# [START import]
from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
# [END import]

# [START data]
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['num_vehicles'] = 1
    data['num_locations'] = 5
    data['depot'] = 0
    return data
# [END data]


# [START arc_cost_evaluator]
def create_arc_cost_evaluator():
    """Creates callback to return arc cost."""
    def arc_cost_evaluator(from_node, to_node):
        """Returns the arc cost between the two nodes."""
        del from_node  # unused
        del to_node  # unused
        return 1

    return arc_cost_evaluator
# [END arc_cost_evaluator]


# [START printer]
def print_solution(routing, manager, assignment):
    """Prints assignment on console."""
    print('Objective: {}'.format(assignment.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    plan_output += 'Distance of the route: {}m\n'.format(route_distance)
    print(plan_output)
# [END printer]

def main():
    """Entry point of the program"""

    # Instantiate the data problem.
    # [START data_model]
    data = create_data_model()
    # [END data_model]

    # Create the routing index manager
    # [START index_manager]
    manager = pywrapcp.RoutingIndexManager(
            data['num_locations'], data['num_vehicles'], data['depot'])
    # [END index_manager]

    # Create Routing Model
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)
    # [END routing_model]

    # Define arc cost
    # [START arc_cost]
    arc_cost_evaluator_index = routing.RegisterTransitCallback(
            create_arc_cost_evaluator())
    routing.SetArcCostEvaluatorOfAllVehicles(arc_cost_evaluator_index)
    # [END arc_cost]

    # Setting first solution heuristic (cheapest addition).
    # [START parameters]
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)  # pylint: disable=no-member
    # [END parameters]

    # Solve the problem.
    # [START solve]
    assignment = routing.SolveWithParameters(search_parameters)
    # [END solve]

    # [START print_solution]
    print_solution(routing, manager, assignment)
    # [END print_solution]


if __name__ == '__main__':
    main()
# [END program]
