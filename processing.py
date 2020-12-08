
import pandas as pd
import numpy as np

# import data
dist = pd.read_csv('data/distances.csv')
net = pd.read_csv('data/connections.csv')


# substitute , by . and to numeric
def clean_table(data, net=False):

    data.replace({',': '.'}, regex=True, inplace=True)
    data.replace({'-': np.nan}, regex=True, inplace=True)
    data.drop('Unnamed: 0', axis=1, inplace=True)
    data = data.apply(pd.to_numeric)

    cols = data.columns

    if net == True:
        data = pd.DataFrame(np.matrix.transpose(np.matrix(data)))
        data.set_index(cols, inplace=True)
        data.columns = cols 
    else:
        data.set_index(cols, inplace=True)
        data = data.stack().reset_index()
        data.columns = ['from', 'to', 'distance']

    return data


dist = clean_table(dist)
net = clean_table(net, net=True)


# NODE CLASS
class Node():

    def __init__(self, station_node, mother_node):
        self.state = station_node
        self.cost = 0
        self.path_to_node = []
        self.mother_node = mother_node
        self.path_to_node.append(self.mother_node)


class Search():

    def __init__(self, initial_station, target_station):

        self.initial_station = initial_station
        self.target_station = target_station
        self.next_nodes = []


## debugg and develop

initial_station = 'E2'
target_station = 'E13'

distance_grid = dist
net_grid = net

def heuristic_cost_function(node, distance_grid):
    #### distance cost 
    disti = distance_grid[distance_grid['from'] == node.mother_node]
    if node.state not in list(disti['to']):
        disti = distance_grid[distance_grid['to'] == node.mother_node]
        disti = disti[disti['from'] == node.state]
    else:
        disti = disti[disti['to'] == node.state]
    
    print(disti.distance)
    cost_distance = float(disti.distance)

    #### path cost
    path_cost = len(node.path_to_node)

    #### final euristic
    final_cost = cost_distance + path_cost

    return final_cost


def heuristic_search(self, distance_grid, net_grid):
    next_nodes = {}
    visited_nodes = {}

    # search from initial node to next nodes
    next_pos = net_grid[[initial_station]] 
    next_pos = next_pos.dropna()

        
    for pos in list(next_pos.index):
        
        node = Node(pos, initial_station)
        node.cost = heuristic_cost_function(node, distance_grid)
        if node.state not in visited_nodes.keys(): # verify if is not on visited nodes
            next_nodes[node] = node.cost


    while found == False:
        # get min node
        olhaonode = min(d, key=d.get)
        
        # evaluate if it is target, if yes, return

        # add to visited
        visited_nodes[pos] = node

        # expand next nodes from visited


        next_pos = net_grid[[olhaonode_POS]] 
        next_pos = next_pos.dropna()

            
        for pos in list(next_pos.index):
            
            node = Node(pos, initial_station)
            node.cost = heuristic_cost_function(node, distance_grid)
            if node.state not in visited_nodes.keys(): # verify if is not on visited nodes
                next_nodes[node] = node.cost












    
    