
import pandas as pd
import numpy as np

# import data
dist = pd.read_csv('data/distances.csv')
net = pd.read_csv('data/connections.csv')


def clean_table(data):
    ''' function for preprocessing data of distance
        and connections among stations
    '''

    data.replace({',': '.'}, regex=True, inplace=True)
    data.replace({'-': np.nan}, regex=True, inplace=True)
    data.drop('Unnamed: 0', axis=1, inplace=True)
    data = data.apply(pd.to_numeric)

    cols = data.columns
    
    data.set_index(cols, inplace=True)
    data = data.stack().reset_index()
    data.columns = ['from', 'to', 'distance']

    return data


dist = clean_table(dist)
net = clean_table(net)


# NODE CLASS
class Node():

    def __init__(self, station_node, mother_node, path_to_node):
        self.state = station_node
        self.cost = 0
        self.path_to_node = path_to_node
        self.mother_node = mother_node



class AStarSearch():

    def __init__(self, initial_station, target_station):

        self.initial_station = initial_station
        self.target_station = target_station
        self.next_nodes = {}
        self.visited_nodes = {}
        self.closed_path = []
        self.search_cost = []


    
    def heuristic_cost_function(self, node, distance_grid, net_grid):

        if node.state != self.target_station:
            #### capture distance cost from node to target
            disti = distance_grid[(distance_grid['from'] == node.state) | (distance_grid['to'] == node.state)]
            disti = disti[(disti['from'] == self.target_station) | (disti['to'] == self.target_station)]
            cost_distance = float(disti.distance)
        else:
            cost_distance = 0

        #### capture path cost 
        netdisti = net_grid[(net_grid['from'] == node.state) | (net_grid['to'] == node.state)]
        netdisti = netdisti[(netdisti['from'] == node.mother_node) | (netdisti['to'] == node.mother_node)]
        cost_directly_distance = float(netdisti.distance)

        #### final heuristic cost
        final_cost = cost_distance + cost_directly_distance

        return final_cost



    def heuristic_search(self, distance_grid, net_grid):

        # go to first node
        self.visited_nodes[self.initial_station] = Node(self.initial_station, self.initial_station, [self.initial_station])
        
        # get first stations
        next_pos = net_grid[(net_grid['from'] == self.initial_station) | (net_grid['to'] == self.initial_station)]
        next_pos = list(next_pos['from']) + list(next_pos['to'])
        next_pos = [x for x in next_pos if x != self.initial_station]
        print('First expansion of search: ', next_pos)

        # build first next stations nodes
        for pos in next_pos:
            node = Node(pos, self.initial_station, [self.initial_station])
            node.cost = self.heuristic_cost_function(node, distance_grid, net_grid)
            if node.state not in self.visited_nodes.keys(): # verify if is not on visited nodes
                self.next_nodes[node] = node.cost
        
        print('Costs for each node: ',self.next_nodes)
        ### search next stations
        found=False
        while found == False:

            # get min cost node
            min_node = min(self.next_nodes, key=self.next_nodes.get)
            print('Go to: ', min_node.state,', with cost:', min_node.cost, '\n')

            # go to node
            self.visited_nodes[min_node.state] = node

            ### evaluate if it is target, if yes, return
            if min_node.state == self.target_station:
                # evaluate if it has a closed path during search and remove it
                # to generate best path
                print('Found Path!')
                print('Total search cost: ', sum(self.search_cost))
                for station in min_node.path_to_node:
                    if min_node.path_to_node.count(station) > 1:
                        position = [i for i, x in enumerate( min_node.path_to_node) if x == station][-1]
                        best_path = min_node.path_to_node[position:]
                        print('Best path cost: ', sum(self.search_cost[position:]))
                        return min_node.state, best_path,  min_node.path_to_node
                # return path
                print('Best path cost: ', sum(self.search_cost))
                return min_node.state, min_node.path_to_node
            else:            
                self.next_nodes = {}
                # get next stations
                next_pos = net_grid[(net_grid['from'] == min_node.state) | (net_grid['to'] == min_node.state)]
                next_pos = list(next_pos['from']) + list(next_pos['to'])
                next_pos = [x for x in next_pos if x != min_node.state]

                ### treat closed path
                # remove closed path stations
                next_pos = [x for x in next_pos if x not in self.closed_path]
                print('Next expansion of search: ', next_pos)

                # if path is closed, go back
                if len(next_pos) == 1 and next_pos[0] in self.visited_nodes.keys():
                    self.visited_nodes.pop(next_pos[0], None) 
                    # append min_node.state to closed_gates
                    self.closed_path.append(next_pos[0])

                ### update path to node
                min_node.path_to_node.append(min_node.state)
                self.search_cost.append(min_node.cost)
                
                ### build next stations nodes and calculate cost function
                for pos in list(next_pos):
                    
                    node = Node(pos, min_node.state, min_node.path_to_node)
                    node.cost = self.heuristic_cost_function(node, distance_grid, net_grid)
                    if node.state not in self.visited_nodes.keys():
                        self.next_nodes[node] = node.cost
                print('Costs for each node: ', self.next_nodes)



# build search agent
search_agent = AStarSearch('E1', 'E13')

# execute search
search_agent.heuristic_search(dist, net)
