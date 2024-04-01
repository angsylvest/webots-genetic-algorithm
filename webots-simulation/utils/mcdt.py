import math
import random

class DecisionTree:
    def __init__(self, num_actions=2):  # TODO: if dispersion, would be 3
        self.num_actions = num_actions
        self.root = Node()
        self.use_preset = False

    def select(self, node, exploration_weight=1.0):
        """Select a child node based on UCB1 exploration-exploitation strategy."""
        if not node.children:
            return node

        ucb_values = {
            child_action: child.total_reward / (child.visits + 1e-6) +
            exploration_weight * math.sqrt(math.log(node.visits) / (child.visits + 1e-6))
            for child_action, child in node.children.items()
        }

        selected_action = max(ucb_values, key=ucb_values.get)
        return node.children[selected_action]
        

    # def select(self, node, exploration_weight=1.0):
    #     """Select a child node based on UCB1 exploration-exploitation strategy."""
    #     if not node.children:
    #         return node

    #     ucb_values = {}
    #     for child_action, child in node.children.items():
    #         if child.visits == 0:
    #             ucb_values[child_action] = float('inf')  # Assign a high value for unvisited nodes
    #         else:
    #             exploration_term = exploration_weight * math.sqrt(math.log(node.visits) / child.visits)
    #             ucb_values[child_action] = child.total_reward / child.visits + exploration_term

    #     selected_action = max(ucb_values, key=ucb_values.get)
    #     return node.children[selected_action]


    def expand(self, node, action):
        """Expand the tree by adding a new child node."""
        new_node = Node(parent=node, action=action)
        node.children[action] = new_node
        return new_node

    def flocking_reward(self, path_length, time_stagnation):
        time_stag_pen = 0.01
        alpha = 0.1
        gen_path_reward = math.exp(-alpha * path_length)
        return gen_path_reward + (-time_stag_pen * time_stagnation)

    def update_tree(self, selected_node, reward):
        # undergo backpropagation
        current_node = selected_node
        while current_node is not None:
            current_node.visits += 1
            current_node.total_reward += reward
            current_node = current_node.parent

class Node:
    def __init__(self, parent=None, action=None):
        self.parent = parent
        self.children = {}
        self.action = action
        self.visits = 0
        self.total_reward = 0

def iterate(tree, max_depth=10):  # TODO: if queueing, max_depth should be based on number of agents
    selected_node = tree.root
    action = random.choice(range(tree.num_actions))

    depth_exists = True
    curr_depth = 0
    accumulated_actions = []

    while depth_exists and curr_depth < max_depth:
        if action not in selected_node.children:
            selected_node = tree.expand(selected_node, action)
            accumulated_actions.append(action)

            if not tree.use_preset:
                # return f'action: {accumulated_actions}', selected_node
                return accumulated_actions, selected_node
            else:
                action = random.choice(range(tree.num_actions))
        else:
            selected_node = selected_node.children[action]
            action = random.choice(range(tree.num_actions))
            accumulated_actions.append(action)

            if action == 1:
                # return f'action: {accumulated_actions}', selected_node
                return accumulated_actions, selected_node
            else:
                curr_depth += 1

    return accumulated_actions, selected_node  # action is a trajectory (sequence of behaviors agent should do)

def iterate_strategically(tree, max_depth=10, exploration_weight=1.0):
    selected_node = tree.root

    depth_exists = True
    curr_depth = 0
    accumulated_actions = []

    while depth_exists and curr_depth < max_depth:
        selected_node = tree.select(selected_node, exploration_weight)
        action = max(selected_node.children, key=lambda a: selected_node.children[a].total_reward / (selected_node.children[a].visits + 1e-6))
        
        if action not in selected_node.children:
            selected_node = tree.expand(selected_node, action)
            accumulated_actions.append(action)

            if not tree.use_preset:
                return accumulated_actions, selected_node
        else:
            selected_node = selected_node.children[action]
            accumulated_actions.append(action)

            if action == 1:
                return accumulated_actions, selected_node
            else:
                curr_depth += 1

    return accumulated_actions, selected_node

def get_best_action(tree):
    root_node = tree.root
    best_action = max(root_node.children, key=lambda a: root_node.children[a].total_reward / (root_node.children[a].visits + 1e-6))
    return best_action

# def main():
#     tree = DecisionTree()

#     for i in range(5):
#         act, node = iterate(tree)
#         print(f'random action selected: {act} for node {node}')

#     for i in range(5): 
#         act, node = iterate_strategically(tree)
#         print(f'strategic action selected: {act} for node {node}')


# main() 