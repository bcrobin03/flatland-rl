import numpy as np

from flatland.core.grid.rail_env_grid import RailEnvTransitions
from flatland.core.transition_map import GridTransitionMap
from flatland.envs.grid4_generators_utils import connect_rail_in_grid_map, connect_straight_line_in_grid_map, \
    fix_inner_nodes


rail_trans = RailEnvTransitions()
grid_map = GridTransitionMap(width=20, height=20, transitions=rail_trans)
grid_map.grid.fill(0)

# Make connection with dead-ends on both sides
start_point = (2, 2)
end_point = (8, 8)
connection_001 = connect_rail_in_grid_map(grid_map, start_point, end_point, rail_trans, flip_start_node_trans=True,
                                            flip_end_node_trans=True, respect_transition_validity=True,
                                            forbidden_cells=None)
connection_001_expected = [(2, 2), (2, 3), (2, 4), (2, 5), (2, 6), (2, 7), (2, 8), (3, 8), (4, 8), (5, 8), (6, 8),
                            (7, 8), (8, 8)]

print(connection_001)

# Make connection with open ends on both sides
start_point = (1, 3)
end_point = (1, 7)
connection_002 = connect_rail_in_grid_map(grid_map, start_point, end_point, rail_trans, flip_start_node_trans=False,
                                            flip_end_node_trans=False, respect_transition_validity=True,
                                            forbidden_cells=None)
connection_002_expected = [(1, 3), (1, 4), (1, 5), (1, 6), (1, 7)]

# Make connection with open end at beginning and dead end on end
start_point = (6, 2)
end_point = (4, 9)
connection_003 = connect_rail_in_grid_map(grid_map, start_point, end_point, rail_trans, flip_start_node_trans=False,
                                            flip_end_node_trans=True, respect_transition_validity=True,
                                            forbidden_cells=None)
connection_003_expected = [(6, 2), (6, 3), (6, 4), (6, 5)]

for (k, (connection, connection_expected)) in enumerate(zip(
    [connection_003],
    [connection_003_expected],
)):
    assert len(connection) == len(connection_expected), \
        "map {}. actual length={}, expected length={}".format(+1, len(connection), len(connection_expected))
    assert connection[0] == connection_expected[0], \
        "map {}. actual start={}, expected start={}".format(k+1, connection[0], connection_expected[0])
    assert connection[len(connection)-1] == connection_expected[len(connection_expected)-1], \
        "map {}. actual end={}, expected end={}".format(k+1, connection[0], connection_expected[0])