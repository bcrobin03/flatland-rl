import networkx as nx
import numpy as np
from numpy import random as rand
from typing import List, Tuple, Set, Union
import graphviz as gv
from collections import defaultdict



class MotionCheck(object):
    """ Class to find chains of agents which are "colliding" with a stopped agent.
        This is to allow close-packed chains of agents, ie a train of agents travelling
        at the same speed with no gaps between them,
    """

    def __init__(self):
        self.G = nx.Graph()
        self.Grev = nx.DiGraph()  # reversed graph for finding predecessors
        self.nDeadlocks = 0
        self.svDeadlocked = set()
        self._G_reversed: Union[nx.DiGraph, None] = None
        self.target = defaultdict(list)
        self.pos = defaultdict(list)
        
    
    def construct_graph(self):

        # fill suitors for each vertex first
        for v1 in self.G.nodes:
            #create the list of cells that could have a swap conflict or a target conflict

            node_v1 = self.G.nodes[v1]
            p1 = node_v1['pos']
            t1 = node_v1['target']

            swap_and_conflict = self.target[p1] + self.target[t1]

            for v2 in swap_and_conflict: # don't iterate through everyone, only suitors of the target of v1 (?) + swapping
                
                node_v2 = self.G.nodes[v2]
                p2 = node_v2['pos']
                t2 = node_v2['target']
                
                #2 agents want the same cell
                if t2 == t1 and v1 != v2:
                    self.G.add_edge(v1, v2)
                #swap conflict
                elif t1 == p2 and p1 == t2 and v1 != v2:
                    self.svDeadlocked.add(v1)
                    self.svDeadlocked.add(v2)

                    self.G.add_edge(v1, v2)

    

    def update_graph(self, v_stopped, p_stopped):
        # we just stopped one agent
        # which edges do we need to add?
        for v1 in self.target[p_stopped]:  # don't iterate through everyone, only suitors of the pos of v_stopped
            if v_stopped != v1 :#and self.G.nodes[v1]['target'] == self.G.nodes[v_stopped]['pos']:
                self.G.add_edge(v1,v_stopped)
                
    def addAgent(self, iAg, pos, target):
        if pos is None:
            pos = (-1, iAg)
        if target is None:
            target = (-1, iAg)
        
        self.G.add_node(iAg, pos = pos, target = target)


        self.target[target].append(iAg)
        self.pos[pos].append(iAg)

    



    def fix_conflicts(self):
        while self.G.edges:
            #get a random edge 
            #v1, v2 = list(self.G.edges)[0] #PRENDRE LE PREMIER
            v1, v2 = next(iter(self.G.edges))
            #stop one of the agent. Choose one that is not already stopped
            node_v1 = self.G.nodes[v1]
            node_v2 = self.G.nodes[v2]

            t1 = node_v1['target']
            p1 = node_v1['pos']

            t2 = node_v2['target']
            p2 = node_v2['pos']

            if p1==t1:
                self.target[t2].remove(v2) 
                self.G.nodes[v2]['target'] = p2
                self.target[p2].append(v2)
                self.G.remove_edge(v1, v2)
                self.update_graph(v2, p2)
                #self.construct_graph()
            else:
                #update the list of suitors of target cell t1
                self.target[t1].remove(v1)
                self.G.nodes[v1]['target'] = p1
                self.target[p1].append(v1)
                self.G.remove_edge(v1, v2)
                self.update_graph(v1, p1)
                #self.construct_graph()

    
    def check_motion(self, iAg):
        a = self.G.nodes[iAg]
        if a['pos'] == a['target']:
            return False
        else:
            return True

  
   


    def find_same_dest(self):
        """ find groups of agents which are trying to land on the same cell.
            ie there is a gap of one cell between them and they are both landing on it.
        """
        pass

    def block_preds(self, svStops, color="red"):
        """ Take a list of stopped agents, and apply a stop color to any chains/trees
            of agents trying to head toward those cells.
            Count the number of agents blocked, ignoring those which are already marked.
            (Otherwise it can double count swaps)

        """
        iCount = 0
        svBlocked = set()
        if len(svStops) == 0:
            return svBlocked

        # The reversed graph allows us to follow directed edges to find affected agents.
        Grev = self.get_G_reversed()
        for v in svStops:

            # Use depth-first-search to find a tree of agents heading toward the blocked cell.
            lvPred = list(nx.traversal.dfs_postorder_nodes(Grev, source=v))
            svBlocked |= set(lvPred)
            svBlocked.add(v)
            # print("node:", v, "set", svBlocked)
            # only count those not already marked
            for v2 in [v] + lvPred:
                if self.G.nodes[v2].get("color") != color:
                    self.G.nodes[v2]["color"] = color
                    iCount += 1

        return svBlocked
    #
#
    #def find_conflicts(self):
    #    self.reset_G_reversed()
#
    #    svStops = self.find_stops2()  # voluntarily stopped agents - have self-loops
    #    # svSwaps = self.find_swaps()   # deadlocks - adjacent head-on collisions
    #    svSwaps = self.find_swaps2()  # faster version of find_swaps
#
    #    # Block all swaps and their tree of predessors
    #    self.svDeadlocked = self.block_preds(svSwaps, color="purple")
#
    #    # Take the union of the above, and find all the predecessors
    #    # svBlocked = self.find_stop_preds(svStops.union(svSwaps))
#
    #    # Just look for the tree of preds for each voluntarily stopped agent
    #    svBlocked = self.find_stop_preds(svStops)
#
    #    # iterate the nodes v with their predecessors dPred (dict of nodes->{})
    #    for (v, dPred) in self.G.pred.items():
    #        # mark any swaps with purple - these are directly deadlocked
    #        # if v in svSwaps:
    #        #    self.G.nodes[v]["color"] = "purple"
    #        # If they are not directly deadlocked, but are in the union of stopped + deadlocked
    #        # elif v in svBlocked:
#
    #        # if in blocked, it will not also be in a swap pred tree, so no need to worry about overwriting
    #        if v in svBlocked:
    #            self.G.nodes[v]["color"] = "red"
    #        # not blocked but has two or more predecessors, ie >=2 agents waiting to enter this node
    #        elif len(dPred) > 1:
#
    #            # if this agent is already red/blocked, ignore. CHECK: why?
    #            # certainly we want to ignore purple so we don't overwrite with red.
    #            if self.G.nodes[v].get("color") in ("red", "purple"):
    #                continue
#
    #            # if this node has no agent, and >=2 want to enter it.
    #            if self.G.nodes[v].get("agent") is None:
    #                self.G.nodes[v]["color"] = "blue"
    #            # this node has an agent and >=2 want to enter
    #            else:
    #                self.G.nodes[v]["color"] = "magenta"
#
    #            # predecessors of a contended cell: {agent index -> node}
    #            diAgCell = {self.G.nodes[vPred].get("agent"): vPred for vPred in dPred}
#
    #            # remove the agent with the lowest index, who wins
    #            iAgWinner = min(diAgCell)
    #            diAgCell.pop(iAgWinner)
#
    #            # Block all the remaining predessors, and their tree of preds
    #            # for iAg, v in diAgCell.items():
    #            #    self.G.nodes[v]["color"] = "red"
    #            #    for vPred in nx.traversal.dfs_postorder_nodes(self.G.reverse(), source=v):
    #            #        self.G.nodes[vPred]["color"] = "red"
    #            self.block_preds(diAgCell.values(), "red")
#
#
def render(omc: MotionCheck, horizontal=True):
    try:
        oAG = nx.drawing.nx_agraph.to_agraph(omc.G)
        oAG.layout("dot")
        sDot = oAG.to_string()
        if horizontal:
            sDot = sDot.replace('{', '{ rankdir="LR" ')
        # return oAG.draw(format="png")
        # This returns a graphviz object which implements __repr_svg
        return gv.Source(sDot)
    except ImportError as oError:
        print("Flatland agent_chains ignoring ImportError - install pygraphviz to render graphs")
        return None


class ChainTestEnv(object):
    """ Just for testing agent chains
    """

    def __init__(self, omc: MotionCheck):
        self.iAgNext = 0
        self.iRowNext = 1
        self.omc = omc

    def addAgent(self, rc1, rc2, xlabel=None):
        self.omc.addAgent(self.iAgNext, rc1, rc2, xlabel=xlabel)
        self.iAgNext += 1

    def addAgentToRow(self, c1, c2, xlabel=None):
        self.addAgent((self.iRowNext, c1), (self.iRowNext, c2), xlabel=xlabel)

    def create_test_chain(self,
                          nAgents: int,
                          rcVel: Tuple[int] = (0, 1),
                          liStopped: List[int] = [],
                          xlabel=None):
        """ create a chain of agents
        """
        lrcAgPos = [(self.iRowNext, i * rcVel[1]) for i in range(nAgents)]

        for iAg, rcPos in zip(range(nAgents), lrcAgPos):
            if iAg in liStopped:
                rcVel1 = (0, 0)
            else:
                rcVel1 = rcVel
            self.omc.addAgent(iAg + self.iAgNext, rcPos, (rcPos[0] + rcVel1[0], rcPos[1] + rcVel1[1]))

        if xlabel:
            self.omc.G.nodes[lrcAgPos[0]]["xlabel"] = xlabel

        self.iAgNext += nAgents
        self.iRowNext += 1

    def nextRow(self):
        self.iRowNext += 1


def create_test_agents(omc: MotionCheck):
    # blocked chain
    omc.addAgent(1, (1, 2), (1, 3))
    omc.addAgent(2, (1, 3), (1, 4))
    omc.addAgent(3, (1, 4), (1, 5))
    omc.addAgent(31, (1, 5), (1, 5))

    # unblocked chain
    omc.addAgent(4, (2, 1), (2, 2))
    omc.addAgent(5, (2, 2), (2, 3))

    # blocked short chain
    omc.addAgent(6, (3, 1), (3, 2))
    omc.addAgent(7, (3, 2), (3, 2))

    # solitary agent
    omc.addAgent(8, (4, 1), (4, 2))

    # solitary stopped agent
    omc.addAgent(9, (5, 1), (5, 1))

    # blocked short chain (opposite direction)
    omc.addAgent(10, (6, 4), (6, 3))
    omc.addAgent(11, (6, 3), (6, 3))

    # swap conflict
    omc.addAgent(12, (7, 1), (7, 2))
    omc.addAgent(13, (7, 2), (7, 1))


def create_test_agents2(omc: MotionCheck):
    # blocked chain
    cte = ChainTestEnv(omc)
    cte.create_test_chain(4, liStopped=[3], xlabel="stopped\nchain")
    cte.create_test_chain(4, xlabel="running\nchain")

    cte.create_test_chain(2, liStopped=[1], xlabel="stopped \nshort\n chain")

    cte.addAgentToRow(1, 2, "swap")
    cte.addAgentToRow(2, 1)

    cte.nextRow()

    cte.addAgentToRow(1, 2, "chain\nswap")
    cte.addAgentToRow(2, 3)
    cte.addAgentToRow(3, 2)

    cte.nextRow()

    cte.addAgentToRow(1, 2, "midchain\nstop")
    cte.addAgentToRow(2, 3)
    cte.addAgentToRow(3, 4)
    cte.addAgentToRow(4, 4)
    cte.addAgentToRow(5, 6)
    cte.addAgentToRow(6, 7)

    cte.nextRow()

    cte.addAgentToRow(1, 2, "midchain\nswap")
    cte.addAgentToRow(2, 3)
    cte.addAgentToRow(3, 4)
    cte.addAgentToRow(4, 3)
    cte.addAgentToRow(5, 4)
    cte.addAgentToRow(6, 5)

    cte.nextRow()

    cte.addAgentToRow(1, 2, "Land on\nSame")
    cte.addAgentToRow(3, 2)

    cte.nextRow()
    cte.addAgentToRow(1, 2, "chains\nonto\nsame")
    cte.addAgentToRow(2, 3)
    cte.addAgentToRow(3, 4)
    cte.addAgentToRow(5, 4)
    cte.addAgentToRow(6, 5)
    cte.addAgentToRow(7, 6)

    cte.nextRow()
    cte.addAgentToRow(1, 2, "3-way\nsame")
    cte.addAgentToRow(3, 2)
    cte.addAgent((cte.iRowNext + 1, 2), (cte.iRowNext, 2))
    cte.nextRow()

    if False:
        cte.nextRow()
        cte.nextRow()
        cte.addAgentToRow(1, 2, "4-way\nsame")
        cte.addAgentToRow(3, 2)
        cte.addAgent((cte.iRowNext + 1, 2), (cte.iRowNext, 2))
        cte.addAgent((cte.iRowNext - 1, 2), (cte.iRowNext, 2))
        cte.nextRow()

    cte.nextRow()
    cte.addAgentToRow(1, 2, "Tee")
    cte.addAgentToRow(2, 3)
    cte.addAgentToRow(3, 4)
    cte.addAgent((cte.iRowNext + 1, 3), (cte.iRowNext, 3))
    cte.nextRow()

    cte.nextRow()
    cte.addAgentToRow(1, 2, "Tree")
    cte.addAgentToRow(2, 3)
    cte.addAgentToRow(3, 4)
    r1 = cte.iRowNext
    r2 = cte.iRowNext + 1
    r3 = cte.iRowNext + 2
    cte.addAgent((r2, 3), (r1, 3))
    cte.addAgent((r2, 2), (r2, 3))
    cte.addAgent((r3, 2), (r2, 3))

    cte.nextRow()


def test_agent_following():
    omc = MotionCheck()
    create_test_agents2(omc)

    svStops = omc.find_stops()
    svBlocked = omc.find_stop_preds()
    llvSwaps = omc.find_swaps()
    svSwaps = {v for lvSwap in llvSwaps for v in lvSwap}
    print(list(svBlocked))

    lvCells = omc.G.nodes()

    lColours = ["magenta" if v in svStops
                else "red" if v in svBlocked
    else "purple" if v in svSwaps
    else "lightblue"
                for v in lvCells]
    dPos = dict(zip(lvCells, lvCells))

    nx.draw(omc.G,
            with_labels=True, arrowsize=20,
            pos=dPos,
            node_color=lColours)


def main():
    test_agent_following()


if __name__ == "__main__":
    main()
