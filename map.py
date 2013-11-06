import utils

class Node(object):
    """This is a node in the map"""
    def __init__(self, name, point, adjlist):
        self.name = name
        self.point = point
        self.adjacents = adjlist
        self.home_distance = 0
        

class NodeMap:
    """This is a graph of the nodes in the map"""
    def __init__(self, lnodes):
        self.nodes = lnodes
        home = self.search_node("h")
        for node in home.adjacents:
            self.calculate_home_distance(node, ["h"])

    def search_node(self, nodename):
        for n in self.nodes:
            if n.name == nodename:
                return n
        return None

    def calculate_home_distance(self, nodename, checked):
        node = self.search_node(nodename)
        checked.append(node.name)
        try:
            if node.name == "h":
                node.home_distance = 0
            else:
            	e = node.adjacents.index("h")
                node.home_distance = utils.mm_between(node.point, utils.home_position)
        except ValueError:
            mindist = 99999999 
            for pname in node.adjacents:
                p = self.search_node(pname)
                if p.home_distance != 0: 
                    aux = p.home_distance+utils.mm_between(p.point, node.point)
                    if aux < mindist:
                        mindist = aux
            node.home_distance = mindist
        for n in node.adjacents:
            try: 
                e = checked.index(n)
            except ValueError:
                self.calculate_home_distance(n, checked)

    def new_node(self, node):
        mindist = 99999999 
        for pname in node.adjacents:
            p = self.search_node(pname)
            aux = p.home_distance+utils.mm_between(p.point, node.point)
            if aux < mindist:
                mindist = aux
        node.home_distance = mindist
        self.nodes.append(node)


def find_path_to_home(nodemap, origin):
    path = []
    now = origin
    while now != "h":
        node = nodemap.search_node(now)
        mdist = 999999
        best = "Nonode"
        for pname in node.adjacents:
            p = nodemap.search_node(pname)
            aux = p.home_distance
            if aux < mdist:
                mdist = aux
                best = p.name
        path.append(best)
        now = best
    return path


