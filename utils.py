def find_shortest_path(nodes, start, end, path=None):
    if path is None:
        path = [start]
    else:
        path = path + [start]
    if start == end:
        return path
    if start not in nodes:
        return None
    shortest_path = None
    for node in nodes[start]['connects']:
        if node not in path:
            new_path = find_shortest_path(node, end, path)
            if new_path:
                if not shortest_path or len(new_path) < len(shortest_path):
                    shortest_path = new_path
    return shortest_path
