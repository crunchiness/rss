"""Run this to generate map images (saved in rss/images)
"""
import cv2
from robot.state.map import NODES

def draw_nodes(drawing, images_path, save):
    for key in NODES.keys():
        node = NODES[key]
        x = node['x']
        y = node['y']
        node_id = node['id']
        ambiguous = node['ambiguous']
        color = (100, 100, 100) if ambiguous else (0, 255, 0)
        drawing.add_big_point(x, y, color)
        drawing.image = cv2.flip(drawing.image, 1)
        cv2.putText(drawing.image, node_id, (320-x, y), cv2.FONT_HERSHEY_PLAIN, color=(255, 255, 255), fontScale=1)
        drawing.image = cv2.flip(drawing.image, 1)
    if save:
        print 'mm'
        drawing.save('map_nodes.png', images_path)


def draw_graph(drawing, images_path, save):
    used = []
    for key in NODES.keys():
        node = NODES[key]
        for sibling_key in node['connects']:
            sibling = NODES[sibling_key]
            if sibling['id'] + node['id'] not in used:
                cv2.line(drawing.image, (node['x'], node['y']), (sibling['x'], sibling['y']), (100, 0, 200), 1)
                used.append(node['id'] + sibling['id'])
            else:
                cv2.line(drawing.image, (node['x'], node['y']), (sibling['x'], sibling['y']), (100, 0, 200), 2)
    if save:
        drawing.save('map_graph.png', images_path)

def draw_path(path, drawing, images_path, save):
    for i in range(1, len(path)):
        start = NODES[path[i-1]]
        end = NODES[path[i]]
        cv2.line(drawing.image, (start['x'], start['y']), (end['x'], end['y']), (200, 100, 200), 2)
    if save:
        drawing.save('map_path.png', images_path)
