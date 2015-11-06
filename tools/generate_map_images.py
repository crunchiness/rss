from robot.visualisation import map_drawing
from robot.visualisation.drawing import Drawing


images_path = 'images/'

drawing = Drawing()

# draw nodes
map_drawing.draw_nodes(drawing, images_path, True)

# draw nodes + graph
map_drawing.draw_nodes(drawing, images_path, False)
map_drawing.draw_graph(drawing, images_path, True)
