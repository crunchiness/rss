import sys
import os.path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
import robot.state.particle_filtering as pf


print sys.argv
min, max = int(sys.argv[1]), int(sys.argv[2])
a = pf.Particles.generate_raycasting_distances(min, max)

pf.Particles.save_numpy_array('RAYCASTING_DISTANCES_{}_{}'.format(min, max), a)
