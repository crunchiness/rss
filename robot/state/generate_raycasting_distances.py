import particle_filtering
import sys

a = particle_filtering.Particles.generate_raycasting_distances(sys.args[0], sys.args[1])
particle_filtering.Particles.save_numpy_array('RAYCASTING_DISTANCES_{}_{}'.format(sys.args[0], sys.args[1]), a)
