# pybullet-kuka-block-push

Note: need to figure out how to speed up simulations without degrading contact
forces. Currently at about 3s for running a simulation with 30k timesteps and
delta_y = 5e-6 and the default timestep of 240Hz, using pybullet.DIRECT. Fiddling
with these any of variables seems to result in spiking behavior of the contact forces again.
Currently, 42% of the runtime of _main_ is due to p.stepSimulation().
