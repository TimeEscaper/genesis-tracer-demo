import torch
import genesis as gs

if torch.cuda.is_available():
    print("GPU available")
    gs.init(backend=gs.gpu)
else:
    print("GPU not available")
    gs.init(backend=gs.cpu)

scene = gs.Scene(
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    show_viewer = True,
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)

franka = scene.add_entity(
    gs.morphs.URDF(
        file  = './urdf/tracer/tracer.urdf',
        pos   = (0.0, 0.0, 0.15),
        euler = (0, 0, 0),
    ),
)

scene.build()

for _ in range(1000):
    scene.step()
