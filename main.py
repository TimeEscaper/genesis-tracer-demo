import torch
import numpy as np
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

tracer = scene.add_entity(
    gs.morphs.URDF(
        file  = './urdf/tracer/tracer.urdf',
        # convexify=False,
        # decimate=False,
        pos   = (0.0, 0.0, 0.15),
        euler = (0, 0, 0),
    ),
)

scene.build()

jnt_names = [
    'left_wheel',
    'right_wheel',
]
dofs_idx = [tracer.get_joint(name).dof_idx_local for name in jnt_names]

for _ in range(100000):
    tracer.control_dofs_velocity(
        # Minus sign is a hack to achieve forward motion
        # Probably wheel joint should be rotated to fix this
        np.array([10.0, 10.0]),
        dofs_idx,
    )
    scene.step()
