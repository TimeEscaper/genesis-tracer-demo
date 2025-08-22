import torch
import numpy as np
import genesis as gs


class DiffDrive:

    def __init__(self, r: float, l: float):
        self._r = r
        self._l = l

    def __call__(self, v: float, w: float):
        c1 = 1. / (2. * self._r)
        c2 = 2. * v
        c3 = w * self._l
        w_r = c1 * (c2 + c3)
        w_l = c1 * (c2 - c3)
        return np.array([w_r, w_l])


if torch.cuda.is_available():
    print("GPU available")
    gs.init(backend=gs.gpu)
else:
    print("GPU not available")
    gs.init(backend=gs.cpu)

controller = DiffDrive(r=0.0605, l=0.341)

scene = gs.Scene(
    sim_options = gs.options.SimOptions(
        dt = 0.01,
        substeps=10,
        requires_grad=False
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
    'right_wheel',
    'left_wheel',
]
dofs_idx = [tracer.get_joint(name).dof_idx_local for name in jnt_names]

action = controller(v=1.5, w=np.deg2rad(0.))

for i in range(100):
    tracer.control_dofs_velocity(
        np.array([action[0], action[1]]),
        dofs_idx,
    )
    scene.step()

print(action)
print(f"Pose: {tracer.get_pos()}")
