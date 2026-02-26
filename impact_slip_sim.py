import os
from pathlib import Path
from typing import Dict, Any, Optional, Tuple
import time

import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, ContactModel
from pydrake.multibody.tree import RevoluteJoint, PrismaticJoint, RevoluteSpring, LinearSpringDamper
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector
from pydrake.visualization import AddDefaultVisualization
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.primitives import LogVectorOutput


# -----------------------------
# Joint elements
# -----------------------------
def add_joint_springs_and_dampers(plant: MultibodyPlant, joint_params: Dict[str, Dict[str, Any]]):
    for joint_name, p in joint_params.items():
        j = plant.GetJointByName(joint_name)

        if isinstance(j, RevoluteJoint):
            j.set_default_damping(float(p.get("d", 0.0)))
            plant.AddForceElement(
                RevoluteSpring(
                    joint=j,
                    nominal_angle=float(p.get("q0", 0.0)),
                    stiffness=float(p.get("k", 0.0)),
                )
            )

        elif isinstance(j, PrismaticJoint):
            A = plant.GetBodyByName(p["A"])
            B = plant.GetBodyByName(p["B"])

            axis_B = np.asarray(p.get("axis_B", [1.0, 0.0, 0.0]), dtype=float).reshape(3,)
            axis_B /= np.linalg.norm(axis_B) + 1e-12

            d0 = float(p.get("anchor_distance", 0.10))
            if d0 <= 0:
                raise ValueError(f"anchor_distance must be > 0 for joint '{joint_name}'")

            p_AP = np.array([0.0, 0.0, 0.0])  # in A frame
            p_BQ = d0 * axis_B               # in B frame

            plant.AddForceElement(
                LinearSpringDamper(
                    bodyA=A, p_AP=p_AP,
                    bodyB=B, p_BQ=p_BQ,
                    free_length=d0 + float(p.get("x0", 0.0)),
                    stiffness=float(p.get("k", 0.0)),
                    damping=float(p.get("d", 0.0)),
                )
            )

            j.set_default_damping(float(p.get("d", 0.0)))

        else:
            raise TypeError(f"Joint '{joint_name}' must be Revolute or Prismatic (got {type(j)}).")


class _BodySpeedFromState(LeafSystem):
    """
    Input:  plant state x = [q; v]
    Output: scalar speed = || v_WB || (translational, world frame) for a chosen body
    """
    def __init__(self, *, plant: MultibodyPlant, body_name: str, model_instance=None):
        super().__init__()
        self._plant = plant
        self._body = plant.GetBodyByName(body_name) if model_instance is None else plant.GetBodyByName(body_name, model_instance)

        self._nq = plant.num_positions()
        self._nv = plant.num_velocities()
        self._nx = self._nq + self._nv

        self._plant_context = plant.CreateDefaultContext()

        self.DeclareVectorInputPort("x", BasicVector(self._nx))
        self.DeclareVectorOutputPort("speed", BasicVector(1), self._CalcSpeed)

    def _CalcSpeed(self, context, output):
        x = self.get_input_port(0).Eval(context)
        q = x[:self._nq]
        v = x[self._nq:]

        self._plant.SetPositions(self._plant_context, q)
        self._plant.SetVelocities(self._plant_context, v)

        V_WB = self._plant.EvalBodySpatialVelocityInWorld(self._plant_context, self._body)
        speed = float(np.linalg.norm(V_WB.translational()))
        output.SetAtIndex(0, speed)


def add_body_speed_logger(*, builder: DiagramBuilder, plant: MultibodyPlant, body_name: str, model_instance=None):
    speed_sys = builder.AddSystem(_BodySpeedFromState(plant=plant, body_name=body_name, model_instance=model_instance))
    speed_sys.set_name(f"{body_name}_speed")

    builder.Connect(plant.get_state_output_port(), speed_sys.get_input_port(0))

    speed_logger = LogVectorOutput(speed_sys.get_output_port(0), builder)
    speed_logger.set_name(f"{body_name}_speed_logger")
    return speed_logger


def plot_body_speed_logger(*, simulator: Simulator, speed_logger, title: str, ylims=None):
    log = speed_logger.FindLog(simulator.get_context())
    t = log.sample_times()
    speed = log.data().reshape(-1)

    plt.figure()
    plt.plot(t, speed)
    plt.xlabel("time (s)")
    plt.ylabel("speed (m/s)")
    plt.title(title)
    if ylims is not None:
        plt.ylim(ylims)
    plt.grid(True)
    plt.show()


# -----------------------------
# Helpers: final pose + speed
# -----------------------------
def get_body_position_and_speed_at_end(
    *,
    plant: MultibodyPlant,
    plant_context,
    body_name: str,
) -> Tuple[np.ndarray, float]:
    """
    Returns:
      position_WB (3,) in meters
      speed (scalar) in m/s (translational speed of body origin, world frame)
    """
    body = plant.GetBodyByName(body_name)
    X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
    p_WB = X_WB.translation().copy()

    V_WB = plant.EvalBodySpatialVelocityInWorld(plant_context, body)
    speed = float(np.linalg.norm(V_WB.translational()))

    return p_WB, speed


# -----------------------------
# Build scene / diagram
# -----------------------------
def create_scene(
    *,
    urdf_path: str,
    terr_path: str,
    slope: float,
    sim_time_step: float,
    joint_params: Dict[str, Dict[str, Any]],
    contact_model: ContactModel,
    visualize_meshcat: bool,
):
    """
    Returns:
      diagram, meshcat_or_none, lower_leg_speed_logger_or_none
    """
    urdf_path = Path(urdf_path)
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path.resolve()}")

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)

    parser = Parser(plant)
    parser.AddModels(str(urdf_path))
    terrain = parser.AddModels(str(terr_path))[0]

    slope_rad = np.deg2rad(slope)
    length = 1.5
    y_offset = 0.45 - (length - length * np.cos(slope_rad)) / 2
    z_offset = 0.15 - (length * np.sin(slope_rad)) / 2
    T = RigidTransform(RollPitchYaw(0, slope_rad, np.pi / 2), [0, y_offset, z_offset])

    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("terrain", terrain),
        T
    )

    add_joint_springs_and_dampers(plant, joint_params)

    plant.set_contact_model(contact_model)
    plant.Finalize()

    meshcat = None
    if visualize_meshcat:
        meshcat = StartMeshcat()
        meshcat.Delete()
        meshcat.DeleteAddedControls()
        AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Keep your logger (optional plotting later)
    lower_leg_speed_logger = add_body_speed_logger(builder=builder, plant=plant, body_name="lower_leg")

    diagram = builder.Build()
    return diagram, meshcat, lower_leg_speed_logger


# -----------------------------
# Run simulation
# -----------------------------
def simulate_impact_sliptest(
    *,
    urdf_path: str,
    terr_path: str,
    sim_time_step: float,
    duration: float,
    realtime_rate: float,
    rail_q0: float,
    rail_v0: float,
    thigh_q0: float,
    thigh_v0: float,
    slope: float,
    joint_params: Dict[str, Dict[str, Any]],
    contact_model: ContactModel,
    test_mode: Optional[bool],
    visualize_meshcat: bool,
    plot_lower_leg_speed: bool,
):
    """
    Returns:
      (weight_position_W, weight_speed, simulator, diagram)
    """
    if test_mode is None:
        test_mode = True if "TEST_SRCDIR" in os.environ else False
    if test_mode:
        duration = min(duration, 0.1)

    diagram, meshcat, lower_leg_speed_logger = create_scene(
        urdf_path=urdf_path,
        terr_path=terr_path,
        slope=slope,
        sim_time_step=sim_time_step,
        joint_params=joint_params,
        contact_model=contact_model,
        visualize_meshcat=visualize_meshcat,
    )

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_sub = diagram.GetSubsystemByName("plant")
    plant_context = plant_sub.GetMyMutableContextFromRoot(context)

    rail = plant_sub.GetJointByName("rail")
    rail.set_translation(plant_context, rail_q0)
    rail.set_translation_rate(plant_context, rail_v0)

    thigh = plant_sub.GetJointByName("thigh")
    thigh.set_translation(plant_context, thigh_q0)
    thigh.set_translation_rate(plant_context, thigh_v0)

    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate)

    if visualize_meshcat and meshcat is not None:
        meshcat.StartRecording()

    time.sleep(0.5)

    simulator.AdvanceTo(duration)

    # ---- end-of-sim readout: weight pose + speed ----
    weight_pos_W, weight_speed = get_body_position_and_speed_at_end(
        plant=plant_sub,
        plant_context=plant_context,
        body_name="weight",
    )

    if plot_lower_leg_speed:
        plot_body_speed_logger(
            simulator=simulator,
            speed_logger=lower_leg_speed_logger,
            title="lower_leg speed",
            ylims=None,
        )

    if visualize_meshcat and meshcat is not None:
        meshcat.StopRecording()
        meshcat.PublishRecording()

    return weight_pos_W, weight_speed, simulator, diagram


# -----------------------------
# Set Parameters
# -----------------------------
joint_params = {
    "rail":  dict(A="base", B="prismatic_coupler",
                  k=0.0, x0=0.00, d=30.50),

    "thigh": dict(A="prismatic_coupler", B="upper_leg",
                  k=0.0, x0=0.00, d=0.00),

    "knee":  dict(A="upper_leg", B="lower_leg",
                  k=129600, x0=0.40, d=2677.0),

    "ankle": dict(k=132, q0=0.00, d=2.344),
}
test_params = {
    "duration": 2.0,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 1.0,
    "slope": 40,
}

def sim_cost(
        test_params: dict,
        joint_params: dict,
        vis = False
    ):
    weight_pos_W, weight_speed, simulator, diagram = simulate_impact_sliptest(
        urdf_path="impact_slip_rig.urdf",
        terr_path="terr_geom.urdf",
        sim_time_step=1e-4,
        duration=test_params["duration"],
        realtime_rate=0.0,          # 0.0 = run as fast as possible (no real-time pacing)
        rail_q0=test_params["y_pos0"],
        rail_v0=test_params["y_vel0"],
        thigh_q0=test_params["z_pos0"],
        thigh_v0=test_params["z_vel0"],
        slope=test_params["slope"],
        joint_params=joint_params,
        contact_model=ContactModel.kHydroelasticWithFallback,
        test_mode=None,
        visualize_meshcat=vis,    # <- toggle this
        plot_lower_leg_speed=False, # <- toggle this if you still want the plot
    )
    return [weight_pos_W, weight_speed]
# -----------------------------
# Example call
# -----------------------------
if __name__ == "__main__":
    weight_pos_W, weight_speed, simulator, diagram = simulate_impact_sliptest(
        urdf_path="impact_slip_rig.urdf",
        terr_path="terr_geom.urdf",
        sim_time_step=1e-4,
        duration=test_params["duration"],
        realtime_rate=0.0,          # 0.0 = run as fast as possible (no real-time pacing)
        rail_q0=test_params["y_pos0"],
        rail_v0=test_params["y_vel0"],
        thigh_q0=test_params["z_pos0"],
        thigh_v0=test_params["z_vel0"],
        slope=test_params["slope"],
        joint_params=joint_params,
        contact_model=ContactModel.kHydroelasticWithFallback,
        test_mode=None,
        visualize_meshcat=True,    # <- toggle this
        plot_lower_leg_speed=False, # <- toggle this if you still want the plot
    )

    print(f"[END] weight position (world) [m]: {weight_pos_W[1]}")
    print(f"[END] weight speed [m/s]: {weight_speed:.6f}")