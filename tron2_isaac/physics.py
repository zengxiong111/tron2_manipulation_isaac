"""Isaac Sim 5.1 physics backend for the TRON2 motor-level simulator."""

from __future__ import annotations

import tempfile
import time
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

from .model import HOME_Q, MOTOR_NAMES, TORQUE_LIMITS


class IsaacRobot:
    """Own an Isaac Sim app and a fixed-base TRON2 articulation.

    ``rate`` is the physics stepping frequency in Hz. State vectors and input
    efforts always use the upstream wire order: left arm, right arm, head.
    The source URDF's mimic grasper linkage is fixed. The upstream motor
    interface exposes sixteen arm and head axes.
    """

    def __init__(self, urdf: Path, *, headless: bool, rate: float) -> None:
        urdf = Path(urdf).expanduser().resolve()
        if not urdf.is_file():
            raise FileNotFoundError(f"TRON2 URDF does not exist: {urdf}")
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError("rate must be a finite positive frequency in Hz")

        self._app: Any = None
        self._kit_app: Any = None
        self._world: Any = None
        self._robot: Any = None
        self._joint_indices: np.ndarray | None = None
        self._temporary_assets: tempfile.TemporaryDirectory[str] | None = None
        self._headless = bool(headless)
        self._gui_render_stride = max(1, round(float(rate) / 30.0))
        self._step_count = 0
        self._last_idle_render = 0.0
        self._closed = False

        try:
            from isaacsim import SimulationApp

            self._app = SimulationApp({"headless": self._headless, "fast_shutdown": True})

            # Isaac Sim imports must happen only after SimulationApp starts.
            from omni.kit.app import get_app

            self._kit_app = get_app()
            import omni.kit.commands
            import omni.usd
            from isaacsim.core.api import World
            from isaacsim.core.prims import SingleArticulation
            from isaacsim.core.utils.types import ArticulationAction
            from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdLux

            self._ArticulationAction = ArticulationAction

            stage = omni.usd.get_context().get_stage()
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)

            self._world = World(
                stage_units_in_meters=1.0,
                physics_dt=1.0 / float(rate),
                rendering_dt=1.0 / float(rate),
            )

            ok, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            if not ok:
                raise RuntimeError("Isaac Sim could not create a URDF import configuration")
            import_config.fix_base = True
            import_config.merge_fixed_joints = False
            import_config.make_default_prim = False
            import_config.create_physics_scene = False
            import_config.parse_mimic = False
            import_config.self_collision = False
            import_config.distance_scale = 1.0

            import_urdf = self._prepare_motor_only_urdf(urdf)
            ok, robot_path = omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=str(import_urdf),
                import_config=import_config,
                get_articulation_root=True,
            )
            if not ok or not robot_path:
                raise RuntimeError(f"Isaac Sim failed to import TRON2 URDF: {urdf}")

            source_armatures = self._apply_mjcf_joint_dynamics(
                stage=stage,
                urdf=urdf,
                physx_schema=PhysxSchema,
                usd=Usd,
                sdf=Sdf,
            )
            if not self._headless:
                light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
                light.CreateIntensityAttr().Set(1000.0)
                from isaacsim.core.utils.viewports import set_camera_view

                set_camera_view(
                    eye=[2.6, -3.0, 1.9],
                    target=[0.0, 0.0, 0.9],
                    camera_prim_path="/OmniverseKit_Persp",
                )

            self._robot = self._world.scene.add(
                SingleArticulation(prim_path=str(robot_path), name="tron2")
            )
            self._world.reset()

            dof_names = tuple(self._robot.dof_names)
            missing = [name for name in MOTOR_NAMES if name not in dof_names]
            if missing:
                raise ValueError(
                    "Imported URDF does not contain all sixteen TRON2 motor joints; "
                    f"missing {missing}; available={dof_names}"
                )
            self._joint_indices = np.asarray(
                [self._robot.get_dof_index(name) for name in MOTOR_NAMES],
                dtype=np.int32,
            )
            applied_armatures = np.asarray(
                self._robot._articulation_view.get_armatures(
                    joint_indices=self._joint_indices
                )[0],
                dtype=np.float64,
            )
            if not np.allclose(applied_armatures, source_armatures, rtol=1e-5, atol=1e-7):
                raise RuntimeError(
                    "PhysX did not retain the pinned MuJoCo rotor inertias; "
                    f"expected={source_armatures.tolist()}, actual={applied_armatures.tolist()}"
                )

            # Direct effort commands must not also be opposed by imported
            # position/velocity drives. Zero all drives, including passive
            # model joints, and issue effort commands only to the 16 motors.
            controller = self._robot.get_articulation_controller()
            controller.set_gains(
                kps=np.zeros(self._robot.num_dof, dtype=np.float32),
                kds=np.zeros(self._robot.num_dof, dtype=np.float32),
            )

            self._robot.set_joint_positions(
                np.asarray(HOME_Q, dtype=np.float32),
                joint_indices=self._joint_indices,
            )
            self._robot.set_joint_velocities(
                np.zeros(len(MOTOR_NAMES), dtype=np.float32),
                joint_indices=self._joint_indices,
            )
            self._robot.apply_action(
                ArticulationAction(
                    joint_efforts=np.zeros(len(MOTOR_NAMES), dtype=np.float32),
                    joint_indices=self._joint_indices,
                )
            )
            self._world.play()
        except BaseException as exc:
            self.close(exc)
            raise

    def __enter__(self) -> "IsaacRobot":
        return self

    def __exit__(self, exc_type: Any, exc: BaseException | None, tb: Any) -> None:
        self.close(exc)

    def state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return measured position, velocity and effort in motor wire order."""
        robot, indices = self._require_ready()
        q = np.asarray(robot.get_joint_positions()[indices], dtype=np.float64).copy()
        dq = np.asarray(robot.get_joint_velocities()[indices], dtype=np.float64).copy()
        effort = np.asarray(robot.get_measured_joint_efforts()[indices], dtype=np.float64).copy()
        return q, dq, effort

    def step(self, effort: np.ndarray) -> None:
        """Apply one saturated motor-effort vector and advance one physics step."""
        robot, indices = self._require_ready()
        values = np.asarray(effort, dtype=np.float64)
        if values.shape != (len(MOTOR_NAMES),):
            raise ValueError(f"effort must have shape ({len(MOTOR_NAMES)},), got {values.shape}")
        if not np.isfinite(values).all():
            raise ValueError("effort must contain only finite values")

        limits = np.asarray(TORQUE_LIMITS, dtype=np.float64)
        values = np.clip(values, -limits, limits).astype(np.float32)
        robot.apply_action(
            self._ArticulationAction(joint_efforts=values, joint_indices=indices)
        )
        self._step_count += 1
        render = (
            not self._headless
            and self._app.is_running()
            and self._step_count % self._gui_render_stride == 0
        )
        self._world.step(render=render)

    def render_idle(self) -> None:
        """Refresh the GUI without advancing physics while awaiting commands."""
        now = time.monotonic()
        if self.is_running() and not self._headless and now - self._last_idle_render >= 1 / 30:
            self._world.render()
            self._last_idle_render = now

    def is_running(self) -> bool:
        return not self._closed and self._app is not None and self._app.is_running()

    def close(self, failure: BaseException | None = None) -> None:
        if self._closed:
            return
        self._closed = True
        exit_code = 0
        if isinstance(failure, SystemExit) and isinstance(failure.code, int):
            exit_code = failure.code
        elif isinstance(failure, KeyboardInterrupt):
            exit_code = 130
        elif failure is not None:
            traceback.print_exception(failure)
            exit_code = 1
        try:
            if self._world is not None:
                self._world.stop()
        except Exception:
            pass
        finally:
            self._world = None
            if self._temporary_assets is not None:
                self._temporary_assets.cleanup()
                self._temporary_assets = None
            if self._app is not None:
                # Kit terminates the process during fast shutdown. Preserve the
                # failure status for shell callers before asking Kit to close.
                if self._kit_app is not None:
                    self._kit_app.post_uncancellable_quit(exit_code)
                self._app.close(wait_for_replicator=False)
                self._app = None

    def _require_ready(self) -> tuple[Any, np.ndarray]:
        if self._closed or self._robot is None or self._joint_indices is None:
            raise RuntimeError("IsaacRobot is closed or was not initialized")
        return self._robot, self._joint_indices

    def _prepare_motor_only_urdf(self, urdf: Path) -> Path:
        """Resolve asset URIs and fix the grasper joints outside the motor interface.

        Prepare a temporary URDF with the pin's link inertials, visuals,
        collision geometry, and all 16 motor joints.
        """
        temporary_assets = tempfile.TemporaryDirectory(prefix="tron2-isaac-assets-")
        self._temporary_assets = temporary_assets
        root = Path(temporary_assets.name)
        copied_urdf = root / "asset" / "urdf" / urdf.name
        copied_urdf.parent.mkdir(parents=True)

        source_meshes = urdf.resolve().parent.parent / "meshes"
        copied_meshes = copied_urdf.parent.parent / "meshes"
        if source_meshes.is_dir():
            copied_meshes.symlink_to(source_meshes, target_is_directory=True)

        tree = ET.parse(urdf)
        xml_root = tree.getroot()
        for mesh in xml_root.iter("mesh"):
            filename = mesh.get("filename", "")
            if filename.startswith("package://"):
                mesh_name = filename.rsplit("/", 1)[-1]
                mesh_path = source_meshes / mesh_name
                if not mesh_path.is_file():
                    raise FileNotFoundError(
                        f"URDF mesh referenced by {urdf} is missing: {mesh_path}"
                    )
                mesh.set("filename", f"../meshes/{mesh_name}")

        fixed_grasper_joints = 0
        for joint in xml_root.iter("joint"):
            if not joint.get("name", "").startswith("grasper_"):
                continue
            joint.set("type", "fixed")
            for child in tuple(joint):
                if child.tag in {"axis", "limit", "mimic", "dynamics", "safety_controller"}:
                    joint.remove(child)
            fixed_grasper_joints += 1
        if fixed_grasper_joints == 0:
            raise ValueError(f"Expected grasper linkage joints in DACH_TRON2A URDF: {urdf}")

        tree.write(copied_urdf, encoding="utf-8", xml_declaration=True)
        return copied_urdf

    @staticmethod
    def _apply_mjcf_joint_dynamics(
        *, stage: Any, urdf: Path, physx_schema: Any, usd: Any, sdf: Any
    ) -> np.ndarray:
        """Copy the pinned MJCF motor armatures and joint damping into PhysX."""
        mjcf = urdf.resolve().parent.parent / "xml" / f"{urdf.stem}.xml"
        if not mjcf.is_file():
            raise FileNotFoundError(f"Pinned TRON2 MuJoCo model is missing: {mjcf}")
        xml_root = ET.parse(mjcf).getroot()
        default_joint = xml_root.find("./default/joint")
        default_damping = float(default_joint.get("damping", "0")) if default_joint is not None else 0.0
        armature_by_name = {
            joint.get("name"): float(joint.get("armature"))
            for joint in xml_root.iter("joint")
            if joint.get("name") and joint.get("armature")
        }
        missing = [name for name in MOTOR_NAMES if name not in armature_by_name]
        if missing:
            raise ValueError(f"Pinned MuJoCo model has no armature for TRON2 motors: {missing}")

        source_armatures = np.asarray(
            [armature_by_name[name] for name in MOTOR_NAMES], dtype=np.float64
        )
        for name in MOTOR_NAMES:
            matches = [prim for prim in stage.Traverse() if prim.GetName() == name]
            if len(matches) != 1:
                raise ValueError(f"Expected one imported USD joint named {name}, found {len(matches)}")
            prim = matches[0]
            physx_joint = physx_schema.PhysxJointAPI.Apply(prim)
            physx_joint.CreateArmatureAttr().Set(armature_by_name[name])

            # Isaac Sim 5.1 registers PhysxJointAxisAPI as a codeless schema.
            # Apply it through the USD registry so PhysX sees the per-axis
            # armature and viscous damping attributes instead of ignoring them.
            if not prim.HasAPI("PhysxJointAxisAPI", "angular"):
                axis_api = usd.SchemaRegistry.GetTypeFromSchemaTypeName("PhysxJointAxisAPI")
                if not axis_api or not prim.ApplyAPI(axis_api, "angular"):
                    raise RuntimeError(f"Could not apply PhysxJointAxisAPI to {name}")
            armature_attr = prim.GetAttribute("physxJointAxis:angular:armature")
            if not armature_attr:
                armature_attr = prim.CreateAttribute(
                    "physxJointAxis:angular:armature", sdf.ValueTypeNames.Float
                )
            armature_attr.Set(armature_by_name[name])
            damping_attr = prim.GetAttribute(
                "physxJointAxis:angular:viscousFrictionCoefficient"
            )
            if not damping_attr:
                damping_attr = prim.CreateAttribute(
                    "physxJointAxis:angular:viscousFrictionCoefficient",
                    sdf.ValueTypeNames.Float,
                )
            damping_attr.Set(default_damping)
        return source_armatures
