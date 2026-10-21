"""Fusion 360 script: export the arm's structure for URDF generation.

Writes into the chosen folder:
  export_log.txt      every step is logged BEFORE it runs (find the crash point)
  arm_export.json     links, joints, mass properties (rewritten after each stage)
  meshes/<link>.stl   one low-resolution binary STL per link (optional stage)

Stages run in increasing cost: structure -> joints -> mass properties -> STL.
A failure in one link or joint is logged and skipped; the export continues.

Raw Fusion units are kept in the JSON (cm, kg, kg*cm^2, rad) and converted
to SI by the URDF generator in the repository.
"""

import json
import os
import traceback

import adsk.core
import adsk.fusion

JOINT_TYPES = {
    adsk.fusion.JointTypes.RigidJointType: "rigid",
    adsk.fusion.JointTypes.RevoluteJointType: "revolute",
    adsk.fusion.JointTypes.SliderJointType: "slider",
    adsk.fusion.JointTypes.CylindricalJointType: "cylindrical",
    adsk.fusion.JointTypes.PinSlotJointType: "pin_slot",
    adsk.fusion.JointTypes.PlanarJointType: "planar",
    adsk.fusion.JointTypes.BallJointType: "ball",
}


class Exporter:
    def __init__(self, ui, design, folder):
        self.ui = ui
        self.design = design
        self.root = design.rootComponent
        self.folder = folder
        self.data = {"design": design.parentDocument.name, "units": {
            "length": "cm", "mass": "kg", "inertia": "kg*cm^2", "angle": "rad"},
            "links": {}, "joints": {}, "warnings": []}
        self.log_file = open(os.path.join(folder, "export_log.txt"), "w", encoding="utf-8", buffering=1)

    # -- helpers ---------------------------------------------------------
    def log(self, msg):
        self.log_file.write(msg + "\n")
        self.log_file.flush()
        os.fsync(self.log_file.fileno())
        adsk.doEvents()

    def warn(self, msg):
        self.data["warnings"].append(msg)
        self.log("WARNING: " + msg)

    def save_json(self):
        with open(os.path.join(self.folder, "arm_export.json"), "w", encoding="utf-8") as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)

    @staticmethod
    def point(p):
        return [p.x, p.y, p.z]

    @staticmethod
    def vector(v):
        return [v.x, v.y, v.z]

    # -- stage 1: structure ------------------------------------------------
    def structure(self):
        self.log("STAGE 1: structure")
        for occ in self.root.occurrences:
            name = occ.component.name
            self.log("  link " + name)
            comp = occ.component
            solid = sum(1 for b in comp.bRepBodies if b.isSolid)
            link = {
                "occurrence": occ.name,
                "visible": occ.isLightBulbOn,
                "referenced": occ.isReferencedComponent,
                "transform": list(occ.transform2.asArray()),
                "solid_bodies": solid,
                "other_brep_bodies": comp.bRepBodies.count - solid,
                "mesh_bodies": comp.meshBodies.count,
                "child_occurrences": occ.childOccurrences.count,
                "bodies": [{"name": b.name, "solid": b.isSolid, "visible": b.isLightBulbOn,
                            "material": b.material.name if b.material else None,
                            "faces": b.faces.count} for b in comp.bRepBodies],
            }
            if link["child_occurrences"]:
                self.warn(name + ": contains nested components (not supported)")
            if link["mesh_bodies"]:
                self.warn(name + ": contains mesh bodies (ignored)")
            if solid == 0:
                self.warn(name + ": no solid bodies")
            for b in link["bodies"]:
                if b["faces"] > 2000:
                    self.warn("{}: body '{}' has {} faces (very heavy, likely converted mesh)".format(
                        name, b["name"], b["faces"]))
            self.data["links"][name] = link
        if self.root.bRepBodies.count:
            self.warn("root component has {} bodies outside any link component".format(
                self.root.bRepBodies.count))
        self.save_json()

    # -- stage 2: joints ---------------------------------------------------
    def joint_geometry(self, geo):
        if geo is None:
            return None
        if hasattr(geo, "geometry") and not hasattr(geo, "primaryAxisVector"):
            geo = geo.geometry  # JointOrigin -> JointGeometry
        return {
            "origin": self.point(geo.origin),
            "primary_axis": self.vector(geo.primaryAxisVector),
            "secondary_axis": self.vector(geo.secondaryAxisVector),
            "third_axis": self.vector(geo.thirdAxisVector),
        }

    @staticmethod
    def limits(lim):
        return {
            "min_enabled": lim.isMinimumValueEnabled, "min": lim.minimumValue,
            "max_enabled": lim.isMaximumValueEnabled, "max": lim.maximumValue,
            "rest_enabled": lim.isRestValueEnabled, "rest": lim.restValue,
        }

    def joints(self):
        self.log("STAGE 2: joints")
        for abj in self.root.asBuiltJoints:
            self.warn("as-built joint '{}' is not exported - recreate it as a regular Joint".format(abj.name))
        for j in self.root.joints:
            self.log("  joint " + j.name)
            entry = {"suppressed": j.isSuppressed}
            try:
                motion = j.jointMotion
                entry["type"] = JOINT_TYPES.get(motion.jointType, str(motion.jointType))
                # occurrenceOne is the child, occurrenceTwo the parent (Fusion selection order)
                entry["child"] = j.occurrenceOne.component.name if j.occurrenceOne else None
                entry["parent"] = j.occurrenceTwo.component.name if j.occurrenceTwo else None
                if entry["type"] == "revolute":
                    entry["axis"] = self.vector(motion.rotationAxisVector)
                    entry["limits"] = self.limits(motion.rotationLimits)
                    entry["value"] = motion.rotationValue
                elif entry["type"] == "slider":
                    entry["axis"] = self.vector(motion.slideDirectionVector)
                    entry["limits"] = self.limits(motion.slideLimits)
                    entry["value"] = motion.slideValue
            except Exception:
                self.warn("joint '{}': motion not readable\n{}".format(j.name, traceback.format_exc()))
            for key, attr in (("geometry_one", "geometryOrOriginOne"), ("geometry_two", "geometryOrOriginTwo")):
                try:
                    entry[key] = self.joint_geometry(getattr(j, attr))
                except Exception:
                    entry[key] = None
                    self.warn("joint '{}': {} not readable".format(j.name, attr))
            if entry.get("type") in ("revolute", "slider"):
                lim = entry.get("limits", {})
                if not (lim.get("min_enabled") and lim.get("max_enabled")):
                    self.warn("joint '{}': min and max limits are not both enabled".format(j.name))
            elif entry.get("type") not in ("rigid", None):
                self.warn("joint '{}': unsupported type {}".format(j.name, entry.get("type")))
            self.data["joints"][j.name] = entry
            self.save_json()

    # -- stage 3: mass properties ------------------------------------------
    def mass_properties(self):
        self.log("STAGE 3: mass properties (low accuracy)")
        accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy
        for occ in self.root.occurrences:
            name = occ.component.name
            self.log("  START mass properties " + name)
            try:
                props = occ.getPhysicalProperties(accuracy)
                _, xx, yy, zz, xy, yz, xz = props.getXYZMomentsOfInertia()
                self.data["links"][name]["mass_properties"] = {
                    "mass": props.mass,
                    "volume": props.volume,
                    "center_of_mass": self.point(props.centerOfMass),
                    "inertia_world_origin": {"xx": xx, "yy": yy, "zz": zz, "xy": xy, "yz": yz, "xz": xz},
                }
                self.log("  OK   {}: mass {:.4f} kg".format(name, props.mass))
            except Exception:
                self.warn("{}: mass properties failed\n{}".format(name, traceback.format_exc()))
            self.save_json()

    # -- stage 4: meshes ---------------------------------------------------
    def meshes(self):
        self.log("STAGE 4: STL meshes (low refinement)")
        mesh_dir = os.path.join(self.folder, "meshes")
        os.makedirs(mesh_dir, exist_ok=True)
        export_mgr = self.design.exportManager
        for occ in self.root.occurrences:
            name = occ.component.name
            path = os.path.join(mesh_dir, name + ".stl")
            self.log("  START stl " + name)
            try:
                opts = export_mgr.createSTLExportOptions(occ, path)
                opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                opts.isBinaryFormat = True
                opts.sendToPrintUtility = False
                export_mgr.execute(opts)
                self.data["links"][name]["mesh"] = "meshes/" + name + ".stl"
                self.log("  OK   stl " + name)
            except Exception:
                self.warn("{}: STL export failed\n{}".format(name, traceback.format_exc()))
            self.save_json()

    def close(self):
        self.log("DONE")
        self.log_file.close()


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open the arm design in the DESIGN workspace first.")
            return

        dialog = ui.createFolderDialog()
        dialog.title = "Output folder for the arm export (use a simple path, e.g. C:\\arm_export)"
        if dialog.showDialog() != adsk.core.DialogResults.DialogOK:
            return

        exporter = Exporter(ui, design, dialog.folder)
        exporter.structure()
        exporter.joints()

        ask = adsk.core.MessageBoxButtonTypes.YesNoButtonType
        yes = adsk.core.DialogResults.DialogYes
        if ui.messageBox("Structure and joints exported.\nCompute mass properties?", "Arm export", ask) == yes:
            exporter.mass_properties()
        if ui.messageBox("Export STL meshes (low resolution)?", "Arm export", ask) == yes:
            exporter.meshes()
        exporter.close()

        summary = "Done. {} links, {} joints, {} warnings.\nSee export_log.txt in:\n{}".format(
            len(exporter.data["links"]), len(exporter.data["joints"]),
            len(exporter.data["warnings"]), dialog.folder)
        ui.messageBox(summary, "Arm export")
    except Exception:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
