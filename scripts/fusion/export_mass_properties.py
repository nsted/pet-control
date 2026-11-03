"""
Run INSIDE Autodesk Fusion (not with python): exports mass properties and joint geometry of the active design.

Writes ~/Desktop/pet_mass_properties.json with, for every component occurrence:
  mass [kg], volume [cm^3], density [kg/cm^3], centre of mass [cm, world frame],
  principal moments of inertia about the centre of mass [kg*cm^2] and the principal axes (world frame),
  moments about the world origin [kg*cm^2], each body's material, and the occurrence's transform;
and for every joint: type, rotation axis and origin points (world frame, cm).
Units are Fusion's internal units: centimetres, kilograms, kg*cm^2.

HOW TO RUN (Fusion): Utilities tab > Add-Ins > Scripts and Add-Ins (or Shift+S) > the green "+" next to "My Scripts"
> "Create new script" (Python) > name it e.g. PETExport > Create > open its .py in an editor, replace its contents with this
file, save > select the script and press Run. A message box shows the output path. Open the PET assembly first.

NOTE: written against the Fusion API from memory and NOT tested inside Fusion. If a property errors, the JSON still
records the error text for that item and carries on, so send the file anyway.
"""

import json
import os
import traceback

import adsk.core
import adsk.fusion


def _pt(p):
    return [p.x, p.y, p.z] if p is not None else None


def _props(obj):
    """Physical properties of a component/occurrence/body, or an {"error": ...} record."""
    try:
        p = obj.physicalProperties
        try:
            p.accuracy = adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        except Exception:
            pass
        d = {
            "mass_kg": p.mass,
            "volume_cm3": p.volume,
            "density_kg_per_cm3": p.density,
            "area_cm2": p.area,
            "center_of_mass_cm": _pt(p.centerOfMass),
        }
        ok, ixx, iyy, izz, ixy, iyz, ixz = p.getXYZMomentsOfInertia()
        d["moments_about_world_origin_kg_cm2"] = {"xx": ixx, "yy": iyy, "zz": izz, "xy": ixy, "yz": iyz, "xz": ixz}
        ok, i1, i2, i3 = p.getPrincipalMomentsOfInertia()
        d["principal_moments_about_com_kg_cm2"] = [i1, i2, i3]
        ok, a1, a2, a3 = p.getPrincipalAxes()
        d["principal_axes_world"] = [[a1.x, a1.y, a1.z], [a2.x, a2.y, a2.z], [a3.x, a3.y, a3.z]]
        return d
    except Exception:
        return {"error": traceback.format_exc()}


def _transform(occ):
    try:
        ok, origin, xa, ya, za = occ.transform2.getAsCoordinateSystem()
        return {"origin_cm": _pt(origin), "x": [xa.x, xa.y, xa.z], "y": [ya.x, ya.y, ya.z], "z": [za.x, za.y, za.z]}
    except Exception:
        return {"error": traceback.format_exc()}


def _joint(j):
    d = {"name": j.name}
    try:
        m = j.jointMotion
        d["type"] = str(m.jointType)
        try:
            v = m.rotationAxisVector
            d["rotation_axis_vector"] = [v.x, v.y, v.z]
        except Exception:
            pass
        try:
            d["rotation_value_rad"] = m.rotationValue
        except Exception:
            pass
    except Exception:
        d["motion_error"] = traceback.format_exc()
    for label in ("geometryOrOriginOne", "geometryOrOriginTwo"):
        try:
            g = getattr(j, label)
            origin = getattr(g, "origin", None)
            if origin is None and hasattr(g, "geometry"):
                origin = g.geometry.origin
            d[label + "_origin_cm"] = _pt(origin)
        except Exception:
            d[label + "_error"] = traceback.format_exc()
    for label in ("occurrenceOne", "occurrenceTwo"):
        try:
            d[label] = getattr(j, label).fullPathName
        except Exception:
            pass
    return d


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        out = {
            "document": app.activeDocument.name,
            "units": "cm, kg, kg*cm^2 (Fusion internal)",
            "root": _props(root),
            "occurrences": [],
            "joints": [],
        }
        for occ in root.allOccurrences:
            entry = {
                "path": occ.fullPathName,
                "component": occ.component.name,
                "transform": _transform(occ),
                "properties": _props(occ),
                "bodies": [],
            }
            for body in occ.component.bRepBodies:
                b = {"name": body.name}
                try:
                    b["material"] = body.material.name if body.material else None
                except Exception:
                    b["material"] = None
                b["properties"] = _props(body)
                entry["bodies"].append(b)
            out["occurrences"].append(entry)
        seen = set()
        components = [root] + [o.component for o in root.allOccurrences]
        for comp in components:
            for j in comp.joints:
                if j.entityToken not in seen:
                    seen.add(j.entityToken)
                    out["joints"].append(_joint(j))
        path = os.path.expanduser("~/Desktop/pet_mass_properties.json")
        with open(path, "w") as fh:
            json.dump(out, fh, indent=2)
        ui.messageBox("Wrote %d occurrences and %d joints to\n%s" % (len(out["occurrences"]), len(out["joints"]), path))
    except Exception:
        ui.messageBox("Export failed:\n%s" % traceback.format_exc())
