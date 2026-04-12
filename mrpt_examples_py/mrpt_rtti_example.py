#!/usr/bin/env python3
"""
mrpt_rtti_example.py — runtime type information with mrpt.rtti.

Demonstrates:
  - getAllRegisteredClasses(): inspect the class registry
  - findRegisteredClass(): look up by name
  - TRuntimeClassId: className, derivedFrom(), getBaseClass()
  - classFactory(): create objects by name
"""

import mrpt.rtti as rtti
import mrpt.poses   # importing poses registers CPose2D, CPose3D, etc.
import mrpt.obs     # registers CObservation2DRangeScan, etc.
import mrpt.maps    # registers CSimplePointsMap, etc.

# ---------------------------------------------------------------------------
# getAllRegisteredClasses — list everything in the RTTI registry
# ---------------------------------------------------------------------------
all_classes = rtti.getAllRegisteredClasses()
print(f"Total registered classes: {len(all_classes)}")

# Print the first few
for cls in sorted(all_classes, key=lambda c: c.className)[:10]:
    print(f"  {cls.className}")
print("  ...")

# ---------------------------------------------------------------------------
# findRegisteredClass — look up by name
# ---------------------------------------------------------------------------
cls_pose2d = rtti.findRegisteredClass("CPose2D")
if cls_pose2d:
    print(f"\nFound 'CPose2D': className='{cls_pose2d.className}'")
    base = cls_pose2d.getBaseClass()
    print(f"  base class: {base.className if base else '(none)'}")
    print(f"  derivedFrom('CSerializable'): {cls_pose2d.derivedFrom('CSerializable')}")
else:
    print("\n'CPose2D' not found (poses module may not be loaded)")

# ---------------------------------------------------------------------------
# classFactory — create an object by class name
# ---------------------------------------------------------------------------
obj = rtti.classFactory("CPose2D")
if obj:
    print(f"\nclassFactory('CPose2D'): {obj.GetRuntimeClass().className}")
else:
    print("\nclassFactory returned None")

# ---------------------------------------------------------------------------
# getAllRegisteredClassesChildrenOf — filter by base class
# ---------------------------------------------------------------------------
obs_cls = rtti.findRegisteredClass("CObservation")
if obs_cls:
    obs_children = rtti.getAllRegisteredClassesChildrenOf(obs_cls)
    print(f"\nClasses derived from CObservation: {len(obs_children)}")
    for c in sorted(obs_children, key=lambda x: x.className)[:8]:
        print(f"  {c.className}")
