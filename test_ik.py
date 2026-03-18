import sys
import numpy as np
import json

from cinematica_inversa import solve_ik_from_robtarget
from especificaciones_robot import ROBOTS

robot_limits = ROBOTS["ABB_IRB_140"]["limites_articulares"]
quaternion = [4.14816E-8, 6.1133E-9, -1, -2.53589E-16]

res1 = solve_ik_from_robtarget([450, 0, 450], quaternion, robot_limits, from_robotstudio=False)
res1_true = solve_ik_from_robtarget([450, 0, 450], quaternion, robot_limits, from_robotstudio=True)
res2 = solve_ik_from_robtarget([450, 0, 400], quaternion, robot_limits, from_robotstudio=False)
res3 = solve_ik_from_robtarget([550, 0, 400], quaternion, robot_limits, from_robotstudio=False)

output = {
    "P10_False": res1,
    "P10_True": res1_true,
    "P20_False": res2,
    "P30_False": res3
}

with open("output.json", "w", encoding="utf-8") as f:
    json.dump(output, f, indent=2)
