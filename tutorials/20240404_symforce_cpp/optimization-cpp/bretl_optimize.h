#pragma once

namespace bretl_optimize {

enum Var : char {
  T_INB_OFA = 'A',              // Pose3d
  P_INA = 'p',                  // Vector3d
  B = 'b',                      // Vector2d
  FX = 'f',                     // Scalar
  FY = 'g',                     // Scalar
  CX = 'h',                     // Scalar
  CY = 'i',                     // Scalar
  EPSILON = 'e',                // Scalar
};

void RunOptimization();

}
