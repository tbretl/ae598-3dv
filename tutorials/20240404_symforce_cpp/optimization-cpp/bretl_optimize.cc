/* ----------------------------------------------------------------------------
 * SymForce - Copyright 2022, Skydio, Inc.
 * This source code is under the Apache 2.0 license found in the LICENSE file.
 * ---------------------------------------------------------------------------- */

#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>

#include <sym/pose3.h>
#include <symforce/opt/optimizer.h>
#include <symforce/opt/values.h>

#include "bretl_optimize.h"
#include "bretl_projection_factor.h"
#include "bretl_projection_nopose_factor.h"
#include "bretl_scale_factor.h"

namespace bretl_optimize {

void RunOptimization(const std::filesystem::path &inpath, const std::filesystem::path &outpath) {
  std::vector<sym::Factor<double>> factors;
  sym::Valuesd values;
  
  std::cout << "Reading from : " << inpath << std::endl;
  std::cout << "Writing to   : " << outpath << std::endl;
  std::cout << std::endl;

  std::ifstream ifile(inpath);
  std::ofstream ofile(outpath);
  
  double fx, fy, cx, cy;
  int num_views, num_tracks, num_matches;
  int i_view, i_track;
  double qx, qy, qz, qw, x, y, z;
  
  // Intrinsics
  ifile >> fx >> fy >> cx >> cy;
  values.Set({Var::FX}, fx);
  values.Set({Var::FY}, fy);
  values.Set({Var::CX}, cx);
  values.Set({Var::CY}, cy);

  // Views
  ifile >> num_views;
  for (int i=0; i < num_views; ++i) {
    ifile >> i_view >> qx >> qy >> qz >> qw >> x >> y >> z;
    values.Set({Var::T_INB_OFA, i_view}, sym::Pose3d((sym::Pose3d::DataVec() << qx, qy, qz, qw, x, y, z).finished()));
  }

  factors.push_back(
    sym::Factor<double>::Hessian(
      bretl_scale::BretlScaleFactor<double>,                // name of factor
      { // keys of all variables (including the ones to optimize)
        {Var::T_INB_OFA, 1},
        {Var::EPSILON}
      },
      { // keys of variables to optimize
        {Var::T_INB_OFA, 1},
      }
  ));

  // Tracks
  ifile >> num_tracks;
  for (int i=0; i < num_tracks; ++i) {
    ifile >> i_track >> num_matches >> x >> y >> z;
    values.Set({Var::P_INA, i_track}, sym::Matrix31d(x, y, z));
    for (int j=0; j < num_matches; ++j) {
      ifile >> i_view >> x >> y;
      values.Set({Var::B, i_track, i_view}, sym::Matrix21d(x, y));
      if (i_view == 0) {
        factors.push_back(
          sym::Factor<double>::Hessian(
            bretl_projection_nopose::BretlProjectionNoposeFactor<double>,                // name of factor
            { // keys of all variables (including the ones to optimize)
              {Var::T_INB_OFA, i_view},
              {Var::P_INA, i_track},
              {Var::B, i_track, i_view},
              {Var::FX},
              {Var::FY},
              {Var::CX},
              {Var::CY},
              {Var::EPSILON}
            },
            { // keys of variables to optimize
              {Var::P_INA, i_track}
            }
        ));
      } else {
        factors.push_back(
          sym::Factor<double>::Hessian(
            bretl_projection::BretlProjectionFactor<double>,                // name of factor
            { // keys of all variables (including the ones to optimize)
              {Var::T_INB_OFA, i_view},
              {Var::P_INA, i_track},
              {Var::B, i_track, i_view},
              {Var::FX},
              {Var::FY},
              {Var::CX},
              {Var::CY},
              {Var::EPSILON}
            },
            { // keys of variables to optimize
              {Var::T_INB_OFA, i_view},
              {Var::P_INA, i_track}
            }
        ));
      }
    }
  }

  values.Set(Var::EPSILON, sym::kDefaultEpsilond);

  auto params = sym::DefaultOptimizerParams();
  params.verbose = true;
  sym::Optimizer<double> optimizer(params, factors, sym::kDefaultEpsilon<double>);
  const auto stats = optimizer.Optimize(values);
  
  ifile.clear();
  ifile.seekg(0);

  // Intrinsics
  ifile >> fx >> fy >> cx >> cy;

  // Views
  ifile >> num_views;
  ofile << num_views << std::endl;
  for (int i=0; i < num_views; ++i) {
    ifile >> i_view >> qx >> qy >> qz >> qw >> x >> y >> z;
    ofile << i_view << " " << values.At<sym::Pose3d>({Var::T_INB_OFA, i_view}).Data().transpose() << std::endl;
  }

  // Tracks
  ifile >> num_tracks;
  ofile << num_tracks  << std::endl;
  for (int i=0; i < num_tracks; ++i) {
    ifile >> i_track >> num_matches >> x >> y >> z;
    ofile << i_track << " " << values.At<sym::Matrix31d>({Var::P_INA, i_track}).transpose() << std::endl;
    for (int j=0; j < num_matches; ++j) {
      ifile >> i_view >> x >> y;
    }
  }

}


}  // namespace bretl_optimize


int main(int argc, char* argv[]) {
  const std::filesystem::path inpath = std::filesystem::absolute(argv[1]);
  const std::filesystem::path outpath = std::filesystem::absolute(argv[2]);
  bretl_optimize::RunOptimization(inpath, outpath);
  
  return 0;
}
