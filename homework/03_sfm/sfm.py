import sys
import numpy as np
import matplotlib.pyplot as plt
import cv2

import symforce
# symforce.set_epsilon_to_symbol()
import symforce.symbolic as sf
from symforce.values import Values
from symforce.opt.factor import Factor
from symforce.opt.optimizer import Optimizer
from symforce.opt.noise_models import PseudoHuberNoiseModel
from symforce.opt.noise_models import BarronNoiseModel
import sym



"""
FUNCTIONS THAT YOU NEED TO MODIFY
"""

"""
Functions for two-view reconstruction.
"""

def getE(a, b, K, rng, threshold=1e-3, num_iters=1000):
    """
    INPUTS
    
    -- required --
    a is n x 2
    b is n x 2
    K is 3 x 3
    rng is a random number generator
    
    -- optional --
    threshold is the max error (e.g., epipolar or sampson) for inliers
    num_iters is the number of RANSAC iterations
    
    OUTPUTS

    E is 3 x 3
    num_inliers is a scalar
    mask is length n (truthy value for each match that is an inlier, falsy otherwise)
    """
    
    return None, None, None

def decomposeE(a, b, K, E):
    """
    INPUTS
    
    -- required --
    a is n x 2
    b is n x 2
    K is 3 x 3
    E is 3 x 3
    
    OUTPUTS

    R_inB_ofA is 3 x 3
    p_inB_ofA is length 3
    p_inA is n x 3
    """

    return None, None, None


"""
Functions for resectioning and triangulation.
"""

def resection(p_inA, c, K, rng, threshold=1., num_iters=1000):
    """
    INPUTS
    
    -- required --
    p_inA is n x 3
    c is n x 2
    K is 3 x 3
    rng is a random number generator
    
    -- optional --
    threshold is the max error (e.g., reprojection) for inliers
    num_iters is the number of RANSAC iterations
    
    OUTPUTS

    R_inC_ofA is 3 x 3
    p_inC_ofA is length 3
    num_inliers is a scalar
    mask is length n (truthy value for each match that is an inlier, falsy otherwise)
    """
    
    return None, None, None, None

def triangulate(track, views, K):
    """
    INPUTS
    track is **one** track of matches to triangulate
    views is the list of all views
    K is 3 x 3
    
    OUTPUTS

    p_inA is length 3
    """

    return None


"""
Functions for optimization
"""

def get_optimizer(views, tracks, K):
    """
    Returns a symforce optimizer and a set of initial_values that
    allow you to solve the bundle adjustment problem corresponding
    to the given views and tracks.
    """

    # Create data structures
    initial_values = Values(
        fx=1.,                                                              # <-- FIXME
        fy=1.,                                                              # <-- FIXME
        cx=1.,                                                              # <-- FIXME
        cy=1.,                                                              # <-- FIXME
        tracks=[],
        epsilon=sym.epsilon,
    )
    optimized_keys = []
    factors = []

    # For each view that has a pose estimate, add this pose estimate as an initial
    # value and (if not the first view) as an optimized key.
    print(f'Iterate over {len(views)} views:')
    for i, view in enumerate(views):
        if (view['R_inB_ofA'] is None) or (view['p_inB_ofA'] is None):
            continue
        
        initial_values[f'T_inB{i}_ofA'] = sym.Pose3(
            R=sym.Rot3.identity(),                                          # <-- FIXME
            t=np.zeros(3),                                                  # <-- FIXME
        )

        if i > 0:
            optimized_keys.append(f'T_inB{i}_ofA')
            print(f' T_inB{i}_ofA has an initial value and is an optimized key')
        else:
            print(f' T_inB{i}_ofA has an initial value')

    # Add a factor to fix the scale (the relative distance between frames B0 and
    # B1 will be something close to one).
    print(f'T_inB{1}_ofA has an sf_scale_residual factor')
    factors = [
        Factor(
            residual=sf_scale_residual,
            keys=[
                f'T_inB{1}_ofA',
                'epsilon',
            ],
        )
    ]

    # For each valid track, add its 3d point as an initial value and an optimized
    # key, and then, for each match in this track, add its 2d point as an initial
    # value and add a factor to penalize reprojection error.
    print(f'Iterate over {len(tracks)} tracks:')
    for i_track, track in enumerate(tracks):
        if not track['valid']:
            continue
        
        if (i_track == 0) or (i_track == len(tracks) - 1):
            print(f' track {i_track}:')
            print(f'  track_{i_track}_p_inA has an initial value and is an optimized key')
        elif (i_track == 1):
            print('\n ...\n')
        initial_values[f'track_{i_track}_p_inA'] = np.zeros(3)              # <-- FIXME
        optimized_keys.append(f'track_{i_track}_p_inA')

        for match in track['matches']:
            view_id = match['view_id']
            feature_id = match['feature_id']
            if (i_track == 0) or (i_track == len(tracks) - 1):
                print(f'  track_{i_track}_b_{view_id} has an initial value and an sf_projection_residual factor')
            initial_values[f'track_{i_track}_b_{view_id}'] = np.zeros(2)    # <-- FIXME
            factors.append(Factor(
                residual=sf_projection_residual,
                keys=[
                    f'T_inB{view_id}_ofA',
                    f'track_{i_track}_p_inA',
                    f'track_{i_track}_b_{view_id}',
                    'fx',
                    'fy',
                    'cx',
                    'cy',
                    'epsilon',
                ],
            ))
    
    # Create optimizer
    optimizer = Optimizer(
        factors=factors,
        optimized_keys=optimized_keys,
        debug_stats=True,
        params=Optimizer.Params(
            iterations=100,
            use_diagonal_damping=True,
            lambda_down_factor=0.1,
            lambda_up_factor=5.,
            early_exit_min_reduction=1e-4,
        ),
    )

    return optimizer, initial_values






"""
FUNCTIONS THAT YOU DO NOT NEED TO MODIFY
"""

def myprint(M):
    """
    Prints either a scalar or a numpy array with four digits after the
    decimal and with consistent spacing so it is easier to read.
    """
    if M.shape:
        with np.printoptions(linewidth=150, formatter={'float': lambda x: f'{x:10.4f}'}):
            print(M)
    else:
        print(f'{M:10.4f}')


def apply_transform(R_inB_ofA, p_inB_ofA, p_inA):
    """
    Returns p_inB.
    """
    p_inB = np.row_stack([
        (R_inB_ofA @ p_inA_i + p_inB_ofA) for p_inA_i in p_inA
    ])
    return p_inB

def project(K, R_inB_ofA, p_inB_ofA, p_inA, warn=False):
    """
    Returns the projection (n x 2) of p_inA (n x 3) into an image
    taken from a calibrated camera at frame B.
    """
    p_inB = apply_transform(R_inB_ofA, p_inB_ofA, p_inA)
    if not np.all(p_inB[:, 2] > 0):
        if warn:
            print('WARNING: non-positive depths')
    q = np.row_stack([K @ p_inB_i / p_inB_i[2] for p_inB_i in p_inB])
    return q[:, 0:2]

def projection_error(K, R_inB_ofA, p_inB_ofA, p_inA, b, warn=False):
    """
    Returns the projection error - the difference between the projection
    of p_inA into an image taken from a calibrated camera at frame B, and
    the observed image coordinates b.

    If p_inA is n x 3 and b is n x 2, then the error is length n.

    If p_inA is length 3 and b is length 2, then the error is a scalar.
    """
    if len(b.shape) == 1:
        b_pred = project(K, R_inB_ofA, p_inB_ofA, np.reshape(p_inA, (1, -1)), warn=warn).flatten()
        return np.linalg.norm(b_pred - b)
    elif len(b.shape) == 2:
        b_pred = project(K, R_inB_ofA, p_inB_ofA, p_inA, warn=warn)
        return np.linalg.norm(b_pred - b, axis=1)
    else:
        raise Exception(f'b has bad shape: {b.shape}')
    
def skew(v):
    """
    Returns the 3 x 3 skew-symmetric matrix that corresponds
    to v, a vector of length 3.
    """
    assert(type(v) == np.ndarray)
    assert(v.shape == (3,))
    return np.array([[0., -v[2], v[1]],
                     [v[2], 0., -v[0]],
                     [-v[1], v[0], 0.]])

def show_results(views, tracks, K, show_pose_estimates=True, show_reprojection_errors=True):
    """
    Show the pose estimates (text) and reprojection errors (both text and plots)
    corresponding to views and tracks.
    """

    # Pose estimates
    if show_pose_estimates:
        print('POSE ESTIMATES')
        for i_view, view in enumerate(views):
            if (view['R_inB_ofA'] is None) or (view['p_inB_ofA'] is None):
                continue

            R_inA_ofB = view['R_inB_ofA'].T
            p_inA_ofB = - view['R_inB_ofA'].T @ view['p_inB_ofA']
            s = f' [R_inA_ofB{i_view}, p_inA_ofB{i_view}] = '
            s += np.array2string(
                np.column_stack([R_inA_ofB, p_inA_ofB]),
                formatter={'float': lambda x: f'{x:10.4f}'},
                prefix=s,
            )
            print(s)
    
    # Get reprojection errors
    e = [[] for view in views]
    for track in tracks:
        if not track['valid']:
            continue
        
        for match in track['matches']:
            view_id = match['view_id']
            feature_id = match['feature_id']
            view = views[view_id]
            e[view_id].append(
                projection_error(
                    K,
                    view['R_inB_ofA'],
                    view['p_inB_ofA'],
                    track['p_inA'],
                    view['pts'][feature_id]['pt2d'],
                )
            )
    
    # Show reprojection errors
    if show_reprojection_errors:
        print('\nREPROJECTION ERRORS')

        # Text
        for i_view, (e_i, view) in enumerate(zip(e, views)):
            if len(e_i) == 0:
                assert((view['R_inB_ofA'] is None) or (view['p_inB_ofA'] is None))
                continue

            assert(not ((view['R_inB_ofA'] is None) or (view['p_inB_ofA'] is None)))
            print(f' Image {i_view:2d} ({len(e_i):5d} points) : (mean, std, max, min) =' + \
                  f' ({np.mean(e_i):6.2f}, {np.std(e_i):6.2f}, {np.max(e_i):6.2f}, {np.min(e_i):6.2f})')
        
        # Figure
        bins = np.linspace(0, 5, 50)
        counts = [len(e_i) for e_i in e if len(e_i) > 0]
        max_count = np.max(counts)
        num_views = len(counts)
        num_cols = 3
        num_rows = (num_views // num_cols) + 1
        fig = plt.figure(figsize=(num_cols * 4, num_rows * 2), tight_layout=True)
        index = 0
        for i_view, e_i in enumerate(e):
            if len(e_i) == 0:
                continue

            index += 1
            ax = fig.add_subplot(num_rows, num_cols, index)
            ax.hist(e_i, bins, label=f'Image {i_view}')
            ax.set_xlim([bins[0], bins[-1]])
            ax.set_ylim([0, max_count])
            ax.legend()
            ax.grid()
        plt.show()

def is_duplicate_match(mA, matches):
    """
    Returns True if the match mA shares either a queryIdx or a trainIdx
    with any match in the list matches, False otherwise.
    """
    for mB in matches:
        if (mA.queryIdx == mB.queryIdx) or (mA.trainIdx == mB.trainIdx):
            return True
    return False

def get_good_matches(descA, descB, threshold=0.5):
    """
    Returns a list of matches that satisfy the ratio test with
    a given threshold. Makes sure these matches are unique - no
    feature in either image will be part of more than one match.
    """

    # Create a brute-force matcher
    bf = cv2.BFMatcher(
        normType=cv2.NORM_L2,
        crossCheck=False,       # <-- IMPORTANT - must be False for kNN matching
    )

    # Find the two best matches between descriptors
    matches = bf.knnMatch(descA, descB, k=2)

    # Find the subset of good matches
    good_matches = []
    for m, n in matches:
        if m.distance / n.distance < threshold:
            good_matches.append(m)

    # Sort the good matches by distance (smallest first)
    sorted_matches = sorted(good_matches, key = lambda m: m.distance)

    # VERY IMPORTANT - Eliminate duplicate matches
    unique_matches = []
    for sorted_match in sorted_matches:
        if not is_duplicate_match(sorted_match, unique_matches):
            unique_matches.append(sorted_match)
    
    # Return good matches, sorted by distance (smallest first)
    return unique_matches

def add_next_view(views, tracks, K, matching_threshold=0.5):
    """
    Updates views and tracks after matching the next available image.
    """
    iC = None
    for i_view, view in enumerate(views):
        if (view['R_inB_ofA'] is None) or (view['p_inB_ofA'] is None):
            iC = i_view
            break
    
    if iC is None:
        raise Exception('all views have been added')
    
    print(f'ADDING VIEW {iC}')

    for iB in range(0, iC):

        viewB = views[iB]
        viewC = views[iC]

        # Get good matches
        matches = get_good_matches(viewB['desc'], viewC['desc'], threshold=matching_threshold)
        num_matches = len(matches)
        print('')
        print(f'matching image {iB} with image {iC} with threshold {matching_threshold}:')
        print(f' {num_matches:4d} good matches found')
        
        num_created = 0
        num_added_to_B = 0
        num_added_to_C = 0
        num_merged_trivial = 0
        num_merged_nontrivial = 0
        num_invalid = 0

        for m in matches:
            # Get the corresponding points
            ptB = viewB['pts'][m.queryIdx]
            ptC = viewC['pts'][m.trainIdx]

            # Get the corresponding tracks (if they exist)
            trackB = ptB['track']
            trackC = ptC['track']

            # Create, extend, or merge tracks
            if (trackC is None) and (trackB is None):
                num_created += 1
                # Create a new track
                track = {
                    'p_inA': None,
                    'valid': True,
                    'matches': [
                        {'view_id': iB, 'feature_id': m.queryIdx},
                        {'view_id': iC, 'feature_id': m.trainIdx},
                    ],
                }
                tracks.append(track)
                ptB['track'] = track
                ptC['track'] = track
            elif (trackC is not None) and (trackB is None):
                num_added_to_C += 1
                # Add ptB to trackC
                track = trackC
                trackC['matches'].append({'view_id': iB, 'feature_id': m.queryIdx})
                ptB['track'] = track
            elif (trackC is None) and (trackB is not None):
                num_added_to_B += 1
                # Add ptC to trackB
                track = trackB
                trackB['matches'].append({'view_id': iC, 'feature_id': m.trainIdx})
                ptC['track'] = track
            elif (trackC is not None) and (trackB is not None):
                
                # If trackB and trackC are identical, then nothing further needs to be done
                if trackB is trackC:
                    num_merged_trivial += 1
                    s = f'       trivial merge - ({iB:2d}, {m.queryIdx:4d}) ({iC:2d}, {m.trainIdx:4d}) - '
                    for track_m in trackB['matches']:
                        s += f'({track_m["view_id"]:2d}, {track_m["feature_id"]:4d}) '
                    print(s)
                    continue
                
                num_merged_nontrivial += 1

                s = f'       non-trivial merge - ({iB:2d}, {m.queryIdx:4d}) ({iC:2d}, {m.trainIdx:4d})\n'
                s += '                           '
                for track_m in trackB['matches']:
                    s += f'({track_m["view_id"]:2d}, {track_m["feature_id"]:4d}) '
                s += '\n'
                s += '                           '
                for track_m in trackC['matches']:
                    s += f'({track_m["view_id"]:2d}, {track_m["feature_id"]:4d}) '
                print(s)

                # Merge
                # - triangulated point
                if trackB['p_inA'] is None:
                    if trackC['p_inA'] is None:
                        p_inA = None
                    else:
                        p_inA = trackC['p_inA']
                else:
                    if trackC['p_inA'] is None:
                        p_inA = trackB['p_inA']
                    else:
                        # FIXME: May want to re-triangulate rather than averaging
                        p_inA = 0.5 * (trackB['p_inA'] + trackC['p_inA'])
                trackC['p_inA'] = p_inA
                # - valid
                valid = trackB['valid'] and trackC['valid']
                trackC['valid'] = valid
                # - matches and points
                iters = 0
                for trackB_m in trackB['matches']:
                    iters += 1
                    if iters > 10:
                        raise Exception('iters exceeded maximum')
                    # Don't add duplicate matches to track (this would happen if we closed a loop)
                    if (trackB_m['view_id'] != iC) or (trackB_m['feature_id'] != m.trainIdx):
                        trackC['matches'].append(trackB_m)
                        views[trackB_m['view_id']]['pts'][trackB_m['feature_id']]['track'] = trackC
                track = trackC
                
                s = '                           '
                for track_m in track['matches']:
                    s += f'({track_m["view_id"]:2d}, {track_m["feature_id"]:4d}) '
                s += f' - {str(track["valid"])}'
                print(s)

            else:
                raise Exception('Should never get here!')
            
            # Check if track is self-consistent (i.e., check that it does not contain two
            # matches from the same image)
            view_ids = [track_m['view_id'] for track_m in track['matches']]
            if len(set(view_ids)) != len(view_ids):
                num_invalid += 1
                track['valid'] = False
                s = f'       FOUND INCONSISTENT - '
                for track_m in track['matches']:
                    s += f'({track_m["view_id"]:2d}, {track_m["feature_id"]:4d}) '
                print(s)

        print(f' {num_created:4d} tracks created')
        print(f' {num_added_to_C:4d} tracks in C extended with point in B')
        print(f' {num_added_to_B:4d} tracks in B extended with point in C')
        print(f' {num_merged_trivial:4d} tracks merged (trivial)')
        print(f' {num_merged_trivial:4d} tracks merged (non-trivial)')
        print(f' {num_invalid:4d} inconsistent tracks')
    
    return iC

def store_results(views, tracks, K, result, max_reprojection_err=1.):
    """
    Updates views and tracks given the result from optimization.
    """

    # Get pose estimates
    num_views = 0
    for i_view, view in enumerate(views):
        if (view['R_inB_ofA'] is None) or (view['p_inB_ofA'] is None):
            continue

        T_inB_ofA = result.optimized_values[f'T_inB{i_view}_ofA'].to_homogenous_matrix()
        R_inB_ofA = T_inB_ofA[0:3, 0:3]
        p_inB_ofA = T_inB_ofA[0:3, 3]
        view['R_inB_ofA'] = R_inB_ofA
        view['p_inB_ofA'] = p_inB_ofA
        num_views += 1

    # Get position estimates
    num_invalid_old = 0
    num_invalid_new = 0
    num_valid = 0
    for i_track, track in enumerate(tracks):
        if not track['valid']:
            num_invalid_old += 1
            continue
        
        p_inA = result.optimized_values[f'track_{i_track}_p_inA']
        track['p_inA'] = p_inA
        valid = track['valid']
        for match in track['matches']:
            view_id = match['view_id']
            feature_id = match['feature_id']
            view = views[view_id]
            R_inB_ofA = view['R_inB_ofA']
            p_inB_ofA = view['p_inB_ofA']
            p_inB = R_inB_ofA @ p_inA + p_inB_ofA
            b = views[view_id]['pts'][feature_id]['pt2d']
            e = projection_error(K, R_inB_ofA, p_inB_ofA, p_inA, b)
            
            # Remain valid if depth is positive
            valid = valid and p_inB[2] > 0.
            
            # Remain valid if reprojection error is below threshold
            valid = valid and e < max_reprojection_err
        
        track['valid'] = valid
        if valid:
            num_valid += 1
        else:
            num_invalid_new += 1

    # Show diagnostics
    print(f'{num_views:6d} views with updated pose estimate')
    print(f'{num_valid:6d} valid tracks with updated position estimate')
    print(f'{num_invalid_old:6d} already invalid tracks')
    print(f'{num_invalid_new:6d} newly invalid tracks')

def get_match_with_view_id(matches, view_id):
    """
    From a list of matches, return the one with the given view_id.
    """
    for match in matches:
        if match['view_id'] == view_id:
            return match
    return None

def get_pt2d_from_match(views, match):
    """
    Get image coordinates of the given match.
    """
    return views[match['view_id']]['pts'][match['feature_id']]['pt2d']

def sf_projection(
    T_inC_ofW: sf.Pose3,
    p_inW: sf.V3,
    fx: sf.Scalar,
    fy: sf.Scalar,
    cx: sf.Scalar,
    cy: sf.Scalar,
    epsilon: sf.Scalar,
) -> sf.V2:
    """
    Symbolic function that projects a point into an image. (If the depth
    of this point is non-positive, then the projection will be pushed far
    away from the image center.)
    """
    p_inC = T_inC_ofW * p_inW
    z = sf.Max(p_inC[2], epsilon)   # <-- if depth is non-positive, then projection
                                    #     will be pushed far away from image center
    return sf.V2(
        fx * (p_inC[0] / z) + cx,
        fy * (p_inC[1] / z) + cy,
    )

def sf_projection_residual(
    T_inC_ofW: sf.Pose3,
    p_inW: sf.V3,
    q: sf.V2,
    fx: sf.Scalar,
    fy: sf.Scalar,
    cx: sf.Scalar,
    cy: sf.Scalar,
    epsilon: sf.Scalar,  
) -> sf.V2:
    """
    Symbolic function that computes the difference between a projected point
    and an image point. This error is "whitened" so that taking its norm will
    be equivalent to applying a robust loss function (Geman-McClure).
    """
    q_proj = sf_projection(T_inC_ofW, p_inW, fx, fy, cx, cy, epsilon)
    unwhitened_residual = sf.V2(q_proj - q)
    
    noise_model = BarronNoiseModel(
        alpha=-2,
        scalar_information=1,
        x_epsilon=epsilon,
        alpha_epsilon=epsilon,
    )
    
    return noise_model.whiten_norm(unwhitened_residual)

def sf_scale_residual(
    T_inC_ofW: sf.Pose3,
    epsilon: sf.Scalar,
) -> sf.V1:
    """
    Symbolic function that computes the relative distance between two frames.
    """
    return sf.V1(T_inC_ofW.t.norm() - 1)







"""
MY OWN INTERNAL FUNCTIONS
"""

# Define a function to get the similarity transformation that normalizes points
def get_transformation(p):
    # The input should be an ndarray with shape (n, 2)
    # where n is the number of points
    assert(p.shape[1] == 2)

    # Get centroid of points
    c = np.mean(p, 0)

    # Get mean distance of points from centroid
    d = np.mean(np.linalg.norm(p - c, axis=1))

    # Create and return transformation matrix
    return np.linalg.inv(
            np.array(
                [[d / np.sqrt(2), 0., c[0]],
                 [0., d / np.sqrt(2), c[1]],
                 [0., 0., 1.]]
            )
        )

# Define a function to get the similarity transformation that normalizes points
def get_transformation_3d(p):
    # The input should be an ndarray with shape (n, 3)
    # where n is the number of points
    assert(p.shape[1] == 3)

    # Get centroid of points
    c = np.mean(p, 0)

    # Get mean distance of points from centroid
    d = np.mean(np.linalg.norm(p - c, axis=1))

    # Create and return transformation matrix
    return np.linalg.inv(
            np.row_stack([
                np.column_stack([np.diag([d, d, d]), c]),
                np.array([0., 0., 0., 1.])
            ])
        )

def _resection(p_inA, c, K):
    # Normalize image coordinates
    K_inv = np.linalg.inv(K)
    gamma = np.row_stack([K_inv @ np.concatenate([c_i, [1.]]) for c_i in c])

    # Do preconditioning
    # - get transformation
    T_2d = get_transformation(gamma[:, 0:2])
    T_3d = get_transformation_3d(p_inA)
    # - apply transformation
    gamma_n = np.array([T_2d @ gamma_i for gamma_i in gamma])
    p_inA_n = np.array([T_3d @ np.concatenate([p_inA_i, [1.]]) for p_inA_i in p_inA])[:, 0:3]
    
    # # Find solution
    # M = np.row_stack([
    #     np.column_stack([
    #         np.kron(p_inA_i, skew(gamma_i)), skew(gamma_i)
    #     ]) for p_inA_i, gamma_i in zip(p_inA_n, gamma_n)
    # ])
    # U, S, V_T = np.linalg.svd(M)
    # R_inC_ofA = np.reshape(V_T[-1, 0:9], (3, 3), 'F')
    # p_inC_ofA = V_T[-1, 9:12]

    # # Correct solution
    # # - Should have the right scale
    # m = np.linalg.norm(R_inC_ofA[:, 0])
    # R_inC_ofA /= m
    # p_inC_ofA /= m
    # # - Should be right-handed
    # m = np.linalg.det(R_inC_ofA)
    # R_inC_ofA *= m
    # p_inC_ofA *= m
    # # - Should be a rotation matrix
    # U, S, V_T = np.linalg.svd(R_inC_ofA)
    # R_inC_ofA = np.linalg.det(U @ V_T) * (U @ V_T)

    # Find solution
    M = np.row_stack([
        np.column_stack([
            np.kron(p_inA_i, skew(gamma_i)), skew(gamma_i)
        ]) for p_inA_i, gamma_i in zip(p_inA_n, gamma_n)
    ])
    U, S, V_T = np.linalg.svd(M)
    P_n = np.reshape(V_T[-1, :], (3, 4), 'F')

    # Do postconditioning
    P = np.linalg.inv(T_2d) @ P_n @ T_3d

    # Decompose and correct solution
    # - Decompose
    R_inC_ofA = P[:, 0:3]
    p_inC_ofA = P[:, 3]
    # - Should have the right scale
    m = np.linalg.norm(R_inC_ofA[:, 0])
    R_inC_ofA /= m
    p_inC_ofA /= m
    # - Should be right-handed
    m = np.linalg.det(R_inC_ofA)
    R_inC_ofA *= m
    p_inC_ofA *= m
    # - Should be a rotation matrix
    U, S, V_T = np.linalg.svd(R_inC_ofA)
    R_inC_ofA = np.linalg.det(U @ V_T) * (U @ V_T)

    return R_inC_ofA, p_inC_ofA

def _resection_without_normalization(p_inA, c, K):
    # Normalize image coordinates
    K_inv = np.linalg.inv(K)
    gamma = np.row_stack([K_inv @ np.concatenate([c_i, [1.]]) for c_i in c])

    # Find solution
    M = np.row_stack([
        np.column_stack([
            np.kron(p_inA_i, skew(gamma_i)), skew(gamma_i)
        ]) for p_inA_i, gamma_i in zip(p_inA, gamma)
    ])
    U, S, V_T = np.linalg.svd(M)
    P = np.reshape(V_T[-1, :], (3, 4), 'F')

    # Decompose and correct solution
    # - Decompose
    R_inC_ofA = P[:, 0:3]
    p_inC_ofA = P[:, 3]
    # - Should have the right scale
    m = np.linalg.norm(R_inC_ofA[:, 0])
    R_inC_ofA /= m
    p_inC_ofA /= m
    # - Should be right-handed
    m = np.linalg.det(R_inC_ofA)
    R_inC_ofA *= m
    p_inC_ofA *= m
    # - Should be a rotation matrix
    U, S, V_T = np.linalg.svd(R_inC_ofA)
    R_inC_ofA = np.linalg.det(U @ V_T) * (U @ V_T)

    return R_inC_ofA, p_inC_ofA

def epipolar_distance(E, alpha, beta):
    l_B = E @ alpha
    d_B = beta.T @ l_B / np.linalg.norm(l_B[0:2])
    l_A = E.T @ beta
    d_A = alpha.T @ l_A / np.linalg.norm(l_A[0:2])
    return np.abs(d_A), np.abs(d_B)

def epipolar_distance_3dv(E, alpha, beta):
    e3 = np.array([0., 0., 1.])
    l_B = E @ alpha
    d_B = beta.T @ l_B / np.linalg.norm(skew(e3) @ l_B)
    l_A = E.T @ beta
    d_A = alpha.T @ l_A / np.linalg.norm(skew(e3) @ l_A)
    return np.abs(d_A), np.abs(d_B)

def epipolar_distances(E, alpha, beta):
    d_A = []
    d_B = []
    for alpha_i, beta_i in zip(alpha, beta):
        d_A_i, d_B_i = epipolar_distance(E, alpha_i, beta_i)
        d_A.append(d_A_i)
        d_B.append(d_B_i)
    return np.array(d_A), np.array(d_B)

def epipolar_distances_unnormalized(E, a, b, K):
    K_inv = np.linalg.inv(K)
    alpha = np.row_stack([K_inv @ np.concatenate([a_i, [1.]]) for a_i in a])
    beta = np.row_stack([K_inv @ np.concatenate([b_i, [1.]]) for b_i in b])
    return epipolar_distances(E, alpha, beta)

def sampson_distance(E, alpha, beta):
    l_A = E.T @ beta
    l_B = E @ alpha
    return (beta.T @ E @ alpha)**2 / (l_A[0]**2 + l_A[1]**2 + l_B[0]**2 + l_B[1]**2)

def sampson_distances(E, alpha, beta):
    return np.array([sampson_distance(E, alpha_i, beta_i) for alpha_i, beta_i in zip(alpha, beta)])

def sampson_distances_unnormalized(E, a, b, K):
    K_inv = np.linalg.inv(K)
    alpha = np.row_stack([K_inv @ np.concatenate([a_i, [1.]]) for a_i in a])
    beta = np.row_stack([K_inv @ np.concatenate([b_i, [1.]]) for b_i in b])
    return sampson_distances(E, alpha, beta)

def twoview_triangulate(alpha, beta, R_inB_ofA, p_inB_ofA):
    p_inA = []
    p_inB = []
    for alpha_i, beta_i in zip(alpha, beta):
        M = np.row_stack([
            skew(alpha_i),
            skew(beta_i) @ R_inB_ofA,
        ])
        n = np.concatenate([
            np.zeros(3),
            -skew(beta_i) @ p_inB_ofA,
        ])
        p_inA_i = np.linalg.pinv(M) @ n
        p_inB_i = R_inB_ofA @ p_inA_i + p_inB_ofA
        p_inA.append(p_inA_i)
        p_inB.append(p_inB_i)
    p_inA = np.array(p_inA)
    p_inB = np.array(p_inB)
    
    # Get mask
    mask = np.bitwise_and(p_inA[:, 2] > 0, p_inB[:, 2] > 0)

    return p_inA, p_inB, mask

def _getE(alpha, beta):
    # Estimate essential matrix
    M = np.row_stack([np.kron(alpha_i, beta_i) for alpha_i, beta_i in zip(alpha, beta)])
    assert(M.shape == (len(alpha), 9))
    U, S, V_T = np.linalg.svd(M)
    E = np.reshape(V_T[-1, :], (3, 3), order='F')
    # Normalize essential matrix
    E *= (np.sqrt(2) / np.linalg.norm(E))
    # Project estimate onto essential space
    U, S, V_T = np.linalg.svd(E)
    # - Change singular values
    S = np.diag([1., 1., 0.])
    # - Ensure that U and V_T are rotation matrices
    U[:, 2] *= np.linalg.det(U)
    V_T[2, :] *= np.linalg.det(V_T)
    assert(np.isclose(np.linalg.det(U), 1.))
    assert(np.isclose(np.linalg.det(V_T), 1.))
    # - Reconstruct E
    E = U @ S @ V_T
    return E