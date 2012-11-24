

/* DEFAULT
// number of frames between keyframe-adds.
// original 20 (assuming 30hz camera)
#define FRAMES_BETWEEN_KEYFRAMES 20


// distance between two keyframes (additional factor).
// is not an absolute distance anyway, depends inversely on mean depth of current keyframe...
#define MIN_DIST_BETWEEN_KEYFRAMES 1

#define TRACKER_COARSE_MIN_DEFAULT 20
#define TRACKER_COARSE_MAX_DEFAULT 60
#define TRACKER_COARSE_RANGE_DEFAULT 30
#define TRACKER_COARSE_SUBPIXELITS_DEFAULT 8
#define TRACKER_COARSE_DISABLE_DEFAULT 0
#define TRACKER_COARSE_MIN_VELOCITY_DEFAULT 0.006
#define TRACKER_DRAW_FAST_CORNERS_DEFAULT 1
#define TRACKER_MAX_PATCHES_PER_FRAME_DEFAULT 1000
#define TRACKER_MINIPATCH_MAX_SSD_DEFAULT 1000000
#define TRACKER_QUALITY_GOOD_DEFAULT 0.3
#define TRACKER_QUALITY_LOST_DEFAULT 0.13
#define TRACKER_M_ESTIMATOR_DEFAULT "Tukey"		// choices are Tukey, Cauchy, Huber
#define TRACKER_ROTATION_ESTIMATOR_BLUR 0.75


#define BUNDLE_MAX_ITERATIONS 20
#define BUNDLE_UPDATE_SQUARED_CONV_LIMIT 1e-006
#define BUNDLE_COUT 0
#define BUNDLE_MIN_TUKEY_SIGMA 0.4
#define BUNDLE_M_ESTIMATOR "Tukey"		// choices are Tukey, Cauchy, Huber

#define MAPMAKER_MIN_SHI_THOMASI_SCORE 70
*/



// number of frames between keyframe-adds.
// original 20 (assuming 30hz camera)
#define FRAMES_BETWEEN_KEYFRAMES 10


// distance between two keyframes (additional factor).
// is not an absolute distance anyway, depends inversely on mean depth of current keyframe...
#define MIN_DIST_BETWEEN_KEYFRAMES 1

#define TRACKER_COARSE_MIN_DEFAULT 20
#define TRACKER_COARSE_MAX_DEFAULT 60
#define TRACKER_COARSE_RANGE_DEFAULT 30
#define TRACKER_COARSE_SUBPIXELITS_DEFAULT 4
#define TRACKER_COARSE_DISABLE_DEFAULT 0
#define TRACKER_COARSE_MIN_VELOCITY_DEFAULT 0.006
#define TRACKER_DRAW_FAST_CORNERS_DEFAULT 0
#define TRACKER_MAX_PATCHES_PER_FRAME_DEFAULT 1000
#define TRACKER_MINIPATCH_MAX_SSD_DEFAULT 1000000
#define TRACKER_QUALITY_GOOD_DEFAULT 0.3
#define TRACKER_QUALITY_LOST_DEFAULT 0.15
#define TRACKER_M_ESTIMATOR_DEFAULT "Tukey"		// choices are Tukey, Cauchy, Huber
#define TRACKER_ROTATION_ESTIMATOR_BLUR 0.75


#define BUNDLE_MAX_ITERATIONS 20
#define BUNDLE_UPDATE_SQUARED_CONV_LIMIT 1e-006
#define BUNDLE_COUT 0
#define BUNDLE_MIN_TUKEY_SIGMA 0.4
#define BUNDLE_M_ESTIMATOR "Tukey"		// choices are Tukey, Cauchy, Huber

#define MAPMAKER_MIN_SHI_THOMASI_SCORE 70