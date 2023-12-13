
#define RTABMAP_PARAM_COND(PREFIX, NAME, TYPE, COND, DEFAULT_VALUE1, DEFAULT_VALUE2, DESCRIPTION) \
*         int width = Parameters::defaultVideoImageWidth(); // theDefaultValue = 640
*         std::string theKey = Parameters::kVideoImageWidth(); // theKey = "Video/ImageWidth"
*         std::string strValue = Util::value(Parameters::getDefaultParameters(), theKey); // strValue = "640"
RTABMAP_PARAM(Rtabmap, MaxRepublished,           unsigned int, 2, uFormat("Maximum nodes republished when requesting missing data. When %s = false, only loop closure data is republished, otherwise the closest nodes from the current localization are republished first. Ignored if %s=false.", kRGBDEnabled().c_str(), kRtabmapPublishLastSignature().c_str()));
RTABMAP_PARAM(Mem, LocalizationDataSaved,       bool, false,     uFormat("Save localization data during localization session (when %s = false). When enabled, the database will then also grow in localization mode. This mode would be used only for debugging purpose.", kMemIncrementalMemory().c_str()).c_str());
RTABMAP_PARAM(Mem, GenerateIds,                 bool, true,     "True = Generate location IDs, False=use input image IDs.");
RTABMAP_PARAM(Kp, NNStrategy,               int, 1,       "kNNFlannNaive = 0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4");
RTABMAP_PARAM(Kp, FlannRebalancingFactor,   float, 2.0,   uFormat("Factor used when rebuilding the incremental FLANN index (see \"%s\"). Set < = 1 to disable.", kKpIncrementalFlann().c_str()));
RTABMAP_PARAM(Kp, ByteToFloat,              bool, false,  uFormat("For %s = 1, binary descriptors are converted to float by converting each byte to float instead of converting each bit to float. When converting bytes instead of bits, less memory is used and search is faster at the cost of slightly less accurate matching.", kKpNNStrategy().c_str()));
RTABMAP_PARAM(Kp, MaxDepth,                 float, 0,     "Filter extracted keypoints by depth (0 = inf).");
RTABMAP_PARAM(Kp, BadSignRatio,             float, 0.5,   "Bad signature ratio (less than Ratio x AverageWordsPerImage = bad).");

#if CV_MAJOR_VERSION > 2 && !defined(HAVE_OPENCV_XFEATURES2D)
RTABMAP_PARAM(Kp, DetectorStrategy,         int, 8,       "0 = SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");

#else
RTABMAP_PARAM(Kp, DetectorStrategy,         int, 6,       "0 = SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");

#endif
RTABMAP_PARAM(DbSqlite3, JournalMode,  int, 3,           "0 = DELETE, 1=TRUNCATE, 2=PERSIST, 3=MEMORY, 4=OFF (see sqlite3 doc : \"PRAGMA journal_mode\")");
RTABMAP_PARAM(DbSqlite3, Synchronous,  int, 0,           "0 = OFF, 1=NORMAL, 2=FULL (see sqlite3 doc : \"PRAGMA synchronous\")");
RTABMAP_PARAM(DbSqlite3, TempStore,    int, 2,           "0 = DEFAULT, 1=FILE, 2=MEMORY (see sqlite3 doc : \"PRAGMA temp_store\")");
RTABMAP_PARAM(ORB, ScaleFactor,   float, 2,  "Pyramid decimation ratio, greater than 1. scaleFactor = =2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.");
RTABMAP_PARAM(ORB, WTA_K,         int, 2,      "The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K = 4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).");
RTABMAP_PARAM(ORB, ScoreType,     int, 0,      "The default HARRIS_SCORE = 0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.");
RTABMAP_PARAM(KAZE, Diffusivity,    int, 1,        "Diffusivity type: 0 = DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER.");
RTABMAP_PARAM(SuperPoint, NMSRadius,     int,  4,      uFormat("[%s = true] Minimum distance (pixels) between keypoints.", kSuperPointNMS().c_str()));
RTABMAP_PARAM_STR(Bayes, PredictionLC, "0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23", "Prediction of loop closures (Gaussian-like, here with sigma = 1.6) - Format: {VirtualPlaceProb, LoopClosureProb, NeighborLvl1, NeighborLvl2, ...}.");
RTABMAP_PARAM(RGBD, NewMapOdomChangeDistance, float, 0,    "A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled).");
RTABMAP_PARAM(RGBD, OptimizeMaxError,         float, 3.0,   uFormat("Reject loop closures if optimization error ratio is greater than this value (0 = disabled). Ratio is computed as absolute error over standard deviation of each link. This will help to detect when a wrong loop closure is added to the graph. Not compatible with \"%s\" if enabled.", kOptimizerRobust().c_str()));
RTABMAP_PARAM(RGBD, MaxLoopClosureDistance,   float, 0.0,   "Reject loop closures/localizations if the distance from the map is over this distance (0 = disabled).");
RTABMAP_PARAM(RGBD, StartAtOrigin,            bool, false, uFormat("If true, rtabmap will assume the robot is starting from origin of the map. If false, rtabmap will assume the robot is restarting from the last saved localization pose from previous session (the place where it shut down previously). Used only in localization mode (%s = false).", kMemIncrementalMemory().c_str()));
RTABMAP_PARAM(RGBD, PlanStuckIterations,      int, 0,      "Mark the current goal node on the path as unreachable if it is not updated after X iterations (0 = disabled). If all upcoming nodes on the path are unreachabled, the plan fails.");
RTABMAP_PARAM(RGBD, MaxLocalRetrieved,        unsigned int, 2, "Maximum local locations retrieved (0 = disabled) near the current pose in the local map or on the current planned path (those on the planned path have priority).");
RTABMAP_PARAM(RGBD, MaxOdomCacheSize,             int,  10,      uFormat("Maximum odometry cache size. Used only in localization mode (when %s = false). This is used to get smoother localizations and to verify localization transforms (when %s!=0) to make sure we don't teleport to a location very similar to one we previously localized on. Set 0 to disable caching.", kMemIncrementalMemory().c_str(), kRGBDOptimizeMaxError().c_str()));
RTABMAP_PARAM(RGBD, ProximityGlobalScanMap,       bool, false, uFormat("Create a global assembled map from laser scans for one-to-many proximity detection, replacing the original one-to-many proximity detection (i.e., detection against local paths). Only used in localization mode (%s = false), otherwise original one-to-many proximity detection is done. Note also that if graph is modified (i.e., memory management is enabled or robot jumps from one disjoint session to another in same database), the global scan map is cleared and one-to-many proximity detection is reverted to original approach.", kMemIncrementalMemory().c_str()));

#ifdef RTABMAP_GTSAM
RTABMAP_PARAM(Optimizer, Strategy,        int, 2,          "Graph optimization strategy: 0 = TORO, 1=g2o, 2=GTSAM and 3=Ceres.");

#else
#ifdef RTABMAP_G2O
RTABMAP_PARAM(Optimizer, Strategy,        int, 1,          "Graph optimization strategy: 0 = TORO, 1=g2o, 2=GTSAM and 3=Ceres.");

#else
#ifdef RTABMAP_CERES
RTABMAP_PARAM(Optimizer, Strategy,        int, 3,          "Graph optimization strategy: 0 = TORO, 1=g2o, 2=GTSAM and 3=Ceres.");

#else
RTABMAP_PARAM(Optimizer, Strategy,        int, 0,          "Graph optimization strategy: 0 = TORO, 1=g2o, 2=GTSAM and 3=Ceres.");

#if defined(RTABMAP_G2O) || defined(RTABMAP_GTSAM)
RTABMAP_PARAM(Optimizer, GravitySigma,    float, 0.3,      uFormat("Gravity sigma value (> = 0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches, for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints. Currently supported only with g2o and GTSAM optimization strategies (see %s).", kOptimizerStrategy().c_str()));

#else
RTABMAP_PARAM(Optimizer, GravitySigma,    float, 0.0,      uFormat("Gravity sigma value (> = 0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches, for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints. Currently supported only with g2o and GTSAM optimization strategies (see %s).", kOptimizerStrategy().c_str()));

#endif

#ifdef RTABMAP_ORB_SLAM
RTABMAP_PARAM(g2o, Solver,            int, 3,          "0 = csparse 1=pcg 2=cholmod 3=Eigen");

#else
RTABMAP_PARAM(g2o, Solver,            int, 0,          "0 = csparse 1=pcg 2=cholmod 3=Eigen");

#endif
RTABMAP_PARAM(g2o, Optimizer,         int, 0,          "0 = Levenberg 1=GaussNewton");
RTABMAP_PARAM(GTSAM, Optimizer,       int, 1,          "0 = Levenberg 1=GaussNewton 2=Dogleg");
RTABMAP_PARAM(Odom, Strategy,               int, 0,       "0 = Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D");
RTABMAP_PARAM(Odom, ResetCountdown,         int, 0,       "Automatically reset odometry after X consecutive images on which odometry cannot be computed (value = 0 disables auto-reset).");
RTABMAP_PARAM(Odom, Holonomic,              bool, true,   "If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y = x*tan(yaw)).");
RTABMAP_PARAM(Odom, FilteringStrategy,      int, 0,       "0 = No filtering 1=Kalman filtering 2=Particle filtering. This filter is used to smooth the odometry output.");

#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
RTABMAP_PARAM(OdomF2M, BundleAdjustment,          int, 1, "Local bundle adjustment: 0 = disabled, 1=g2o, 2=cvsba, 3=Ceres.");

#else
RTABMAP_PARAM(OdomF2M, BundleAdjustment,          int, 0, "Local bundle adjustment: 0 = disabled, 1=g2o, 2=cvsba, 3=Ceres.");

#endif
RTABMAP_PARAM(OdomF2M, BundleAdjustmentMaxFrames, int, 10, "Maximum frames used for bundle adjustment (0 = inf or all current frames in the local map).");
RTABMAP_PARAM(OdomViso2, MatchRefinement,           int, 1,      "Refinement (0 = none,1=pixel,2=subpixel).");
RTABMAP_PARAM(OdomLOAM, Sensor,     int,    2,    "Velodyne sensor: 0 = VLP-16, 1=HDL-32, 2=HDL-64E");
RTABMAP_PARAM(OdomOpenVINS, Integration,               int,    1,      "0 = discrete, 1=rk4, 2=analytical (if rk4 or analytical used then analytical covariance propagation is used)");
RTABMAP_PARAM(OdomOpen3D, Method,           int, 0,  "Registration method: 0 = PointToPlane, 1=Intensity, 2=Hybrid.");
RTABMAP_PARAM(Reg, Strategy,                 int, 0,        "0 = Vis, 1=Icp, 2=VisIcp");
RTABMAP_PARAM(Vis, InlierDistance,           float, 0.1,    uFormat("[%s = 0] Maximum distance for feature correspondences. Used by 3D->3D estimation approach.", kVisEstimationType().c_str()));
RTABMAP_PARAM(Vis, RefineIterations,         int, 5,        uFormat("[%s = 0] Number of iterations used to refine the transformation found by RANSAC. 0 means that the transformation is not refined.", kVisEstimationType().c_str()));
RTABMAP_PARAM(Vis, PnPReprojError,           float, 2,      uFormat("[%s = 1] PnP reprojection error.", kVisEstimationType().c_str()));
RTABMAP_PARAM(Vis, PnPFlags,                 int, 0,        uFormat("[%s = 1] PnP flags: 0=Iterative, 1=EPNP, 2=P3P", kVisEstimationType().c_str()));

#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
RTABMAP_PARAM(Vis, PnPRefineIterations,      int, 0,        uFormat("[%s = 1] Refine iterations. Set to 0 if \"%s\" is also used.", kVisEstimationType().c_str(), kVisBundleAdjustment().c_str()));

#else
RTABMAP_PARAM(Vis, PnPRefineIterations,      int, 1,        uFormat("[%s = 1] Refine iterations. Set to 0 if \"%s\" is also used.", kVisEstimationType().c_str(), kVisBundleAdjustment().c_str()));

#endif
RTABMAP_PARAM(Vis, PnPVarianceMedianRatio,   int, 4,        uFormat("[%s = 1] Ratio used to compute variance of the estimated transformation if 3D correspondences are provided (should be > 1). The higher it is, the smaller the covariance will be. With accurate depth estimation, this could be set to 2. For depth estimated by stereo, 4 or more maybe used to ignore large errors of very far points.", kVisEstimationType().c_str()));
RTABMAP_PARAM(Vis, PnPMaxVariance,           float, 0.0,    uFormat("[%s = 1] Max linear variance between 3D point correspondences after PnP. 0 means disabled.", kVisEstimationType().c_str()));
RTABMAP_PARAM(Vis, PnPSamplingPolicy,      unsigned int, 1, uFormat("[%s = 1] Multi-camera random sampling policy: 0=AUTO, 1=ANY, 2=HOMOGENEOUS. With HOMOGENEOUS policy, RANSAC will be done uniformly against all cameras, so at least 2 matches per camera are required. With ANY policy, RANSAC is not constraint to sample on all cameras at the same time. AUTO policy will use HOMOGENEOUS if there are at least 2 matches per camera, otherwise it will fallback to ANY policy.", kVisEstimationType().c_str()).c_str());
RTABMAP_PARAM(Vis, EpipolarGeometryVar,      float, 0.1,    uFormat("[%s = 2] Epipolar geometry maximum variance to accept the transformation.", kVisEstimationType().c_str()));

#if CV_MAJOR_VERSION > 2 && !defined(HAVE_OPENCV_XFEATURES2D)
RTABMAP_PARAM(Vis, FeatureType, int, 8, "0 = SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");

#else
RTABMAP_PARAM(Vis, FeatureType, int, 6, "0 = SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");

#endif
RTABMAP_PARAM(Vis, CorType,                  int, 0,      "Correspondences computation approach: 0 = Features Matching, 1=Optical Flow");
RTABMAP_PARAM(Vis, CorNNType,                int, 1,    uFormat("[%s = 0] kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7. Used for features matching approach.", kVisCorType().c_str()));
RTABMAP_PARAM(Vis, CorNNDR,                  float, 0.8,  uFormat("[%s = 0] NNDR: nearest neighbor distance ratio. Used for knn features matching approach.", kVisCorType().c_str()));
RTABMAP_PARAM(Vis, CorGuessWinSize,          int, 40,     uFormat("[%s = 0] Matching window size (pixels) around projected points when a guess transform is provided to find correspondences. 0 means disabled.", kVisCorType().c_str()));
RTABMAP_PARAM(Vis, CorGuessMatchToProjection, bool, false, uFormat("[%s = 0] Match frame's corners to source's projected points (when guess transform is provided) instead of projected points to frame's corners.", kVisCorType().c_str()));
RTABMAP_PARAM(Vis, CorFlowWinSize,           int, 16,     uFormat("[%s = 1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));
RTABMAP_PARAM(Vis, CorFlowIterations,        int, 30,     uFormat("[%s = 1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));
RTABMAP_PARAM(Vis, CorFlowEps,               float, 0.01, uFormat("[%s = 1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));
RTABMAP_PARAM(Vis, CorFlowMaxLevel,          int, 3,      uFormat("[%s = 1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));

#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
RTABMAP_PARAM(Vis, BundleAdjustment,         int, 1,      "Optimization with bundle adjustment: 0 = disabled, 1=g2o, 2=cvsba, 3=Ceres.");

#else
RTABMAP_PARAM(Vis, BundleAdjustment,         int, 0,      "Optimization with bundle adjustment: 0 = disabled, 1=g2o, 2=cvsba, 3=Ceres.");

#ifdef RTABMAP_POINTMATCHER
RTABMAP_PARAM(Icp, Strategy,                  int, 1,       "ICP implementation: 0 = Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).");

#else
RTABMAP_PARAM(Icp, Strategy,                  int, 0,       "ICP implementation: 0 = Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).");

#endif
RTABMAP_PARAM(Icp, VoxelSize,                 float, 0.05,  "Uniform sampling voxel size (0 = disabled).");
RTABMAP_PARAM(Icp, DownsamplingStep,          int, 1,       "Downsampling step size (1 = no sampling). This is done before uniform sampling.");
RTABMAP_PARAM(Icp, RangeMin,                  float, 0,     "Minimum range filtering (0 = disabled).");
RTABMAP_PARAM(Icp, RangeMax,                  float, 0,     "Maximum range filtering (0 = disabled).");

#endif
RTABMAP_PARAM(Icp, PointToPlaneMinComplexity,   float, 0.02,  uFormat("Minimum structural complexity (0.0 = low, 1.0=high) of the scan to do PointToPlane registration, otherwise PointToPoint registration is done instead and strategy from %s is used. This check is done only when %s=true.", kIcpPointToPlaneLowComplexityStrategy().c_str(), kIcpPointToPlane().c_str()));
RTABMAP_PARAM(Stereo, SSD,                   bool, true,    uFormat("[%s = false] Use Sum of Squared Differences (SSD) window, otherwise Sum of Absolute Differences (SAD) window is used.", kStereoOpticalFlow().c_str()));
RTABMAP_PARAM(Stereo, Eps,                   double, 0.01,  uFormat("[%s = true] Epsilon stop criterion.", kStereoOpticalFlow().c_str()));
RTABMAP_PARAM(Stereo, DenseStrategy,         int, 0,  "0 = cv::StereoBM, 1=cv::StereoSGBM");

#endif
RTABMAP_PARAM(Grid, Sensor,                  int,    1,       "Create occupancy grid from selected sensor: 0 = laser scan, 1=depth image(s) or 2=both laser scan and depth image(s).");
RTABMAP_PARAM(Grid, DepthDecimation,         unsigned int,  4, uFormat("[%s = true] Decimation of the depth image before creating cloud.", kGridDepthDecimation().c_str()));
RTABMAP_PARAM(Grid, RangeMax,                float,  5.0,     "Maximum range from sensor. 0 = inf.");
RTABMAP_PARAM_STR(Grid, DepthRoiRatios,      "0.0 0.0 0.0 0.0", uFormat("[%s> = 1] Region of interest ratios [left, right, top, bottom].", kGridSensor().c_str()));
RTABMAP_PARAM(Grid, ScanDecimation,          int,    1,       uFormat("[%s = 0 or 2] Decimation of the laser scan before creating cloud.", kGridSensor().c_str()));
RTABMAP_PARAM(Grid, MaxObstacleHeight,       float,  0.0,     "Maximum obstacles height (0 = disabled).");
RTABMAP_PARAM(Grid, MinGroundHeight,         float,  0.0,     "Minimum ground height (0 = disabled).");
RTABMAP_PARAM(Grid, MaxGroundHeight,         float,  0.0,     uFormat("Maximum ground height (0 = disabled). Should be set if \"%s\" is false.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, MaxGroundAngle,          float,  45,      uFormat("[%s = true] Maximum angle (degrees) between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, NormalK,                 int,    20,      uFormat("[%s = true] K neighbors to compute normals.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, ClusterRadius,           float,  0.1,     uFormat("[%s = true] Cluster maximum radius.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, MinClusterSize,          int,    10,      uFormat("[%s = true] Minimum cluster size to project the points.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, FlatObstacleDetected,    bool,   true,    uFormat("[%s = true] Flat obstacles detected.", kGridNormalsSegmentation().c_str()));

#endif
RTABMAP_PARAM(Grid, GroundIsObstacle,           bool,   false,   uFormat("[%s = true] Ground segmentation (%s) is ignored, all points are obstacles. Use this only if you want an OctoMap with ground identified as an obstacle (e.g., with an UAV).", kGrid3D().c_str(), kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, NoiseFilteringRadius,       float,   0.0,    "Noise filtering radius (0 = disabled). Done after segmentation.");
RTABMAP_PARAM(Grid, RayTracing,                 bool,   false,   uFormat("Ray tracing is done for each occupied cell, filling unknown space between the sensor and occupied cells. If %s = true, RTAB-Map should be built with OctoMap support, otherwise 3D ray tracing is ignored.", kGrid3D().c_str()));
RTABMAP_PARAM(GridGlobal, MaxNodes,             int,    0,       "Maximum nodes assembled in the map starting from the last node (0 = unlimited).");
RTABMAP_PARAM(GridGlobal, AltitudeDelta,        float,  0,       "Assemble only nodes that have the same altitude of +-delta meters of the current pose (0 = disabled). This is used to generate 2D occupancy grid based on the current altitude (e.g., multi-floor building).");
RTABMAP_PARAM(GridGlobal, FloodFillDepth,       unsigned int, 0, "Flood fill filter (0 = disabled), used to remove empty cells outside the map. The flood fill is done at the specified depth (between 1 and 16) of the OctoMap.");
RTABMAP_PARAM(Marker, Dictionary,             int,   0,     "Dictionary to use: DICT_ARUCO_4X4_50 = 0, DICT_ARUCO_4X4_100=1, DICT_ARUCO_4X4_250=2, DICT_ARUCO_4X4_1000=3, DICT_ARUCO_5X5_50=4, DICT_ARUCO_5X5_100=5, DICT_ARUCO_5X5_250=6, DICT_ARUCO_5X5_1000=7, DICT_ARUCO_6X6_50=8, DICT_ARUCO_6X6_100=9, DICT_ARUCO_6X6_250=10, DICT_ARUCO_6X6_1000=11, DICT_ARUCO_7X7_50=12, DICT_ARUCO_7X7_100=13, DICT_ARUCO_7X7_250=14, DICT_ARUCO_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16, DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20");
RTABMAP_PARAM(Marker, VarianceAngular,        float, 0.01,  "Angular variance to set on marker detections. Set to > = 9999 to use only position (xyz) constraint in graph optimization.");
RTABMAP_PARAM(Marker, MaxRange,               float, 0.0,   "Maximum range in which markers will be detected. < = 0 for unlimited range.");
RTABMAP_PARAM(Marker, MinRange,               float, 0.0,   "Miniminum range in which markers will be detected. < = 0 for unlimited range.");
static ParametersMap parseArguments(int argc, char * argv[], bool onlyParameters = false);
static ParametersMap getDefaultOdometryParameters(bool stereo = false, bool vis = true, bool icp = false);
* If remove = true: remove parameters of the specified group.
static ParametersMap filterParameters(const ParametersMap & parameters, const std::string & group, bool remove = false);
static void readINI(const std::string & configFile, ParametersMap & parameters, bool modifiedOnly = false);
* <OldKeyName, <isEqual, NewKeyName> >, when isEqual = true, the old value can be safely copied to new parameter


[Core]
Version = 0.20.23
BRIEF\Bytes = 32
BRISK\Octaves = 3
BRISK\PatternScale = 1
BRISK\Thresh = 30
Bayes\FullPredictionUpdate = false
Bayes\PredictionLC = 0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23
Bayes\VirtualPlacePriorThr = 0.9
Db\TargetVersion = 
DbSqlite3\CacheSize = 10000
DbSqlite3\InMemory = false
DbSqlite3\JournalMode = 3
DbSqlite3\Synchronous = 0
DbSqlite3\TempStore = 2
FAST\CV = 0
FAST\Gpu = false
FAST\GpuKeypointsRatio = 0.05
FAST\GridCols = 0
FAST\GridRows = 0
FAST\MaxThreshold = 200
FAST\MinThreshold = 7
FAST\NonmaxSuppression = true
FAST\Threshold = 20
FREAK\NOctaves = 4
FREAK\OrientationNormalized = true
FREAK\PatternScale = 22
FREAK\ScaleNormalized = true
GFTT\BlockSize = 3
GFTT\K = 0.04
GFTT\MinDistance = 7
GFTT\QualityLevel = 0.001
GFTT\UseHarrisDetector = false
GMS\ThresholdFactor = 6.0
GMS\WithRotation = false
GMS\WithScale = false
GTSAM\Optimizer = 1
Grid\3D = true
Grid\CellSize = 0.05
Grid\ClusterRadius = 0.1
Grid\DepthDecimation = 4
Grid\DepthRoiRatios = 0.0 0.0 0.0 0.0
Grid\FlatObstacleDetected = true
Grid\FootprintHeight = 0.0
Grid\FootprintLength = 0.0
Grid\FootprintWidth = 0.0
Grid\GroundIsObstacle = false
Grid\MapFrameProjection = false
Grid\MaxGroundAngle = 45
Grid\MaxGroundHeight = 0.0
Grid\MaxObstacleHeight = 0.0
Grid\MinClusterSize = 10
Grid\MinGroundHeight = 0.0
Grid\NoiseFilteringMinNeighbors = 5
Grid\NoiseFilteringRadius = 0.0
Grid\NormalK = 20
Grid\NormalsSegmentation = true
Grid\PreVoxelFiltering = true
Grid\RangeMax = 5
Grid\RangeMin = 0.0
Grid\RayTracing = false
Grid\Scan2dUnknownSpaceFilled = false
Grid\ScanDecimation = 1
Grid\Sensor = 0
GridGlobal\AltitudeDelta = 0
GridGlobal\Eroded = false
GridGlobal\FloodFillDepth = 0
GridGlobal\FootprintRadius = 0.0
GridGlobal\FullUpdate = true
GridGlobal\MaxNodes = 0
GridGlobal\MinSize = 0.0
GridGlobal\OccupancyThr = 0.5
GridGlobal\ProbClampingMax = 0.971
GridGlobal\ProbClampingMin = 0.1192
GridGlobal\ProbHit = 0.7
GridGlobal\ProbMiss = 0.4
GridGlobal\UpdateError = 0.01
Icp\CCFilterOutFarthestPoints = false
Icp\CCMaxFinalRMS = 0.2
Icp\CCSamplingLimit = 50000
Icp\CorrespondenceRatio = 0.1
Icp\DebugExportFormat = 
Icp\DownsamplingStep = 1
Icp\Epsilon = 0
Icp\Force4DoF = false
Icp\Iterations = 30
Icp\MaxCorrespondenceDistance = 0.1
Icp\MaxRotation = 0.78
Icp\MaxTranslation = 0.2
Icp\OutlierRatio = 0.85
Icp\PMConfig = 
Icp\PMMatcherEpsilon = 0.0
Icp\PMMatcherIntensity = false
Icp\PMMatcherKnn = 1
Icp\PointToPlane = true
Icp\PointToPlaneGroundNormalsUp = 0.0
Icp\PointToPlaneK = 5
Icp\PointToPlaneLowComplexityStrategy = 1
Icp\PointToPlaneMinComplexity = 0.02
Icp\PointToPlaneRadius = 0
Icp\RangeMax = 0
Icp\RangeMin = 0
Icp\Strategy = 1
Icp\VoxelSize = 0.05
ImuFilter\ComplementaryBiasAlpha = 0.01
ImuFilter\ComplementaryDoAdpativeGain = true
ImuFilter\ComplementaryDoBiasEstimation = true
ImuFilter\ComplementaryGainAcc = 0.01
ImuFilter\MadgwickGain = 0.1
ImuFilter\MadgwickZeta = 0.0
KAZE\Diffusivity = 1
KAZE\Extended = false
KAZE\NOctaveLayers = 4
KAZE\NOctaves = 4
KAZE\Threshold = 0.001
KAZE\Upright = false
Kp\BadSignRatio = 0.5
Kp\ByteToFloat = false
Kp\DetectorStrategy = 8
Kp\DictionaryPath = 
Kp\FlannRebalancingFactor = 2.0
Kp\GridCols = 1
Kp\GridRows = 1
Kp\IncrementalDictionary = true
Kp\IncrementalFlann = true
Kp\MaxDepth = 0
Kp\MaxFeatures = -1
Kp\MinDepth = 0
Kp\NNStrategy = 1
Kp\NewWordsComparedTogether = true
Kp\NndrRatio = 0.8
Kp\Parallelized = true
Kp\RoiRatios = 0.0 0.0 0.0 0.0
Kp\SubPixEps = 0.02
Kp\SubPixIterations = 0
Kp\SubPixWinSize = 3
Kp\TfIdfLikelihoodUsed = true
Marker\CornerRefinementMethod = 0
Marker\Dictionary = 0
Marker\Length = 0
Marker\MaxDepthError = 0.01
Marker\MaxRange = 0.0
Marker\MinRange = 0.0
Marker\Priors = 
Marker\PriorsVarianceAngular = 0.001
Marker\PriorsVarianceLinear = 0.001
Marker\VarianceAngular = 0.01
Marker\VarianceLinear = 0.001
Mem\BadSignaturesIgnored = false
Mem\BinDataKept = true
Mem\CompressionParallelized = true
Mem\CovOffDiagIgnored = true
Mem\DepthAsMask = true
Mem\GenerateIds = true
Mem\ImageCompressionFormat = .jpg
Mem\ImageKept = false
Mem\ImagePostDecimation = 1
Mem\ImagePreDecimation = 1
Mem\IncrementalMemory = true
Mem\InitWMWithAllNodes = false
Mem\IntermediateNodeDataKept = false
Mem\LaserScanDownsampleStepSize = 1
Mem\LaserScanNormalK = 0
Mem\LaserScanNormalRadius = 0.0
Mem\LaserScanVoxelSize = 0.0
Mem\LocalizationDataSaved = false
Mem\MapLabelsAdded = true
Mem\NotLinkedNodesKept = true
Mem\RawDescriptorsKept = true
Mem\RecentWmRatio = 0.2
Mem\ReduceGraph = false
Mem\RehearsalIdUpdatedToNewOne = false
Mem\RehearsalSimilarity = 0.6
Mem\RehearsalWeightIgnoredWhileMoving = false
Mem\STMSize = 10
Mem\SaveDepth16Format = false
Mem\StereoFromMotion = false
Mem\TransferSortingByWeightId = false
Mem\UseOdomFeatures = true
Mem\UseOdomGravity = false
ORB\EdgeThreshold = 19
ORB\FirstLevel = 0
ORB\Gpu = false
ORB\NLevels = 3
ORB\PatchSize = 31
ORB\ScaleFactor = 2
ORB\ScoreType = 0
ORB\WTA_K = 2
Optimizer\Epsilon = 0.0
Optimizer\GravitySigma = 0.3
Optimizer\Iterations = 50
Optimizer\LandmarksIgnored = false
Optimizer\PriorsIgnored = true
Optimizer\Robust = false
Optimizer\Strategy = 1
Optimizer\VarianceIgnored = false
PyDetector\Cuda = true
PyDetector\Path = 
PyMatcher\Cuda = true
PyMatcher\Iterations = 20
PyMatcher\Model = indoor
PyMatcher\Path = 
PyMatcher\Threshold = 0.2
RGBD\AngularSpeedUpdate = 0.0
RGBD\AngularUpdate = 0.1
RGBD\CreateOccupancyGrid = true
RGBD\Enabled = true
RGBD\GoalReachedRadius = 0.5
RGBD\GoalsSavedInUserData = false
RGBD\InvertedReg = false
RGBD\LinearSpeedUpdate = 0.0
RGBD\LinearUpdate = 0.1
RGBD\LocalBundleOnLoopClosure = false
RGBD\LocalImmunizationRatio = 0.25
RGBD\LocalRadius = 10
RGBD\LoopClosureIdentityGuess = false
RGBD\LoopClosureReextractFeatures = false
RGBD\LoopCovLimited = false
RGBD\MarkerDetection = false
RGBD\MaxLocalRetrieved = 2
RGBD\MaxLoopClosureDistance = 0.0
RGBD\MaxOdomCacheSize = 10
RGBD\NeighborLinkRefining = false
RGBD\NewMapOdomChangeDistance = 0
RGBD\OptimizeFromGraphEnd = false
RGBD\OptimizeMaxError = 3.0
RGBD\PlanAngularVelocity = 0
RGBD\PlanLinearVelocity = 0
RGBD\PlanStuckIterations = 0
RGBD\ProximityAngle = 45
RGBD\ProximityBySpace = true
RGBD\ProximityByTime = false
RGBD\ProximityGlobalScanMap = false
RGBD\ProximityMaxGraphDepth = 50
RGBD\ProximityMaxPaths = 3
RGBD\ProximityMergedScanCovFactor = 100.0
RGBD\ProximityOdomGuess = false
RGBD\ProximityPathFilteringRadius = 1
RGBD\ProximityPathMaxNeighbors = 0
RGBD\ProximityPathRawPosesUsed = true
RGBD\ScanMatchingIdsSavedInLinks = true
RGBD\StartAtOrigin = false
Reg\Force3DoF = false
Reg\RepeatOnce = true
Reg\Strategy = 1
Rtabmap\ComputeRMSE = true
Rtabmap\CreateIntermediateNodes = false
Rtabmap\DetectionRate = 1
Rtabmap\ImageBufferSize = 1
Rtabmap\ImagesAlreadyRectified = true
Rtabmap\LoopGPS = true
Rtabmap\LoopRatio = 0
Rtabmap\LoopThr = 0.11
Rtabmap\MaxRepublished = 2
Rtabmap\MaxRetrieved = 2
Rtabmap\MemoryThr = 0
Rtabmap\PublishLastSignature = true
Rtabmap\PublishLikelihood = true
Rtabmap\PublishPdf = true
Rtabmap\PublishRAMUsage = false
Rtabmap\PublishStats = true
Rtabmap\RectifyOnlyFeatures = false
Rtabmap\SaveWMState = false
Rtabmap\StartNewMapOnGoodSignature = false
Rtabmap\StartNewMapOnLoopClosure = false
Rtabmap\StatisticLogged = false
Rtabmap\StatisticLoggedHeaders = true
Rtabmap\StatisticLogsBufferedInRAM = true
Rtabmap\TimeThr = 0
Rtabmap\WorkingDirectory = /home/ros/.ros
SIFT\ContrastThreshold = 0.04
SIFT\EdgeThreshold = 10
SIFT\NFeatures = 0
SIFT\NOctaveLayers = 3
SIFT\RootSIFT = false
SIFT\Sigma = 1.6
SURF\Extended = false
SURF\GpuKeypointsRatio = 0.01
SURF\GpuVersion = false
SURF\HessianThreshold = 500
SURF\OctaveLayers = 2
SURF\Octaves = 4
SURF\Upright = false
Stereo\DenseStrategy = 0
Stereo\Eps = 0.01
Stereo\Iterations = 30
Stereo\MaxDisparity = 128.0
Stereo\MaxLevel = 5
Stereo\MinDisparity = 0.5
Stereo\OpticalFlow = true
Stereo\SSD = true
Stereo\WinHeight = 3
Stereo\WinWidth = 15
StereoBM\BlockSize = 15
StereoBM\Disp12MaxDiff = -1
StereoBM\MinDisparity = 0
StereoBM\NumDisparities = 128
StereoBM\PreFilterCap = 31
StereoBM\PreFilterSize = 9
StereoBM\SpeckleRange = 4
StereoBM\SpeckleWindowSize = 100
StereoBM\TextureThreshold = 10
StereoBM\UniquenessRatio = 15
StereoSGBM\BlockSize = 15
StereoSGBM\Disp12MaxDiff = 1
StereoSGBM\MinDisparity = 0
StereoSGBM\Mode = 2
StereoSGBM\NumDisparities = 128
StereoSGBM\P1 = 2
StereoSGBM\P2 = 5
StereoSGBM\PreFilterCap = 31
StereoSGBM\SpeckleRange = 4
StereoSGBM\SpeckleWindowSize = 100
StereoSGBM\UniquenessRatio = 20
SuperPoint\Cuda = true
SuperPoint\ModelPath = 
SuperPoint\NMS = true
SuperPoint\NMSRadius = 4
SuperPoint\Threshold = 0.010
VhEp\Enabled = false
VhEp\MatchCountMin = 8
VhEp\RansacParam1 = 3
VhEp\RansacParam2 = 0.99
Vis\BundleAdjustment = 1
Vis\CorFlowEps = 0.01
Vis\CorFlowIterations = 30
Vis\CorFlowMaxLevel = 3
Vis\CorFlowWinSize = 16
Vis\CorGuessMatchToProjection = false
Vis\CorGuessWinSize = 40
Vis\CorNNDR = 0.8
Vis\CorNNType = 1
Vis\CorType = 0
Vis\DepthAsMask = true
Vis\EpipolarGeometryVar = 0.1
Vis\EstimationType = 1
Vis\FeatureType = 8
Vis\ForwardEstOnly = true
Vis\GridCols = 1
Vis\GridRows = 1
Vis\InlierDistance = 0.1
Vis\Iterations = 300
Vis\MaxDepth = 0
Vis\MaxFeatures = 1000
Vis\MeanInliersDistance = 0.0
Vis\MinDepth = 0
Vis\MinInliers = 20
Vis\MinInliersDistribution = 0.0
Vis\PnPFlags = 0
Vis\PnPMaxVariance = 0.0
Vis\PnPRefineIterations = 0
Vis\PnPReprojError = 2
Vis\RefineIterations = 5
Vis\RoiRatios = 0.0 0.0 0.0 0.0
Vis\SubPixEps = 0.02
Vis\SubPixIterations = 0
Vis\SubPixWinSize = 3
g2o\Baseline = 0.075
g2o\Optimizer = 0
g2o\PixelVariance = 1.0
g2o\RobustKernelDelta = 8
g2o\Solver = 0
