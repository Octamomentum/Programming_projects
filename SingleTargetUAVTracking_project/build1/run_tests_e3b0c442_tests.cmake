add_test([=[KinematicPrediction.TestExample]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=KinematicPrediction.TestExample]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[KinematicPrediction.TestExample]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_physconst.cpp:8]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[EstimationParams.NonPositiveParameters]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=EstimationParams.NonPositiveParameters]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[EstimationParams.NonPositiveParameters]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_UAVTracking.cpp:7]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[Variances.NonPositiveParameters]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=Variances.NonPositiveParameters]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[Variances.NonPositiveParameters]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_UAVTracking.cpp:23]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[ChoosingMethods.UnknownMethod]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=ChoosingMethods.UnknownMethod]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[ChoosingMethods.UnknownMethod]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_UAVTracking.cpp:39]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[UAVTracking.UnstableSolution]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=UAVTracking.UnstableSolution]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[UAVTracking.UnstableSolution]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_UAVTracking.cpp:56]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[UAVTracking.GuessInsideTheRadarAndUnstableSolution]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=UAVTracking.GuessInsideTheRadarAndUnstableSolution]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[UAVTracking.GuessInsideTheRadarAndUnstableSolution]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_UAVTracking.cpp:92]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[UAVTracking.WGNAndUnstableSolution]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=UAVTracking.WGNAndUnstableSolution]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[UAVTracking.WGNAndUnstableSolution]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_UAVTracking.cpp:128]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[StateEstimator.GaussNewton]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=StateEstimator.GaussNewton]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[StateEstimator.GaussNewton]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_StateEstimator.cpp:10]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[StateEstimator.LevenbergMarquardt]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=StateEstimator.LevenbergMarquardt]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[StateEstimator.LevenbergMarquardt]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_StateEstimator.cpp:66]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[StateEstimator.KalmanFilter]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=StateEstimator.KalmanFilter]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[StateEstimator.KalmanFilter]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_StateEstimator.cpp:127]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[NegativeSeeds.InvalidSeedValues]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=NegativeSeeds.InvalidSeedValues]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[NegativeSeeds.InvalidSeedValues]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_RadarSystem.cpp:9]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LandmarksInitialization.TooFewLandmarks]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=LandmarksInitialization.TooFewLandmarks]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LandmarksInitialization.TooFewLandmarks]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_RadarSystem.cpp:21]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[UAVsInitializations.TooFewUAVs]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=UAVsInitializations.TooFewUAVs]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[UAVsInitializations.TooFewUAVs]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_RadarSystem.cpp:30]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[GenerateRadar.negativeposParams]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=GenerateRadar.negativeposParams]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[GenerateRadar.negativeposParams]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_RadarSystem.cpp:39]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[GenerateRadarSystem.AWorkingExample]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=GenerateRadarSystem.AWorkingExample]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[GenerateRadarSystem.AWorkingExample]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_RadarSystem.cpp:51]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[MatrixDimension.DimensionIndexTest]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=MatrixDimension.DimensionIndexTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[MatrixDimension.DimensionIndexTest]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_MatrixObject.cpp:6]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[MatrixCopy.CopyTest]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=MatrixCopy.CopyTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[MatrixCopy.CopyTest]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_MatrixObject.cpp:27]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[MatrixMove.MoveTest]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=MatrixMove.MoveTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[MatrixMove.MoveTest]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_MatrixObject.cpp:38]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[MatrixOperations.CalcTest]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=MatrixOperations.CalcTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[MatrixOperations.CalcTest]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_MatrixObject.cpp:49]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LeastSquare.StandardOperations]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=LeastSquare.StandardOperations]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LeastSquare.StandardOperations]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_MatrixCalculations.cpp:10]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LeastSquare.QRExample]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=LeastSquare.QRExample]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LeastSquare.QRExample]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_MatrixCalculations.cpp:101]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LinkedListTest.AddingAndRemovingElements]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=LinkedListTest.AddingAndRemovingElements]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LinkedListTest.AddingAndRemovingElements]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_LinkedListObject.cpp:7]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LinkedListTest.CopyTest]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=LinkedListTest.CopyTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LinkedListTest.CopyTest]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_LinkedListObject.cpp:40]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LinkedListTest.MoveTest]=]  C:/Programming_projects/SingleTargetUAVTracking_project/build1/run_tests.exe [==[--gtest_filter=LinkedListTest.MoveTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LinkedListTest.MoveTest]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Programming_projects\SingleTargetUAVTracking_project\tests\test_LinkedListObject.cpp:56]==]
    WORKING_DIRECTORY [==[C:/Programming_projects/SingleTargetUAVTracking_project/build1]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
set(run_tests_TESTS [==[KinematicPrediction.TestExample]==] [==[EstimationParams.NonPositiveParameters]==] [==[Variances.NonPositiveParameters]==] [==[ChoosingMethods.UnknownMethod]==] [==[UAVTracking.UnstableSolution]==] [==[UAVTracking.GuessInsideTheRadarAndUnstableSolution]==] [==[UAVTracking.WGNAndUnstableSolution]==] [==[StateEstimator.GaussNewton]==] [==[StateEstimator.LevenbergMarquardt]==] [==[StateEstimator.KalmanFilter]==] [==[NegativeSeeds.InvalidSeedValues]==] [==[LandmarksInitialization.TooFewLandmarks]==] [==[UAVsInitializations.TooFewUAVs]==] [==[GenerateRadar.negativeposParams]==] [==[GenerateRadarSystem.AWorkingExample]==] [==[MatrixDimension.DimensionIndexTest]==] [==[MatrixCopy.CopyTest]==] [==[MatrixMove.MoveTest]==] [==[MatrixOperations.CalcTest]==] [==[LeastSquare.StandardOperations]==] [==[LeastSquare.QRExample]==] [==[LinkedListTest.AddingAndRemovingElements]==] [==[LinkedListTest.CopyTest]==] [==[LinkedListTest.MoveTest]==])
