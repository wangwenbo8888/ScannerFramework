# CMake generated Testfile for 
# Source directory: E:/workfold/framework/framework/workflow/tests
# Build directory: E:/workfold/framework/build/framework/workflow/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[test_pipeline]=] "E:/workfold/framework/build/bin/Debug/test_pipeline.exe")
  set_tests_properties([=[test_pipeline]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;4;add_test;E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[test_pipeline]=] "E:/workfold/framework/build/bin/Release/test_pipeline.exe")
  set_tests_properties([=[test_pipeline]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;4;add_test;E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[test_pipeline]=] "E:/workfold/framework/build/bin/MinSizeRel/test_pipeline.exe")
  set_tests_properties([=[test_pipeline]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;4;add_test;E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[test_pipeline]=] "E:/workfold/framework/build/bin/RelWithDebInfo/test_pipeline.exe")
  set_tests_properties([=[test_pipeline]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;4;add_test;E:/workfold/framework/framework/workflow/tests/CMakeLists.txt;0;")
else()
  add_test([=[test_pipeline]=] NOT_AVAILABLE)
endif()
