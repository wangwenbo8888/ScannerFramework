# CMake generated Testfile for 
# Source directory: E:/workfold/framework/framework/tests
# Build directory: E:/workfold/framework/build/framework/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[test_framework_smoke]=] "E:/workfold/framework/build/bin/Debug/test_framework_smoke.exe")
  set_tests_properties([=[test_framework_smoke]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/tests/CMakeLists.txt;8;add_test;E:/workfold/framework/framework/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[test_framework_smoke]=] "E:/workfold/framework/build/bin/Release/test_framework_smoke.exe")
  set_tests_properties([=[test_framework_smoke]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/tests/CMakeLists.txt;8;add_test;E:/workfold/framework/framework/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[test_framework_smoke]=] "E:/workfold/framework/build/bin/MinSizeRel/test_framework_smoke.exe")
  set_tests_properties([=[test_framework_smoke]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/tests/CMakeLists.txt;8;add_test;E:/workfold/framework/framework/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[test_framework_smoke]=] "E:/workfold/framework/build/bin/RelWithDebInfo/test_framework_smoke.exe")
  set_tests_properties([=[test_framework_smoke]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/workfold/framework/framework/tests/CMakeLists.txt;8;add_test;E:/workfold/framework/framework/tests/CMakeLists.txt;0;")
else()
  add_test([=[test_framework_smoke]=] NOT_AVAILABLE)
endif()
