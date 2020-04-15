# CMake generated Testfile for 
# Source directory: /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2
# Build directory: /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(out_low_linv_bound "test_linvel_controller" "1")
add_test(low_linv_bound "test_linvel_controller" "2")
add_test(upp_linv_bound "test_linvel_controller" "3")
add_test(out_upp_linv_bound "test_linvel_controller" "4")
add_test(zero_angle "test_angvel_controller" "1")
add_test(positive_angle "test_angvel_controller" "2")
add_test(positive_angle_outside_limits "test_angvel_controller" "3")
add_test(negative_angle "test_angvel_controller" "4")
add_test(negative_angle_outside_limits "test_angvel_last")
add_test(upper_posbound "test_posbounds" "1")
add_test(lower_posbound "test_posbounds" "2")
add_test(left_posbound "test_posbounds" "3")
add_test(right_posbound "test_posbounds" "4")
add_test(low_ang_bound "test_posbounds" "5")
add_test(upp_ang_bound "test_posbounds" "6")
add_test(up_linx_bound "test_velbounds" "1")
add_test(up_liny_bound "test_velbounds" "2")
add_test(up_th_bound "test_velbounds" "3")
add_test(low_linx_bound "test_velbounds" "4")
add_test(low_liny_bound "test_velbounds" "5")
add_test(low_th_bound "test_velbounds" "6")
