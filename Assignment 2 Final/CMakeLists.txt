cmake_minimum_required(VERSION 2.6.0)

project(VehicleSimulator C)
find_package(Threads)

include_directories(include/)

set (SimulatorFiles)
list(APPEND SimulatorFiles
	include/client.h
	include/simulator.h
	include/controller.h
	include/vehicle.h
	src/client.c
	src/simulator.c
	src/vehicle.c
	src/controller.c
)
add_definitions(-DPORTNUM=1635 -DIP="127.0.0.1")

# create the executable
add_executable(main src/main.c ${SimulatorFiles})
target_link_libraries(main m ${CMAKE_THREAD_LIBS_INIT}) 

# note that we would normally create a library for the simulator files and
# link our tests and main function against that. for this assignment, to keep
# things simple, we'll compile separate binaries.

# now create our unit tests
enable_testing() # enables the CTEST framework

add_executable(test_linvel_controller tests/test_linvel_controller.c ${SimulatorFiles})
target_link_libraries(test_linvel_controller m)
## this created the executable, now lets add a test
add_test(out_low_linv_bound test_linvel_controller 1)
add_test(low_linv_bound test_linvel_controller 2)
add_test(upp_linv_bound test_linvel_controller 3)
add_test(out_upp_linv_bound test_linvel_controller 4)

add_executable(test_angvel_controller tests/test_linvel_controller.c ${SimulatorFiles})
target_link_libraries(test_angvel_controller m)
add_test(zero_angle test_angvel_controller 1)
add_test(positive_angle test_angvel_controller 2)
add_test(positive_angle_outside_limits test_angvel_controller 3)
add_test(negative_angle test_angvel_controller 4)

add_executable(test_angvel_last tests/test_angvel_last.c ${SimulatorFiles})
target_link_libraries(test_angvel_last m)
add_test(negative_angle_outside_limits test_angvel_last)

add_executable(test_posbounds tests/test_posbounds.c ${SimulatorFiles})
target_link_libraries(test_posbounds m)
add_test(upper_posbound test_posbounds 1)
add_test(lower_posbound test_posbounds 2)
add_test(left_posbound test_posbounds 3)
add_test(right_posbound test_posbounds 4)
add_test(low_ang_bound test_posbounds 5)
add_test(upp_ang_bound test_posbounds 6)

add_executable(test_velbounds tests/test_velbounds.c ${SimulatorFiles})
target_link_libraries(test_velbounds m)
add_test(up_linx_bound test_velbounds 1)
add_test(up_liny_bound test_velbounds 2)
add_test(up_th_bound test_velbounds 3)
add_test(low_linx_bound test_velbounds 4)
add_test(low_liny_bound test_velbounds 5)
add_test(low_th_bound test_velbounds 6)

