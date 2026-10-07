add_library(mm_test_support INTERFACE)
target_include_directories(mm_test_support INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

function(add_mm_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE ballistics-multimodel mm_test_support)
  add_test(NAME ${name} COMMAND ${name} ${ARGN})
endfunction()

add_mm_test(mm_test_atmosphere)
add_mm_test(mm_test_drag)
add_mm_test(mm_test_wind)
add_mm_test(mm_test_geo)
add_mm_test(mm_test_catalogue)
add_mm_test(mm_test_models)
add_mm_test(mm_test_swap)
add_mm_test(mm_test_conservation)
add_mm_test(mm_test_adaptive)
add_mm_test(mm_test_rangetable)

add_test(NAME mm_self_contained
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_self_contained.sh ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Against a file recorded by ballistics-pointmass, so the check survives the two libraries living
# in separate repositories and needs no link-time dependency on it.
add_mm_test(mm_test_crosslibrary ${CMAKE_CURRENT_SOURCE_DIR}/data/pointmass-reference.csv)

# Locks the committed catalogue to the generator, so every derived number in it stays traceable.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME mm_catalogue_generated
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/../tools/generate_catalogue.py --check)
endif()
