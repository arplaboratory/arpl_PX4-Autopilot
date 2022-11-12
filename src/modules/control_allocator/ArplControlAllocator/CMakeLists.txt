px4_add_library(ArplControlAllocator
        ArplControlAllocator.cpp
        ArplControlAllocator.hpp
        )
target_compile_options(ArplControlAllocator PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(ArplControlAllocator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ArplControlAllocator PRIVATE mathlib)

#px4_add_unit_gtest(SRC ControlAllocationPseudoInverseTest.cpp LINKLIBS ControlAllocation)