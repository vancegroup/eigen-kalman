
if (SUPERLU_INCLUDES AND SUPERLU_LIBRARIES)
  set(SUPERLU_FIND_QUIETLY TRUE)
endif (SUPERLU_INCLUDES AND SUPERLU_LIBRARIES)

find_package(BLAS)

if(BLAS_FOUND)
  
  find_path(SUPERLU_INCLUDES
    NAMES
    superlu/supermatrix.h
    PATHS
    $ENV{SUPERLUDIR}
    ${INCLUDE_INSTALL_DIR}
  )

  find_library(SUPERLU_LIBRARIES superlu PATHS $ENV{SUPERLUDIR} ${LIB_INSTALL_DIR})
  
  if(SUPERLU_LIBRARIES)
    set(SUPERLU_LIBRARIES ${SUPERLU_LIBRARIES} ${BLAS_LIBRARIES})
  endif(SUPERLU_LIBRARIES)
  
endif(BLAS_FOUND)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SUPERLU DEFAULT_MSG
                                  SUPERLU_INCLUDES SUPERLU_LIBRARIES)

mark_as_advanced(SUPERLU_INCLUDES SUPERLU_LIBRARIES)
