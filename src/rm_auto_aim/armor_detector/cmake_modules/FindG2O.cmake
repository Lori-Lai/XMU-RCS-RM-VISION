# FindG2O.cmake

# Locate the g2o libraries
# A general framework for graph optimization.

# This module defines:
#   G2O_FOUND         - set to TRUE if G2O was found
#   G2O_INCLUDE_DIR   - where to find the g2o headers
#   G2O_LIBRARIES     - the list of libraries to link against

if(UNIX)
  message(STATUS "Searching for g2o ...")

  # 查找头文件路径
  find_path(G2O_INCLUDE_DIR
    NAMES core math_groups types
    PATHS /usr/local /usr
    PATH_SUFFIXES include/g2o include
  )

  if(G2O_INCLUDE_DIR)
    message(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  endif()

  # 查找你现有的库
  find_library(G2O_CORE_LIB             NAMES g2o_core             PATHS /usr/local /usr PATH_SUFFIXES lib)
  find_library(G2O_STUFF_LIB            NAMES g2o_stuff            PATHS /usr/local /usr PATH_SUFFIXES lib)
  find_library(G2O_TYPES_SLAM2D_LIB     NAMES g2o_types_slam2d     PATHS /usr/local /usr PATH_SUFFIXES lib)
  find_library(G2O_TYPES_SLAM3D_LIB     NAMES g2o_types_slam3d     PATHS /usr/local /usr PATH_SUFFIXES lib)
  find_library(G2O_SOLVER_DENSE_LIB     NAMES g2o_solver_dense     PATHS /usr/local /usr PATH_SUFFIXES lib)
  find_library(G2O_SOLVER_EIGEN_LIB     NAMES g2o_solver_eigen     PATHS /usr/local /usr PATH_SUFFIXES lib)

  # 构建库列表（仅添加找到的库）
  set(G2O_LIBRARIES
    ${G2O_CORE_LIB}
    ${G2O_STUFF_LIB}
    ${G2O_TYPES_SLAM2D_LIB}
    ${G2O_TYPES_SLAM3D_LIB}
    ${G2O_SOLVER_DENSE_LIB}
    ${G2O_SOLVER_EIGEN_LIB}
  )

  # 删除未找到（为空）的项
  list(REMOVE_ITEM G2O_LIBRARIES "")

  # 判断是否找到所有必要组件
  if(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    set(G2O_FOUND TRUE)
    message(STATUS "Found libg2o: ${G2O_LIBRARIES}")
  else()
    set(G2O_FOUND FALSE)
    message(WARNING "Could not find complete libg2o setup. Found: ${G2O_LIBRARIES}, Headers: ${G2O_INCLUDE_DIR}")
  endif()
endif()