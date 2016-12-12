
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-visualizer)
find_package (Qt5 REQUIRED COMPONENTS Widgets Core)
find_package (VTK REQUIRED)
find_package (PCL 1.8 REQUIRED))

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp pclviewer.cpp)
set  (project_HEADERS pclviewer.h)
set  (project_FORMS   pclviewer.ui)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcl_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})


TARGET_LINK_LIBRARIES (pcl_visualizer ${PCL_LIBRARIES})

qt5_use_modules (pcl_visualizer Widgets)


