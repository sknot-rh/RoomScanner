cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (RoomScanner)
find_package (Qt5 REQUIRED COMPONENTS Widgets Core)
#find_package (VTK 6.3.0 REQUIRED)
find_package (VTK 7.1.0 REQUIRED)
find_package (PCL 1.8 REQUIRED)


include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (CMAKE_AUTORCC ON)
set  (project_SOURCES main.cpp application.cpp filters.cpp mesh.cpp registration.cpp)
set  (project_HEADERS application.h parameters.h filters.h pointrepr.h mesh.h registration.h types.h)
set  (project_FORMS   application.ui)
set  (project_RESOURCES Resources/Resources.qrc)
#set  (CMAKE_CXX_FLAGS -g)
set(CMAKE_CXX_STANDARD 11)


QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})
QT5_ADD_RESOURCES(project_RESOURCES_RCC ${project_RESOURCES})

ADD_DEFINITIONS (${QT_DEFINITIONS})


ADD_EXECUTABLE  (RoomScanner ${project_SOURCES}
                                ${project_FORMS_HEADERS}
								${project_RESOURCES}
                                ${project_HEADERS_MOC})


TARGET_LINK_LIBRARIES (RoomScanner ${PCL_LIBRARIES})

qt5_use_modules (RoomScanner Widgets)
