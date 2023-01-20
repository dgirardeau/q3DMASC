cmake_minimum_required(VERSION 2.8)

option( INSTALL_Q3DMASC_PLUGIN "Check to install q3DMASC plugin" OFF )

if (INSTALL_Q3DMASC_PLUGIN)

	#New classification plugin (Dimitri Lague/OSUR)
	project( Q3DMASC_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	include( CMakePolicies NO_POLICY_SCOPE )

	#need OpenCV
	find_package( OpenCV REQUIRED )

	#As we are going to re-use several components of the main qCC project
	#we define its sources path relatively to the plugin's dir
	set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../qCC)

	#include the whole CC directory (easier)
	include_directories( ${CloudCompare_SOURCE_DIR} )
	#include_directories( ${OpenCV_INCLUDE_DIR} )
	
	file( GLOB PLUGIN_HDR_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.h )
	file( GLOB PLUGIN_SRC_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp )
	file( GLOB PLUGIN_UI_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.ui )
	
	#we need the "order choice" dialog
	file( GLOB CC_HDR_LIST ${CloudCompare_SOURCE_DIR}/ccOrderChoiceDlg*.h )
	file( GLOB CC_SRC_LIST ${CloudCompare_SOURCE_DIR}/ccOrderChoiceDlg*.cpp )
	set( CC_UI_LIST	${CloudCompare_SOURCE_DIR}/ui_templates/roleChoiceDlg.ui )

	# add source files
	target_sources( ${PROJECT_NAME}
		PRIVATE
			${PLUGIN_HDR_LIST}
			${PLUGIN_SRC_LIST}
			${PLUGIN_UI_LIST}
			${CC_HDR_LIST}
			${CC_SRC_LIST}
			${CC_UI_LIST}
	)

    #include( CMakePluginTemplate )
	#include( ../../CMakePluginTpl.cmake )
	
    set_target_properties( ${PROJECT_NAME} PROPERTIES
        # FIXME Remove after everything has moved to targets
        AUTOUIC_SEARCH_PATHS "${CloudCompare_SOURCE_DIR}/ui_templates;${CMAKE_CURRENT_SOURCE_DIR}/ui"
    )

    # FIXME These are required because we have dependencies outside of the api
    target_include_directories( ${PROJECT_NAME}
        PRIVATE
            ${CloudCompare_SOURCE_DIR}
            ${CloudCompare_SOURCE_DIR}/../common
    )

        target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
		set( OPENCV_DEP_DLL_FILES ${OpenCV_DIR}/x64/vc15/bin/opencv_world340.dll )
        copy_files("${OPENCV_DEP_DLL_FILES}" "${CLOUDCOMPARE_DEST_FOLDER}") #mind the quotes!


endif()
