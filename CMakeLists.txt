cmake_minimum_required(VERSION 2.8)

option( INSTALL_QCANUPO2_PLUGIN "Check to install qCanupo2 plugin" OFF )

if (INSTALL_QCANUPO2_PLUGIN)

#New classification plugin (Dimitri Lague/OSUR)
project( QCANUPO2_PLUGIN_DLL )

	#need OpenCV
	find_package( OpenCV REQUIRED )

	#As we are going to re-use several components of the main qCC project
	#we define its sources path relatively to the plugin's dir
	set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../qCC)

	#include the whole CC directory (easier)
	include_directories( ${CloudCompare_SOURCE_DIR} )
	
	#we need the "order choice" dialog
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccOrderChoiceDlg*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccOrderChoiceDlg*.cpp )

	#don't forget associated widgets(.ui)
	set( CC_PLUGIN_CUSTOM_UI_LIST	${CloudCompare_SOURCE_DIR}/ui_templates/roleChoiceDlg.ui )

	include( ../../CMakePluginTpl.cmake )
	
	target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

endif()
