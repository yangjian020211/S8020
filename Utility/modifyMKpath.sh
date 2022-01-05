#!/bin/bash
# Author: Minzhao
# Date: 2016-12-15
# Verison: 0.1
# This script is modify the related path arguments for sample code
# you could do like this to convert the file to linux line-ending format:
#       vi modifyMKpath.sh
#       :set ff=unix and :wq 

SDK_DIR=../../../Output/AR8020SDK
echo $SDK_DIR
#modify the sample top makefile
cd $SDK_DIR/Application
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ..'  ./Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./Makefile
sed -i '/cp \*\.bin/d' ./Makefile

#modify config data src in src.cfg
sed -i '/^CONFIG_DATA_DIR ?= /cCONFIG_DATA_DIR ?= ./ConfigData'  ./src.cfg

#modify the cpu0 sample top makefile
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ../..'  ./cpu0/Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./cpu0/Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./cpu0/Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./cpu0/Makefile

#modify the cpu1 sample top makefile
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ../..'  ./cpu1/Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./cpu1/Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./cpu1/Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./cpu1/Makefile

#modify the cpu2 sample top makefile
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ../..'  ./cpu2/Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./cpu2/Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./cpu2/Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./cpu2/Makefile


cd $SDK_DIR/Backup
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ..'  ./Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./Makefile
sed -i '/cp \*\.bin/d' ./Makefile

#modify the cpu0 sample top makefile
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ../..'  ./cpu0/Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./cpu0/Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./cpu0/Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./cpu0/Makefile

#modify the cpu1 sample top makefile
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ../..'  ./cpu1/Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./cpu1/Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./cpu1/Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./cpu1/Makefile

#modify the cpu2 sample top makefile
sed -i '/^TOP_DIR ?= /cTOP_DIR ?= ../..'  ./cpu2/Makefile
sed -i '/^OUTPUT_DIR ?= /cOUTPUT_DIR ?= $(TOP_DIR)'  ./cpu2/Makefile
sed -i '/^OUTPUT_HEADER_STAGING_DIR ?= /cOUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Inc'  ./cpu2/Makefile
sed -i '/^OUTPUT_LIB_STAGING_DIR ?= /cOUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Lib'  ./cpu2/Makefile

#modify the joint2flash.sh
cd $SDK_DIR/Utility
echo `pwd`
sed -i '/^bootload=/cbootload=../Lib/ar8020_boot.bin'  ./joint2flash.sh
sed -i '/^upgrade=/cupgrade=../Lib/ar8020_upgrade.bin'  ./joint2flash.sh
sed -i '/^skycpu0=/cskycpu0=../Lib/ar8020_skycpu0.bin'  ./joint2flash.sh
sed -i '/^skycpu1=/cskycpu1=../Lib/ar8020_skycpu1.bin'  ./joint2flash.sh
sed -i '/^skycpu2=/cskycpu2=../Lib/ar8020_skycpu2.bin'  ./joint2flash.sh
sed -i '/^gndcpu0=/cgndcpu0=../Lib/ar8020_gndcpu0.bin'  ./joint2flash.sh
sed -i '/^gndcpu1=/cgndcpu1=../Lib/ar8020_gndcpu1.bin'  ./joint2flash.sh
sed -i '/^gndcpu2=/cgndcpu2=../Lib/ar8020_gndcpu2.bin'  ./joint2flash.sh
sed -i '/^bckcpu0=/cbckcpu0=../Lib/ar8020_backupcpu0.bin'  ./joint2flash.sh
sed -i '/^bckcpu1=/cbckcpu1=../Lib/ar8020_backupcpu1.bin'  ./joint2flash.sh
sed -i '/^bckcpu2=/cbckcpu2=../Lib/ar8020_backupcpu2.bin'  ./joint2flash.sh
sed -i '/^ve=/cve=../Utility/imageinfo'  ./joint2flash.sh
sed -i '/^Bin=/cBin=../Bin'  ./joint2flash.sh

#modify the jointbin.sh
sed -i '/^Bin=/cBin=../Output/Staging/Config'  ./jointbin.sh